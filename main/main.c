#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_system.h" // IWYU pragma: keep
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_partition.h"
#include "driver/gpio.h" // IWYU pragma: keep
#include "driver/rmt_tx.h"

static const char *TAG = "SONY_GRX7";

/* =========================================================================
 * CONFIGURACAO DE WIFI (DUAL SSID COM FAILOVER) E PINO IR
 * Se existir 'main/wifi_secrets.h' (ignorado pelo Git), usa as credenciais
 * locais; caso contrario, usa os valores do 'idf.py menuconfig'.
 * ========================================================================= */
#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#endif

#ifndef CONFIG_SONY_WIFI_SSID_1
#define CONFIG_SONY_WIFI_SSID_1 "SEU_WIFI_PRINCIPAL"
#endif

#ifndef CONFIG_SONY_WIFI_PASSWORD_1
#define CONFIG_SONY_WIFI_PASSWORD_1 "SUA_SENHA_1"
#endif

#ifndef CONFIG_SONY_WIFI_SSID_2
#define CONFIG_SONY_WIFI_SSID_2 "SEU_WIFI_SECUNDARIO"
#endif

#ifndef CONFIG_SONY_WIFI_PASSWORD_2
#define CONFIG_SONY_WIFI_PASSWORD_2 "SUA_SENHA_2"
#endif

#ifndef CONFIG_SONY_IR_GPIO
#define CONFIG_SONY_IR_GPIO 13
#endif

#ifdef WIFI_SECRET_SSID_1
#define WIFI_SSID_1    WIFI_SECRET_SSID_1
#define WIFI_PASS_1    WIFI_SECRET_PASS_1
#define WIFI_SSID_2    WIFI_SECRET_SSID_2
#define WIFI_PASS_2    WIFI_SECRET_PASS_2
#else
#define WIFI_SSID_1    CONFIG_SONY_WIFI_SSID_1
#define WIFI_PASS_1    CONFIG_SONY_WIFI_PASSWORD_1
#define WIFI_SSID_2    CONFIG_SONY_WIFI_SSID_2
#define WIFI_PASS_2    CONFIG_SONY_WIFI_PASSWORD_2
#endif

#define IR_TX_GPIO_NUM CONFIG_SONY_IR_GPIO

/* =========================================================================
 * PARAMETROS DO PROTOCOLO SONY SIRC (40 kHz, resolucao de 1 us)
 * ========================================================================= */
#define RMT_RESOLUTION_HZ       1000000 // 1 MHz = 1 tick por microssegundo (1 us)
#define SONY_CARRIER_FREQ_HZ    40000   // Portadora Sony SIRC = 40 kHz
#define SONY_HEADER_MARK_US     2400
#define SONY_HEADER_SPACE_US    600
#define SONY_BIT_ONE_MARK_US    1200
#define SONY_BIT_ZERO_MARK_US   600
#define SONY_BIT_SPACE_US       600
#define SONY_FRAME_PERIOD_US    45000   // Periodo total CONST_LENGTH oficial Sony SIRC = 45 ms (inicio a inicio)
#define SONY_REPEAT_FRAMES      3       // Padrao oficial Sony SIRC: 3 frames por comando (~135 ms)
#define SONY_MAX_BATCH_FRAMES   5       // Maximo de frames por transacao unica na RAM de hardware RMT (5 * 21 = 105 < 128)
#define SONY_TX_TIMEOUT_MS      1000    // Timeout amplo em MILISSEGUNDOS para aguardar rajada completa no RMT
#define AUTO_POWER_ON_DELAY_MS  3000    // Espera 3s ao energizar a placa antes de enviar POWER automaticamente
#define SONY_POWER_CMD_HEX      0xA81   // POWER I/O 12-bit (Addr 0x10, Cmd 21) confirmado no MHC-GRX7
#define SONY_POWER_CMD_BITS     12

static rmt_channel_handle_t s_rmt_tx_chan = NULL;
static rmt_encoder_handle_t s_rmt_copy_encoder = NULL;
static DRAM_ATTR rmt_symbol_word_t s_tx_symbols[128]; // Buffer estatico em SRAM interna para DMA/RMT sem risco de stack

typedef struct {
    uint32_t data;
    uint8_t nbits;
    int repeats;
} ir_tx_msg_t;

static QueueHandle_t s_ir_tx_queue = NULL;

/* =========================================================================
 * INICIALIZACAO DO HARDWARE RMT (ESP-IDF v5.x / v6.x)
 * ========================================================================= */
static void ir_rmt_init(void)
{
    ESP_LOGI(TAG, "Configurando RMT TX no GPIO %d (Portadora 40kHz Sony SIRC)...", IR_TX_GPIO_NUM);

    // Garante que o GPIO 13 (RTC_GPIO14 / MTCK) saia do modo RTC/JTAG e sem pull-up antes do RMT assumir a matriz
    gpio_reset_pin(IR_TX_GPIO_NUM);
    gpio_pullup_dis(IR_TX_GPIO_NUM);
    gpio_pulldown_dis(IR_TX_GPIO_NUM);

    rmt_tx_channel_config_t tx_chan_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = IR_TX_GPIO_NUM,
        // Usa 4 blocos de memoria RMT dedicados (256 simbolos, ping_pong = 128) para enviar os 3 frames de
        // 12, 15 ou 20 bits (ate 63 simbolos) em UMA UNICA transacao de hardware sem interrupcoes no meio!
        .mem_block_symbols = 256,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .trans_queue_depth = 4,
        .flags = {
            .invert_out = false,
            .with_dma = false,
        },
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_cfg, &s_rmt_tx_chan));

    // Usa drive padrao (GPIO_DRIVE_CAP_2 = 20mA, suficiente com sobra para os ~9.5mA do resistor de 220R)
    // evitando bordas de chaveamento ultra-rapidas (harmônicos de RF/EMI no cabo de audio) do DRIVE_CAP_3
    ESP_ERROR_CHECK(gpio_set_drive_capability(IR_TX_GPIO_NUM, GPIO_DRIVE_CAP_2));

    rmt_carrier_config_t carrier_cfg = {
        .duty_cycle = 0.33f, // Duty cycle oficial Sony SIRC (1/3 = 33%): mesmo alcance IR com 34% menos ripple na fonte!
        .frequency_hz = SONY_CARRIER_FREQ_HZ,
        .flags = {
            .polarity_active_low = false,
        },
    };
    ESP_ERROR_CHECK(rmt_apply_carrier(s_rmt_tx_chan, &carrier_cfg));

    rmt_copy_encoder_config_t copy_encoder_cfg = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_encoder_cfg, &s_rmt_copy_encoder));

    ESP_ERROR_CHECK(rmt_enable(s_rmt_tx_chan));

    // Estabiliza o latch de saida RMT em nivel LOW antes do primeiro comando (igual ao setup do ESPHome)
    rmt_symbol_word_t idle_prime = { .level0 = 0, .duration0 = 1, .level1 = 0, .duration1 = 0 };
    rmt_transmit_config_t prime_cfg = { .loop_count = 0, .flags = { .eot_level = 0 } };
    ESP_ERROR_CHECK(rmt_transmit(s_rmt_tx_chan, s_rmt_copy_encoder, &idle_prime, sizeof(idle_prime), &prime_cfg));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(s_rmt_tx_chan, 100));

    ESP_LOGI(TAG, "RMT TX inicializado com sucesso (256 simbolos HW, Drive Cap = MAX 40mA)!");
}

/* =========================================================================
 * TRANSMISSAO SONY SIRC (12, 15 ou 20 bits - formato MSB-first estilo ESPHome/LIRC)
 * Exemplo: Power = 0xA81 (12 bits), Function = 0x4B09 (15 bits)
 *
 * CORRECAO CRITICA PARA 15 E 20 BITS:
 * 1) O protocolo Sony SIRC usa CONST_LENGTH = 45000 us (45 ms do inicio de um Header
 *    ate o inicio do proximo Header). Antes, o espaco fixo de 24 ms somado aos ~25 ms
 *    de um frame de 15 bits estourava o periodo para ~50+ ms, fazendo o som rejeitar!
 * 2) Agora todos os 3 frames sao empacotados em um unico buffer na SRAM interna do RMT
 *    e disparados em UMA UNICA chamada rmt_transmit(), eliminando jitter de Wi-Fi/RTOS.
 * ========================================================================= */
static void sony_transmit_msb(uint32_t data, uint8_t nbits, int repeats)
{
    if (nbits != 12 && nbits != 15 && nbits != 20) {
        nbits = 12;
    }
    if (repeats < 1) {
        repeats = SONY_REPEAT_FRAMES;
    }

    // Mascara de seguranca para garantir que nao haja bits extras acima de nbits
    uint32_t mask_valid = (1UL << nbits) - 1UL;
    data &= mask_valid;

    // Calcula a duracao ativa exata do frame (do inicio do Header ate o fim do ultimo pulso HIGH)
    uint32_t frame_active_us = SONY_HEADER_MARK_US + SONY_HEADER_SPACE_US;
    for (int i = nbits - 1; i >= 0; i--) {
        bool bit_is_one = (data >> i) & 0x1;
        frame_active_us += bit_is_one ? SONY_BIT_ONE_MARK_US : SONY_BIT_ZERO_MARK_US;
        if (i > 0) {
            frame_active_us += SONY_BIT_SPACE_US;
        }
    }

    // Calcula o espaco LOW restante para fechar cravado o periodo oficial de 45.000 us (CONST_LENGTH)
    // Cabe sempre nos 15 bits do RMT (max 32767 us): p/ 12b ~25800 us, p/ 15b ~21000 us, p/ 20b ~13000 us
    uint32_t gap_calc_us = (frame_active_us + 5000U < SONY_FRAME_PERIOD_US)
                           ? (SONY_FRAME_PERIOD_US - frame_active_us)
                           : 10000U;
    if (gap_calc_us > 32767U) {
        gap_calc_us = 32767U;
    }
    uint16_t interframe_space_us = (uint16_t)gap_calc_us;

    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,
        .flags = {
            .eot_level = 0, // Garante que o LED IR fique desligado ao terminar
        },
    };

    ESP_LOGI(TAG, "TX Sony SIRC: data=0x%lX (%u bits) x%d [ativo=%lu us, gap=%u us, periodo=%lu us]",
             (unsigned long)data, nbits, repeats,
             (unsigned long)frame_active_us, interframe_space_us,
             (unsigned long)(frame_active_us + interframe_space_us));

    int remaining = repeats;
    while (remaining > 0) {
        int batch = (remaining > SONY_MAX_BATCH_FRAMES) ? SONY_MAX_BATCH_FRAMES : remaining;
        size_t num_symbols = 0;

        for (int r = 0; r < batch; r++) {
            // Header Sony: 2400us HIGH + 600us LOW
            s_tx_symbols[num_symbols++] = (rmt_symbol_word_t){
                .level0 = 1,
                .duration0 = SONY_HEADER_MARK_US,
                .level1 = 0,
                .duration1 = SONY_HEADER_SPACE_US,
            };

            // Bits de dados MSB-first (identico ao SonyProtocol::encode do ESPHome)
            for (int i = nbits - 1; i >= 0; i--) {
                bool bit_is_one = (data >> i) & 0x1;
                // No ultimo bit (i == 0) de cada frame, aplica interframe_space_us (level LOW) para cravar
                // exatamente 45.000 us de periodo inicio-a-inicio no hardware RMT e proteger contra colisoes
                uint16_t space_us = (i == 0) ? interframe_space_us : SONY_BIT_SPACE_US;
                s_tx_symbols[num_symbols++] = (rmt_symbol_word_t){
                    .level0 = 1,
                    .duration0 = bit_is_one ? SONY_BIT_ONE_MARK_US : SONY_BIT_ZERO_MARK_US,
                    .level1 = 0,
                    .duration1 = space_us,
                };
            }
        }

        esp_err_t err = rmt_transmit(
            s_rmt_tx_chan,
            s_rmt_copy_encoder,
            s_tx_symbols,
            num_symbols * sizeof(rmt_symbol_word_t),
            &tx_cfg
        );
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Falha em rmt_transmit: %s", esp_err_to_name(err));
            break;
        }
        err = rmt_tx_wait_all_done(s_rmt_tx_chan, SONY_TX_TIMEOUT_MS);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Timeout aguardando fim da transmissao RMT: %s", esp_err_to_name(err));
            break;
        }

        remaining -= batch;
    }
}

static void ir_tx_task(void *arg)
{
    ir_tx_msg_t msg;
    while (1) {
        if (xQueueReceive(s_ir_tx_queue, &msg, portMAX_DELAY) == pdTRUE) {
            sony_transmit_msb(msg.data, msg.nbits, msg.repeats);
        }
    }
}

static void ir_enqueue_tx(uint32_t data, uint8_t nbits, int repeats)
{
    if (s_ir_tx_queue != NULL) {
        ir_tx_msg_t msg = {
            .data = data,
            .nbits = nbits,
            .repeats = repeats,
        };
        if (xQueueSend(s_ir_tx_queue, &msg, 0) != pdTRUE) {
            // Se a fila estiver cheia (ex.: segurando botao de volume), substitui o ultimo comando pendente
            ir_tx_msg_t dropped;
            xQueueReceive(s_ir_tx_queue, &dropped, 0);
            xQueueSend(s_ir_tx_queue, &msg, 0);
        }
    } else {
        sony_transmit_msb(data, nbits, repeats);
    }
}

/* =========================================================================
 * TRANSMISSAO POR (COMMAND, ADDRESS) NATIVO SONY (LSB-first)
 * Facilita testar qualquer comando da tabela oficial da Sony (0..127)
 * ========================================================================= */
static uint32_t sony_cmd_addr_to_msb(uint8_t command, uint16_t address, uint8_t nbits)
{
    uint32_t msb_data = 0;
    // Os primeiros 7 bits enviados na rede sao o Command (LSB first)
    for (int i = 0; i < 7; i++) {
        msb_data = (msb_data << 1) | ((command >> i) & 0x1);
    }
    // Os bits restantes (5, 8 ou 13) sao o Address (LSB first)
    int addr_bits = nbits - 7;
    for (int i = 0; i < addr_bits; i++) {
        msb_data = (msb_data << 1) | ((address >> i) & 0x1);
    }
    return msb_data;
}

/* =========================================================================
 * ARQUIVO HTML EMBUTIDO NA FLASH (main/index.html via EMBED_TXTFILES)
 * ========================================================================= */
extern const char index_html_start[] asm("_binary_index_html_start");
extern const char index_html_end[]   asm("_binary_index_html_end");

/* =========================================================================
 * HANDLERS DO SERVIDOR HTTP + PWA (MANIFEST, ICONE SVG, SERVICE WORKER)
 * ========================================================================= */
static const char MANIFEST_JSON[] =
"{\n"
"  \"name\": \"Sony MHC-GRX7 Remote\",\n"
"  \"short_name\": \"Sony GRX7\",\n"
"  \"description\": \"Controle Remoto IR Wi-Fi Sony MHC-GRX7\",\n"
"  \"start_url\": \"/\",\n"
"  \"scope\": \"/\",\n"
"  \"display\": \"standalone\",\n"
"  \"display_override\": [\"fullscreen\", \"standalone\"],\n"
"  \"orientation\": \"portrait\",\n"
"  \"background_color\": \"#0d0f12\",\n"
"  \"theme_color\": \"#111318\",\n"
"  \"icons\": [\n"
"    {\n"
"      \"src\": \"/icon.svg\",\n"
"      \"sizes\": \"any\",\n"
"      \"type\": \"image/svg+xml\",\n"
"      \"purpose\": \"any maskable\"\n"
"    },\n"
"    {\n"
"      \"src\": \"/icon.svg\",\n"
"      \"sizes\": \"192x192 512x512\",\n"
"      \"type\": \"image/svg+xml\"\n"
"    }\n"
"  ]\n"
"}";

static const char ICON_SVG[] =
"<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 192 192\">"
"<rect width=\"192\" height=\"192\" rx=\"42\" fill=\"#141820\"/>"
"<rect x=\"48\" y=\"20\" width=\"96\" height=\"152\" rx=\"22\" fill=\"#232834\" stroke=\"#ff5722\" stroke-width=\"4\"/>"
"<circle cx=\"74\" cy=\"48\" r=\"10\" fill=\"#e53935\"/>"
"<circle cx=\"118\" cy=\"48\" r=\"10\" fill=\"#00acc1\"/>"
"<rect x=\"64\" y=\"74\" width=\"64\" height=\"24\" rx=\"8\" fill=\"#ff6f00\"/>"
"<circle cx=\"76\" cy=\"124\" r=\"11\" fill=\"#323949\"/>"
"<circle cx=\"116\" cy=\"124\" r=\"11\" fill=\"#323949\"/>"
"<text x=\"96\" y=\"160\" text-anchor=\"middle\" fill=\"#eceff4\" font-family=\"sans-serif\" font-weight=\"bold\" font-size=\"15\">SONY</text>"
"</svg>";

static const char SW_JS[] =
"self.addEventListener('install', () => self.skipWaiting());\n"
"self.addEventListener('activate', (e) => e.waitUntil(self.clients.claim()));\n"
"self.addEventListener('fetch', () => {});\n";

static esp_err_t index_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, index_html_start, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t manifest_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/manifest+json; charset=utf-8");
    return httpd_resp_send(req, MANIFEST_JSON, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t icon_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "image/svg+xml");
    return httpd_resp_send(req, ICON_SVG, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t sw_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/javascript; charset=utf-8");
    return httpd_resp_send(req, SW_JS, HTTPD_RESP_USE_STRLEN);
}

typedef struct {
    const char *key;
    uint32_t data;
    uint8_t nbits;
    int repeats;
    const char *desc;
} sony_key_map_t;

static const sony_key_map_t s_key_table[] = {
    /* Comandos 12-bit (Addr 0x10 = Sony:16) - Confirmados no MHC-GRX7 Manaus */
    { "power",       0xA81,   12, SONY_REPEAT_FRAMES, "POWER I/O" },     // Addr 0x10, Cmd 21 (Confirmado)
    { "vol_up",      0x481,   12, SONY_REPEAT_FRAMES, "VOLUME +" },      // Addr 0x10, Cmd 18 (Confirmado)
    { "vol_down",    0xC81,   12, SONY_REPEAT_FRAMES, "VOLUME -" },      // Addr 0x10, Cmd 19 (Confirmado)
    { "display",     0xD21,   12, SONY_REPEAT_FRAMES, "DISPLAY" },       // Addr 0x10, Cmd 75 (Confirmado)
    { "sleep",       0x061,   12, SONY_REPEAT_FRAMES, "SLEEP TIMER" },   // Addr 0x10, Cmd 96 (Confirmado)
    { "mute",        0x281,   12, SONY_REPEAT_FRAMES, "MUTING" },        // Addr 0x10, Cmd 20
    { "groove",      0x5E1,   12, SONY_REPEAT_FRAMES, "GROOVE" },        // Addr 0x10, Cmd 122 (Sony_cdcombo Sony:16)

    /* Seletores de Fonte (FUNCTION 0x966 confirmado em 12-bit Addr 0x0C + MD/VID em 0x10) */
    { "function",    0x966,   12, SONY_REPEAT_FRAMES, "FUNCTION" },      // Addr 0x0C, Cmd 105 (Confirmado)
    { "md_video",    0x961,   12, SONY_REPEAT_FRAMES, "MD / VIDEO" },    // Addr 0x10, Cmd 105

    /* Processamento de Audio / DSP (Addr 0x0C = Sony:12) e P.FILE -/+ (Addr 0x12 = Sony:18) */
    { "dbfb",        0x8C6,   12, SONY_REPEAT_FRAMES, "DBFB BASS" },     // Addr 0x0C, Cmd 49 (Confirmado)
    { "effect",      0x146,   12, SONY_REPEAT_FRAMES, "EFFECT" },        // Addr 0x0C, Cmd 40 (Confirmado)
    { "surround",    0xFC6,   12, SONY_REPEAT_FRAMES, "SURROUND" },      // Addr 0x0C, Cmd 63 (Confirmado)
    { "pfile_down",  0xBE9,   12, SONY_REPEAT_FRAMES, "P.FILE -" },      // Addr 0x12, Cmd 125 (Confirmado)
    { "pfile_up",    0x3E9,   12, SONY_REPEAT_FRAMES, "P.FILE +" },      // Addr 0x12, Cmd 124 (Confirmado)

    /* Utilitario de teste visual do LED IR pela camera do celular */
    { "cam_test",    0xFFF,   12, 35,                 "CAMERA IR TEST" },
};

static esp_err_t api_cmd_handler(httpd_req_t *req)
{
    char query[64] = {0};
    char key[32] = {0};

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        httpd_query_key_value(query, "key", key, sizeof(key));
    }

    for (size_t i = 0; i < sizeof(s_key_table) / sizeof(s_key_table[0]); i++) {
        if (strcmp(key, s_key_table[i].key) == 0) {
            ir_enqueue_tx(s_key_table[i].data, s_key_table[i].nbits, s_key_table[i].repeats);
            httpd_resp_set_type(req, "text/plain");
            return httpd_resp_sendstr(req, s_key_table[i].desc);
        }
    }

    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Tecla desconhecida");
    return ESP_FAIL;
}

static esp_err_t api_raw_handler(httpd_req_t *req)
{
    char query[96] = {0};
    char data_str[32] = {0};
    char bits_str[16] = {0};

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        httpd_query_key_value(query, "data", data_str, sizeof(data_str));
        httpd_query_key_value(query, "bits", bits_str, sizeof(bits_str));
    }

    uint32_t data = (uint32_t)strtoul(data_str, NULL, 0);
    uint8_t nbits = (uint8_t)atoi(bits_str);
    if (nbits == 0) nbits = 12;

    ir_enqueue_tx(data, nbits, SONY_REPEAT_FRAMES);

    char resp[64];
    snprintf(resp, sizeof(resp), "ENVIADO RAW 0x%lX (%u bits)", (unsigned long)data, nbits);
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, resp);
}

static esp_err_t api_sirc_handler(httpd_req_t *req)
{
    char query[96] = {0};
    char cmd_str[16] = {0};
    char addr_str[16] = {0};
    char bits_str[16] = {0};

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        httpd_query_key_value(query, "cmd", cmd_str, sizeof(cmd_str));
        httpd_query_key_value(query, "addr", addr_str, sizeof(addr_str));
        httpd_query_key_value(query, "bits", bits_str, sizeof(bits_str));
    }

    uint8_t cmd = (uint8_t)atoi(cmd_str);
    uint16_t addr = (uint16_t)atoi(addr_str);
    uint8_t nbits = (uint8_t)atoi(bits_str);
    if (nbits != 12 && nbits != 15 && nbits != 20) nbits = 12;

    uint32_t msb_data = sony_cmd_addr_to_msb(cmd, addr, nbits);
    ir_enqueue_tx(msb_data, nbits, SONY_REPEAT_FRAMES);

    char resp[80];
    snprintf(resp, sizeof(resp), "CMD=%u ADDR=%u -> HEX=0x%lX (%ub)", cmd, addr, (unsigned long)msb_data, nbits);
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, resp);
}

/* =========================================================================
 * HANDLER WEBSOCKET (/ws) PARA COMANDOS EM TEMPO REAL (BAIXISSIMA LATENCIA)
 * Formatos aceitos:
 *   "cmd:<tecla>"               ex: "cmd:vol_up"
 *   "raw:<hex>:<bits>"          ex: "raw:0xA81:12"
 *   "sirc:<cmd>:<addr>:<bits>"  ex: "sirc:21:16:12"
 * ========================================================================= */
static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Cliente WebSocket conectado!");
        return ESP_OK;
    }

    uint8_t buf[96] = {0};
    httpd_ws_frame_t ws_pkt = {
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = buf,
    };

    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, sizeof(buf) - 1);
    if (ret != ESP_OK) {
        return ret;
    }
    buf[ws_pkt.len] = '\0';

    char resp[80] = "ERR // UNKNOWN";
    const char *msg = (const char *)buf;

    if (strncmp(msg, "cmd:", 4) == 0) {
        const char *key = msg + 4;
        for (size_t i = 0; i < sizeof(s_key_table) / sizeof(s_key_table[0]); i++) {
            if (strcmp(key, s_key_table[i].key) == 0) {
                ir_enqueue_tx(s_key_table[i].data, s_key_table[i].nbits, s_key_table[i].repeats);
                snprintf(resp, sizeof(resp), "OK // %s", s_key_table[i].desc);
                break;
            }
        }
    } else if (strncmp(msg, "raw:", 4) == 0) {
        char copy[64];
        strncpy(copy, msg + 4, sizeof(copy) - 1);
        copy[sizeof(copy) - 1] = '\0';
        char *sep = strchr(copy, ':');
        uint8_t nbits = 12;
        if (sep) {
            *sep = '\0';
            nbits = (uint8_t)atoi(sep + 1);
            if (nbits == 0) nbits = 12;
        }
        uint32_t data = (uint32_t)strtoul(copy, NULL, 0);
        ir_enqueue_tx(data, nbits, SONY_REPEAT_FRAMES);
        snprintf(resp, sizeof(resp), "RAW 0x%lX (%uB)", (unsigned long)data, nbits);
    } else if (strncmp(msg, "sirc:", 5) == 0) {
        unsigned int cmd = 0, addr = 16, nbits = 12;
        if (sscanf(msg + 5, "%u:%u:%u", &cmd, &addr, &nbits) >= 2) {
            if (nbits != 12 && nbits != 15 && nbits != 20) nbits = 12;
            uint32_t msb_data = sony_cmd_addr_to_msb((uint8_t)cmd, (uint16_t)addr, (uint8_t)nbits);
            ir_enqueue_tx(msb_data, (uint8_t)nbits, SONY_REPEAT_FRAMES);
            snprintf(resp, sizeof(resp), "CMD=%u ADDR=%u -> 0x%lX", cmd, addr, (unsigned long)msb_data);
        }
    }

    httpd_ws_frame_t resp_pkt = {
        .final = true,
        .fragmented = false,
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)resp,
        .len = strlen(resp),
    };
    return httpd_ws_send_frame(req, &resp_pkt);
}

/* =========================================================================
 * ATUALIZACAO OTA VIA WEB COM PARTICAO DUPLA (ota_0 / ota_1) E FALLBACK
 * - Valida cabecalho do .bin antes de gravar
 * - No pos-boot em estado PENDING_VERIFY, so confirma a imagem depois que
 *   Wi-Fi (IP obtido) + HTTP Server estiverem 100% operacionais!
 * - Se nao conectar em ate 90s apos um OTA, executa rollback automatico!
 * ========================================================================= */
#define OTA_VERIFY_TIMEOUT_SEC  90
#define OTA_RECV_BUF_SIZE       4096

static volatile bool s_wifi_got_ip = false;
static volatile bool s_webserver_ready = false;
static volatile bool s_ota_in_progress = false;

static void ota_reboot_task(void *arg)
{
    bool do_rollback = (arg != NULL);
    vTaskDelay(pdMS_TO_TICKS(800));
    if (do_rollback) {
        ESP_LOGW(TAG, "OTA: Executando fallback/rollback para a particao anterior...");
        esp_ota_mark_app_invalid_rollback_and_reboot();
    }
    ESP_LOGI(TAG, "OTA: Reiniciando ESP32 para carregar o novo firmware...");
    esp_restart();
    vTaskDelete(NULL);
}

static void ota_rollback_watchdog_task(void *arg)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    ESP_LOGW(TAG, "OTA: Particao '%s' em estado PENDING_VERIFY. Aguardando Wi-Fi + HTTP para confirmar...",
             running ? running->label : "?");

    const int max_ticks = OTA_VERIFY_TIMEOUT_SEC * 10;
    for (int i = 0; i < max_ticks; i++) {
        if (s_wifi_got_ip && s_webserver_ready) {
            // Aguarda 2s extras de estabilidade apos obter IP e subir o servidor HTTP
            vTaskDelay(pdMS_TO_TICKS(2000));
            if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
                ESP_LOGI(TAG, "OTA: Auto-diagnostico APROVADO! Particao '%s' marcada como VALIDA (fallback cancelado).",
                         running ? running->label : "?");
            }
            vTaskDelete(NULL);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGE(TAG, "OTA: Falha no auto-diagnostico em %d s (sem Wi-Fi/HTTP)! Acionando FALLBACK automatico...",
             OTA_VERIFY_TIMEOUT_SEC);
    esp_ota_mark_app_invalid_rollback_and_reboot();
    vTaskDelete(NULL);
}

static void ota_boot_check(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    if (running) {
        ESP_LOGI(TAG, "Particao ativa: '%s' @ 0x%lx (%lu KB) | Proxima OTA: '%s'",
                 running->label, (unsigned long)running->address, (unsigned long)(running->size / 1024),
                 next ? next->label : "N/A");

        esp_ota_img_states_t ota_state;
        if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
            if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
                xTaskCreate(ota_rollback_watchdog_task, "ota_wdg", 3072, NULL, 8, NULL);
            } else {
                ESP_LOGI(TAG, "Estado OTA da particao atual: %d (VALIDA)", (int)ota_state);
            }
        }
    }
}

static const char *ota_state_to_str(esp_ota_img_states_t st)
{
    switch (st) {
        case ESP_OTA_IMG_NEW:            return "NEW";
        case ESP_OTA_IMG_PENDING_VERIFY: return "PENDING_VERIFY";
        case ESP_OTA_IMG_VALID:          return "VALID";
        case ESP_OTA_IMG_INVALID:        return "INVALID";
        case ESP_OTA_IMG_ABORTED:        return "ABORTED";
        default:                         return "FACTORY_BOOT";
    }
}

static esp_err_t api_ota_info_handler(httpd_req_t *req)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    const esp_app_desc_t *desc = esp_app_get_description();

    const char *state_str = "READY";
    esp_ota_img_states_t st;
    if (running && esp_ota_get_state_partition(running, &st) == ESP_OK) {
        state_str = ota_state_to_str(st);
    }

    bool can_rb = esp_ota_check_rollback_is_possible();

    char json[256];
    snprintf(json, sizeof(json),
             "{\"running\":\"%s\",\"addr\":\"0x%lx\",\"size_kb\":%lu,"
             "\"next\":\"%s\",\"ver\":\"%s\",\"state\":\"%s\",\"can_rollback\":%s}",
             running ? running->label : "unknown",
             running ? (unsigned long)running->address : 0UL,
             running ? (unsigned long)(running->size / 1024) : 0UL,
             next ? next->label : "none",
             desc ? desc->version : "1.0",
             state_str,
             can_rb ? "true" : "false");

    httpd_resp_set_type(req, "application/json; charset=utf-8");
    return httpd_resp_sendstr(req, json);
}

static esp_err_t api_ota_upload_handler(httpd_req_t *req)
{
    if (s_ota_in_progress) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA ja em andamento");
        return ESP_FAIL;
    }

    const esp_partition_t *update_part = esp_ota_get_next_update_partition(NULL);
    if (!update_part) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Particao OTA nao encontrada (grave a tabela partitions.csv via USB uma vez)");
        return ESP_FAIL;
    }

    if (req->content_len == 0 || req->content_len > update_part->size) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Tamanho do arquivo .bin invalido ou maior que a particao OTA");
        return ESP_FAIL;
    }

    // Garante que a particao atual esteja confirmada antes de iniciar novo OTA
    esp_ota_mark_app_valid_cancel_rollback();

    char *buf = (char *)malloc(OTA_RECV_BUF_SIZE);
    if (!buf) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Sem memoria RAM para buffer OTA");
        return ESP_FAIL;
    }

    s_ota_in_progress = true;
    esp_ota_handle_t ota_handle = 0;
    bool ota_started = false;
    size_t remaining = req->content_len;
    size_t total_written = 0;

    ESP_LOGI(TAG, "OTA: Iniciando upload de %u bytes para a particao '%s' (0x%lx)...",
             (unsigned)req->content_len, update_part->label, (unsigned long)update_part->address);

    while (remaining > 0) {
        size_t to_read = (remaining > OTA_RECV_BUF_SIZE) ? OTA_RECV_BUF_SIZE : remaining;
        int recv_len = httpd_req_recv(req, buf, to_read);
        if (recv_len <= 0) {
            if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            ESP_LOGE(TAG, "OTA: Erro de rede na recepcao (recv=%d)", recv_len);
            if (ota_started) esp_ota_abort(ota_handle);
            free(buf);
            s_ota_in_progress = false;
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Falha de conexao durante upload OTA");
            return ESP_FAIL;
        }

        if (!ota_started) {
            if ((uint8_t)buf[0] != ESP_IMAGE_HEADER_MAGIC) {
                ESP_LOGE(TAG, "OTA: Arquivo invalido (magic byte 0x%02X != 0xE9)", (uint8_t)buf[0]);
                free(buf);
                s_ota_in_progress = false;
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Arquivo invalido! Envie o binário compilado (sony_grx7_remote.bin)");
                return ESP_FAIL;
            }
            esp_err_t err = esp_ota_begin(update_part, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "OTA: esp_ota_begin falhou (%s)", esp_err_to_name(err));
                free(buf);
                s_ota_in_progress = false;
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(err));
                return ESP_FAIL;
            }
            ota_started = true;
        }

        esp_err_t err = esp_ota_write(ota_handle, buf, (size_t)recv_len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "OTA: esp_ota_write falhou (%s)", esp_err_to_name(err));
            esp_ota_abort(ota_handle);
            free(buf);
            s_ota_in_progress = false;
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Erro ao gravar na Flash");
            return ESP_FAIL;
        }

        remaining -= (size_t)recv_len;
        total_written += (size_t)recv_len;
    }

    free(buf);

    esp_err_t err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA: Validacao da imagem falhou em esp_ota_end (%s)", esp_err_to_name(err));
        s_ota_in_progress = false;
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Imagem corrompida ou assinatura invalida!");
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(update_part);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA: esp_ota_set_boot_partition falhou (%s)", esp_err_to_name(err));
        s_ota_in_progress = false;
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Falha ao definir particao de boot");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA: %u bytes gravados e verificados em '%s'! Agendando reboot...",
             (unsigned)total_written, update_part->label);

    char resp[128];
    snprintf(resp, sizeof(resp), "{\"ok\":true,\"part\":\"%s\",\"bytes\":%u}",
             update_part->label, (unsigned)total_written);
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_sendstr(req, resp);

    xTaskCreate(ota_reboot_task, "ota_reboot", 2048, NULL, 10, NULL);
    return ESP_OK;
}

static esp_err_t api_ota_rollback_handler(httpd_req_t *req)
{
    if (!esp_ota_check_rollback_is_possible()) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Nenhuma particao anterior valida disponivel para fallback");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_sendstr(req, "{\"ok\":true,\"msg\":\"Revertendo para particao anterior...\"}");

    xTaskCreate(ota_reboot_task, "ota_rb", 2048, (void *)1, 10, NULL);
    return ESP_OK;
}

static void start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.max_uri_handlers = 14;
    config.stack_size = 6144;
    config.recv_wait_timeout = 15;
    config.send_wait_timeout = 15;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        const httpd_uri_t index_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = index_get_handler,
            .user_ctx  = NULL
        };
        const httpd_uri_t manifest_uri = {
            .uri       = "/manifest.json",
            .method    = HTTP_GET,
            .handler   = manifest_get_handler,
            .user_ctx  = NULL
        };
        const httpd_uri_t icon_uri = {
            .uri       = "/icon.svg",
            .method    = HTTP_GET,
            .handler   = icon_get_handler,
            .user_ctx  = NULL
        };
        const httpd_uri_t sw_uri = {
            .uri       = "/sw.js",
            .method    = HTTP_GET,
            .handler   = sw_get_handler,
            .user_ctx  = NULL
        };
        const httpd_uri_t cmd_uri = {
            .uri       = "/api/cmd",
            .method    = HTTP_POST,
            .handler   = api_cmd_handler,
            .user_ctx  = NULL
        };
        const httpd_uri_t raw_uri = {
            .uri       = "/api/raw",
            .method    = HTTP_POST,
            .handler   = api_raw_handler,
            .user_ctx  = NULL
        };
        const httpd_uri_t sirc_uri = {
            .uri       = "/api/sirc",
            .method    = HTTP_POST,
            .handler   = api_sirc_handler,
            .user_ctx  = NULL
        };
        const httpd_uri_t ota_info_uri = {
            .uri       = "/api/ota/info",
            .method    = HTTP_GET,
            .handler   = api_ota_info_handler,
            .user_ctx  = NULL
        };
        const httpd_uri_t ota_upload_uri = {
            .uri       = "/api/ota",
            .method    = HTTP_POST,
            .handler   = api_ota_upload_handler,
            .user_ctx  = NULL
        };
        const httpd_uri_t ota_rb_uri = {
            .uri       = "/api/ota/rollback",
            .method    = HTTP_POST,
            .handler   = api_ota_rollback_handler,
            .user_ctx  = NULL
        };
        const httpd_uri_t ws_uri = {
            .uri          = "/ws",
            .method       = HTTP_GET,
            .handler      = ws_handler,
            .user_ctx     = NULL,
            .is_websocket = true
        };

        httpd_register_uri_handler(server, &index_uri);
        httpd_register_uri_handler(server, &manifest_uri);
        httpd_register_uri_handler(server, &icon_uri);
        httpd_register_uri_handler(server, &sw_uri);
        httpd_register_uri_handler(server, &cmd_uri);
        httpd_register_uri_handler(server, &raw_uri);
        httpd_register_uri_handler(server, &sirc_uri);
        httpd_register_uri_handler(server, &ota_info_uri);
        httpd_register_uri_handler(server, &ota_upload_uri);
        httpd_register_uri_handler(server, &ota_rb_uri);
        httpd_register_uri_handler(server, &ws_uri);

        s_webserver_ready = true;
        ESP_LOGI(TAG, "Webserver + WebSocket (/ws) + OTA (/api/ota) iniciado na porta %d!", config.server_port);
    }
}

/* =========================================================================
 * CONEXAO WI-FI STATION (DUAL SSID COM ALTERNANCIA AUTOMATICA)
 * ========================================================================= */
typedef struct {
    const char *ssid;
    const char *password;
} wifi_credential_t;

static const wifi_credential_t s_wifi_aps[] = {
    { WIFI_SSID_1, WIFI_PASS_1 },
    { WIFI_SSID_2, WIFI_PASS_2 },
};

#define WIFI_AP_COUNT          (sizeof(s_wifi_aps) / sizeof(s_wifi_aps[0]))
#define WIFI_MAX_RETRY_PER_AP  3

static size_t s_current_ap_idx = 0;
static int s_retry_count = 0;

static void wifi_apply_current_ap(void)
{
    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, s_wifi_aps[s_current_ap_idx].ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, s_wifi_aps[s_current_ap_idx].password, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK;

    ESP_LOGI(TAG, "Configurando Wi-Fi para SSID [%u/%u]: '%s'...",
             (unsigned)(s_current_ap_idx + 1), (unsigned)WIFI_AP_COUNT, s_wifi_aps[s_current_ap_idx].ssid);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_retry_count++;
        if (s_retry_count < WIFI_MAX_RETRY_PER_AP) {
            ESP_LOGW(TAG, "Falha ao conectar em '%s' (tentativa %d/%d). Tentando novamente...",
                     s_wifi_aps[s_current_ap_idx].ssid, s_retry_count, WIFI_MAX_RETRY_PER_AP);
        } else {
            s_retry_count = 0;
            size_t prev_idx = s_current_ap_idx;
            s_current_ap_idx = (s_current_ap_idx + 1) % WIFI_AP_COUNT;
            ESP_LOGW(TAG, "Nao foi possivel conectar em '%s'. Alternando para '%s'...",
                     s_wifi_aps[prev_idx].ssid, s_wifi_aps[s_current_ap_idx].ssid);
            wifi_apply_current_ap();
        }
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        s_retry_count = 0;
        s_wifi_got_ip = true;
        ESP_LOGI(TAG, "==========================================");
        ESP_LOGI(TAG, "CONECTADO na rede '%s'!", s_wifi_aps[s_current_ap_idx].ssid);
        ESP_LOGI(TAG, "Abra no navegador do celular:");
        ESP_LOGI(TAG, "http://" IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "==========================================");
    }
}

static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(netif, "sony-grx7");

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    wifi_apply_current_ap();
    ESP_ERROR_CHECK(esp_wifi_start());

    // Reduz potencia de pico de RF do Wi-Fi de +20 dBm (100 mW) para +13 dBm (20 mW = 52 * 0.25 dBm):
    // corta em 80% a interferencia de RF induzida nos cabos RCA/pre-amp do som e os picos de corrente ao clicar!
    esp_wifi_set_max_tx_power(52);

    // Mantem recepcao continua (evita pulsos de 10 Hz de DTIM modem-sleep na fonte) e baixa latencia
    esp_wifi_set_ps(WIFI_PS_NONE);
}

static void auto_power_on_task(void *arg)
{
    ESP_LOGI(TAG, "Auto-Power ON: placa energizada! Aguardando %d ms para ligar o som...", AUTO_POWER_ON_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(AUTO_POWER_ON_DELAY_MS));
    ESP_LOGI(TAG, "Auto-Power ON: disparando POWER (0x%X, %u bits)!", SONY_POWER_CMD_HEX, SONY_POWER_CMD_BITS);
    ir_enqueue_tx(SONY_POWER_CMD_HEX, SONY_POWER_CMD_BITS, SONY_REPEAT_FRAMES);
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ir_rmt_init();
    s_ir_tx_queue = xQueueCreate(3, sizeof(ir_tx_msg_t));
    xTaskCreate(ir_tx_task, "ir_tx_task", 3072, NULL, 12, NULL);

    // Envia POWER 3s apos energizar a placa (ignora reboots de software/OTA para nao desligar o som ja ligado)
    esp_reset_reason_t rst_reason = esp_reset_reason();
    if (rst_reason == ESP_RST_POWERON || rst_reason == ESP_RST_EXT || rst_reason == ESP_RST_BROWNOUT) {
        xTaskCreate(auto_power_on_task, "auto_pwr_on", 3072, NULL, 10, NULL);
    } else {
        ESP_LOGI(TAG, "Auto-Power ON ignorado neste boot (reset_reason=%d, ex: reboot OTA/SW).", (int)rst_reason);
    }

    ota_boot_check();
    wifi_init_sta();
    start_webserver();
}
