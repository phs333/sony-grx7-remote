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
#define SONY_INTERFRAME_SPACE_US 24000  // Espaco exato de 24 ms entre frames no hardware RMT (periodo Sony ~45 ms)
#define SONY_REPEAT_FRAMES      3       // Padrao oficial Sony SIRC: 3 frames por comando (~135 ms)
#define SONY_TX_TIMEOUT_MS      200     // Timeout em MILISSEGUNDOS para rmt_tx_wait_all_done()

static rmt_channel_handle_t s_rmt_tx_chan = NULL;
static rmt_encoder_handle_t s_rmt_copy_encoder = NULL;

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
        .mem_block_symbols = 64,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .trans_queue_depth = 4,
        .flags = {
            .invert_out = false,
            .with_dma = false,
        },
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_cfg, &s_rmt_tx_chan));

    // Configura corrente maxima de saida no pad do ESP32 (GPIO_DRIVE_CAP_3 = ~40mA) para o LED TIL32
    ESP_ERROR_CHECK(gpio_set_drive_capability(IR_TX_GPIO_NUM, GPIO_DRIVE_CAP_3));

    rmt_carrier_config_t carrier_cfg = {
        .duty_cycle = 0.50f, // 50% duty cycle para maior potencia luminosa no LED IR
        .frequency_hz = SONY_CARRIER_FREQ_HZ,
        .flags = {
            .polarity_active_low = false,
        },
    };
    ESP_ERROR_CHECK(rmt_apply_carrier(s_rmt_tx_chan, &carrier_cfg));

    rmt_copy_encoder_config_t copy_encoder_cfg = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_encoder_cfg, &s_rmt_copy_encoder));

    ESP_ERROR_CHECK(rmt_enable(s_rmt_tx_chan));
    ESP_LOGI(TAG, "RMT TX inicializado com sucesso (Drive Cap = MAX 40mA)!");
}

/* =========================================================================
 * TRANSMISSAO SONY SIRC (12, 15 ou 20 bits - formato MSB-first estilo ESPHome/LIRC)
 * Exemplo: Power = 0xA81 (12 bits), Vol+ = 0x481 (12 bits)
 * ========================================================================= */
static void sony_transmit_msb(uint32_t data, uint8_t nbits, int repeats)
{
    if (nbits != 12 && nbits != 15 && nbits != 20) {
        nbits = 12;
    }
    if (repeats < 1) {
        repeats = SONY_REPEAT_FRAMES;
    }

    // 1 simbolo de Header + nbits simbolos de dados
    rmt_symbol_word_t symbols[22];
    size_t num_symbols = 0;

    // Header Sony: 2400us HIGH + 600us LOW
    symbols[num_symbols++] = (rmt_symbol_word_t){
        .level0 = 1,
        .duration0 = SONY_HEADER_MARK_US,
        .level1 = 0,
        .duration1 = SONY_HEADER_SPACE_US,
    };

    // Bits de dados (do bit mais significativo para o menos significativo)
    for (int i = nbits - 1; i >= 0; i--) {
        bool bit_is_one = (data >> i) & 0x1;
        // No ultimo bit do frame, estende o espaco LOW para 24000 us (24 ms) direto no hardware RMT
        uint16_t space_us = (i == 0) ? SONY_INTERFRAME_SPACE_US : SONY_BIT_SPACE_US;
        symbols[num_symbols++] = (rmt_symbol_word_t){
            .level0 = 1,
            .duration0 = bit_is_one ? SONY_BIT_ONE_MARK_US : SONY_BIT_ZERO_MARK_US,
            .level1 = 0,
            .duration1 = space_us,
        };
    }

    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,
        .flags = {
            .eot_level = 0, // Garante que o LED IR fique desligado ao terminar
        },
    };

    ESP_LOGI(TAG, "Transmitindo Sony SIRC: data=0x%lX (%u bits) x%d", (unsigned long)data, nbits, repeats);

    for (int r = 0; r < repeats; r++) {
        esp_err_t err = rmt_transmit(
            s_rmt_tx_chan,
            s_rmt_copy_encoder,
            symbols,
            num_symbols * sizeof(rmt_symbol_word_t),
            &tx_cfg
        );
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Falha em rmt_transmit: %s", esp_err_to_name(err));
            break;
        }
        // IMPORTANTE: rmt_tx_wait_all_done recebe o timeout em MILISSEGUNDOS (nao usar pdMS_TO_TICKS aqui!)
        err = rmt_tx_wait_all_done(s_rmt_tx_chan, SONY_TX_TIMEOUT_MS);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Timeout aguardando fim da transmissao RMT: %s", esp_err_to_name(err));
            break;
        }
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
    { "power",     0xA81,  12, SONY_REPEAT_FRAMES, "POWER (0xA81)" },
    { "vol_up",    0x481,  12, SONY_REPEAT_FRAMES, "VOL+ (0x481)" },
    { "vol_down",  0xC81,  12, SONY_REPEAT_FRAMES, "VOL- (0xC81)" },
    { "mute",      0x281,  12, SONY_REPEAT_FRAMES, "MUTE (0x281)" },
    { "md",        0x621,  12, SONY_REPEAT_FRAMES, "MD DIRECT (0x621)" },
    { "video_aux", 0x441,  12, SONY_REPEAT_FRAMES, "VIDEO/AUX (0x441)" },
    { "function",  0x4B09, 15, SONY_REPEAT_FRAMES, "FUNCTION (0x4B09)" },
    { "groove",    0x1309, 15, SONY_REPEAT_FRAMES, "GROOVE (0x1309)" },
    { "dbfb",      0x2309, 15, SONY_REPEAT_FRAMES, "DBFB (0x2309)" },
    { "eq",        0x6309, 15, SONY_REPEAT_FRAMES, "EQ (0x6309)" },
    { "effect",    0xF81,  12, SONY_REPEAT_FRAMES, "EFFECT (0xF81)" },
    { "surround",  0xD41,  12, SONY_REPEAT_FRAMES, "SURROUND (0xD41)" },
    { "display",   0xD21,  12, SONY_REPEAT_FRAMES, "DISPLAY (0xD21)" },
    { "cam_test",  0xFFF,  12, 35,                 "CAMERA IR TEST (1.5s)" },
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

static void start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.max_uri_handlers = 10;

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
        httpd_register_uri_handler(server, &ws_uri);

        ESP_LOGI(TAG, "Webserver + WebSocket (/ws) iniciado na porta %d!", config.server_port);
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

    // Desativa economia de energia do Wi-Fi para resposta imediata ao toque no celular!
    esp_wifi_set_ps(WIFI_PS_NONE);
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
    xTaskCreate(ir_tx_task, "ir_tx_task", 3072, NULL, 10, NULL);
    wifi_init_sta();
    start_webserver();
}
