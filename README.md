# Sony MHC-GRX7 Web IR Remote (ESP32 + ESP-IDF)

Controle remoto infravermelho (IR) via Wi-Fi com interface Web Mobile responsiva para o Mini System **Sony MHC-GRX7** (compatível também com outros aparelhos de áudio Sony que utilizam o protocolo **Sony SIRC 40 kHz** de 12, 15 e 20 bits).

Ideal para controlar o som pelo celular quando integrado à TV/Chromecast pela entrada **MD / VIDEO (AUX)**.

---

## Funcionalidades

- **Protocolo Sony SIRC Nativo via Hardware RMT:** Geração precisa da portadora de **40 kHz** (duty cycle 50%) utilizando o periférico `rmt_tx` do ESP-IDF, enviando 3 quadros por comando conforme especificação da Sony.
- **Interface Web Mobile Embutida:** Página HTML/CSS/JS em arquivo dedicado (`main/index.html`) gravada diretamente na Flash do ESP32 (`EMBED_TXTFILES`).
  - Controle de **Power**, **Entrada Direta MD**, **Function**, **Video/Aux**.
  - Controle de **Volume (+ / -)** com suporte a **segurar o botão** (*touch & hold*) e **Mute**.
  - Efeitos e equalização do GRX7: **GROOVE**, **DBFB**, **EQ**, **EFFECT** e **SURROUND**.
  - **Scanner / Testador SIRC integrado:** Permite enviar qualquer código Hex (12/15/20 bits) ou varrer comandos (`0..127`) para descobrir funções extras do aparelho.
- **Dual Wi-Fi com Failover Automático:** Conecta em até 2 redes Wi-Fi configuradas; se a principal falhar após 3 tentativas, alterna automaticamente para a rede secundária.

---

## Circuito Sugerido (LED IR TIL32 / 940 nm)

Por padrão, o pino de saída IR é o **GPIO 13** (configurável). Para obter alcance de vários metros, recomenda-se acionar o LED IR através de um transistor NPN (ex.: `BC337` ou `2N2222`):

```text
ESP32 GPIO 13 ───[ 220 Ω ]───► Base (BC337 / 2N2222)
                               Emissor ───► GND

ESP32 5V (VIN) ──[ 33 Ω ]────► Anodo (+) LED IR (TIL32)
                               Catodo (-) LED IR ───► Coletor (BC337 / 2N2222)
```

---

## Como Configurar o Wi-Fi

Para proteger suas senhas ao publicar ou clonar o repositório, o projeto suporta um arquivo local ignorado pelo Git:

1. Copie o arquivo de exemplo:
   ```bash
   cp main/wifi_secrets.example.h main/wifi_secrets.h
   ```
2. Edite `main/wifi_secrets.h` com o nome e senha das suas duas redes Wi-Fi 2.4 GHz.
   - Alternativamente, você pode configurar via `idf.py menuconfig` no menu **Sony GRX7 Remote Configuration**.

---

## Compilação e Gravação (ESP-IDF v5.x / v6.x)

```bash
idf.py set-target esp32
idf.py build
idf.py -p COMx flash monitor
```

Ao conectar no Wi-Fi, o endereço IP será exibido no monitor serial:

```text
CONECTADO na rede 'SUA_REDE'!
Abra no navegador do celular:
http://192.168.x.x
```

---

## Como Instalar como App em Tela Cheia (Android Chrome / PWA)

A interface possui manifesto PWA (`/manifest.json`), ícone vetorial (`/icon.svg`), Service Worker (`/sw.js`) e um botão **`⛶ Tela Cheia`** direto no topo do controle.

Para que o Chrome do Android permita **Instalar como Aplicativo Nativo** mesmo sem HTTPS (em IP local `http://192.168.x.x`):

1. Abra no Chrome do celular:
   ```text
   chrome://flags/#unsafely-treat-insecure-origin-as-secure
   ```
2. Digite o endereço do ESP32 (ex.: `http://192.168.1.100`), mude para **Enabled** e toque em **Relaunch**.
3. Acesse o IP do controle, toque no menu do Chrome (**⋮**) e selecione **Adicionar à tela inicial -> Instalar aplicativo**.

---

## Tabela de Códigos Sony SIRC Utilizados

| Função | Código Hex (MSB) | Bits | Comando (LSB) | Endereço (LSB) |
| :--- | :---: | :---: | :---: | :---: |
| **POWER** | `0xA81` | 12 | 21 | 16 |
| **VOL +** | `0x481` | 12 | 18 | 16 |
| **VOL −** | `0xC81` | 12 | 19 | 16 |
| **MUTE** | `0x281` | 12 | 20 | 16 |
| **MD (Entrada Direta)** | `0x621` | 12 | 38 | 16 |
| **VIDEO / AUX** | `0x441` | 12 | 34 | 16 |
| **FUNCTION** | `0x4B09` | 15 | 75 | 144 |
| **GROOVE** | `0x1309` | 15 | 72 | 144 |
| **DBFB** | `0x2309` | 15 | 68 | 144 |
| **EQ** | `0x6309` | 15 | 70 (`0x46`) | 144 (`0x90`) |
| **EFFECT** | `0xF81` | 12 | 31 (`0x1F`) | 16 (`0x10`) |
| **SURROUND** | `0xD41` | 12 | 43 (`0x2B`) | 16 (`0x10`) |
| **DISPLAY** | `0xD21` | 12 | 75 (`0x4B`) | 16 (`0x10`) |

