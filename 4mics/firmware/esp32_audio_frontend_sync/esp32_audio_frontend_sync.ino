/*
 * esp32_audio_frontend_sync.ino
 *
 * Captura 4 micrófonos INMP441 con 2 buses I2S SINCRONIZADOS por clock
 * compartido (master + slave) y transmite las muestras por USB-serial a la RPi.
 *
 * =============================================================================
 * CABLEADO 
 *
 *   Señal      │ ESP32 GPIO │ Mic 0 │ Mic 1 │ Mic 2 │ Mic 3
 *   ───────────┼────────────┼───────┼───────┼───────┼───────
 *   SCK bus0   │     26     │  SCK  │  SCK  │  SCK  │  SCK
 *   WS  bus0   │     25     │  WS   │  WS   │  WS   │  WS
 *   SD  bus0   │     22     │  SD   │  SD   │   -   │   -
 *   SD  bus1   │     32     │   -   │   -   │  SD   │  SD
 *   SCK bus1(in)│    14     │   -   │   -   │   -   │   -   (jumper desde 26)
 *   WS  bus1(in)│    15     │   -   │   -   │   -   │   -   (jumper desde 25)
 *   L/R        │  (fijo)    │  GND  │  VDD  │  GND  │  VDD
 *
 */

#include <driver/i2s.h>
#include <esp_system.h>      // esp_reset_reason()
#include <string.h>          // memcpy en send_hop()

// ============================================================================
// CONFIGURACIÓN
// ============================================================================

#define SAMPLE_RATE    11025 
#define HOP_SIZE       256
#define DMA_BUF_LEN    256
#define DMA_BUF_COUNT  6     


// Shift de del dato de 24 bits que entrega el micrófono para tomar los MSB
// ajustar de forma que no se sature y recorten los datos, si la ganancia es poca pero
// los datos están, MUSIC no es afectado.
#define GAIN_SHIFT     12   

#define SERIAL_BAUD    921600

#define SYNC_BYTE      0xAA
#define END_BYTE       0x55


#define I2S_READ_TIMEOUT_MS  100
// Fallos consecutivos de captura antes de reiniciar el periférico I2S.
#define FALLOS_PARA_REINICIO  10

// Bus 0 = MASTER (genera el clock para los 4 mics)
#define PIN_SCK0  26
#define PIN_WS0   25
#define PIN_SD0   22 

// Bus 1 = SLAVE (recibe el clock del master)
#define PIN_SCK1  14   // ENTRADA SCK del slave  <- jumper desde GPIO26
#define PIN_WS1   15   // ENTRADA WS  del slave  <- jumper desde GPIO25
#define PIN_SD1   32


// Tamaño total del paquete: SYNC(1) + counter(2) + datos + END(1)
#define PKT_BYTES  (3 + HOP_SIZE * 4 * (int)sizeof(int16_t) + 1)

static int32_t  buf0[HOP_SIZE * 2];       // DMA bus 0
static int32_t  buf1[HOP_SIZE * 2];       // DMA bus 1
static int16_t  out_buf[HOP_SIZE * 4];    // 4 canales interleaved, alineado
static uint8_t  packet[PKT_BYTES];        // paquete completo, listo para enviar
static uint16_t frame_counter = 0;

// Contadores de diagnóstico.
static uint32_t err_i2s_read   = 0;   // i2s_read con timeout o lectura corta
static uint32_t n_reinicios    = 0;   // reinicios del periférico I2S
static uint32_t pkts_truncados = 0;   // Serial.write escribió menos de PKT_BYTES

// ============================================================================
// BANNER DE ARRANQUE — el firmware se identifica solo
// ============================================================================

static const char* motivo_reset() {
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:   return "POWERON";    // alimentación recién puesta
        case ESP_RST_EXT:       return "EXT";        // pin EN (RTS/DTR)
        case ESP_RST_SW:        return "SW";
        case ESP_RST_PANIC:     return "PANIC";      // excepción no atrapada
        case ESP_RST_INT_WDT:   return "INT_WDT";
        case ESP_RST_TASK_WDT:  return "TASK_WDT";   // loop() bloqueado
        case ESP_RST_WDT:       return "WDT";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";   // la fuente se cayó
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "UNKNOWN";
    }
}

static void imprimir_banner() {
    Serial.printf("\r\n[SSL-FW] build " __DATE__ " " __TIME__
                  " | reset=%s | fs=%d hop=%d ch=4 shift=%d cpu=%luMHz\r\n",
                  motivo_reset(), SAMPLE_RATE, HOP_SIZE, GAIN_SHIFT,
                  (unsigned long)getCpuFrequencyMhz());
    Serial.flush();
}

// ============================================================================
// INICIALIZACIÓN I2S
// ============================================================================

static void setup_i2s(i2s_port_t port, i2s_mode_t rxmode,
                      int sck, int ws, int sd) {
    i2s_config_t cfg = {
        .mode                 = (i2s_mode_t)(rxmode | I2S_MODE_RX),
        .sample_rate          = SAMPLE_RATE,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = DMA_BUF_COUNT,
        .dma_buf_len          = DMA_BUF_LEN,
        .use_apll             = false,
        .tx_desc_auto_clear   = false,
        .fixed_mclk           = 0
    };

    i2s_pin_config_t pins = {
        .mck_io_num   = I2S_PIN_NO_CHANGE,
        .bck_io_num   = sck,   // master: salida | slave: entrada
        .ws_io_num    = ws,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num  = sd
    };

    i2s_driver_install(port, &cfg, 0, NULL);
    i2s_set_pin(port, &pins);
}

/* Arranca los dos buses en el orden correcto y con los DMA limpios.
 * i2s_driver_install() deja el periférico CORRIENDO, así que sin este stop/start
 * explícito el clock empieza antes de que los micrófonos estén listos. */
static void arrancar_i2s() {
    i2s_stop(I2S_NUM_0);
    i2s_stop(I2S_NUM_1);
    delay(200);                       // mics alimentados y quietos, sin clock
    i2s_zero_dma_buffer(I2S_NUM_0);
    i2s_zero_dma_buffer(I2S_NUM_1);
    i2s_start(I2S_NUM_1);             // SLAVE primero: queda esperando el WS
    i2s_start(I2S_NUM_0);             // MASTER: recién ahora empieza el clock
}

static void reiniciar_i2s() {
    n_reinicios++;
    arrancar_i2s();
    for (int i = 0; i < 10; i++) {    // descartar transitorios
        size_t br;
        i2s_read(I2S_NUM_1, buf1, sizeof(buf1), &br, pdMS_TO_TICKS(I2S_READ_TIMEOUT_MS));
        i2s_read(I2S_NUM_0, buf0, sizeof(buf0), &br, pdMS_TO_TICKS(I2S_READ_TIMEOUT_MS));
    }
}

// ============================================================================
// CAPTURA Y TRANSMISIÓN
// ============================================================================

static bool capture_hop() {
    size_t br0 = 0, br1 = 0;
    const size_t expected = HOP_SIZE * 2 * sizeof(int32_t);
    const TickType_t tout = pdMS_TO_TICKS(I2S_READ_TIMEOUT_MS);

    esp_err_t e1 = i2s_read(I2S_NUM_1, buf1, expected, &br1, tout);
    esp_err_t e0 = i2s_read(I2S_NUM_0, buf0, expected, &br0, tout);

    if (e0 != ESP_OK || e1 != ESP_OK || br0 < expected || br1 < expected) {
        err_i2s_read++;
        return false;
    }

    for (int i = 0; i < HOP_SIZE; i++) {
        int32_t s0 = buf0[i * 2 + 0] >> GAIN_SHIFT;  // Mic 0
        int32_t s1 = buf0[i * 2 + 1] >> GAIN_SHIFT;  // Mic 1
        int32_t s2 = buf1[i * 2 + 0] >> GAIN_SHIFT;  // Mic 2
        int32_t s3 = buf1[i * 2 + 1] >> GAIN_SHIFT;  // Mic 3

        if (s0 >  32767) s0 =  32767; if (s0 < -32768) s0 = -32768;
        if (s1 >  32767) s1 =  32767; if (s1 < -32768) s1 = -32768;
        if (s2 >  32767) s2 =  32767; if (s2 < -32768) s2 = -32768;
        if (s3 >  32767) s3 =  32767; if (s3 < -32768) s3 = -32768;

        out_buf[i * 4 + 0] = (int16_t)s0;
        out_buf[i * 4 + 1] = (int16_t)s1;
        out_buf[i * 4 + 2] = (int16_t)s2;
        out_buf[i * 4 + 3] = (int16_t)s3;
    }
    return true;
}

static void send_hop() {
    packet[0] = SYNC_BYTE;
    packet[1] = (uint8_t)((frame_counter >> 8) & 0xFF);
    packet[2] = (uint8_t)(frame_counter & 0xFF);
    memcpy(packet + 3, out_buf, sizeof(out_buf));   // int16 little-endian nativo
    packet[PKT_BYTES - 1] = END_BYTE;

    size_t n = Serial.write(packet, PKT_BYTES);
    if (n != (size_t)PKT_BYTES) pkts_truncados++;

    frame_counter++;
}

// ============================================================================
// SETUP Y LOOP
// ============================================================================

void setup() {
    // Frecuencia de CPU explícita
    setCpuFrequencyMhz(240);

    // Buffer de TX antes de begin(). Dos paquetes de colchón para absorber los
    // hipos del USB-CDC sin que Serial.write bloquee el loop.
    Serial.setTxBufferSize(2 * PKT_BYTES);
    Serial.begin(SERIAL_BAUD);

    // Para que los microfonos se estabilicen antes de arrancar I2S
    delay(500);

    imprimir_banner();

    // Instalar el SLAVE primero (queda esperando clocks) y después el MASTER.
    setup_i2s(I2S_NUM_1, I2S_MODE_SLAVE,  PIN_SCK1, PIN_WS1, PIN_SD1);
    setup_i2s(I2S_NUM_0, I2S_MODE_MASTER, PIN_SCK0, PIN_WS0, PIN_SD0);

    arrancar_i2s();

    // Descartar transitorios (cápsula + alineación de DMA)
    for (int i = 0; i < 20; i++) capture_hop();

    err_i2s_read = 0;   // los fallos del arranque no cuentan
    frame_counter = 0;
}

void loop() {
    static uint32_t fallos_seguidos = 0;

    if (capture_hop()) {
        fallos_seguidos = 0;
        send_hop();
        return;
    }

    // Captura fallida: NO se manda paquete (mejor un hueco en el counter, que
    // la Pi detecta, que un paquete con datos viejos que parece bueno).
    if (++fallos_seguidos >= FALLOS_PARA_REINICIO) {
        Serial.printf("[SSL-FW] i2s sin datos: err=%lu reinicios=%lu trunc=%lu\r\n",
                      (unsigned long)err_i2s_read, (unsigned long)n_reinicios,
                      (unsigned long)pkts_truncados);
        reiniciar_i2s();
        fallos_seguidos = 0;
    }
}
