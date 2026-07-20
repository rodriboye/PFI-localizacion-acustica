/*
 * esp32_audio_frontend_sync.ino
 *
 * Captura 4 micrófonos INMP441 con 2 buses I2S SINCRONIZADOS por clock
 * compartido (master + slave) y transmite las muestras por USB-serial a la RPi.
 *
 * CAMBIO RESPECTO A LA VERSIÓN ANTERIOR (dos masters independientes)
 * -----------------------------------------------------------------------------
 * La versión anterior usaba I2S_NUM_0 e I2S_NUM_1 como DOS masters, cada uno
 * generando su propio SCK/WS. Con use_apll=false el clock sale de PLL_D2 con
 * división FRACCIONARIA (160 MHz / 512 kHz = 312.5), y cada periférico corre su
 * propio divisor: misma frecuencia promedio pero la FASE entre buses DERIVA en
 * el tiempo. Resultado: los pares de mics que cruzan de bus (elevación) pierden
 * coherencia y MUSIC no puede localizar (pseudoespectro plano).
 *
 * Solución: un solo clock físico para los 4 mics.
 *   - Bus0 (I2S_NUM_0) = MASTER: genera SCK (GPIO26) y WS (GPIO25).
 *   - Bus1 (I2S_NUM_1) = SLAVE: NO genera clock; recibe SCK y WS del master.
 *   Los 4 mics muestrean en el MISMO flanco -> sincronía exacta, sin deriva.
 *
 * ============================================================================
 * CABLEADO  (cambios marcados con  <<<)
 *
 *   Señal      │ ESP32 GPIO │ Mic 0 │ Mic 1 │ Mic 2 │ Mic 3
 *   ───────────┼────────────┼───────┼───────┼───────┼───────
 *   SCK bus0   │     26     │  SCK  │  SCK  │  SCK  │  SCK   <<< ahora alimenta los 4
 *   WS  bus0   │     25     │  WS   │  WS   │  WS   │  WS    <<< ahora alimenta los 4
 *   SD  bus0   │     27     │  SD   │  SD   │   -   │   -
 *   SD  bus1   │     32     │   -   │   -   │  SD   │  SD
 *   SCK bus1(in)│    14     │   -   │   -   │   -   │   -    <<< entrada del slave
 *   WS  bus1(in)│    15     │   -   │   -   │   -   │   -    <<< entrada del slave
 *   L/R        │  (fijo)    │  GND  │  VDD  │  GND  │  VDD
 *   VDD/GND    │ 3.3V / GND │  ...  │  ...  │  ...  │  ...
 *
 *   DOS JUMPERS NUEVOS (la única modificación de hardware):
 *       GPIO26  ──►  GPIO14     (SCK del master entra al slave)
 *       GPIO25  ──►  GPIO15     (WS  del master entra al slave)
 *
 *   Y mover la alimentación de clock de Mic2/Mic3:
 *       SCK de Mic2 y Mic3:  de GPIO14  ->  GPIO26  (compartido con Mic0/Mic1)
 *       WS  de Mic2 y Mic3:  de GPIO15  ->  GPIO25  (compartido con Mic0/Mic1)
 *   SD de Mic2/Mic3 queda en GPIO32. SD de Mic0/Mic1 queda en GPIO27.
 *
 *   Nota: GPIO14/15 ya NO van a los mics; quedan SOLO como entrada de clock del
 *   periférico slave, alimentados por los jumpers desde 26/25. (Alternativa sin
 *   mover los mics: dejá Mic2/Mic3 en 14/15 y los jumpers 26->14, 25->15 los
 *   alimentan igual, porque 14/15 ahora son entradas manejadas por el master.)
 *
 * ============================================================================
 * NOTAS
 * ============================================================================
 *   - mck_io_num = I2S_PIN_NO_CHANGE en ambos. El INMP441 no necesita MCLK.
 *   - use_apll = false en ambos. Ya no importa para la sincronía: el slave no
 *     genera clock, lo recibe del master.
 *   - En modo SLAVE el campo sample_rate se ignora para el clock (lo impone el
 *     master); se deja igual solo para el dimensionamiento de buffers DMA.
 *   - Orden de arranque: se instala primero el SLAVE (queda esperando clocks) y
 *     después el MASTER (empieza a clockear). Así el slave no pierde el frame
 *     inicial de alineación.
 *   - Con WS compartido ambos periféricos cuentan los frames desde el mismo
 *     flanco -> correspondencia muestra-a-muestra exacta, sin offset entre buses.
 *   - GAIN_SHIFT toma los MSBs del dato de 24 bits del INMP441 (ganancia nominal).
 *     Todos los canales usan el mismo shift: L/R=GND y L/R=VDD tienen la misma
 *     amplitud una vez que el bias del INMP441 se estabiliza (~3 s de arranque).
 *     Si hay clipping, subir GAIN_SHIFT. Si la señal es muy débil, bajarlo.
 */

#include <driver/i2s.h>

// ============================================================================
// CONFIGURACIÓN
// ============================================================================

// 16 bits por muestra a 11.025 kHz, little-endian. Cuenta de ancho de banda CP2102:
//   921600 baud 8N1 = 92160 B/s efectivos.
//   11025 Hz x 4 ch x 2 bytes + framing(4 B/hop) = 11025 x 8.016 = 88.4 kB/s (~96%).
// Por que 11025/16-bit y no 22050/8-bit: el aliasing espacial topa la localizacion
// en ~2425 Hz (lo fija la apertura, no la fs), asi que el ancho de banda extra de
// 22 kHz es inutil para DOA; lo que importa es el rango dinamico, y 16 bits (~96 dB)
// ayuda a detectar los armonicos debiles de un dron lejano (gate espectral). Ambos
// formatos entran al ~96% del enlace; 22 kHz/16-bit NO entra (~1.9x). Si hay drops
// (pkts_lost>0 en el display), bajar SAMPLE_RATE a 10000.
#define SAMPLE_RATE    11025
#define HOP_SIZE       256
#define DMA_BUF_LEN    256
#define DMA_BUF_COUNT  4

// Shift de int32(I2S) -> int16. Toma los MSBs del dato de 24 bits del INMP441.
// Todos los canales usan el mismo valor (L/R=GND y L/R=VDD tienen la misma
// amplitud una vez estabilizados).
// >>> ESTE ES EL PARAMETRO QUE CASI SEGURO VAS A TENER QUE TUNEAR <<<
// Mira el display de diagnose_serial: apunta a std ~2000-8000 y clip% ~0.
//   muy bajo (std<500)  -> bajar GAIN_SHIFT (mas ganancia)
//   clip alto (>1%)     -> subir GAIN_SHIFT
#define GAIN_SHIFT     9
#define SERIAL_BAUD    921600

#define SYNC_BYTE      0xAA
#define END_BYTE       0x55

// Bus 0 = MASTER (genera el clock para los 4 mics)
#define PIN_SCK0  26   // salida SCK del master  -> los 4 mics + jumper a GPIO14
#define PIN_WS0   25   // salida WS  del master  -> los 4 mics + jumper a GPIO15
#define PIN_SD0   22   // datos mic0/mic1  <<< MOVIDO de 27 a 22: GPIO27 era
                       // adyacente a SCK/WS (25/26) y captaba crosstalk del clock
                       // -> bus0 salia ruidoso. 22 esta lejos de las lineas de
                       // clock. Mantener el cable SD0 corto. (El swap de SD probo
                       // que el ruido seguia a la ruta de bus0, no a los mics.)

// Bus 1 = SLAVE (recibe el clock del master por jumper)
#define PIN_SCK1  14   // ENTRADA SCK del slave  <- jumper desde GPIO26
#define PIN_WS1   15   // ENTRADA WS  del slave  <- jumper desde GPIO25
#define PIN_SD1   32   // datos mic2/mic3

// ============================================================================
// BUFFERS GLOBALES
// ============================================================================

static int32_t  buf0[HOP_SIZE * 2];
static int32_t  buf1[HOP_SIZE * 2];
static int16_t  out_buf[HOP_SIZE * 4];   // 16 bits/muestra, interleaved 4 canales
static uint16_t frame_counter = 0;

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
        .bck_io_num   = sck,   // master: salida | slave: entrada (lo maneja el driver)
        .ws_io_num    = ws,    // master: salida | slave: entrada
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num  = sd
    };

    i2s_driver_install(port, &cfg, 0, NULL);
    i2s_set_pin(port, &pins);
}

// ============================================================================
// CAPTURA Y TRANSMISIÓN
// ============================================================================

static bool capture_hop() {
    size_t br0 = 0, br1 = 0;
    size_t expected = HOP_SIZE * 2 * sizeof(int32_t);

    // El slave se lee primero: como comparte clock con el master, ambos DMA
    // se llenan en lockstep; el orden de lectura no introduce desfase.
    esp_err_t e1 = i2s_read(I2S_NUM_1, buf1, expected, &br1, portMAX_DELAY);
    esp_err_t e0 = i2s_read(I2S_NUM_0, buf0, expected, &br0, portMAX_DELAY);

    if (e0 != ESP_OK || e1 != ESP_OK) return false;
    if (br0 < expected || br1 < expected) return false;

    for (int i = 0; i < HOP_SIZE; i++) {
        int32_t s0 = buf0[i * 2 + 0] >> GAIN_SHIFT;  // Mic 0
        int32_t s1 = buf0[i * 2 + 1] >> GAIN_SHIFT;  // Mic 1
        int32_t s2 = buf1[i * 2 + 0] >> GAIN_SHIFT;  // Mic 2
        int32_t s3 = buf1[i * 2 + 1] >> GAIN_SHIFT;  // Mic 3

        // Clamp a rango int16 (-32768..32767)
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
    uint8_t header[3] = {
        SYNC_BYTE,
        (uint8_t)((frame_counter >> 8) & 0xFF),
        (uint8_t)(frame_counter & 0xFF)
    };
    Serial.write(header, 3);
    // int16 little-endian (nativo ESP32, sin byte-swap): audio_input.py /
    // capture_wav.py / diagnose_serial.py leen con '<i2' en coherencia.
    Serial.write((uint8_t*)out_buf, HOP_SIZE * 4 * sizeof(int16_t));
    Serial.write(END_BYTE);
    frame_counter++;
}

// ============================================================================
// SETUP Y LOOP
// ============================================================================

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(100);

    // IMPORTANTE: instalar el SLAVE primero (queda esperando clocks) y luego el
    // MASTER (empieza a generarlos). Así el slave se alinea desde el primer WS.
    setup_i2s(I2S_NUM_1, I2S_MODE_SLAVE,  PIN_SCK1, PIN_WS1, PIN_SD1);
    setup_i2s(I2S_NUM_0, I2S_MODE_MASTER, PIN_SCK0, PIN_WS0, PIN_SD0);
   //setup_i2s(I2S_NUM_0, I2S_MODE_SLAVE,  PIN_SCK0, PIN_WS0, PIN_SD0);
   //setup_i2s(I2S_NUM_1, I2S_MODE_MASTER, PIN_SCK1, PIN_WS1, PIN_SD1);

    // Estabilización de los INMP441 (~85 ms mínimo) + asentamiento del slave
    delay(200);

    // Descartar transitorios iniciales (cápsula + alineación de DMA)
    for (int i = 0; i < 20; i++) {
        capture_hop();
    }
}

void loop() {
    if (capture_hop()) {
        send_hop();
    }
}
