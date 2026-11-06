/*
 * Project  : Anomaly detection tramite Edge AI
 * File     : ITS_AD_INFERENCE.ino
 * Title    : NanoEdge AI anomaly detection inference
 * Author   : Fabrizio Bogoni
 * Date     : 2026-11-06
 * Hardware : Arduino UNO R4 WiFi + Modulino Movement (Qwiic) or analog mic
 * Notes    : source in DATA_SOURCE, thresholds in ANOMALY_THRESHOLD (90)
 *            and ANOMALY_FULL_LEVEL (50), filter in ANOMALY_FILTER_COUNT
 * Revision : 01
 */

/*
 * ============================================================================
 *  Rilevamento anomalie con libreria NanoEdge AI (Anomaly Detection)
 *  Scheda:   Arduino UNO R4 WiFi
 *  Sorgenti (alternative, scelte con DATA_SOURCE):
 *    - Arduino Modulino Movement (LSM6DSOX) sul connettore Qwiic
 *    - Microfono analogico su pin configurabile (A0 default)
 *
 *  Librerie:
 *    - libreria generata da NanoEdge AI Studio (fase Deploy -> Arduino),
 *      installata come .zip dall'IDE (Sketch -> Include Library -> Add .ZIP)
 *    - Arduino_Modulino (solo con SOURCE_MODULINO)
 *    - Arduino_LED_Matrix (inclusa nel core UNO R4, solo con USE_LED_MATRIX)
 * ============================================================================
 *
 *  Funzionamento:
 *    1. init della libreria
 *    2. AUTO-APPRENDIMENTO: LEARNING_CYCLES buffer di funzionamento NORMALE
 *       (di default il minimo consigliato dalla libreria)
 *    3. RILEVAMENTO: un buffer ogni DETECT_PERIOD_MS, la libreria restituisce
 *       la similarita' (0..100 %) rispetto al comportamento appreso
 *
 *  IMPORTANTE: sorgente, BUFFER_SIZE, SAMPLING_FREQ_HZ, assi e trattamento
 *  del DC devono essere IDENTICI a quelli usati per acquisire il dataset
 *  con lo sketch di raccolta dati.
 *
 *  Segnalazioni:
 *    - Seriale: fase, similarita', livello di anomalia, stato (SERIAL_OUTPUT)
 *    - LED integrato: lampeggio veloce = errore, 1 Hz = funzionamento
 *    - Matrice LED 12x8 (USE_LED_MATRIX):
 *        apprendimento -> barra di avanzamento
 *        NORMALE       -> un pixel che si muove avanti e indietro sulla
 *                         prima riga ("tutto ok, il sistema e' vivo")
 *        ANOMALIA      -> scritta "ERR" che lampeggia sempre piu' veloce e
 *                         barra di livello (righe 7-8) sempre piu' lunga,
 *                         man mano che la similarita' scende da
 *                         ANOMALY_THRESHOLD (livello 0 %) fino a
 *                         ANOMALY_FULL_LEVEL (livello 100 %, massimo)
 *
 *    Livello di anomalia:
 *
 *      similarita'  100 ...... 90 ............ 50 ...... 0
 *                   |  NORMALE |    ANOMALIA    | MASSIMO |
 *      livello            0 %  |  0 % -> 100 %  |  100 %
 *                  (THRESHOLD)            (FULL_LEVEL)
 *
 *  Nota: la conoscenza appresa sta in RAM, a ogni reset si riparte
 *  dall'apprendimento.
 * ============================================================================
 */

// ----------------------------------------------------------------------------
//  SELEZIONE SORGENTE
// ----------------------------------------------------------------------------

#define SOURCE_MODULINO    1
#define SOURCE_MICROPHONE  2

#define DATA_SOURCE        SOURCE_MODULINO   // SOURCE_MODULINO oppure SOURCE_MICROPHONE

// ----------------------------------------------------------------------------
//  PARAMETRI ACCELEROMETRO (uguali allo sketch di acquisizione)
// ----------------------------------------------------------------------------

#define ACC_BUFFER_SIZE        256
#define ACC_SAMPLING_FREQ_HZ   100
#define USE_AXIS_X             1
#define USE_AXIS_Y             1
#define USE_AXIS_Z             1

// ----------------------------------------------------------------------------
//  PARAMETRI MICROFONO (uguali allo sketch di acquisizione)
// ----------------------------------------------------------------------------

#define MIC_PIN                A0
#define MIC_BUFFER_SIZE        1024
#define MIC_SAMPLING_FREQ_HZ   20000
#define MIC_ADC_BITS           12
#define MIC_REMOVE_DC          1

// ----------------------------------------------------------------------------
//  PARAMETRI DI RILEVAMENTO
// ----------------------------------------------------------------------------

// Periodo di rilevamento (ms). Se l'acquisizione di un buffer dura di piu'
// (es. 256 campioni a 100 Hz = 2,56 s) i rilevamenti avvengono uno dopo
// l'altro, senza attesa.
#define DETECT_PERIOD_MS       1000

// Cicli di apprendimento. 0 = usa il minimo consigliato dalla libreria
// (MINIMUM_ITERATION_CALLS_FOR_EFFICIENT_LEARNING)
#define LEARNING_CYCLES        0

// Soglia di errore: similarita' (%) sotto la quale il buffer e' anomalo.
// A questa similarita' il livello di anomalia vale 0 %.
#define ANOMALY_THRESHOLD      90

// Similarita' (%) alla quale la segnalazione raggiunge il MASSIMO
// (livello 100 %). Sotto questo valore resta al massimo.
// Deve essere minore di ANOMALY_THRESHOLD.
#define ANOMALY_FULL_LEVEL     50

// Filtro: numero di buffer anomali CONSECUTIVI per dichiarare l'anomalia
// (1 = nessun filtro)
#define ANOMALY_FILTER_COUNT   3

// ----------------------------------------------------------------------------
//  USCITE
// ----------------------------------------------------------------------------

#define SERIAL_OUTPUT          1        // 1 = invia i risultati su seriale
#define SERIAL_BAUD            115200

#define USE_LED_MATRIX         1        // 1 = segnalazioni sulla matrice

// Stato NORMALE: passo del pixel in movimento sulla prima riga (ms)
#define OK_PIXEL_STEP_MS       100

// Stato ANOMALIA: semiperiodo di lampeggio della scritta ERR (ms)
//   ERR_BLINK_SLOW_MS -> a livello 0 %   (appena sotto ANOMALY_THRESHOLD)
//   ERR_BLINK_FAST_MS -> a livello 100 % (ANOMALY_FULL_LEVEL o meno)
//   ERR_BLINK_FAST_MS = 0 -> al massimo la scritta resta accesa fissa
#define ERR_BLINK_SLOW_MS      600
#define ERR_BLINK_FAST_MS      70

// 1 = barra di livello sotto la scritta ERR (righe 7-8, da 1 a 12 colonne)
#define ERR_SHOW_LEVEL_BAR     1

#define ERROR_BLINK_MS         100      // semiperiodo lampeggio di errore fatale
#define RUN_BLINK_MS           500      // semiperiodo lampeggio normale (1 Hz)

// Pin per oscilloscopio (-1 = disabilitato), come nello sketch di acquisizione
#define PIN_SAMPLE_TOGGLE      2
#define PIN_BUSY               3
#define USE_FAST_GPIO          1

// ----------------------------------------------------------------------------
//  INCLUDE
// ----------------------------------------------------------------------------

#include "NanoEdgeAI.h"

#if DATA_SOURCE == SOURCE_MODULINO
  #include <Modulino.h>
#endif

#if USE_LED_MATRIX
  #include "Arduino_LED_Matrix.h"
#endif

// ----------------------------------------------------------------------------
//  DERIVATI E CONTROLLI
// ----------------------------------------------------------------------------

#if DATA_SOURCE == SOURCE_MODULINO
  #define BUFFER_SIZE        ACC_BUFFER_SIZE
  #define SAMPLING_FREQ_HZ   ACC_SAMPLING_FREQ_HZ
  #define NUM_AXES           (USE_AXIS_X + USE_AXIS_Y + USE_AXIS_Z)
  #if NUM_AXES == 0
    #error "Abilita almeno un asse"
  #endif
#elif DATA_SOURCE == SOURCE_MICROPHONE
  #define BUFFER_SIZE        MIC_BUFFER_SIZE
  #define SAMPLING_FREQ_HZ   MIC_SAMPLING_FREQ_HZ
  #define NUM_AXES           1
#else
  #error "DATA_SOURCE non valido"
#endif

#if (ANOMALY_THRESHOLD > 100) || (ANOMALY_THRESHOLD < 1)
  #error "ANOMALY_THRESHOLD deve essere tra 1 e 100"
#endif
#if ANOMALY_FULL_LEVEL >= ANOMALY_THRESHOLD
  #error "ANOMALY_FULL_LEVEL deve essere minore di ANOMALY_THRESHOLD"
#endif
#if ERR_BLINK_FAST_MS > ERR_BLINK_SLOW_MS
  #error "ERR_BLINK_FAST_MS deve essere <= ERR_BLINK_SLOW_MS"
#endif

// Coerenza con la libreria generata da NanoEdge
#if defined(DATA_INPUT_USER) && (DATA_INPUT_USER != BUFFER_SIZE)
  #error "BUFFER_SIZE diverso da DATA_INPUT_USER della libreria NanoEdge"
#endif
#if defined(AXIS_NUMBER) && (AXIS_NUMBER != NUM_AXES)
  #error "Numero di assi diverso da AXIS_NUMBER della libreria NanoEdge"
#endif

#if LEARNING_CYCLES > 0
  #define N_LEARNING  LEARNING_CYCLES
#elif defined(MINIMUM_ITERATION_CALLS_FOR_EFFICIENT_LEARNING)
  #define N_LEARNING  MINIMUM_ITERATION_CALLS_FOR_EFFICIENT_LEARNING
#else
  #define N_LEARNING  12
#endif

#define PERIOD_US   (1000000UL / SAMPLING_FREQ_HZ)
#define PERIOD_REM  (1000000UL % SAMPLING_FREQ_HZ)

#define MATRIX_ROWS  8
#define MATRIX_COLS  12

// ----------------------------------------------------------------------------
//  GPIO VELOCI PER L'OSCILLOSCOPIO
// ----------------------------------------------------------------------------

#if USE_FAST_GPIO
struct FastPin {
  R_PORT0_Type* port;
  uint16_t      mask;
};

// Prototipo esplicito (evita il prototipo automatico dell'IDE fuori posto)
static FastPin makeFastPin(int pin);

static FastPin makeFastPin(int pin) {
  FastPin f;
  uint32_t p = (uint32_t)g_pin_cfg[pin].pin;   // (porta << 8) | bit
  f.port = (R_PORT0_Type*)((uint32_t)R_PORT0 + 0x20u * (p >> 8));
  f.mask = (uint16_t)(1u << (p & 0xFFu));
  return f;
}
#endif

#if PIN_SAMPLE_TOGGLE >= 0
  #if USE_FAST_GPIO
    static FastPin fpToggle;
    #define TOGGLE_SAMPLE()  (fpToggle.port->PODR ^= fpToggle.mask)
  #else
    static bool toggleState = false;
    #define TOGGLE_SAMPLE()  digitalWrite(PIN_SAMPLE_TOGGLE, (toggleState = !toggleState))
  #endif
#else
  #define TOGGLE_SAMPLE()
#endif

#if PIN_BUSY >= 0
  #if USE_FAST_GPIO
    static FastPin fpBusy;
    #define BUSY_HIGH()  (fpBusy.port->PODR |=  fpBusy.mask)
    #define BUSY_LOW()   (fpBusy.port->PODR &= (uint16_t)~fpBusy.mask)
  #else
    #define BUSY_HIGH()  digitalWrite(PIN_BUSY, HIGH)
    #define BUSY_LOW()   digitalWrite(PIN_BUSY, LOW)
  #endif
#else
  #define BUSY_HIGH()
  #define BUSY_LOW()
#endif

// ----------------------------------------------------------------------------
//  VARIABILI GLOBALI
// ----------------------------------------------------------------------------

#if DATA_SOURCE == SOURCE_MODULINO
ModulinoMovement movement;
#endif

#if USE_LED_MATRIX
ArduinoLEDMatrix matrix;
static uint8_t frame[MATRIX_ROWS][MATRIX_COLS];
#endif

// La libreria NanoEdge richiede float, assi interleaved
static float dataBuffer[BUFFER_SIZE * NUM_AXES];

static uint32_t lastLedToggle = 0;
static uint8_t  anomalyCounter = 0;
static uint32_t overruns = 0;

// ----------------------------------------------------------------------------
//  LED INTEGRATO
// ----------------------------------------------------------------------------

static inline void ledService() {
  uint32_t now = millis();
  if (now - lastLedToggle >= RUN_BLINK_MS) {
    lastLedToggle = now;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

// ----------------------------------------------------------------------------
//  LIVELLO DI ANOMALIA: 0..1000 (per mille)
//    similarita' >= ANOMALY_THRESHOLD  -> 0
//    similarita' <= ANOMALY_FULL_LEVEL -> 1000
//    in mezzo                          -> interpolazione lineare
// ----------------------------------------------------------------------------

static uint16_t anomalyLevel(uint8_t similarity) {
  if (similarity >= ANOMALY_THRESHOLD)  return 0;
  if (similarity <= ANOMALY_FULL_LEVEL) return 1000;
  return (uint16_t)(((uint32_t)(ANOMALY_THRESHOLD - similarity) * 1000u) /
                    (uint32_t)(ANOMALY_THRESHOLD - ANOMALY_FULL_LEVEL));
}

// ----------------------------------------------------------------------------
//  MATRICE LED
// ----------------------------------------------------------------------------

#if USE_LED_MATRIX

enum DisplayMode : uint8_t { DISP_LEARNING, DISP_OK, DISP_ERR };

static DisplayMode dispMode      = DISP_LEARNING;
static uint32_t    dispLastStep  = 0;
static int8_t      pixPos        = 0;      // colonna del pixel (stato OK)
static int8_t      pixDir        = 1;      // direzione del pixel
static bool        errVisible    = true;   // fase del lampeggio di ERR
static uint16_t    errHalfPeriod = ERR_BLINK_SLOW_MS;
static uint8_t     errLevelCols  = 1;      // lunghezza barra di livello

// Font 3x5: ogni riga e' un nibble, bit 2 = colonna sinistra
static const uint8_t GLYPH_E[5] = { 0b111, 0b100, 0b110, 0b100, 0b111 };
static const uint8_t GLYPH_R[5] = { 0b110, 0b101, 0b110, 0b101, 0b101 };

static void drawGlyph(const uint8_t* g, uint8_t col0, uint8_t row0) {
  for (uint8_t r = 0; r < 5; r++) {
    for (uint8_t c = 0; c < 3; c++) {
      if (g[r] & (0b100 >> c)) frame[row0 + r][col0 + c] = 1;
    }
  }
}

static inline void clearFrame() {
  memset(frame, 0, sizeof(frame));
}

static inline void showFrame() {
  matrix.renderBitmap(frame, MATRIX_ROWS, MATRIX_COLS);
}

// Stato NORMALE: un solo pixel acceso sulla prima riga
static void renderOk() {
  clearFrame();
  frame[0][pixPos] = 1;
  showFrame();
}

// Stato ANOMALIA: "ERR" sulle righe 1-5, barra di livello sulle righe 7-8
static void renderErr() {
  clearFrame();
  if (errVisible) {
    drawGlyph(GLYPH_E, 0, 0);   // colonne 0-2
    drawGlyph(GLYPH_R, 4, 0);   // colonne 4-6
    drawGlyph(GLYPH_R, 8, 0);   // colonne 8-10
  }
#if ERR_SHOW_LEVEL_BAR
  // La barra non lampeggia: il livello resta leggibile anche a scritta spenta
  for (uint8_t c = 0; c < errLevelCols; c++) {
    frame[6][c] = 1;
    frame[7][c] = 1;
  }
#endif
  showFrame();
}

#endif  // USE_LED_MATRIX

// Barra orizzontale 0..100 % (usata durante l'apprendimento)
void drawBar(uint8_t percent) {
#if USE_LED_MATRIX
  if (percent > 100) percent = 100;
  uint8_t cols = (uint8_t)((percent * 12u + 50u) / 100u);   // 0..12 colonne
  for (uint8_t r = 0; r < MATRIX_ROWS; r++) {
    for (uint8_t c = 0; c < MATRIX_COLS; c++) {
      frame[r][c] = (c < cols) ? 1 : 0;
    }
  }
  showFrame();
#else
  (void)percent;
#endif
}

// Passa allo stato NORMALE (il pixel riparte da dove era)
static void displaySetOk() {
#if USE_LED_MATRIX
  if (dispMode != DISP_OK) {
    dispMode     = DISP_OK;
    dispLastStep = millis();
    renderOk();
  }
#endif
}

// Passa allo stato ANOMALIA o ne aggiorna il livello
static void displaySetErr(uint16_t level) {
#if USE_LED_MATRIX
  // Semiperiodo: da ERR_BLINK_SLOW_MS (livello 0) a ERR_BLINK_FAST_MS (1000)
  errHalfPeriod = (uint16_t)(ERR_BLINK_SLOW_MS -
                  ((uint32_t)(ERR_BLINK_SLOW_MS - ERR_BLINK_FAST_MS) * level) / 1000u);
  // Barra: da 1 colonna (livello 0) a 12 colonne (livello 1000)
  errLevelCols  = (uint8_t)(1u + (11u * level + 500u) / 1000u);

  if (dispMode != DISP_ERR) {
    dispMode     = DISP_ERR;
    errVisible   = true;            // la scritta compare subito
    dispLastStep = millis();
  }
  if (errHalfPeriod == 0) errVisible = true;   // massimo = acceso fisso
  renderErr();
#else
  (void)level;
#endif
}

// Animazioni: chiamata spesso (anche durante l'acquisizione),
// ridisegna la matrice solo quando serve
static inline void displayService() {
#if USE_LED_MATRIX
  uint32_t now = millis();

  if (dispMode == DISP_OK) {
    if (now - dispLastStep >= OK_PIXEL_STEP_MS) {
      dispLastStep = now;
      pixPos += pixDir;
      if (pixPos <= 0 || pixPos >= MATRIX_COLS - 1) pixDir = -pixDir;  // rimbalzo
      renderOk();
    }
  } else if (dispMode == DISP_ERR) {
    if (errHalfPeriod > 0 && now - dispLastStep >= errHalfPeriod) {
      dispLastStep = now;
      errVisible = !errVisible;
      renderErr();
    }
  }
  // DISP_LEARNING: la barra di avanzamento e' disegnata da setup()
#endif
}

// Errore bloccante: messaggio e lampeggio veloce per sempre
void fatalError(const char* msg, int code) {
#if SERIAL_OUTPUT
  Serial.print("ERRORE: ");
  Serial.print(msg);
  Serial.print(" (codice ");
  Serial.print(code);
  Serial.println(")");
#endif
#if USE_LED_MATRIX
  clearFrame();
  showFrame();
#endif
  while (true) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(ERROR_BLINK_MS);
  }
}

// ----------------------------------------------------------------------------
//  ACQUISIZIONE (identica allo sketch di raccolta dati)
// ----------------------------------------------------------------------------

static inline void readSample(uint16_t i) {
#if DATA_SOURCE == SOURCE_MODULINO
  uint16_t idx = i * NUM_AXES;
  movement.update();
  #if USE_AXIS_X
    dataBuffer[idx++] = movement.getX();
  #endif
  #if USE_AXIS_Y
    dataBuffer[idx++] = movement.getY();
  #endif
  #if USE_AXIS_Z
    dataBuffer[idx++] = movement.getZ();
  #endif
#else
  dataBuffer[i] = (float)analogRead(MIC_PIN);
#endif
}

void acquireBuffer() {
  overruns = 0;
  uint32_t frac = 0;
  uint32_t next = micros();

  for (uint16_t i = 0; i < BUFFER_SIZE; i++) {
    while ((int32_t)(micros() - next) < 0) { ; }

    TOGGLE_SAMPLE();
    BUSY_HIGH();
    readSample(i);
    BUSY_LOW();

    next += PERIOD_US;
    frac += PERIOD_REM;
    if (frac >= SAMPLING_FREQ_HZ) {
      frac -= SAMPLING_FREQ_HZ;
      next++;
    }
    if ((int32_t)(micros() - next) >= 0) overruns++;

    // LED e animazioni anche durante acquisizioni lunghe
    // (lavorano solo quando scade il loro passo)
    ledService();
    displayService();
  }

#if DATA_SOURCE == SOURCE_MICROPHONE && MIC_REMOVE_DC
  // Nello sketch di acquisizione la media e' intera: stesso arrotondamento
  int32_t sum = 0;
  for (uint16_t i = 0; i < BUFFER_SIZE; i++) sum += (int32_t)dataBuffer[i];
  float mean = (float)(sum / BUFFER_SIZE);
  for (uint16_t i = 0; i < BUFFER_SIZE; i++) dataBuffer[i] -= mean;
#endif
}

// ----------------------------------------------------------------------------
//  SETUP: init + auto-apprendimento
// ----------------------------------------------------------------------------

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

#if SERIAL_OUTPUT
  Serial.begin(SERIAL_BAUD);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 3000)) { ; }   // non blocca senza PC
#endif

#if USE_LED_MATRIX
  matrix.begin();
  drawBar(0);
#endif

#if PIN_SAMPLE_TOGGLE >= 0
  pinMode(PIN_SAMPLE_TOGGLE, OUTPUT);
  digitalWrite(PIN_SAMPLE_TOGGLE, LOW);
  #if USE_FAST_GPIO
    fpToggle = makeFastPin(PIN_SAMPLE_TOGGLE);
  #endif
#endif
#if PIN_BUSY >= 0
  pinMode(PIN_BUSY, OUTPUT);
  digitalWrite(PIN_BUSY, LOW);
  #if USE_FAST_GPIO
    fpBusy = makeFastPin(PIN_BUSY);
  #endif
#endif

  // --- Sensore ---
#if DATA_SOURCE == SOURCE_MODULINO
  Modulino.begin();
  if (!movement.begin()) {
    fatalError("Modulino Movement non trovato", 0);
  }
#else
  analogReadResolution(MIC_ADC_BITS);
  (void)analogRead(MIC_PIN);
#endif
  delay(100);

  // --- Libreria NanoEdge ---
  enum neai_state st = neai_anomalydetection_init(true);
  if (st != NEAI_OK) {
    fatalError("init NanoEdge fallita", (int)st);
  }

  // --- Auto-apprendimento ---
#if SERIAL_OUTPUT
  Serial.print("Apprendimento: ");
  Serial.print(N_LEARNING);
  Serial.println(" cicli (macchina in funzionamento NORMALE)");
#endif

  for (uint16_t n = 0; n < N_LEARNING; n++) {
    acquireBuffer();
    st = neai_anomalydetection_learn(dataBuffer);

    if (st != NEAI_OK &&
        st != NEAI_LEARNING_DONE &&
        st != NEAI_LEARNING_IN_PROGRESS) {
      fatalError("learn NanoEdge fallito", (int)st);
    }

    drawBar((uint8_t)(((n + 1) * 100UL) / N_LEARNING));

#if SERIAL_OUTPUT
    Serial.print("  learn ");
    Serial.print(n + 1);
    Serial.print("/");
    Serial.println(N_LEARNING);
#endif
  }

#if SERIAL_OUTPUT
  Serial.println("Apprendimento completato, inizio rilevamento");
  Serial.print("Soglia anomalia: ");
  Serial.print(ANOMALY_THRESHOLD);
  Serial.print(" % | livello massimo a: ");
  Serial.print(ANOMALY_FULL_LEVEL);
  Serial.print(" % | filtro: ");
  Serial.print(ANOMALY_FILTER_COUNT);
  Serial.println(" buffer consecutivi");
#endif

  displaySetOk();
}

// ----------------------------------------------------------------------------
//  LOOP: un rilevamento ogni DETECT_PERIOD_MS
// ----------------------------------------------------------------------------

void loop() {
  static uint32_t nextDetect = millis();

  // Attesa dell'istante del prossimo rilevamento (animazioni attive)
  while ((int32_t)(millis() - nextDetect) < 0) {
    ledService();
    displayService();
  }
  nextDetect += DETECT_PERIOD_MS;

  acquireBuffer();

  uint8_t similarity = 0;
  enum neai_state st = neai_anomalydetection_detect(dataBuffer, &similarity);
  if (st != NEAI_OK) {
    fatalError("detect NanoEdge fallito", (int)st);
  }

  // Soglia + filtro sui buffer consecutivi
  if (similarity < ANOMALY_THRESHOLD) {
    if (anomalyCounter < 255) anomalyCounter++;
  } else {
    anomalyCounter = 0;
  }
  bool     anomaly = (anomalyCounter >= ANOMALY_FILTER_COUNT);
  uint16_t level   = anomalyLevel(similarity);   // 0..1000

  // Con l'anomalia confermata il buffer corrente e' sempre sotto soglia,
  // quindi il livello parte da 0 % e cresce verso il massimo
  if (anomaly) {
    displaySetErr(level);
  } else {
    displaySetOk();
  }

#if SERIAL_OUTPUT
  Serial.print("similarita': ");
  Serial.print(similarity);
  Serial.print(" % | ");
  if (anomaly) {
    Serial.print("ANOMALIA livello ");
    Serial.print(level / 10);
    Serial.print(" %");
  } else if (anomalyCounter > 0) {
    // Sotto soglia ma non ancora confermata dal filtro
    Serial.print("NORMALE (sotto soglia ");
    Serial.print(anomalyCounter);
    Serial.print("/");
    Serial.print(ANOMALY_FILTER_COUNT);
    Serial.print(")");
  } else {
    Serial.print("NORMALE");
  }
  if (overruns > 0) {
    Serial.print(" | overrun: ");
    Serial.print(overruns);
  }
  Serial.println();
#endif

  // Acquisizione piu' lunga del periodo: si riallinea senza accumulare ritardo
  if ((int32_t)(millis() - nextDetect) >= 0) {
    nextDetect = millis();
  }

  ledService();
  displayService();
}
