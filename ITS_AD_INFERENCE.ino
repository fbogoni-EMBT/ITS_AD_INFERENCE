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
 *    - Seriale: fase, similarita', stato NORMALE/ANOMALIA (SERIAL_OUTPUT)
 *    - LED integrato: lampeggio veloce = errore, 1 Hz = funzionamento
 *    - Matrice LED 12x8 (USE_LED_MATRIX): barra proporzionale alla similarita'
 *      (durante l'apprendimento mostra l'avanzamento)
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

// Similarita' (%) sotto la quale il buffer e' considerato anomalo
#define ANOMALY_THRESHOLD      90

// Filtro: numero di buffer anomali CONSECUTIVI per dichiarare l'anomalia
// (1 = nessun filtro)
#define ANOMALY_FILTER_COUNT   3

// ----------------------------------------------------------------------------
//  USCITE
// ----------------------------------------------------------------------------

#define SERIAL_OUTPUT          1        // 1 = invia i risultati su seriale
#define SERIAL_BAUD            115200

#define USE_LED_MATRIX         1        // 1 = barra di similarita' sulla matrice

#define ERROR_BLINK_MS         100      // semiperiodo lampeggio di errore
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
  #define N_LEARNING  20
#endif

#define PERIOD_US   (1000000UL / SAMPLING_FREQ_HZ)
#define PERIOD_REM  (1000000UL % SAMPLING_FREQ_HZ)

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
static uint8_t frame[8][12];
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
  memset(frame, 0, sizeof(frame));
  matrix.renderBitmap(frame, 8, 12);
#endif
  while (true) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(ERROR_BLINK_MS);
  }
}

// ----------------------------------------------------------------------------
//  MATRICE LED: barra orizzontale 0..100 %
// ----------------------------------------------------------------------------

void drawBar(uint8_t percent) {
#if USE_LED_MATRIX
  if (percent > 100) percent = 100;
  uint8_t cols = (uint8_t)((percent * 12u + 50u) / 100u);   // 0..12 colonne
  for (uint8_t r = 0; r < 8; r++) {
    for (uint8_t c = 0; c < 12; c++) {
      frame[r][c] = (c < cols) ? 1 : 0;
    }
  }
  matrix.renderBitmap(frame, 8, 12);
#else
  (void)percent;
#endif
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

    // LED a 1 Hz anche durante acquisizioni lunghe (operazione brevissima)
    ledService();
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
  enum neai_state st = neai_anomalydetection_init();
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
        st != NEAI_NOT_ENOUGH_CALL_TO_LEARNING &&
        st != NEAI_MINIMAL_RECOMMENDED_LEARNING_DONE) {
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
#endif
}

// ----------------------------------------------------------------------------
//  LOOP: un rilevamento ogni DETECT_PERIOD_MS
// ----------------------------------------------------------------------------

void loop() {
  static uint32_t nextDetect = millis();

  // Attesa dell'istante del prossimo rilevamento
  while ((int32_t)(millis() - nextDetect) < 0) {
    ledService();
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
  bool anomaly = (anomalyCounter >= ANOMALY_FILTER_COUNT);

  drawBar(similarity);

#if SERIAL_OUTPUT
  Serial.print("similarita': ");
  Serial.print(similarity);
  Serial.print(" % | ");
  Serial.print(anomaly ? "ANOMALIA" : "NORMALE");
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
}
