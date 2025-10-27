// Sketch: ESP32 - 4 flex sensors -> 2 x WS2812B (75 leds each)
// Behavior:
// - sensor1 controls cascade (end->start) on strip A
// - sensor2 controls cascade (end->start) on strip B
// - sensor3 controls color pulses on strip A (hue & speed based on flex)
// - sensor4 controls color pulses on strip B (hue & speed based on flex)

#include <FastLED.h>

// ---------- CONFIG ----------
#define NUM_LEDS_PER_STRIP 75
#define NUM_STRIPS 2

#define DATA_PIN_A 5    // strip A DIN
#define DATA_PIN_B 18   // strip B DIN

#define ADC_PIN_1 34    // sensor 1 -> cascade strip A
#define ADC_PIN_2 35    // sensor 2 -> cascade strip B
#define ADC_PIN_3 32    // sensor 3 -> color pulse A
#define ADC_PIN_4 33    // sensor 4 -> color pulse B

#define BRIGHTNESS 180          // 0-255 global max brightness (ajuste para economia)
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
// ----------------------------

CRGB ledsA[NUM_LEDS_PER_STRIP];
CRGB ledsB[NUM_LEDS_PER_STRIP];

const uint8_t smoothingSamples = 8; // média móvel simples para estabilidade

// Buffers para média
uint16_t adcHist1[smoothingSamples], adcHist2[smoothingSamples], adcHist3[smoothingSamples], adcHist4[smoothingSamples];
uint8_t histIndex = 0;

unsigned long lastMillis = 0;

void setup() {
  Serial.begin(115200);
  delay(100);

  FastLED.addLeds<LED_TYPE, DATA_PIN_A, COLOR_ORDER>(ledsA, NUM_LEDS_PER_STRIP).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<LED_TYPE, DATA_PIN_B, COLOR_ORDER>(ledsB, NUM_LEDS_PER_STRIP).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(BRIGHTNESS);

  // Inicializa buffers de média
  uint16_t init = readADC_raw(ADC_PIN_1);
  for (uint8_t i=0;i<smoothingSamples;i++){
    adcHist1[i] = init;
    adcHist2[i] = readADC_raw(ADC_PIN_2);
    adcHist3[i] = readADC_raw(ADC_PIN_3);
    adcHist4[i] = readADC_raw(ADC_PIN_4);
  }
  histIndex = 0;
}

void loop() {
  // le as 4 entradas suavizadas
  uint16_t s1 = readADC_smooth(ADC_PIN_1, adcHist1);
  uint16_t s2 = readADC_smooth(ADC_PIN_2, adcHist2);
  uint16_t s3 = readADC_smooth(ADC_PIN_3, adcHist3);
  uint16_t s4 = readADC_smooth(ADC_PIN_4, adcHist4);

  // Normaliza para 0..1 (float) com ADC do ESP32 = 0..4095
  float n1 = constrain((float)s1 / 4095.0, 0.0, 1.0);
  float n2 = constrain((float)s2 / 4095.0, 0.0, 1.0);
  float n3 = constrain((float)s3 / 4095.0, 0.0, 1.0);
  float n4 = constrain((float)s4 / 4095.0, 0.0, 1.0);

  // ----- STRIP A: cascade from end->start by n1, overlay pulse from n3 -----
  handleCascadeAndPulse(ledsA, NUM_LEDS_PER_STRIP, n1, n3);

  // ----- STRIP B: cascade from end->start by n2, overlay pulse from n4 -----
  handleCascadeAndPulse(ledsB, NUM_LEDS_PER_STRIP, n2, n4);

  FastLED.show();

  // small delay for stability
  delay(20);
}

// ---------------------- helper functions ----------------------

uint16_t readADC_raw(uint8_t pin) {
  // Leitura direta (ESP32 ADC range 0..4095)
  // Remova attenuation se não quiser (aqui assumimos padrão)
  return analogRead(pin);
}

uint16_t readADC_smooth(uint8_t pin, uint16_t *hist) {
  // escreve valor no buffer circular e retorna a média
  uint16_t val = readADC_raw(pin);
  hist[histIndex] = val;
  // increment circular index (apenas uma vez por ciclo)
  if (pin == ADC_PIN_4) { // update histIndex only after populating last sensor (arbitrary)
    histIndex++;
    if (histIndex >= smoothingSamples) histIndex = 0;
  }
  // calcular média
  uint32_t s = 0;
  for (uint8_t i=0;i<smoothingSamples;i++) s += hist[i];
  return (uint16_t)(s / smoothingSamples);
}

void handleCascadeAndPulse(CRGB *leds, uint16_t nLeds, float normCascade, float normPulse) {
  // normCascade: 0..1 -> porcentagem de LEDs acesos (do final para inicio)
  // normPulse: 0..1 -> controla hue e velocidade do pulso
  // 1) Primeiro, base cascade: acende leds do final ao índice start
  uint16_t ledsToLight = (uint16_t) round(normCascade * (float)nLeds); // 0..nLeds

  // Limpa buffer
  for (uint16_t i=0;i<nLeds;i++) leds[i] = CRGB::Black;

  // Acende do final para inicio: final é índice nLeds-1
  for (uint16_t i=0;i<ledsToLight;i++) {
    uint16_t idx = (nLeds - 1) - i;
    // cor base para a parte 'acesa' (pode customizar)
    leds[idx] = CHSV(160, 200, 255); // azul/verde neutro — pode alterar
  }

  // 2) Pulse overlay: cria um pulso global com hue baseado em normPulse
  // map normPulse -> hue (0..255)
  uint8_t hue = (uint8_t) round(normPulse * 255.0); // 0..255
  // map normPulse -> pulse speed (mais dobra = mais rápido)
  // pulsePeriod ms: quando normPulse=0 => slow 1400ms, normPulse=1 => fast 150ms
  uint16_t pulsePeriod = (uint16_t) map((int) round(normPulse * 1000.0), 0, 1000, 1400, 150);
  // pulse amplitude (quanto o pulso afeta o brilho) -> 0..200
  uint8_t pulseAmp = (uint8_t) (normPulse * 200.0);

  // calcula fase do pulso
  uint32_t t = millis();
  float phase = (sin( (2.0 * PI * (float)(t % pulsePeriod)) / (float)pulsePeriod ) + 1.0) / 2.0; // 0..1
  uint8_t pulseBright = (uint8_t) (phase * pulseAmp);

  // Aplica pulso por mistura ADD às cores existentes
  for (uint16_t i=0;i<nLeds;i++) {
    // Color pulse LED
    CRGB pulseCol = CHSV(hue, 200, pulseBright);
    // mistura aditiva para 'bater' o pulso sobre a base
    leds[i] += pulseCol;
  }
}

// --------------------------------------------------------------------

