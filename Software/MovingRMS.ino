#include <avr/interrupt.h>
#include <math.h>

#define OFFSET_VOL 1.504f
#define OFFSET_ADC 308.00f
#define debugPin 2
#define FACTOR_ADC 1.42286217f
#define FACTOR_VOL 294.1176f
#define MOVING_RMS_MAX_BUF 100

bool TEST = 0;
volatile float voltageValue;
volatile float rms; // = 226.5;
String s = "";
String s1 = "";


typedef struct {
  uint16_t L;      // Windows length
  float invL;      // 1 divided by window length
  uint16_t count;  // circular buffer
  float in_sq_L[MOVING_RMS_MAX_BUF];
  float out_sq;  // RMS estimate Squared
  float rms;     // RMS estimate
} MovingRMS;

void MovingRMS_Init(MovingRMS *mrms, uint16_t L) {
  mrms->L = L;
  mrms->invL = 1.0f / ((float)L);
  mrms->count = 0;
  // Clear buffer
  for (uint16_t n = 0; n < L; n++)
    mrms->in_sq_L[n] = 0.0f;
  // Clear output
  mrms->out_sq = 0.0f;
}

void MovingRMS_Update(MovingRMS *mrms, float in) {
  float in_sq = in * in;
  mrms->in_sq_L[mrms->count] = in_sq;
  if (mrms->count == (mrms->L - 1)) {
    mrms->count = 0;
    // mrms->rms = sqrt(mrms->out_sq);
  } else
    mrms->count++;
  mrms->out_sq = mrms->out_sq + mrms->invL * (in_sq - mrms->in_sq_L[mrms->count]);
}

void RMS_Update(MovingRMS *mrms) {
  mrms->rms = sqrt(mrms->out_sq);
}


MovingRMS mrms;

ISR(TIMER1_COMPA_vect) {  //TIMER1_OVF_vect
  voltageValue = (float(analogRead(A0)) - OFFSET_ADC) * FACTOR_ADC;
  MovingRMS_Update(&mrms, voltageValue);

  // digitalWrite(debugPin, TEST);
  // TEST = TEST ? 0 : 1;
  // rms = sqrt(mrms.out_sq);

  // Serial.print(voltageValue);
  // Serial.print(",");
  // Serial.println(rms);
}


void setup() {
  pinMode(A0, INPUT);
  pinMode(debugPin, OUTPUT);
  Serial.begin(115200);
  digitalWrite(debugPin, 1);
  MovingRMS_Init(&mrms, MOVING_RMS_MAX_BUF);

  TCCR1A = 0;
  TCCR1B = 0;  // Reset 2 registers

  TCCR1B |= (1 << WGM12);  // | (1 << WGM13);  // CTC - TOP = ICR1

  OCR1A = 3199;             // Top value [Frequency = 16M / ICR1]
  TCCR1B |= (1 << CS10);    // No Prescaling = F_Clock or F_clock/1=16mhz
  TIMSK1 |= (1 << OCIE1A);  // Timer1 Overflow Interrupt Enable

  sei();
}

void loop() {
  rms = sqrt(mrms.out_sq);
  // Serial.print(voltageValue);
  // Serial.print(",");
  Serial.println(rms);
}
