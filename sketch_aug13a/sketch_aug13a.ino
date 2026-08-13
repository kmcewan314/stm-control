#include <MobaTools.h>

const int PUL_PIN = 12;
const int DIR_PIN = 13;
const int STEP_ENC_PIN = 14;

int STEPS_PER_REV = 8000; // controlled by dip switches
int STEP_PULSES_PER_REV = 1000; // only change if changing motors

int SAMPLE_INTERVAL = 5000; // how often to record motor positions, in milliseconds

MoToStepper stepper(STEPS_PER_REV, STEPDIR);

// --- ISR variables ---
volatile long step_edge_count = 0;
// --- logging variables ---
unsigned long last_sample_time = 0;
long mot_step_count = 0;
// --- interrupt service routines (ISR) ---
void stepEncoderISR() { step_edge_count++; }


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_ENC_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(STEP_ENC_PIN), stepEncoderISR, RISING);

  stepper.attach(PUL_PIN, DIR_PIN);
  stepper.setZero();
  stepper.setRampLen(3200);
  stepper.setMaxSpeed(3200);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long now = micros();

  if (now - last_sample_time >= SAMPLE_INTERVAL) {
    last_sample_time = now;
    noInterrupts();
    long current_step_edges = step_edge_count;
    interrupts();
    mot_step_count = stepper.readSteps();

    Serial.print("Encoder count: ");
    Serial.print(current_step_edges);
    Serial.print("        Motor count: ");
    Serial.println(mot_step_count);
  }
  if (mot_step_count < 32000) {
    stepper.rotate(1);
  } else {
    stepper.rotate(0);
  }

}
