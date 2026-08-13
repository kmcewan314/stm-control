#include <MobaTools.h>

const int PUL_PIN = 12;
const int DIR_PIN = 13;
const int STEP_ENC_PIN = 14;

int STEPS_PER_REV = 3200; // controlled by dip switches
int STEP_PULSES_PER_REV = 1000; // only change if changing motors

float POSITIVE_MAX_ACC = 25.1;
float NEGATIVE_MAX_ACC = -10.0;

int SAMPLE_INTERVAL = 5000; // how often to record motor positions, in microseconds
volatile int targetrpm = 0;
volatile int currentrpm = 0;
volatile int ramp = 800;
volatile int stepspeed = 0;
volatile bool systemActive = false;
volatile int last_change = 0;

MoToStepper stepper(STEPS_PER_REV, STEPDIR);

// --- ISR variables ---
volatile long step_edge_count = 0;
// --- logging variables ---
unsigned long last_sample_time = 0;
long mot_step_count = 0;
// --- interrupt service routines (ISR) ---
void stepEncoderISR() { step_edge_count++; }

int calcRamp(int oldrpm, int newrpm) {
  int delta = newrpm - oldrpm;
  if (delta > 0) {
    // speeding up
    float exact_ramp = (delta * delta * 0.0055) / POSITIVE_MAX_ACC;
    return static_cast<int>(exact_ramp);
  } else {
    float exact_ramp = (delta * delta * 0.0055) / NEGATIVE_MAX_ACC;
    return -1 * static_cast<int>(exact_ramp);
  }
}

int calcStepSpeed(int RPM) {
  return static_cast<int>(RPM*3200/6);
}

// start/stop control with keyboard 
void checkSerialCommands() {
  while (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.length() == 0) continue;

    if (cmd == "START") {
      systemActive = true;
      if (targetrpm > 0) {
        stepspeed = calcStepSpeed(targetrpm);
        ramp = calcRamp(currentrpm, targetrpm);
        stepper.setSpeedSteps(stepspeed, ramp);
        stepper.rotate(1);
      }
      Serial.println("SYSTEM_STARTED");
    } else if (cmd == "STOP") {
      systemActive = false;
      targetrpm = 0;
      stepper.rotate(0);
      Serial.println("SYSTEM_STOPPED");
      currentrpm = 0;
    } else {
      // if not start/stop signal, try parsing as new target RPM
      int newrpm = cmd.toInt();
      if (newrpm > 0) {
        targetrpm = newrpm;
        stepspeed = calcStepSpeed(targetrpm);
        ramp = calcRamp(currentrpm, targetrpm);
        if (systemActive) {
          stepper.setSpeedSteps(stepspeed, ramp);
          stepper.rotate(1);
        }
        currentrpm = targetrpm;
        Serial.print("RPM_UPDATED:");
        Serial.println(targetrpm);
      }
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_ENC_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(STEP_ENC_PIN), stepEncoderISR, RISING);

  stepper.attach(PUL_PIN, DIR_PIN);
  stepper.setZero();
}

void loop() {
  // put your main code here, to run repeatedly:
  checkSerialCommands();


}
