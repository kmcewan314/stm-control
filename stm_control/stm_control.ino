#include <AccelStepper.h>

// --- pin definitions ---
const int TRIG_PIN = 0;
const int PUL_PIN = 12;
const int DIR_PIN = 13;
const int STEP_ENC_PIN = 14;
const int MIR_ENC_PIN = 15;


// **** EDITABLE VARIABLES ***

// --- motor resolutions ---
int STEPS_PER_REV = 8000; // controlled by dip switches
int STEP_PULSES_PER_REV = 1000; // only change if changing motors
int MIR_PULSES_PER_REV = 4320; // gear ratio of motor * (CPR of encoder / 4)

// --- output timing ---
int SAMPLE_INTERVAL_MS = 5; // how often to record motor positions, in milliseconds
int TRIG_INTERVAL_MS = 100; // how often to send trigger signal to camera, in milliseconds
int TRIG_LENGTH = 2; // how long the trigger pulse is, in *micro*seconds

// *** END EDITABLE VARIABLES ***

// set up stepper object for motor control
AccelStepper stepper(AccelStepper::DRIVER, PUL_PIN, DIR_PIN);
// acceleration limit in steps/sec^2 (STEPS_PER_REV steps/sec^2 = ramps up/down by 1 RPS per second)
const float ACCELERATION_STEPS_SEC2 = 4000.0;

// shared states
volatile bool systemActive = false;
volatile float target_steps_per_sec = 1600.0;

// --- ISR variables ---
volatile long step_edge_count = 0;
volatile long mir_edge_count = 0;

// --- logging variables ---
unsigned long last_sample_time = 0;
unsigned long last_trig_time = 0;
// convert intervals to microseconds
int TRIG_INTERVAL_US = TRIG_INTERVAL_MS * 1000;
int SAMPLE_INTERVAL_US = SAMPLE_INTERVAL_MS * 1000;


// --- interrupt service routines (ISR) ---
void stepEncoderISR() { step_edge_count++; }
void mirEncoderISR() { mir_edge_count++; }

// --- other functions ---
// send pulse
void sendTrigPulse() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(TRIG_LENGTH);
  digitalWrite(TRIG_PIN, HIGH);
}

// send start synchronization signal
void sendSyncPulse() {
  sendTrigPulse();
  delay(10);
  sendTrigPulse();
}

// start/stop control with keyboard 
void checkSerialCommands() {
  while (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.length() == 0) continue;

    if (cmd == "START") {
      sendSyncPulse();      // fire sync pattern for camera
      systemActive = true;  // start motor and logging loop
      Serial.println("SYSTEM_STARTED");
    } else if (cmd == "STOP") {
      systemActive = false; // pause motor and logging
      Serial.println("SYSTEM_STOPPED");
    } else {
      // if not start/stop signal, try parsing as new target RPM
      int newRPM = cmd.toInt();
      if (newRPM > 0) {
        target_steps_per_sec = ((float)newRPM * STEPS_PER_REV) / 60.0;
        Serial.print("RPM_UPDATED:");
        Serial.println(newRPM);
      }
    }
  }
}

// --- runtime code ---
// *** core 0: encoders and trigger ***
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, HIGH);

  pinMode(STEP_ENC_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(STEP_ENC_PIN), stepEncoderISR, RISING);

  pinMode(MIR_ENC_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(MIR_ENC_PIN), mirEncoderISR, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  // --- time check ---
  unsigned long now_us = micros();

  // --- check for commands ---
  checkSerialCommands();

  if (systemActive) {
    bool trig_sent = false;

    // --- trigger signal ---
    if (now_us - last_trig_time >= TRIG_INTERVAL_US) {
      last_trig_time = now_us;
      trig_sent = true;
      sendTrigPulse();
    }
  
    // --- motor logging ---
    if (now_us - last_sample_time >= SAMPLE_INTERVAL_US) {
      last_sample_time = now_us;

      // safely copy encoder counts from ISR
      noInterrupts();
      long current_step_edges = step_edge_count;
      long current_mir_edges = mir_edge_count;
      interrupts();

      // calculate angles of motors
      float step_deg = fmod((current_step_edges % STEP_PULSES_PER_REV) * (360.0 / STEP_PULSES_PER_REV), 360.0);
      float mir_deg  = fmod((current_mir_edges % MIR_PULSES_PER_REV) * (360.0 / MIR_PULSES_PER_REV), 360.0);

      // csv ouput to serial port
      // format: "<timestamp>,<object angle>,<mirror angle>,<trigger sent T/F>"
      Serial.print(now_us);
      Serial.print(",");
      Serial.print(step_deg, 2);
      Serial.print(",");
      Serial.print(mir_deg, 2);
      Serial.print(",");
      Serial.println(trig_sent);
    }
  }
}


// *** core 1: stepper motor control ***
void setup1() {
  stepper.setAcceleration(ACCELERATION_STEPS_SEC2); // smooth acceleration rate
  stepper.setMaxSpeed(target_steps_per_sec); // speed it will move after acceleration
  stepper.moveTo(2000000000L); // set destination 2 billion steps away
}

void loop1() {
  if (systemActive) {
    // update speed limit
    stepper.setMaxSpeed(target_steps_per_sec);

    // move destination further ahead when it gets close to keep continuous motion
    if (stepper.distanceToGo() < 100000) {
      stepper.moveTo(stepper.currentPosition() + 2000000000L);
    }

    // run with smooth acceleration
    stepper.run();

  } else {
    if (stepper.speed() != 0) {
      // if not already stopped, stop as quickly as possible within acceleration limit
      stepper.stop();
      stepper.run();
    } else {
      // make sure we're ready to start again
      stepper.moveTo(stepper.currentPosition() + 2000000000L);
    }
  }
}

