#include <FastAccelStepper.h>

// --- pin definitions ---
const int TRIG_PIN = 5;

const int MIR_PUL_PIN = 10;
const int MIR_DIR_PIN = 11;

const int OBJ_PUL_PIN = 12;
const int OBJ_DIR_PIN = 13;

// const int MIR_ENC_PIN = 16;
// const int OBJ_ENC_PIN = 17;


// **** EDITABLE VARIABLES ***

// --- motor resolutions ---
int OBJ_STEPS_PER_REV = 8000; // controlled by dip switches
int MIR_STEPS_PER_REV = 8000;

// int OBJ_ENC_PPR = 1000; // only change if changing motors
// int MIR_ENC_PPR = 2500; 

// --- output timing ---
int SAMPLE_INTERVAL_MS = 5; // how often to record motor positions, in milliseconds
int TRIG_INTERVAL_MS = 1000; // how often to send trigger signal to camera, in milliseconds
int TRIG_LENGTH = 2; // how long the trigger pulse is, in *micro*seconds

// *** END EDITABLE VARIABLES ***

// convert intervals to microseconds
unsigned long TRIG_INTERVAL_US = TRIG_INTERVAL_MS * 1000;
unsigned long SAMPLE_INTERVAL_US = SAMPLE_INTERVAL_MS * 1000;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *obj_stepper = NULL;
FastAccelStepper *mir_stepper = NULL;

// shared states
volatile bool systemActive = false;
volatile int target_obj_rpm = 60;
volatile int target_mir_rpm = 20;

// // --- ISR variables ---
// volatile unsigned long obj_edge_count = 0;
// volatile unsigned long obj_last_edge_time = 0;
// volatile unsigned long mir_edge_count = 0;
// volatile unsigned long mir_last_edge_time = 0;

// --- logging variables ---
unsigned long last_sample_time = 0;
unsigned long last_trig_time = 0;
bool pending_trig = false;


// --- interrupt service routines (ISR) ---
// void objEncoderISR() {
//   unsigned long now = micros();
//   if (now - obj_last_edge_time > 450) {
//     obj_edge_count++;
//     obj_last_edge_time = now;
//   }
// }

// void mirEncoderISR() {
//   unsigned long now = micros();
//   if (now - mir_last_edge_time > 500) {
//     mir_edge_count++;
//     mir_last_edge_time = now;
//   }
// }

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

// speed setter with smooth acceleration curve
void setRPM(FastAccelStepper *stepper, int rpm, int steps_per_rev) {
  if (!stepper) return;

  if (rpm <= 0) {
    stepper->stopMove();
  } else {
    uint32_t steps_per_sec = ((uint32_t)rpm * (uint32_t)steps_per_rev) / 60;
    stepper->setSpeedInHz(steps_per_sec);
    stepper->runForward();
  }
}

// start/stop control with keyboard 
void checkSerialCommands() {
  while (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.length() == 0) continue;

    if (cmd == "START") {
      sendSyncPulse();      // fire sync pattern for camera
      systemActive = true;  // start trigger and logging loop
      setRPM(obj_stepper, target_obj_rpm, OBJ_STEPS_PER_REV);
      setRPM(mir_stepper, target_mir_rpm, MIR_STEPS_PER_REV);
      Serial.println("SYSTEM_STARTED");

    } else if (cmd == "STOP") {
      systemActive = false; // pause motor and logging
      setRPM(obj_stepper, 0, OBJ_STEPS_PER_REV);
      setRPM(mir_stepper, 0, MIR_STEPS_PER_REV);
      Serial.println("SYSTEM_STOPPED");
    } else if (cmd.startsWith("OBJ:")) {
      int newRPM = cmd.substring(4).toInt();
      if (newRPM >= 0) {
        target_obj_rpm = newRPM;
        if (systemActive) {
          setRPM(obj_stepper, target_obj_rpm, OBJ_STEPS_PER_REV);
        }
        Serial.print("SUCCESS:OBJ_RPM_UPDATED:");
        Serial.println(target_obj_rpm);
      }
    } else if (cmd.startsWith("MIR:")) {
      int newRPM = cmd.substring(4).toInt();
      if (newRPM >= 0) {
        target_mir_rpm = newRPM;
        if (systemActive) {
          setRPM(mir_stepper, target_mir_rpm, MIR_STEPS_PER_REV);
        }
        Serial.print("SUCCESS:MIR_RPM_UPDATED:");
        Serial.println(target_mir_rpm);
      }
    }
  }
}

// --- runtime code ---
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(10);

  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, HIGH);

  // pinMode(OBJ_ENC_PIN, INPUT);
  // attachInterrupt(digitalPinToInterrupt(OBJ_ENC_PIN), objEncoderISR, RISING);

  // pinMode(MIR_ENC_PIN, INPUT);
  // attachInterrupt(digitalPinToInterrupt(MIR_ENC_PIN), mirEncoderISR, RISING);

  // initialize engine and attach steppers
  engine.init();
  obj_stepper = engine.stepperConnectToPin(OBJ_PUL_PIN);
  mir_stepper = engine.stepperConnectToPin(MIR_PUL_PIN);

  // configure acceleration curves
  if (obj_stepper) {
    obj_stepper->setDirectionPin(OBJ_DIR_PIN, false);
    obj_stepper->setAcceleration(1000);
    obj_stepper->setLinearAcceleration(800);
  }
  if (mir_stepper) {
    mir_stepper->setDirectionPin(MIR_DIR_PIN);
    mir_stepper->setAcceleration(2500);
    mir_stepper->setLinearAcceleration(400);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  // --- time check ---
  unsigned long now_us = micros();

  // --- check for commands ---
  checkSerialCommands();

  // camera trigger & data logging only sent when system is running
  if (systemActive) {
    // --- trigger signal ---
    if (now_us - last_trig_time >= TRIG_INTERVAL_US) {
      last_trig_time = now_us;
      pending_trig = true;
      sendTrigPulse();
    }
  
    // --- motor logging ---
    if (now_us - last_sample_time >= SAMPLE_INTERVAL_US) {
      last_sample_time = now_us;

      // safely copy encoder counts from ISR
      // noInterrupts();
      // long current_obj_edges = obj_edge_count;
      // long current_mir_edges = mir_edge_count;
      // interrupts();

      // calculate angles of motors
      // float obj_deg = fmod((current_obj_edges % OBJ_ENC_PPR) * (360.0 / OBJ_ENC_PPR), 360.0);
      // float mir_deg  = fmod((current_mir_edges % MIR_ENC_PPR) * (360.0 / MIR_ENC_PPR), 360.0);

      // csv ouput to serial port
      // format: "<timestamp>,<object angle>,<mirror angle>,<trigger sent T/F>"
      Serial.print(now_us);
      Serial.print(",");
      // Serial.print(obj_deg, 2);
      // Serial.print(",");
      // Serial.print(mir_deg, 2);
      // Serial.print(",");
      Serial.println(pending_trig? 1 : 0);
      pending_trig = false;
    }
  }
}