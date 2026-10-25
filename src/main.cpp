// Step 1: Basic stepper test
// ESP32 + CNC Shield V3 + DRV8825 + NEMA 17 (full step, no jumpers)

#include <Arduino.h>

#define STEP_PIN 26   // X.STEP
#define DIR_PIN  27   // X.DIR
#define EN_PIN   25   // EN (LOW = driver ON)

const int STEPS_PER_REV = 600;   // full step
const int STEP_DELAY_US = 1500;   // smaller = faster

void moveSteps(int steps, bool forward) {
  digitalWrite(DIR_PIN, forward ? HIGH : LOW);
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);   // enable driver
  Serial.println("Stepper ready");
}

void loop() {
  Serial.println("Forward 1 rev");
  moveSteps(STEPS_PER_REV, true);
  delay(1000);

  Serial.println("Backward 1 rev");
  moveSteps(STEPS_PER_REV, false);
  delay(1000);
}
