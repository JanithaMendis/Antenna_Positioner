// Milestones 1-2 + 5: kill-button interrupt, position control, DHT22
// ESP32 + CNC Shield V3 + DRV8825 + NEMA 17 (full step, no jumpers)
// Serial commands: f <cm>, b <cm>, stop, pos, zero, home, env, reset

#include <Arduino.h>
#include <FastAccelStepper.h>
#include <DHT.h>

#define STEP_PIN 26   // X.STEP
#define DIR_PIN  27   // X.DIR
#define EN_PIN   25   // EN (LOW = driver ON)
#define KILL_PIN 4    // kill button to GND
#define DHT_PIN  13   // DHT22 module data pin

const float STEPS_PER_CM = 200.0;  // 200 steps = 1 cm
const int   SPEED        = 625;      // steps/s (~3.1 cm/s, same as the first test code)
const int   ACCEL        = 1000000;  // steps/s^2 (effectively no ramp - smoothest in full step)

FastAccelStepperEngine engine;
FastAccelStepper *stepper;

DHT dht(DHT_PIN, DHT22);
float temperature = NAN;         // latest reading, °C
float humidity    = NAN;         // latest reading, %RH
unsigned long lastDhtRead = 0;

volatile bool killed = false;
bool killReported = false;

// Kill button interrupt: switch the driver off immediately
void IRAM_ATTR onKill() {
  killed = true;
  digitalWrite(EN_PIN, HIGH);
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(60000);            // wait for Enter while typing a command
  delay(2000);

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);           // enable driver
  pinMode(KILL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(KILL_PIN), onKill, FALLING);

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  stepper->setDirectionPin(DIR_PIN, false);   // false = reversed direction
  stepper->setSpeedInHz(SPEED);
  stepper->setAcceleration(ACCEL);

  dht.begin();

  Serial.println("Ready. Commands: f <cm>, b <cm>, stop, pos, zero, home, env, reset");
}

void loop() {
  // Read the DHT22 every 2 s (it cannot be read faster)
  if (millis() - lastDhtRead >= 2000) {
    lastDhtRead = millis();
    temperature = dht.readTemperature();
    humidity    = dht.readHumidity();
  }

  // Handle a kill: stop the step pulses
  if (killed && !killReported) {
    stepper->forceStop();
    killReported = true;
    Serial.println("KILL pressed - motor stopped. Type 'reset' to continue.");
  }

  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.startsWith("f ") || cmd.startsWith("b ")) {
    if (killed) {
      Serial.println("Killed - type 'reset' first.");
      return;
    }
    float cm = cmd.substring(2).toFloat();
    long steps = lround(cm * STEPS_PER_CM);
    if (cmd[0] == 'b') steps = -steps;
    Serial.printf("Moving %.2f cm\n", cmd[0] == 'b' ? -cm : cm);
    stepper->move(steps);
  } else if (cmd == "stop") {
    stepper->stopMove();
    Serial.println("Stopping");
  } else if (cmd == "pos") {
    Serial.printf("Position: %.2f cm\n", stepper->getCurrentPosition() / STEPS_PER_CM);
  } else if (cmd == "zero") {
    stepper->setCurrentPosition(0);          // current position becomes 0 cm
    Serial.println("Position set to 0 cm");
  } else if (cmd == "home") {
    if (killed) {
      Serial.println("Killed - type 'reset' first.");
      return;
    }
    stepper->moveTo(0);                      // drive back to the 0 cm point
    Serial.println("Going to 0 cm");
  } else if (cmd == "env") {
    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("DHT22 read failed - check wiring");
    } else {
      Serial.printf("Temperature: %.1f C   Humidity: %.1f %%\n", temperature, humidity);
    }
  } else if (cmd == "reset") {
    killed = false;
    killReported = false;
    digitalWrite(EN_PIN, LOW);
    Serial.println("Reset - driver enabled");
  } else {
    Serial.println("Commands: f <cm>, b <cm>, stop, pos, zero, home, env, reset");
  }
}
