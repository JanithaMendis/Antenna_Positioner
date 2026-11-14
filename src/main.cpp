// Milestone 3: two axes (X + Y) with limit switches and homing
// ESP32 + CNC Shield V3 + 2x DRV8825 + 2x NEMA 17 (full step, no jumpers)
// Serial commands (axis = x or y):
//   x f <cm>, x b <cm>, x home, x zero, y f <cm>, ...
//   stop, pos, sw, env, reset
// Limit switches: NO, wired between the pin and GND (LOW = pressed)

#include <Arduino.h>
#include <FastAccelStepper.h>
#include <DHT.h>

#define X_STEP_PIN 26   // X.STEP
#define X_DIR_PIN  27   // X.DIR
#define Y_STEP_PIN 16   // Y.STEP
#define Y_DIR_PIN  17   // Y.DIR
#define EN_PIN     25   // EN, shared by both drivers (LOW = driver ON)
#define KILL_PIN   4    // kill button to GND
#define DHT_PIN    13   // DHT22 module data pin

#define X_MIN_PIN  32   // limit switches (NO, to GND)
#define X_MAX_PIN  33
#define Y_MIN_PIN  18
#define Y_MAX_PIN  19

const float STEPS_PER_CM = 200.0;  // 200 steps = 1 cm (both axes)
const int   SPEED        = 625;      // steps/s (~3.1 cm/s)
const int   ACCEL        = 1000000;  // steps/s^2 (effectively no ramp)
const float MAX_CM       = 20.0;     // position set when the MAX switch is hit

FastAccelStepperEngine engine;
FastAccelStepper *stepper[2];       // 0 = X, 1 = Y
const char AXIS_NAME[2] = {'X', 'Y'};
const int  MIN_PIN[2]   = {X_MIN_PIN, Y_MIN_PIN};
const int  MAX_PIN[2]   = {X_MAX_PIN, Y_MAX_PIN};
int  moveDir[2] = {0, 0};           // -1 towards MIN, +1 towards MAX, 0 idle
bool homing[2]  = {false, false};

DHT dht(DHT_PIN, DHT22);
float temperature = NAN;
float humidity    = NAN;
unsigned long lastDhtRead = 0;

volatile bool killed = false;
bool killReported = false;
String line = "";                   // serial input being typed

// Kill button interrupt: switch both drivers off immediately
void IRAM_ATTR onKill() {
  killed = true;
  digitalWrite(EN_PIN, HIGH);
}

bool pressed(int pin) { return digitalRead(pin) == LOW; }

void stopAxis(int a) {
  stepper[a]->forceStopAndNewPosition(stepper[a]->getCurrentPosition());
  moveDir[a] = 0;
  homing[a]  = false;
}

// Stop an axis that runs into a limit switch and set its position:
// MIN switch = 0 cm, MAX switch = 20 cm
void checkLimits(int a) {
  if (moveDir[a] < 0 && pressed(MIN_PIN[a])) {
    bool wasHoming = homing[a];
    stepper[a]->forceStopAndNewPosition(0);
    moveDir[a] = 0;
    homing[a]  = false;
    if (wasHoming) Serial.printf("%c homed - position 0 cm\n", AXIS_NAME[a]);
    else           Serial.printf("%c MIN limit hit - position 0 cm\n", AXIS_NAME[a]);
  } else if (moveDir[a] > 0 && pressed(MAX_PIN[a])) {
    stepper[a]->forceStopAndNewPosition(lround(MAX_CM * STEPS_PER_CM));
    moveDir[a] = 0;
    Serial.printf("%c MAX limit hit - position %.0f cm\n", AXIS_NAME[a], MAX_CM);
  }
}

void handleCommand(String cmd) {
  // Axis commands: "x f 5", "y b 2", "x home", "y zero"
  if (cmd.startsWith("x ") || cmd.startsWith("y ")) {
    int a = (cmd[0] == 'x') ? 0 : 1;
    String sub = cmd.substring(2);
    sub.trim();

    if (sub == "zero") {
      stepper[a]->setCurrentPosition(0);
      Serial.printf("%c position set to 0 cm\n", AXIS_NAME[a]);
      return;
    }
    if (killed) {
      Serial.println("Killed - type 'reset' first.");
      return;
    }
    if (sub.startsWith("f ") || sub.startsWith("b ")) {
      float cm = sub.substring(2).toFloat();
      int dir = (sub[0] == 'f') ? 1 : -1;
      dir = -dir;   // backward = away from MIN switch, position counts up 0 -> 20 cm
      if ((dir > 0 && pressed(MAX_PIN[a])) || (dir < 0 && pressed(MIN_PIN[a]))) {
        Serial.printf("%c is on the limit switch - can't move that way\n", AXIS_NAME[a]);
        return;
      }
      moveDir[a] = dir;
      stepper[a]->move(dir * lround(cm * STEPS_PER_CM));
      Serial.printf("%c moving %.2f cm\n", AXIS_NAME[a], dir * cm);
    } else if (sub == "home") {
      if (pressed(MIN_PIN[a])) {          // already on the switch
        stepper[a]->setCurrentPosition(0);
        Serial.printf("%c homed - position 0 cm\n", AXIS_NAME[a]);
        return;
      }
      homing[a]  = true;
      moveDir[a] = -1;
      stepper[a]->runBackward();          // run until the MIN switch is pressed
      Serial.printf("%c homing...\n", AXIS_NAME[a]);
    } else {
      Serial.println("Axis commands: f <cm>, b <cm>, home, zero");
    }
  } else if (cmd == "stop") {
    stopAxis(0);
    stopAxis(1);
    Serial.println("Stopped");
  } else if (cmd == "pos") {
    Serial.printf("X: %.2f cm   Y: %.2f cm\n",
                  stepper[0]->getCurrentPosition() / STEPS_PER_CM,
                  stepper[1]->getCurrentPosition() / STEPS_PER_CM);
  } else if (cmd == "sw") {               // show limit switch states (1 = pressed)
    Serial.printf("X min=%d max=%d   Y min=%d max=%d\n",
                  pressed(X_MIN_PIN), pressed(X_MAX_PIN),
                  pressed(Y_MIN_PIN), pressed(Y_MAX_PIN));
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
    Serial.println("Reset - drivers enabled");
  } else {
    Serial.println("Commands: x/y f <cm>, x/y b <cm>, x/y home, x/y zero, stop, pos, sw, env, reset");
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);           // enable drivers
  pinMode(KILL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(KILL_PIN), onKill, FALLING);

  pinMode(X_MIN_PIN, INPUT_PULLUP);
  pinMode(X_MAX_PIN, INPUT_PULLUP);
  pinMode(Y_MIN_PIN, INPUT_PULLUP);
  pinMode(Y_MAX_PIN, INPUT_PULLUP);

  engine.init();
  stepper[0] = engine.stepperConnectToPin(X_STEP_PIN);
  stepper[0]->setDirectionPin(X_DIR_PIN, true);    // X: + steps = backward (away from MIN)
  stepper[1] = engine.stepperConnectToPin(Y_STEP_PIN);
  stepper[1]->setDirectionPin(Y_DIR_PIN, true);    // Y: + steps = backward (away from MIN)
  for (int a = 0; a < 2; a++) {
    stepper[a]->setSpeedInHz(SPEED);
    stepper[a]->setAcceleration(ACCEL);
  }

  dht.begin();

  Serial.println("Ready. Commands: x/y f <cm>, x/y b <cm>, x/y home, x/y zero, stop, pos, sw, env, reset");
}

void loop() {
  // Read the DHT22 every 2 s (it cannot be read faster)
  if (millis() - lastDhtRead >= 2000) {
    lastDhtRead = millis();
    temperature = dht.readTemperature();
    humidity    = dht.readHumidity();
  }

  // Handle a kill: stop both motors
  if (killed && !killReported) {
    stopAxis(0);
    stopAxis(1);
    killReported = true;
    Serial.println("KILL pressed - motors stopped. Type 'reset' to continue.");
  }

  checkLimits(0);
  checkLimits(1);

  // Read serial without blocking, so the limit switches are always checked
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      line.trim();
      if (line.length() > 0) handleCommand(line);
      line = "";
    } else {
      line += c;
    }
  }
}
