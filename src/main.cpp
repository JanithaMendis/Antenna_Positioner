// Step 2: Move a distance in cm entered over Serial
// ESP32 + CNC Shield V3 + DRV8825 + NEMA 17 (full step, no jumpers)
// Serial monitor asks for direction (F/B), then distance in cm

#include <Arduino.h>

#define STEP_PIN 26   // X.STEP
#define DIR_PIN  27   // X.DIR
#define EN_PIN   25   // EN (LOW = driver ON)

const float STEPS_PER_CM  = 200.0;  // measured: 200 steps = 1 cm
const int   STEP_DELAY_US = 800;    // smaller = faster

void moveSteps(long steps, bool forward) {
  digitalWrite(DIR_PIN, forward ? LOW : HIGH);  // swapped to match mechanical forward
  for (long i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
}

void moveCm(float cm) {
  long steps = lround(fabs(cm) * STEPS_PER_CM);
  moveSteps(steps, cm > 0);
}

// Reads one line from Serial (ends with Enter). Returns true when a line is ready.
bool readLine(String &line) {
  static String buffer;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (buffer.length() == 0) continue;  // skip empty lines / CRLF pairs
      line = buffer;
      buffer = "";
      return true;
    }
    buffer += c;
  }
  return false;
}

enum State { ASK_DIRECTION, ASK_DISTANCE };
State state = ASK_DIRECTION;
bool moveForward = true;

void askDirection() {
  Serial.println();
  Serial.print("Direction? (F = forward, B = backward): ");
}

void askDistance() {
  Serial.println();
  Serial.print("How many centimeters should it move? ");
}

void setup() {
  Serial.begin(115200);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);   // enable driver
  Serial.println("Stepper ready");
  askDirection();
}

void loop() {
  String line;
  if (!readLine(line)) return;
  line.trim();
  Serial.println();

  if (state == ASK_DIRECTION) {
    line.toUpperCase();
    if (line == "F" || line == "FORWARD") {
      moveForward = true;
    } else if (line == "B" || line == "BACKWARD") {
      moveForward = false;
    } else {
      Serial.println("Please type F or B.");
      askDirection();
      return;
    }
    state = ASK_DISTANCE;
    askDistance();
    return;
  }

  // ASK_DISTANCE
  float cm = line.toFloat();
  if (cm <= 0.0) {
    Serial.println("Please enter a positive number of centimeters.");
    askDistance();
    return;
  }

  Serial.printf("Moving %.2f cm (%ld steps) %s...\n",
                cm, lround(cm * STEPS_PER_CM),
                moveForward ? "forward" : "backward");
  moveCm(moveForward ? cm : -cm);
  Serial.println("Done.");

  state = ASK_DIRECTION;
  askDirection();
}
