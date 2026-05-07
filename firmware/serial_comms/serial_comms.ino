#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver driver = Adafruit_PWMServoDriver();

//order: base, shoulder, elbow, wrist, claw, gripper
const String SERVO_NAMES[6] = {"base", "shoulder", "elbow", "wrist", "claw", "gripper"};
const int SERVO_COUNT  = 6;
const int SERVO_CH[6]  = {0, 1, 2, 3, 4, 5};

const int TICK_MIN[6]  = {120, 150, 120, 140, 120, 230};
const int TICK_REST[6] = {325, 470, 400, 519, 200, 300};
const int TICK_MAX[6]  = {530, 530, 409, 520, 480, 370};

float currentTick[6];
float targetTick[6];

void setup() {
  Serial.begin(115200);
  Wire.begin();
  driver.begin();
  driver.setOscillatorFrequency(27000000);
  driver.setPWMFreq(50);
  delay(10);
 
  for (int i = 0; i < SERVO_COUNT; i++) {
    currentTick[i] = TICK_REST[i];
    targetTick[i]  = TICK_REST[i];
    driver.setPWM(i, 0, TICK_REST[i]);
  }
  delay(1000);
  Serial.println("At rest");
}

String inputBuffer = "";

void interpolateAll() {
    bool moving = true;
    while (moving) {
        moving = false;
        for (int i = 0; i < SERVO_COUNT; i++) {
            float diff = targetTick[i] - currentTick[i];
            float threshold = (i == 3) ? 5.0 : 1.0; //increased wrist deadzone due to servo inconsistency
            if (abs(diff) < threshold) continue;
            moving = true;
            float step = constrain(diff, -2.0, 2.0);
            currentTick[i] += step;
            currentTick[i] = constrain(currentTick[i], TICK_MIN[i], TICK_MAX[i]);
            driver.setPWM(SERVO_CH[i], 0, (int)round(currentTick[i]));
        }
        delay(10);
    }
}

void readAndRun(String line) {
  line.trim();
  if (line.length() < 1) { Serial.println("ERR"); return; }

  char cmd = line.charAt(0);

  if (cmd == 'S') {
    int comma = line.indexOf(',');
    if (comma < 0) { Serial.println("ERR"); return; }

    int servoID = line.substring(1, comma).toInt();
    int tick    = line.substring(comma + 1).toInt();

    if (servoID < 0 || servoID > 5) { Serial.println("ERR"); return; }
    tick = constrain(tick, TICK_MIN[servoID], TICK_MAX[servoID]);
    targetTick[servoID] = tick;
    interpolateAll();
    Serial.println("OK");
  } 
  else if (cmd == 'M') {
    String payload = line.substring(1);
    bool valid = true;
    int pos = 0;

    while (pos < payload.length()) {
      int c1 = payload.indexOf(',', pos);
      if (c1 < 0) { valid = false; break; }
      int c2 = payload.indexOf(',', c1 + 1);

      int servoID = payload.substring(pos, c1).toInt();
      int tick = (c2 < 0)
                  ? payload.substring(c1 + 1).toInt()
                  : payload.substring(c1 + 1, c2).toInt();

      if (servoID < 0 || servoID > 5) { valid = false; break; }
      tick = constrain(tick, TICK_MIN[servoID], TICK_MAX[servoID]);
      targetTick[servoID] = tick;
      pos = (c2 < 0) ? payload.length() : c2 + 1;
    }

    if (!valid) { Serial.println("ERR"); return; }
    interpolateAll();
    Serial.println("OK");
  } 
  else if (cmd == 'R') {
    for (int i = 0; i < SERVO_COUNT; i++) {
      targetTick[i] = TICK_REST[i];
    }
    interpolateAll();
    Serial.println("OK");
  } 
  else if (cmd == 'Q') {
    for (int i = 0; i < SERVO_COUNT; i++) {
        Serial.print(SERVO_NAMES[i]);
        Serial.print(":");
        Serial.print((int)round(currentTick[i]));
        if (i < SERVO_COUNT - 1) Serial.print(",");
    }
    Serial.println();
  } 
  else {
      Serial.println("ERR");
  }
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      if (inputBuffer.length() > 0) {
        readAndRun(inputBuffer);
        inputBuffer = "";
      }
    } else if (c != '\r') {
      inputBuffer += c;
    }
  } 
}