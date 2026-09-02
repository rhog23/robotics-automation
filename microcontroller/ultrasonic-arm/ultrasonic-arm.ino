#include <ESP32Servo.h>

// --- Servo objects ---
Servo servoBase;
Servo servoShoulder;
Servo servoElbow;
Servo servoClaw;

// --- Pin assignments ---
const int IR_PIN          = 4;   // Infrared sensor OUT pin
const int SERVO_BASE      = 19;
const int SERVO_SHOULDER  = 21;
const int SERVO_ELBOW     = 22;
const int SERVO_CLAW      = 23;

// --- Tunable parameters ---
const int BASE_IDLE       = 90;
const int BASE_GRAB       = 90;

const int SHOULDER_IDLE   = 0;
const int SHOULDER_DOWN   = 60;

const int ELBOW_IDLE      = 0;
const int ELBOW_DOWN      = 80;

const int CLAW_OPEN       = 30;
const int CLAW_CLOSED     = 10;

const int STEP_DELAY      = 20;
const int SETTLE_DELAY    = 400;

// --- State ---
bool isHolding = false;

// --- Helper: smooth servo move ---
void moveServo(Servo &srv, int fromAngle, int toAngle, int stepDelay) {
  int step = (toAngle > fromAngle) ? 1 : -1;
  for (int pos = fromAngle; pos != toAngle; pos += step) {
    srv.write(pos);
    delay(stepDelay);
  }
  srv.write(toAngle);
}

// --- Grab sequence ---
void grabBox() {
  moveServo(servoClaw, CLAW_CLOSED, CLAW_OPEN, STEP_DELAY);
  delay(SETTLE_DELAY);

  moveServo(servoBase, BASE_IDLE, BASE_GRAB, STEP_DELAY);
  delay(SETTLE_DELAY);

  moveServo(servoShoulder, SHOULDER_IDLE, SHOULDER_DOWN, STEP_DELAY);
  delay(SETTLE_DELAY);

  moveServo(servoElbow, ELBOW_IDLE, ELBOW_DOWN, STEP_DELAY);
  delay(SETTLE_DELAY);

  moveServo(servoClaw, CLAW_OPEN, CLAW_CLOSED, STEP_DELAY);
  delay(SETTLE_DELAY);

  moveServo(servoElbow, ELBOW_DOWN, ELBOW_IDLE, STEP_DELAY);
  delay(SETTLE_DELAY);

  moveServo(servoShoulder, SHOULDER_DOWN, SHOULDER_IDLE, STEP_DELAY);
  delay(SETTLE_DELAY);

  isHolding = true;
}

// --- Release sequence ---
void releaseBox() {
  moveServo(servoShoulder, SHOULDER_IDLE, SHOULDER_DOWN, STEP_DELAY);
  delay(SETTLE_DELAY);

  moveServo(servoElbow, ELBOW_IDLE, ELBOW_DOWN, STEP_DELAY);
  delay(SETTLE_DELAY);

  moveServo(servoClaw, CLAW_CLOSED, CLAW_OPEN, STEP_DELAY);
  delay(SETTLE_DELAY);

  moveServo(servoElbow, ELBOW_DOWN, ELBOW_IDLE, STEP_DELAY);
  delay(SETTLE_DELAY);

  moveServo(servoShoulder, SHOULDER_DOWN, SHOULDER_IDLE, STEP_DELAY);
  delay(SETTLE_DELAY);

  isHolding = false;
}

void setup() {
  Serial.begin(9600);

  servoBase.attach(SERVO_BASE);
  servoShoulder.attach(SERVO_SHOULDER);
  servoElbow.attach(SERVO_ELBOW);
  servoClaw.attach(SERVO_CLAW);

  pinMode(IR_PIN, INPUT);

  // Initial position
  servoBase.write(BASE_IDLE);
  servoShoulder.write(SHOULDER_IDLE);
  servoElbow.write(ELBOW_IDLE);
  servoClaw.write(CLAW_OPEN);

  delay(1000);
}

void loop() {
  int irState = digitalRead(IR_PIN);

  Serial.print("IR State: ");
  Serial.println(irState);

  // NOTE: Most IR sensors → LOW = object detected
  if (irState == LOW && !isHolding) {
    Serial.println("Object detected — grabbing...");
    grabBox();
  } 
  else if (irState == HIGH && isHolding) {
    Serial.println("Object gone — releasing...");
    releaseBox();
  }

  delay(300);
}