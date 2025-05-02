#include <Arduino.h>
#include <ESP32Servo.h>

// ==========================
// KONFIGURASI PIN TERATUR
// ==========================

// Motor Driver
#define MOTOR_LEFT_FWD    25
#define MOTOR_LEFT_BWD    26
#define MOTOR_RIGHT_FWD   27
#define MOTOR_RIGHT_BWD   14

// PWM Motor Enable
#define ENABLE_LEFT_PIN   32
#define ENABLE_RIGHT_PIN  33

// Sensor IR (Anti-Jatuh)
#define IR_FRONT_LEFT     35  // input-only, aman
#define IR_FRONT_RIGHT    13
#define IR_BACK_LEFT      18
#define IR_BACK_RIGHT     4   // pin baru: GPIO 4

// Ultrasonik
#define TRIG              16
#define ECHO              17

// Servo
#define SERVO_PIN         19
Servo myServo;

// ==========================
// KONSTANTA SISTEM
// ==========================
#define MAX_PWM_DUTY      255
#define SPEED_FORWARD     160
#define SPEED_REVERSE     140
#define SAFE_DISTANCE_CM  20
#define SENSOR_TIMEOUT    999
#define PWM_FREQ          30000
#define PWM_RESOLUTION    8
#define TURN_DURATION     450
#define REVERSE_DURATION  450

// ==========================
// SETUP
// ==========================
void setup() {
  Serial.begin(115200);

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  pinMode(IR_FRONT_LEFT, INPUT);
  pinMode(IR_FRONT_RIGHT, INPUT);
  pinMode(IR_BACK_LEFT, INPUT);
  pinMode(IR_BACK_RIGHT, INPUT);  // GPIO 4

  pinMode(MOTOR_LEFT_FWD, OUTPUT);
  pinMode(MOTOR_LEFT_BWD, OUTPUT);
  pinMode(MOTOR_RIGHT_FWD, OUTPUT);
  pinMode(MOTOR_RIGHT_BWD, OUTPUT);

  ledcAttach(ENABLE_LEFT_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENABLE_RIGHT_PIN, PWM_FREQ, PWM_RESOLUTION);
  setMotorSpeed(SPEED_FORWARD, SPEED_FORWARD);

  myServo.attach(SERVO_PIN);
  myServo.write(90);  // posisi tengah

  randomSeed(analogRead(34));
}

// ==========================
// FUNGSI MOTOR
// ==========================
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  ledcWrite(ENABLE_LEFT_PIN, constrain(leftSpeed, 0, MAX_PWM_DUTY));
  ledcWrite(ENABLE_RIGHT_PIN, constrain(rightSpeed, 0, MAX_PWM_DUTY));
}

void stopMotors() {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
}

void moveForward() {
  setMotorSpeed(SPEED_FORWARD, SPEED_FORWARD);
  digitalWrite(MOTOR_LEFT_FWD, HIGH);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
}

void moveBackward() {
  setMotorSpeed(SPEED_REVERSE, SPEED_REVERSE);
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, HIGH);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, HIGH);
}

void turnLeft() {
  setMotorSpeed(SPEED_FORWARD, SPEED_FORWARD);
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, HIGH);
  digitalWrite(MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
}

void turnRight() {
  setMotorSpeed(SPEED_FORWARD, SPEED_FORWARD);
  digitalWrite(MOTOR_LEFT_FWD, HIGH);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, HIGH);
}

// ==========================
// SENSOR JARAK
// ==========================
long getDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long duration = pulseIn(ECHO, HIGH, 30000);
  if (duration == 0 || duration > 25000) return SENSOR_TIMEOUT;
  return duration * 0.034 / 2;
}

long getStableDistance() {
  return (getDistance() + getDistance()) / 2;
}

// ==========================
// ANTI-JATUH
// ==========================
void antiFallAvoidance() {
  int fl = digitalRead(IR_FRONT_LEFT);
  int fr = digitalRead(IR_FRONT_RIGHT);
  int bl = digitalRead(IR_BACK_LEFT);
  int br = digitalRead(IR_BACK_RIGHT);

  setMotorSpeed(0, 0);
  stopMotors();

  if (fl == HIGH && fr == HIGH) {
    Serial.println("⚠ Tepi depan ganda! Mundur & belok kanan");
    moveBackward(); delay(500);
    stopMotors(); turnRight(); delay(TURN_DURATION);
  }
  else if (bl == HIGH && br == HIGH) {
    Serial.println("⚠ Tepi belakang ganda! Maju & belok kiri");
    moveForward(); delay(500);
    stopMotors(); turnLeft(); delay(TURN_DURATION);
  }
  else if (fl == HIGH) {
    Serial.println("⚠ Tepi depan kiri! Mundur & belok kanan");
    moveBackward(); delay(400);
    stopMotors(); turnRight(); delay(400);
  }
  else if (fr == HIGH) {
    Serial.println("⚠ Tepi depan kanan! Mundur & belok kiri");
    moveBackward(); delay(400);
    stopMotors(); turnLeft(); delay(400);
  }
  else if (bl == HIGH) {
    Serial.println("⚠ Tepi belakang kiri! Maju & belok kanan");
    moveForward(); delay(400);
    stopMotors(); turnRight(); delay(400);
  }
  else if (br == HIGH) {
    Serial.println("⚠ Tepi belakang kanan! Maju & belok kiri");
    moveForward(); delay(400);
    stopMotors(); turnLeft(); delay(400);
  }

  stopMotors();
  delay(300);
}

// ==========================
// OBSTACLE AVOIDANCE
// ==========================
void obstacleAvoidance() {
  stopMotors();
  Serial.println("⛔ Obstacle! Mundur...");
  delay(300);

  moveBackward(); delay(REVERSE_DURATION);
  stopMotors(); delay(300);

  Serial.println("🔍 Scan kiri...");
  myServo.write(0); delay(300);
  long leftDist = getStableDistance();

  Serial.println("🔍 Scan kanan...");
  myServo.write(180); delay(300);
  long rightDist = getStableDistance();

  myServo.write(90); delay(200);

  Serial.print("↙ Kiri: "); Serial.print(leftDist);
  Serial.print(" cm | ↘ Kanan: "); Serial.println(rightDist);

  if (leftDist == rightDist) {
    Serial.println("🔄 Sama → belok acak");
    if (random(0, 2) == 0) turnLeft();
    else turnRight();
  } else if (leftDist > rightDist) {
    Serial.println("↩ Belok kiri");
    turnLeft();
  } else {
    Serial.println("↪ Belok kanan");
    turnRight();
  }

  delay(TURN_DURATION);
  stopMotors(); delay(300);
}

// ==========================
// LOOP UTAMA
// ==========================
void loop() {
  int fl = digitalRead(IR_FRONT_LEFT);
  int fr = digitalRead(IR_FRONT_RIGHT);
  int bl = digitalRead(IR_BACK_LEFT);
  int br = digitalRead(IR_BACK_RIGHT);

  Serial.print("IR FL: "); Serial.print(fl);
  Serial.print(" FR: "); Serial.print(fr);
  Serial.print(" BL: "); Serial.print(bl);
  Serial.print(" BR: "); Serial.println(br);

  if (fl == HIGH || fr == HIGH || bl == HIGH || br == HIGH) {
    setMotorSpeed(0, 0);
    stopMotors();
    Serial.println("‼ IR Aktif – STOP!");
    delay(100);
    antiFallAvoidance();
    return;
  }

  long frontDist = getStableDistance();
  Serial.print("Ultrasonik: "); Serial.print(frontDist); Serial.println(" cm");

  if (frontDist >= SENSOR_TIMEOUT) {
    stopMotors();
    Serial.println("⚠ Sensor timeout");
    delay(300);
    return;
  }

  if (frontDist > SAFE_DISTANCE_CM) {
    moveForward();
    Serial.println("✅ Jalan aman");
  } else {
    obstacleAvoidance();
  }

  delay(100);
}