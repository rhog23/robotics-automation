#include <Arduino.h>
#include <ESP32Servo.h>

// Ultrasonic Sensor
#define TRIG        5
#define ECHO        18

// Servo
#define SERVO_PIN   19  // moved from GPIO12

// IR Sensors
#define IR_FRONT_LEFT   35
#define IR_FRONT_RIGHT  21
#define IR_BACK_LEFT    13
#define IR_BACK_RIGHT   15  // updated from GPIO33

// Motor Control Pins
#define MOTOR_LEFT_FWD   25
#define MOTOR_LEFT_BWD   26
#define MOTOR_RIGHT_FWD  27
#define MOTOR_RIGHT_BWD  14

// PWM Enable Pins
#define ENABLE_LEFT   22  // ENA
#define ENABLE_RIGHT  23  // ENB

// PWM config
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
const int dutyCycle = 150;

Servo myServo;

void setup() {
  Serial.begin(115200);

  // Ultrasonic
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  // IR sensors
  pinMode(IR_FRONT_LEFT, INPUT);
  pinMode(IR_FRONT_RIGHT, INPUT);
  pinMode(IR_BACK_LEFT, INPUT);
  pinMode(IR_BACK_RIGHT, INPUT);

  // Motor control pins
  pinMode(MOTOR_LEFT_FWD, OUTPUT);
  pinMode(MOTOR_LEFT_BWD, OUTPUT);
  pinMode(MOTOR_RIGHT_FWD, OUTPUT);
  pinMode(MOTOR_RIGHT_BWD, OUTPUT);

  // PWM channels for enable pins
  ledcAttachChannel(ENABLE_LEFT, freq, resolution, pwmChannel);  // Channel 0 for left motor
  ledcAttachChannel(ENABLE_RIGHT, freq, resolution, pwmChannel);  // Channel 1 for right motor
  ledcWrite(ENABLE_LEFT, dutyCycle);
  ledcWrite(ENABLE_RIGHT, dutyCycle);

  // Servo
  myServo.attach(SERVO_PIN);
}

long getDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long duration = pulseIn(ECHO, HIGH);
  return duration * 0.034 / 2;
}

void stopMotors() {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
}

void moveForward() {
  digitalWrite(MOTOR_LEFT_FWD, HIGH);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
}

void turnLeft() {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, HIGH);
  digitalWrite(MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
}

void turnRight() {
  digitalWrite(MOTOR_LEFT_FWD, HIGH);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, HIGH);
}

void loop() {
  long frontDist = getDistance();
  int fl = digitalRead(IR_FRONT_LEFT);
  int fr = digitalRead(IR_FRONT_RIGHT);
  int bl = digitalRead(IR_BACK_LEFT);
  int br = digitalRead(IR_BACK_RIGHT);

  Serial.print("FL: "); Serial.print(fl);
  Serial.print(" FR: "); Serial.print(fr);
  Serial.print(" BL: "); Serial.print(bl);
  Serial.print(" BR: "); Serial.println(br);
  Serial.print("Distance: "); Serial.println(frontDist);

  bool allIRSafe = (fl == LOW && fr == LOW && bl == LOW && br == LOW);
  bool frontSafe = (fl == LOW && fr == LOW);

  if (!frontSafe || bl == HIGH || br == HIGH) {
    stopMotors();
    Serial.println("❌ Edge detected! Stop.");
    delay(500);
    return;
  }

  if (allIRSafe && frontDist > 20) {
    moveForward();
    Serial.println("✅ Moving forward...");
  } else if (allIRSafe && frontDist <= 20) {
    stopMotors();
    Serial.println("⛔ Obstacle ahead! Scanning...");
    delay(500);

    long leftDist, rightDist;

    myServo.write(0);  // look left
    delay(600);
    leftDist = getDistance();

    myServo.write(180);  // look right
    delay(600);
    rightDist = getDistance();

    myServo.write(90);  // center

    if (leftDist > rightDist) {
      Serial.println("↩ Turning left");
      turnLeft();
    } else {
      Serial.println("↪ Turning right");
      turnRight();
    }

    delay(800);  // turn duration
    stopMotors();
    delay(500);
  } else {
    stopMotors();
  }

  delay(100);
}
