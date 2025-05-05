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
#define IR_FRONT_LEFT     35
#define IR_FRONT_RIGHT    13
#define IR_BACK_LEFT      18
#define IR_BACK_RIGHT     4

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
#define SPEED_FORWARD     210
#define SPEED_REVERSE     210
#define SAFE_DISTANCE_CM  20
#define SENSOR_TIMEOUT    999
#define PWM_FREQ          5000
#define PWM_RESOLUTION    8
#define TURN_DURATION     450
#define REVERSE_DURATION  450

// Untuk menghindari busy waiting
unsigned long lastSensorCheck = 0;
const unsigned long SENSOR_CHECK_INTERVAL = 100; // ms

// Status sensor terakhir
bool isOnSurface = true;
bool wasOnSurface = true;

// ==========================
// SETUP
// ==========================
void setup() {
  Serial.begin(115200);
  Serial.println("Robot starting...");

  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  pinMode(IR_FRONT_LEFT, INPUT);
  pinMode(IR_FRONT_RIGHT, INPUT);
  pinMode(IR_BACK_LEFT, INPUT);
  pinMode(IR_BACK_RIGHT, INPUT);

  pinMode(MOTOR_LEFT_FWD, OUTPUT);
  pinMode(MOTOR_LEFT_BWD, OUTPUT);
  pinMode(MOTOR_RIGHT_FWD, OUTPUT);
  pinMode(MOTOR_RIGHT_BWD, OUTPUT);

  ledcAttach(ENABLE_LEFT_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(ENABLE_RIGHT_PIN, PWM_FREQ, PWM_RESOLUTION);
  
  // Pastikan motor speed dimulai dengan 0
  setMotorSpeed(0, 0);

  myServo.attach(SERVO_PIN);
  myServo.write(90);  // posisi tengah

  randomSeed(analogRead(34));
  
  // Pastikan robot dimulai dalam keadaan berhenti
  stopMotors();
  delay(1000); // Berikan waktu untuk semua sensor menginisialisasi
  
  Serial.println("Robot ready!");
  
  // Berikan delay tambahan sebelum robot mulai bergerak
  delay(2000);
  
  // Inisialisasi timer pertama kali
  lastSensorCheck = millis() - SENSOR_CHECK_INTERVAL;
  
  Serial.println("Robot starting movement!");
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
  setMotorSpeed(0, 0);
  Serial.println("Motors stopped");
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

// Gradual acceleration forward
void gradualForward(int duration) {
  digitalWrite(MOTOR_LEFT_FWD, HIGH);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
  for (int pwm = 100; pwm <= SPEED_FORWARD; pwm += 10) {
    setMotorSpeed(pwm, pwm);
    delay(20);
  }
  delay(duration);
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
  long dist1 = getDistance();
  long dist2 = getDistance();
  return (dist1 + dist2) / 2;
}

// ==========================
// ANTI-JATUH
// ==========================

// Fungsi untuk memeriksa apakah robot berada di atas permukaan
bool checkOnSurface() {
  int fl = digitalRead(IR_FRONT_LEFT);
  int fr = digitalRead(IR_FRONT_RIGHT);
  int bl = digitalRead(IR_BACK_LEFT);
  int br = digitalRead(IR_BACK_RIGHT);
  
  // PENTING: Untuk sensor IR ini:
  // LOW = sensor mendeteksi permukaan (normal, aman)
  // HIGH = sensor tidak mendeteksi permukaan (tepi atau diangkat)
  
  // Jika semua sensor HIGH, robot kemungkinan diangkat
  if (fl == HIGH && fr == HIGH && bl == HIGH && br == HIGH) {
    return false;
  }
  return true;
}

void antiFallAvoidance() {
  int fl = digitalRead(IR_FRONT_LEFT);
  int fr = digitalRead(IR_FRONT_RIGHT);
  int bl = digitalRead(IR_BACK_LEFT);
  int br = digitalRead(IR_BACK_RIGHT);

  stopMotors();

  // Jika semua sensor HIGH, robot diangkat
  if (fl == HIGH && fr == HIGH && bl == HIGH && br == HIGH) {
    Serial.println("⚠ EMERGENCY STOP! Robot diangkat!");
    stopMotors();
    return;
  }

  if (fl == HIGH && fr == HIGH) {
    Serial.println("⚠ Tepi depan ganda! Mundur & belok kanan");
    moveBackward(); delay(500);
    stopMotors(); delay(100);
    turnRight(); delay(TURN_DURATION);
    gradualForward(200);
    stopMotors(); delay(100);
  }
  else if (bl == HIGH && br == HIGH) {
    Serial.println("⚠ Tepi belakang ganda! Maju & belok kiri");
    gradualForward(500);
    stopMotors(); delay(100);
    turnLeft(); delay(TURN_DURATION);
    gradualForward(200);
    stopMotors(); delay(100);
  }
  else if (fl == HIGH) {
    Serial.println("⚠ Tepi depan kiri! Mundur & belok kanan");
    moveBackward(); delay(400);
    stopMotors(); delay(100);
    turnRight(); delay(400);
    gradualForward(200);
    stopMotors(); delay(100);
  }
  else if (fr == HIGH) {
    Serial.println("⚠ Tepi depan kanan! Mundur & belok kiri");
    moveBackward(); delay(400);
    stopMotors(); delay(100);
    turnLeft(); delay(400);
    gradualForward(200);
    stopMotors(); delay(100);
  }
  else if (bl == HIGH) {
    Serial.println("⚠ Tepi belakang kiri! Maju & belok kanan");
    gradualForward(400);
    stopMotors(); delay(100);
    turnRight(); delay(400);
    gradualForward(200);
    stopMotors(); delay(100);
  }
  else if (br == HIGH) {
    Serial.println("⚠ Tepi belakang kanan! Maju & belok kiri");
    gradualForward(400);
    stopMotors(); delay(100);
    turnLeft(); delay(400);
    gradualForward(200);
    stopMotors(); delay(100);
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
  gradualForward(200);
  stopMotors(); delay(300);
}

// ==========================
// LOOP UTAMA
// ==========================
void loop() {
  // Gunakan millis() untuk non-blocking timing
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastSensorCheck >= SENSOR_CHECK_INTERVAL) {
    lastSensorCheck = currentMillis;
    
    // Baca status semua sensor IR
    int fl = digitalRead(IR_FRONT_LEFT);
    int fr = digitalRead(IR_FRONT_RIGHT);
    int bl = digitalRead(IR_BACK_LEFT);
    int br = digitalRead(IR_BACK_RIGHT);

    Serial.print("IR FL: "); Serial.print(fl);
    Serial.print(" FR: "); Serial.print(fr);
    Serial.print(" BL: "); Serial.print(bl);
    Serial.print(" BR: "); Serial.println(br);
    
    // Jika SEMUA sensor HIGH, robot diangkat
    if (fl == HIGH && fr == HIGH && bl == HIGH && br == HIGH) {
      Serial.println("⚠⚠⚠ ROBOT DIANGKAT! EMERGENCY STOP! ⚠⚠⚠");
      stopMotors();
      delay(100);
      return;
    }
    
    // Jika ada sensor yang HIGH (mendeteksi tepi)
    if (fl == HIGH || fr == HIGH || bl == HIGH || br == HIGH) {
      stopMotors();
      Serial.println("‼ Deteksi tepi – STOP!");
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
  }
}