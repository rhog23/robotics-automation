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
#define IR_BACK_LEFT      13
#define IR_FRONT_RIGHT    21
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
#define SAFE_DISTANCE_CM  15
#define SENSOR_TIMEOUT    999
#define PWM_FREQ          5000
#define PWM_RESOLUTION    8
#define TURN_DURATION     500
#define REVERSE_DURATION  500
#define STOP_DELAY        200    // Waktu berhenti sebelum melakukan tindakan

// Untuk menghindari busy waiting 
unsigned long lastSensorCheck = 0;
const unsigned long SENSOR_CHECK_INTERVAL = 50;

// Untuk smooth acceleration
int currentSpeed = 0;
const int ACCEL_STEP = 15;           // Diturunkan dari 25
const int ACCEL_DELAY = 3;           // Diturunkan dari 5

// Variabel status
bool isMotorRunning = false;
bool isOnSurface = true;
bool wasRobotLifted = false;         // Flag untuk menandai jika robot diangkat
bool isMovingBackward = false;       // Flag untuk menandai jika robot sedang bergerak mundur
unsigned long lastObstacleTime = 0;
const unsigned long OBSTACLE_IGNORE_TIME = 500; // Prevent oscillation

// Forward declarations (tanpa default arguments)
void setMotorSpeed(int leftSpeed, int rightSpeed);
void stopMotors();
void moveForward(bool smooth);
void moveBackward();
void turnLeft();
void turnRight();
void gradualForward(int duration);
long getDistance();
long getStableDistance();
bool checkOnSurface();
void antiFallAvoidance();
void obstacleAvoidance();
bool checkLedgeDetected();

// ==========================
// SENSOR JARAK
// ==========================
long getDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long duration = pulseIn(ECHO, HIGH, 20000); // Reduced timeout
  if (duration == 0 || duration > 20000) return SENSOR_TIMEOUT;
  return duration * 0.034 / 2;
}

long getStableDistance() {
  // Take only 2 readings and average for faster response
  long dist = getDistance();
  return dist;  // Return single reading for faster response
}

// ==========================
// FUNGSI MOTOR
// ==========================
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  ledcWrite(ENABLE_LEFT_PIN, constrain(leftSpeed, 0, MAX_PWM_DUTY));
  ledcWrite(ENABLE_RIGHT_PIN, constrain(rightSpeed, 0, MAX_PWM_DUTY));
  
  // Track if motors are running
  isMotorRunning = (leftSpeed > 0 || rightSpeed > 0);
  currentSpeed = max(leftSpeed, rightSpeed);
}

void stopMotors() {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
  setMotorSpeed(0, 0);
  currentSpeed = 0;
  isMovingBackward = false;
  Serial.println("Motors stopped");
}

// Default argument hanya di definisi fungsi, tidak di deklarasi
void moveForward(bool smooth = true) {
  // Reset backward movement flag
  isMovingBackward = false;
  
  digitalWrite(MOTOR_LEFT_FWD, HIGH);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
  
  if (smooth && currentSpeed < SPEED_FORWARD) {
    // Mulai dari kecepatan yang lebih rendah untuk smoother acceleration
    int startSpeed = max(currentSpeed, 100);
    for (int pwm = startSpeed; pwm <= SPEED_FORWARD; pwm += ACCEL_STEP) {
      // Check for ledge during acceleration
      if (checkLedgeDetected()) {
        stopMotors();
        Serial.println("Ledge detected during forward acceleration - stopping");
        return;
      }
      setMotorSpeed(pwm, pwm);
      delay(ACCEL_DELAY);
    }
  } else {
    setMotorSpeed(SPEED_FORWARD, SPEED_FORWARD);
  }
  
  Serial.println("Moving forward with speed: " + String(SPEED_FORWARD));
}

void moveBackward() {
  // Set flag to indicate backward movement
  isMovingBackward = true;

  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, HIGH);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, HIGH);
  
  // PERUBAHAN: Smooth deceleration sebelum reverse
  int startSpeed = min(120, SPEED_REVERSE);
  setMotorSpeed(startSpeed, startSpeed);
  
  // Percepat secara bertahap hingga kecepatan penuh
  for (int pwm = startSpeed; pwm <= SPEED_REVERSE; pwm += ACCEL_STEP) {
    // Check for ledge during acceleration when moving backward
    if (checkLedgeDetected()) {
      stopMotors();
      Serial.println("Ledge detected during backward acceleration - stopping");
      return;
    }
    setMotorSpeed(pwm, pwm);
    delay(ACCEL_DELAY);
  }

  Serial.println("Moving backward with speed: " + String(SPEED_REVERSE));
}

void turnLeft() {
  // Reset backward movement flag
  isMovingBackward = false;

  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, HIGH);
  digitalWrite(MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
  
  // PERUBAHAN: Smooth acceleration untuk belok
  int startSpeed = 100;
  for (int pwm = startSpeed; pwm <= SPEED_FORWARD; pwm += ACCEL_STEP) {
    // Check for ledge during turning
    if (checkLedgeDetected()) {
      stopMotors();
      Serial.println("Ledge detected during left turn - stopping");
      return;
    }
    setMotorSpeed(pwm, pwm);
    delay(ACCEL_DELAY);
  }
}

void turnRight() {
  // Reset backward movement flag
  isMovingBackward = false;

  digitalWrite(MOTOR_LEFT_FWD, HIGH);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, HIGH);
  
  // PERUBAHAN: Smooth acceleration untuk belok
  int startSpeed = 100;
  for (int pwm = startSpeed; pwm <= SPEED_FORWARD; pwm += ACCEL_STEP) {
    // Check for ledge during turning
    if (checkLedgeDetected()) {
      stopMotors();
      Serial.println("Ledge detected during right turn - stopping");
      return;
    }
    setMotorSpeed(pwm, pwm);
    delay(ACCEL_DELAY);
  }
}

// Improved gradual acceleration forward
void gradualForward(int duration) {
  // Reset backward movement flag
  isMovingBackward = false;

  digitalWrite(MOTOR_LEFT_FWD, HIGH);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
  
  // Start from a lower speed for smoother acceleration
  for (int pwm = 100; pwm <= SPEED_FORWARD; pwm += ACCEL_STEP) {
    // Check for ledge during acceleration
    if (checkLedgeDetected()) {
      stopMotors();
      Serial.println("Ledge detected during gradual forward - stopping");
      return;
    }
    setMotorSpeed(pwm, pwm);
    delay(ACCEL_DELAY);
  }
  
  if (duration > 0) {
    unsigned long startTime = millis();
    while (millis() - startTime < duration) {
      // Check for ledge during the duration
      if (checkLedgeDetected()) {
        stopMotors();
        Serial.println("Ledge detected during timed movement - stopping");
        return;
      }
      delay(10); // Small delay to avoid excessive checking
    }
  }
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

// New function to check if any ledge is detected
bool checkLedgeDetected() {
  int fl = digitalRead(IR_FRONT_LEFT);
  int fr = digitalRead(IR_FRONT_RIGHT);
  int bl = digitalRead(IR_BACK_LEFT);
  int br = digitalRead(IR_BACK_RIGHT);

  // PENTING: Untuk sensor IR ini:
  // LOW = sensor mendeteksi permukaan (normal, aman)
  // HIGH = sensor tidak mendeteksi permukaan (tepi atau diangkat)

  // Any sensor being HIGH means danger - stop immediately
  if (fl == HIGH || fr == HIGH || bl == HIGH || br == HIGH) {
    // Log specific detection for debugging
    if (isMovingBackward && (bl == HIGH || br == HIGH)) {
      Serial.println("⚠ Ledge detected at back during backward movement!");
    } 
    else if (!isMovingBackward && (fl == HIGH || fr == HIGH)) {
      Serial.println("⚠ Ledge detected at front during forward movement!");
    }
    else {
      Serial.println("⚠ Ledge detected - immediate stop required!");
    }
    return true;
  }

  // Always check for all sensors being HIGH (robot lifted)
  if (fl == HIGH && fr == HIGH && bl == HIGH && br == HIGH) {
    Serial.println("⚠ All sensors HIGH - robot likely lifted!");
    return true;
  }

  return false;
}

// PERUBAHAN: Dimodifikasi agar berhenti dulu sebelum memperbaiki posisi
void antiFallAvoidance() {
  int fl = digitalRead(IR_FRONT_LEFT);
  int fr = digitalRead(IR_FRONT_RIGHT);
  int bl = digitalRead(IR_BACK_LEFT);
  int br = digitalRead(IR_BACK_RIGHT);

  // PERUBAHAN: Pastikan motor berhenti dulu
  stopMotors();
  delay(STOP_DELAY);  // Berikan waktu untuk benar-benar berhenti

  // Jika semua sensor HIGH, robot diangkat
  if (fl == HIGH && fr == HIGH && bl == HIGH && br == HIGH) {
    Serial.println("⚠ EMERGENCY STOP! Robot diangkat!");
    wasRobotLifted = true;  // Set flag untuk menandai robot diangkat
    return;
  }

  // Hindari tepi jika terdeteksi saat mundur
  if (isMovingBackward && (bl == HIGH || br == HIGH)) {
    Serial.println("⚠ Tepi belakang terdeteksi saat mundur! Berhenti!");
    stopMotors();
    delay(STOP_DELAY);
    return;
  }

  if (fl == HIGH && fr == HIGH) {
    Serial.println("⚠ Tepi depan ganda! Mundur & belok kanan");
    moveBackward(); delay(400);
    stopMotors(); delay(100);  // PERUBAHAN: Berhenti sejenak
    turnRight(); delay(TURN_DURATION);
    stopMotors(); delay(100);  // PERUBAHAN: Berhenti sejenak
    moveForward(true);
  }
  else if (bl == HIGH && br == HIGH) {
    Serial.println("⚠ Tepi belakang ganda! Maju & belok kiri");
    moveForward(false); delay(400);
    stopMotors(); delay(100);  // PERUBAHAN: Berhenti sejenak
    turnLeft(); delay(TURN_DURATION);
    stopMotors(); delay(100);  // PERUBAHAN: Berhenti sejenak
    moveForward(true);
  }
  else if (fl == HIGH) {
    Serial.println("⚠ Tepi depan kiri! Mundur & belok kanan");
    moveBackward(); delay(300);
    stopMotors(); delay(100);  // PERUBAHAN: Berhenti sejenak
    turnRight(); delay(350);
    stopMotors(); delay(100);  // PERUBAHAN: Berhenti sejenak
    moveForward(true);
  }
  else if (fr == HIGH) {
    Serial.println("⚠ Tepi depan kanan! Mundur & belok kiri");
    moveBackward(); delay(300);
    stopMotors(); delay(100);  // PERUBAHAN: Berhenti sejenak
    turnLeft(); delay(350);
    stopMotors(); delay(100);  // PERUBAHAN: Berhenti sejenak
    moveForward(true);
  }
  else if (bl == HIGH) {
    Serial.println("⚠ Tepi belakang kiri! Maju & belok kanan");
    moveForward(false); delay(300);
    stopMotors(); delay(100);  // PERUBAHAN: Berhenti sejenak
    turnRight(); delay(350);
    stopMotors(); delay(100);  // PERUBAHAN: Berhenti sejenak
    moveForward(true);
  }
  else if (br == HIGH) {
    Serial.println("⚠ Tepi belakang kanan! Maju & belok kiri");
    moveForward(false); delay(300);
    stopMotors(); delay(100);  // PERUBAHAN: Berhenti sejenak
    turnLeft(); delay(350);
    stopMotors(); delay(100);  // PERUBAHAN: Berhenti sejenak
    moveForward(true);
  }
}

// ==========================
// OBSTACLE AVOIDANCE
// ==========================
void obstacleAvoidance() {
  // Check if we recently handled an obstacle to prevent rapid back-and-forth
  if (millis() - lastObstacleTime < OBSTACLE_IGNORE_TIME) {
    return;
  }
  
  lastObstacleTime = millis();
  
  // PERUBAHAN: Pastikan berhenti dulu
  stopMotors();
  delay(STOP_DELAY);
  
  Serial.println("⛔ Obstacle! Mundur...");
  
  moveBackward(); delay(REVERSE_DURATION);
  stopMotors(); delay(STOP_DELAY);

  Serial.println("🔍 Scan kiri...");
  myServo.write(10); delay(200);
  long leftDist = getDistance();

  Serial.println("🔍 Scan kanan...");
  myServo.write(170); delay(200);
  long rightDist = getDistance();

  myServo.write(90); 

  Serial.print("↙ Kiri: "); Serial.print(leftDist);
  Serial.print(" cm | ↘ Kanan: "); Serial.println(rightDist);

  if (leftDist == rightDist || abs(leftDist - rightDist) < 10) {
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
  stopMotors(); delay(100);  // PERUBAHAN: Berhenti sejenak
  moveForward(true);
}

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
  delay(500);
  
  Serial.println("Robot ready!");
  
  // Berikan delay tambahan sebelum robot mulai bergerak
  delay(1000);
  
  // Inisialisasi timer pertama kali
  lastSensorCheck = millis();
  
  Serial.println("Robot starting movement!");
  
  // PERUBAHAN: Langsung bergerak maju saat startup
  moveForward(true);
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

    // Debug less frequently to reduce serial overhead
    if (currentMillis % 500 == 0) {
      Serial.print("IR FL: "); Serial.print(fl);
      Serial.print(" FR: "); Serial.print(fr);
      Serial.print(" BL: "); Serial.print(bl);
      Serial.print(" BR: "); Serial.println(br);
    }
    
    // PERUBAHAN: Cek apakah wasRobotLifted flag aktif
    if (wasRobotLifted) {
      // Reset flag
      wasRobotLifted = false;
      
      // Periksa apakah masih diangkat
      if (!(fl == HIGH && fr == HIGH && bl == HIGH && br == HIGH)) {
        Serial.println("🔄 Robot dikembalikan ke permukaan! Melanjutkan pergerakan...");
        delay(500);  // Tunggu sebentar sebelum mulai bergerak
        moveForward(true);  // Langsung maju
      }
    }
    
    // Jika SEMUA sensor HIGH, robot diangkat
    if (fl == HIGH && fr == HIGH && bl == HIGH && br == HIGH) {
      if (isMotorRunning) {  // Hanya menampilkan pesan jika motor sedang berputar
        Serial.println("⚠⚠⚠ ROBOT DIANGKAT! EMERGENCY STOP! ⚠⚠⚠");
        stopMotors();
        delay(100);
        wasRobotLifted = true;  // Set flag
      }
      return;
    }
    
    // Cek untuk SEMUA jenis pergerakan (maju atau mundur)
    // Jika ada sensor yang HIGH (mendeteksi tepi), berhenti dan buat keputusan
    if (fl == HIGH || fr == HIGH || bl == HIGH || br == HIGH) {
      Serial.println("‼ Deteksi tepi – STOP!");
      
      // Jika robot bergerak mundur dan sensor belakang mendeteksi tepi, berhenti saja
      if (isMovingBackward && (bl == HIGH || br == HIGH)) {
        Serial.println("⚠ Terdeteksi tepi saat mundur! Berhenti!");
        stopMotors();
        delay(STOP_DELAY);
        // Lakukan koreksi dengan bergerak maju sedikit
        moveForward(false); delay(200);
        stopMotors();
      } else {
        // Untuk kasus lain, lakukan anti-fall avoidance normal
        antiFallAvoidance();
      }
      return;
    }

    // Only check distance if we're not in recovery mode from a recent obstacle
    if (millis() - lastObstacleTime >= OBSTACLE_IGNORE_TIME) {
      long frontDist = getDistance();
      
      // Debug distance less frequently
      if (currentMillis % 500 == 0) {
        Serial.print("Ultrasonik: "); Serial.print(frontDist); Serial.println(" cm");
      }

      if (frontDist >= SENSOR_TIMEOUT) {
        // Handle sensor timeout more gracefully - keep moving if already moving
        if (!isMotorRunning) {
          Serial.println("⚠ Sensor timeout - continuing current action");
          moveForward(true);  // PERUBAHAN: Bergerak maju jika tidak bergerak
        }
      } else if (frontDist <= SAFE_DISTANCE_CM) {
        obstacleAvoidance();
      } else if (!isMotorRunning) {
        // Only call moveForward if we're not already moving
        Serial.println("✅ Jalan maju");
        moveForward(true);
      }
    }
  }
}