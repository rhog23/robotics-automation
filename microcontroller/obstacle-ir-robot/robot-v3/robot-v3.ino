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
#define IR_FRONT_RIGHT    18
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
#define SPEED_FORWARD     190         // Increased from 190
#define SPEED_REVERSE     190         // Increased from 190
#define SAFE_DISTANCE_CM  15          // Reduced from 20 for less stopping
#define SENSOR_TIMEOUT    999
#define PWM_FREQ          5000
#define PWM_RESOLUTION    8
#define TURN_DURATION     400         // Reduced from 450
#define REVERSE_DURATION  400         // Reduced from 450

// Untuk menghindari busy waiting
unsigned long lastSensorCheck = 0;
const unsigned long SENSOR_CHECK_INTERVAL = 50;  // Reduced from 100ms for faster response

// Untuk smooth acceleration
int currentSpeed = 0;
const int ACCEL_STEP = 25;           // Increased from 10
const int ACCEL_DELAY = 5;           // Reduced from 20

// Variabel status
bool isMotorRunning = false;
bool isOnSurface = true;
unsigned long lastObstacleTime = 0;
const unsigned long OBSTACLE_IGNORE_TIME = 500; // Prevent oscillation

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
  delay(500); // Reduced from 1000ms
  
  Serial.println("Robot ready!");
  
  // Berikan delay tambahan sebelum robot mulai bergerak
  delay(1000); // Reduced from 2000ms
  
  // Inisialisasi timer pertama kali
  lastSensorCheck = millis();
  
  Serial.println("Robot starting movement!");
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
  Serial.println("Motors stopped");
}

void moveForward(bool smooth = true) {
  digitalWrite(MOTOR_LEFT_FWD, HIGH);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
  
  if (smooth && currentSpeed < SPEED_FORWARD) {
    // Start from a higher initial speed
    int startSpeed = max(currentSpeed, 150);
    for (int pwm = startSpeed; pwm <= SPEED_FORWARD; pwm += ACCEL_STEP) {
      setMotorSpeed(pwm, pwm);
      delay(ACCEL_DELAY);
    }
  } else {
    setMotorSpeed(SPEED_FORWARD, SPEED_FORWARD);
  }
}

void moveBackward() {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, HIGH);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, HIGH);
  setMotorSpeed(SPEED_REVERSE, SPEED_REVERSE);
}

void turnLeft() {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, HIGH);
  digitalWrite(MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
  setMotorSpeed(SPEED_FORWARD, SPEED_FORWARD);
}

void turnRight() {
  digitalWrite(MOTOR_LEFT_FWD, HIGH);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, HIGH);
  setMotorSpeed(SPEED_FORWARD, SPEED_FORWARD);
}

// Improved gradual acceleration forward
void gradualForward(int duration) {
  digitalWrite(MOTOR_LEFT_FWD, HIGH);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
  
  // Start from a higher speed (150 instead of 180)
  for (int pwm = 150; pwm <= SPEED_FORWARD; pwm += ACCEL_STEP) {
    setMotorSpeed(pwm, pwm);
    delay(ACCEL_DELAY);
  }
  
  if (duration > 0) {
    delay(duration);
  }
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
    moveBackward(); delay(400); // Reduced from 500
    turnRight(); delay(TURN_DURATION);
    moveForward(false);
  }
  else if (bl == HIGH && br == HIGH) {
    Serial.println("⚠ Tepi belakang ganda! Maju & belok kiri");
    moveForward(false); delay(400); // Reduced from 500
    turnLeft(); delay(TURN_DURATION);
    moveForward(false);
  }
  else if (fl == HIGH) {
    Serial.println("⚠ Tepi depan kiri! Mundur & belok kanan");
    moveBackward(); delay(300); // Reduced from 400
    turnRight(); delay(350);
    moveForward(false);
  }
  else if (fr == HIGH) {
    Serial.println("⚠ Tepi depan kanan! Mundur & belok kiri");
    moveBackward(); delay(300); // Reduced from 400
    turnLeft(); delay(350);
    moveForward(false);
  }
  else if (bl == HIGH) {
    Serial.println("⚠ Tepi belakang kiri! Maju & belok kanan");
    moveForward(false); delay(300); // Reduced from 400
    turnRight(); delay(350);
    moveForward(false);
  }
  else if (br == HIGH) {
    Serial.println("⚠ Tepi belakang kanan! Maju & belok kiri");
    moveForward(false); delay(300); // Reduced from 400
    turnLeft(); delay(350);
    moveForward(false);
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
  stopMotors();
  Serial.println("⛔ Obstacle! Mundur...");
  
  moveBackward(); delay(REVERSE_DURATION);
  stopMotors(); 

  Serial.println("🔍 Scan kiri...");
  myServo.write(10); delay(200); // Reduced from 300ms
  long leftDist = getDistance();

  Serial.println("🔍 Scan kanan...");
  myServo.write(170); delay(200); // Reduced from 300ms
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
  moveForward();
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
    
    // Jika SEMUA sensor HIGH, robot diangkat
    if (fl == HIGH && fr == HIGH && bl == HIGH && br == HIGH) {
      Serial.println("⚠⚠⚠ ROBOT DIANGKAT! EMERGENCY STOP! ⚠⚠⚠");
      stopMotors();
      delay(100);
      return;
    }
    
    // Jika ada sensor yang HIGH (mendeteksi tepi)
    if (fl == HIGH || fr == HIGH || bl == HIGH || br == HIGH) {
      Serial.println("‼ Deteksi tepi – STOP!");
      antiFallAvoidance();
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
        }
      } else if (frontDist <= SAFE_DISTANCE_CM) {
        obstacleAvoidance();
      } else if (!isMotorRunning) {
        // Only call moveForward if we're not already moving
        Serial.println("✅ Jalan maju");
        moveForward();
      }
    }
  }
}