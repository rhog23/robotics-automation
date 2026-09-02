// ============================================================
//  ESP32 Line Follower  –  Arduino Core v3.x
//  Verified & corrected – May 2026
//
//  Wiring asumsi:
//    Motor Driver : L298N (atau kompatibel)
//    Sensor IR    : modul TCRT5000 dengan komparator (LM393)
//
//  Konfigurasi sensor: CONFIG_B
//    Sensor mengAPIT garis — garis hitam berada DI ANTARA
//    sensor kiri dan kanan.
//
//    [IR_L]       [IR_R]
//  _____|_____________|_____
//            |
//          garis
//
//  Kondisi normal maju: keduanya di atas PUTIH (LOW, LOW)
//  Koreksi: salah satu kena hitam → robot koreksi ke arah itu
// ============================================================

#include <Arduino.h>

// ------------------------------------------------------------
// ★ POLARITAS SENSOR
//   HIGH = sensor membaca permukaan HITAM (garis)
//   LOW  = sensor membaca permukaan PUTIH
//   Sesuaikan jika modulmu terbalik.
// ------------------------------------------------------------
#define SENSOR_ON_LINE  HIGH

// ------------------------------------------------------------
// PIN MOTOR DRIVER
//
//  CATATAN GPIO 14: pin ini di-pull HIGH oleh ESP32 saat boot,
//  bisa menyebabkan motor R maju sesaat. Jika ini masalah,
//  pindahkan MOTOR_R_BWD ke pin lain (mis. GPIO 12).
// ------------------------------------------------------------
#define MOTOR_L_FWD   25
#define MOTOR_L_BWD   26
#define MOTOR_R_FWD   27
#define MOTOR_R_BWD   14   // ← bisa diganti GPIO 12 jika perlu

// Pin PWM Enable (harus mendukung LEDC)
#define PWM_LEFT      32
#define PWM_RIGHT     33

// ------------------------------------------------------------
// PIN SENSOR IR
//   GPIO 35 : input-only, TANPA internal pull. Wajib pakai
//             resistor pull-down eksternal 10kΩ ke GND jika
//             modul sensor tidak punya resistor bawaan.
//   GPIO 13 : normal GPIO, diset INPUT_PULLDOWN untuk stabilitas.
// ------------------------------------------------------------
#define IR_LEFT       35
#define IR_RIGHT      13

// ------------------------------------------------------------
// PARAMETER PWM & KECEPATAN
// ------------------------------------------------------------
#define PWM_FREQ        5000   // Hz
#define PWM_RESOLUTION  8      // bit → range duty: 0–255
#define MAX_DUTY        255

#define SPEED_FWD       165    // duty maju lurus  (~70%)
#define SPEED_TURN      165    // duty roda aktif saat belok (~82%)

// ------------------------------------------------------------
// PARAMETER RECOVERY (saat kedua sensor kehilangan garis)
//   Set LOST_TIMEOUT_MS = 0 untuk menonaktifkan batas waktu.
// ------------------------------------------------------------
#define LOST_TIMEOUT_MS   0  // ms sebelum berhenti total

// ============================================================
// STATE INTERNAL
// ============================================================
static int           lastDir = 0;    // -1=kiri, 0=lurus, 1=kanan
static unsigned long lostAt  = 0;    // timestamp saat garis hilang

// ============================================================
// FUNGSI MOTOR (helper internal)
// ============================================================

/** Tulis duty PWM ke kedua enable pin. */
static void _setSpeed(int left, int right) {
  ledcWrite(PWM_LEFT,  constrain(left,  0, MAX_DUTY));
  ledcWrite(PWM_RIGHT, constrain(right, 0, MAX_DUTY));
}

// PENTING: selalu set arah (digitalWrite) SEBELUM set speed (_setSpeed)
// agar H-bridge tidak sesaat mendapat tegangan dengan arah tidak tentu.

void stopMotors() {
  // Set arah dulu → netral
  digitalWrite(MOTOR_L_FWD, LOW);  digitalWrite(MOTOR_L_BWD, LOW);
  digitalWrite(MOTOR_R_FWD, LOW);  digitalWrite(MOTOR_R_BWD, LOW);
  // Baru matikan speed
  _setSpeed(0, 0);
}

void moveForward() {
  // Set arah
  digitalWrite(MOTOR_L_FWD, HIGH); digitalWrite(MOTOR_L_BWD, LOW);
  digitalWrite(MOTOR_R_FWD, HIGH); digitalWrite(MOTOR_R_BWD, LOW);
  // Set speed
  _setSpeed(SPEED_FWD, SPEED_FWD);
}

/**
 * Pivot kiri: roda kiri berhenti, roda kanan maju.
 * Dipakai saat garis ada di sebelah KIRI robot (robot harus belok kiri).
 */
void turnLeft() {
  // Set arah: kiri diam, kanan maju
  digitalWrite(MOTOR_L_FWD, LOW);  digitalWrite(MOTOR_L_BWD, LOW);
  digitalWrite(MOTOR_R_FWD, HIGH); digitalWrite(MOTOR_R_BWD, LOW);
  // Set speed
  _setSpeed(0, SPEED_TURN);
}

/**
 * Pivot kanan: roda kiri maju, roda kanan berhenti.
 * Dipakai saat garis ada di sebelah KANAN robot (robot harus belok kanan).
 */
void turnRight() {
  // Set arah: kiri maju, kanan diam
  digitalWrite(MOTOR_L_FWD, HIGH); digitalWrite(MOTOR_L_BWD, LOW);
  digitalWrite(MOTOR_R_FWD, LOW);  digitalWrite(MOTOR_R_BWD, LOW);
  // Set speed
  _setSpeed(SPEED_TURN, 0);
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("[LINE FOLLOWER] Initializing...");

  // Pin arah motor
  pinMode(MOTOR_L_FWD, OUTPUT);
  pinMode(MOTOR_L_BWD, OUTPUT);
  pinMode(MOTOR_R_FWD, OUTPUT);
  pinMode(MOTOR_R_BWD, OUTPUT);

  // PWM enable (Arduino Core v3.x)
  ledcAttach(PWM_LEFT,  PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PWM_RIGHT, PWM_FREQ, PWM_RESOLUTION);

  // Sensor
  pinMode(IR_LEFT,  INPUT);          // GPIO 35: input-only, tanpa pull internal
  pinMode(IR_RIGHT, INPUT_PULLDOWN); // GPIO 13: aktifkan pull-down internal

  // Pastikan motor mati sebelum loop mulai
  stopMotors();

  // Tunggu sensor dan driver stabil
  delay(500);

  Serial.println("[LINE FOLLOWER] Ready. Listening to sensors...");
}

// ============================================================
// LOOP UTAMA
// ============================================================
void loop() {
  int rawL = digitalRead(IR_LEFT);
  int rawR = digitalRead(IR_RIGHT);

  // Normalisasi: onLine = true jika sensor mendeteksi garis hitam
  bool L = (rawL == SENSOR_ON_LINE);
  bool R = (rawR == SENSOR_ON_LINE);

  // Debug – buka Serial Monitor untuk verifikasi
  Serial.printf("IR raw L:%d R:%d  |  onLine L:%d R:%d\n", rawL, rawR, L, R);

  // ══════════════════════════════════════════════════════════
  //  LOGIKA CONFIG B — sensor mengapit garis
  //  Lintasan: 1 jalur memutar, tanpa simpang
  //
  //  !L && !R → NORMAL  : garis di tengah, maju lurus
  //   L && !R → KOREKSI : robot geser kiri, koreksi kanan
  //  !L &&  R → KOREKSI : robot geser kanan, koreksi kiri
  //   L &&  R → RECOVERY: overshoot tikungan, pakai lastDir
  // ══════════════════════════════════════════════════════════

  // ── Keduanya PUTIH → garis tepat di tengah → MAJU ────────
  if (!L && !R) {
    moveForward();
    lastDir = 0;
    lostAt  = 0;
    Serial.println(">> FORWARD");
  }

  // ── Kiri HITAM → robot geser kiri → koreksi KANAN ────────
  else if (L && !R) {
    turnRight();
    lastDir = 1;
    lostAt  = 0;
    Serial.println(">> CORRECTION RIGHT");
  }

  // ── Kanan HITAM → robot geser kanan → koreksi KIRI ───────
  else if (!L && R) {
    turnLeft();
    lastDir = -1;
    lostAt  = 0;
    Serial.println(">> CORRECTION LEFT");
  }

  // ── Keduanya HITAM → overshoot tikungan → recovery ───────
  // Pakai lastDir: teruskan arah koreksi sebelumnya.
  // Jika melebihi LOST_TIMEOUT_MS → berhenti (safety).
  else {
    if (lostAt == 0) lostAt = millis();

    bool timedOut = (LOST_TIMEOUT_MS > 0) &&
                    ((millis() - lostAt) >= LOST_TIMEOUT_MS);

    if (timedOut) {
      stopMotors();
      Serial.println(">> ANOMALY – TIMEOUT, STOPPED");
    }
    else if (lastDir == 1) {
      turnRight();
      Serial.println(">> RECOVERY RIGHT");
    }
    else if (lastDir == -1) {
      turnLeft();
      Serial.println(">> RECOVERY LEFT");
    }
    else {
      // lastDir == 0: tidak ada riwayat → maju pelan
      moveForward();
      Serial.println(">> RECOVERY FORWARD");
    }
  }

  delay(5); // ~200 Hz loop rate
}
