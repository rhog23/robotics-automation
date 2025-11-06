#define MOTION_SENSOR_PIN  32  // Pin GPIO32 pada ESP32 terhubung ke pin OUTPUT sensor gerak (PIR)
#define LED_PIN            17  // Pin GPIO17 pada ESP32 terhubung ke kaki LED

int motionStateCurrent  = LOW; // kondisi sensor gerak saat ini
int motionStatePrevious = LOW; // kondisi sensor gerak sebelumnya

void setup() {
  Serial.begin(9600);                // memulai komunikasi serial
  pinMode(MOTION_SENSOR_PIN, INPUT); // mengatur pin ESP32 sebagai input (membaca data dari sensor)
  pinMode(LED_PIN, OUTPUT);          // mengatur pin ESP32 sebagai output (mengendalikan LED)
}

void loop() {
  motionStatePrevious = motionStateCurrent;             // menyimpan kondisi lama sensor
  motionStateCurrent  = digitalRead(MOTION_SENSOR_PIN); // membaca kondisi terbaru dari sensor

  // Jika kondisi berubah dari LOW ke HIGH → ada gerakan terdeteksi
  if (motionStatePrevious == LOW && motionStateCurrent == HIGH) {
    Serial.println("Gerakan terdeteksi! LED menyala");
    digitalWrite(LED_PIN, HIGH); // menyalakan LED
  } 
  // Jika kondisi berubah dari HIGH ke LOW → gerakan berhenti
  else if (motionStatePrevious == HIGH && motionStateCurrent == LOW) {
    Serial.println("Gerakan berhenti! LED mati");
    digitalWrite(LED_PIN, LOW);  // mematikan LED
  }
}