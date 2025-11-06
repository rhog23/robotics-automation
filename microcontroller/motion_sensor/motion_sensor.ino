const int PIR_SENSOR = 19; // Pin GPIO19 terhubung ke pin OUTPUT sensor
int pinStateCurrent   = LOW;  // kondisi pin saat ini
int pinStatePrevious  = LOW;  // kondisi pin sebelumnya

void setup() {
  Serial.begin(9600);
  pinMode(PIR_SENSOR, INPUT); // mengatur pin ESP32 sebagai input untuk membaca nilai dari pin OUTPUT sensor
}

void loop() {
  pinStatePrevious = pinStateCurrent; // menyimpan kondisi lama
  pinStateCurrent = digitalRead(PIR_SENSOR);   // membaca kondisi terbaru dari pin

  if (pinStatePrevious == LOW && pinStateCurrent == HIGH) {   // perubahan kondisi pin: dari LOW -> HIGH
    Serial.println("Gerakan terdeteksi!");
    // TODO: nyalakan lampu
  }
  else
  if (pinStatePrevious == HIGH && pinStateCurrent == LOW) {   // perubahan kondisi pin: dari HIGH -> LOW
    Serial.println("Gerakan berhenti!");
    // TODO: matikan lampu
  }
}
