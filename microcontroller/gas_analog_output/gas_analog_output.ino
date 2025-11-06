#define AO_PIN 36  // Pin GPIO36 pada ESP32 terhubung ke pin AO (Analog Output) dari sensor MQ2

void setup() {
  // memulai komunikasi serial dengan komputer
  Serial.begin(9600);

  // mengatur sensitivitas pembacaan ADC (Analog to Digital Converter)
  // 11 dB berarti tegangan maksimum yang bisa dibaca adalah sekitar 3,3V
  analogSetAttenuation(ADC_11db);

  Serial.println("Memanaskan sensor MQ2...");
  delay(20000);  // menunggu selama 20 detik agar sensor MQ2 siap digunakan
}

void loop() {
  // membaca nilai tegangan dari pin AO sensor (hasilnya berupa nilai analog)
  int gasValue = analogRead(AO_PIN);

  // menampilkan nilai hasil pembacaan sensor ke Serial Monitor
  Serial.print("Nilai AO dari sensor MQ2: ");
  Serial.println(gasValue);
}
