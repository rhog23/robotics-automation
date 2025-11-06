#define SENSOR_PIN 18 // Pin GPIO18 pada ESP32 terhubung ke pin OUT sensor IR penghindar halangan

void setup() {
  // memulai komunikasi serial dengan kecepatan 9600 bit per detik
  Serial.begin(9600);
  // mengatur pin ESP32 sebagai input untuk membaca data dari sensor
  pinMode(SENSOR_PIN, INPUT);
}

void loop() {
  // membaca kondisi dari pin input
  int state = digitalRead(SENSOR_PIN);

  // jika sensor mendeteksi halangan (output LOW)
  if (state == LOW)
    Serial.println("Ada halangan di depan sensor");
  else
    Serial.println("Tidak ada halangan di depan sensor");

  // jeda 100 milidetik sebelum membaca lagi
  delay(100);
}