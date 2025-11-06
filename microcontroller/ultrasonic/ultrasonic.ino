#define TRIG_PIN 23              // deklarasikan dan inisialisasi variabel untuk pin 23 pada ESP32
#define ECHO_PIN 22              // deklarasikan dan inisialisasi variabel untuk pin 22 pada ESP32
float duration_us, distance_cm;  // deklarasikan dua variabel bertipe float untuk waktu dan jarak

void setup() {
  Serial.begin(9600);         // inisialisasi komunikasi serial dengan kecepatan 9600 bit per detik
  pinMode(TRIG_PIN, OUTPUT);  // inisialisasi trig sebagai output
  pinMode(ECHO_PIN, INPUT);   // inisialisasi echo sebagai input
}

void loop() {
  digitalWrite(TRIG_PIN, LOW);   // set trig ke LOW
  delayMicroseconds(2);          // tunggu selama 2 mikrodetik
  digitalWrite(TRIG_PIN, HIGH);  // set trig ke HIGH
  delayMicroseconds(10);         // tunggu selama 10 mikrodetik
  digitalWrite(TRIG_PIN, LOW);   // set trig kembali ke LOW

  duration_us = pulseIn(ECHO_PIN, HIGH);   // baca durasi pulsa echo
  distance_cm = duration_us * 0.0343 / 2;  // hitung jarak dalam cm

  Serial.print("Distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");

  delay(500); // tunggu 1/2 detik
}
