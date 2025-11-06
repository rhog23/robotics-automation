#define TRIG_PIN 23  // Pin GPIO23 pada ESP32 terhubung ke pin TRIG sensor ultrasonik
#define ECHO_PIN 22  // Pin GPIO22 pada ESP32 terhubung ke pin ECHO sensor ultrasonik

float duration_us, distance_cm;

void setup() {
  // memulai komunikasi serial
  Serial.begin(9600);

  // mengatur pin trigger sebagai output
  pinMode(TRIG_PIN, OUTPUT);
  // mengatur pin echo sebagai input
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  // menghasilkan pulsa selama 10 mikrodetik ke pin TRIG
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // mengukur durasi pulsa dari pin ECHO
  duration_us = pulseIn(ECHO_PIN, HIGH);

  // menghitung jarak
  distance_cm = 0.017 * duration_us;

  // mencetak nilai ke Serial Monitor
  Serial.print("distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");

  delay(500);
}
