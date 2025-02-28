// Mendefinisikan library yang diperlukan
#include <SoftwareSerial.h>
#include <stdlib.h>

unsigned char check_connection = 0; // variabel yang digunakan untuk memeriksa status koneksi WiFi
unsigned char times_check = 0;      // variabel counter yang digunakan untuk menentukan apakah koneksi akan dilakukan kembali

/*
 * Susunan parameternya yakni RX, TX mengikuti RX, TX di modul ESP8266
 */
SoftwareSerial ESP8266(2, 3);

// Definisikan pin yang terhubung ke relay
const int relayPin0 = 8;
const int relayPin1 = 9;
const int relayPin2 = 10;
const int relayPin3 = 11;

void setup()
{
  // Set pin sebagai output
  pinMode(relayPin0, OUTPUT);
  pinMode(relayPin1, OUTPUT);
  pinMode(relayPin2, OUTPUT);
  pinMode(relayPin3, OUTPUT);

  // Relay dimatikan secara default saat Arduino pertama kali dijalankan
  digitalWrite(relayPin0, LOW);
  digitalWrite(relayPin1, LOW);
  digitalWrite(relayPin2, LOW);
  digitalWrite(relayPin3, LOW);
}

void loop()
{
  // Aktifkan relay
  digitalWrite(relayPin0, HIGH);
  delay(500); // Tunda selama 1 detik

  digitalWrite(relayPin1, HIGH);
  delay(500); // Tunda selama 1 detik

  digitalWrite(relayPin2, HIGH);
  delay(500); // Tunda selama 1 detik

  digitalWrite(relayPin3, HIGH);
  delay(500); // Tunda selama 1 detik

  // Matikan relay
  digitalWrite(relayPin0, LOW);
  delay(500); // Tunda selama 1 detik

  // Matikan relay
  digitalWrite(relayPin1, LOW);
  delay(500); // Tunda selama 1 detik

  // Matikan relay
  digitalWrite(relayPin2, LOW);
  delay(500); // Tunda selama 1 detik

  // Matikan relay
  digitalWrite(relayPin3, LOW);
  delay(500); // Tunda selama 1 detik
}
