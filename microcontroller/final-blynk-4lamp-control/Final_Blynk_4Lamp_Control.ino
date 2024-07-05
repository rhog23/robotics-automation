#define BLYNK_TEMPLATE_ID "TMPL6sTE5_t4K"
#define BLYNK_TEMPLATE_NAME "Smart Home"
#define BLYNK_PRINT Serial

#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>
#include <SoftwareSerial.h>

// Auth Token yang dikirim Blynk ke email Anda
char auth[] = "oOXKGjKFu2gsWoQeusU4Bb_JAn7rNvI-";

// Credentials WiFi
char ssid[] = "Unifi";
char pass[] = "99990000";

// Koneksi ESP8266
SoftwareSerial EspSerial(3, 2); // RX, TX

#define ESP8266_BAUD 9600
ESP8266 wifi(&EspSerial);

// Definisi pin relay
const int RELAY1 = 8;
const int RELAY2 = 9;
const int RELAY3 = 10;
const int RELAY4 = 11;

void setup() {
    Serial.begin(9600);
    delay(10);
    EspSerial.begin(ESP8266_BAUD);
    delay(10);

    // Mulai koneksi dengan Blynk
    Blynk.begin(auth, wifi, ssid, pass);

    // Inisialisasi pin relay sebagai output
    pinMode(RELAY1, OUTPUT);
    pinMode(RELAY2, OUTPUT);
    pinMode(RELAY3, OUTPUT);
    pinMode(RELAY4, OUTPUT);

    // Matikan semua relay di awal
    digitalWrite(RELAY1, HIGH);
    digitalWrite(RELAY2, HIGH);
    digitalWrite(RELAY3, HIGH);
    digitalWrite(RELAY4, HIGH);

    // Tunggu hingga terhubung dengan WiFi
    while (Blynk.connect() == false) {
        delay(1000);
        Serial.println("Mencoba terhubung ke Blynk...");
    }

    // Tampilkan status koneksi di Serial Monitor
    Serial.println("Koneksi WiFi berhasil");
    Serial.print("Status koneksi Blynk: ");
    Serial.println(Blynk.connected() ? "Terhubung" : "Tidak terhubung");
}

void loop() {
    Blynk.run();
}

// Fungsi untuk mengontrol relay melalui Blynk
BLYNK_WRITE(V0) {
    int pinValue = param.asInt();
    Serial.print("Mengubah status RELAY1 ke: ");
    Serial.println(pinValue);
    digitalWrite(RELAY1, pinValue);
}

BLYNK_WRITE(V1) {
    int pinValue = param.asInt();
    Serial.print("Mengubah status RELAY2 ke: ");
    Serial.println(pinValue);
    digitalWrite(RELAY2, pinValue);
}

BLYNK_WRITE(V2) {
    int pinValue = param.asInt();
    Serial.print("Mengubah status RELAY3 ke: ");
    Serial.println(pinValue);
    digitalWrite(RELAY3, pinValue);
}

BLYNK_WRITE(V3) {
    int pinValue = param.asInt();
    Serial.print("Mengubah status RELAY4 ke: ");
    Serial.println(pinValue);
    digitalWrite(RELAY4, pinValue);
}