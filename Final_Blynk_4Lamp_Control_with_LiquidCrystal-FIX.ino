#define BLYNK_TEMPLATE_ID "TMPL6sTE5_t4K"
#define BLYNK_TEMPLATE_NAME "Smart Home"
#define BLYNK_PRINT Serial

#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>

// Auth Token yang dikirim Blynk ke email Anda
char auth[] = "oOXKGjKFu2gsWoQeusU4Bb_JAn7rNvI-";

// Credentials WiFi
char ssid[] = "Galaxy";
char pass[] = "qwertyu8888";

// Koneksi ESP8266
SoftwareSerial EspSerial(3, 2); // RX, TX

#define ESP8266_BAUD 9600
ESP8266 wifi(&EspSerial);

// Definisi pin relay
const int RELAY1 = 13;
const int RELAY2 = 12;
const int RELAY3 = 11;
const int RELAY4 = 10;

// Inisialisasi LiquidCrystal
LiquidCrystal lcd(4, 5, 6, 7, 8, 9); // Sesuaikan dengan pin yang digunakan untuk LCD

void setup() {
    Serial.begin(9600);
    delay(10);
    EspSerial.begin(ESP8266_BAUD);
    delay(10);

    // Inisialisasi LCD
    lcd.begin(16, 2); // Inisialisasi LCD 16x2

    // Mulai koneksi dengan Blynk
    Blynk.begin(auth, wifi, ssid, pass);

    // Inisialisasi pin relay sebagai output
    pinMode(RELAY1, OUTPUT);
    pinMode(RELAY2, OUTPUT);
    pinMode(RELAY3, OUTPUT);
    pinMode(RELAY4, OUTPUT);

    // Matikan semua relay di awal
    digitalWrite(RELAY1, 1);
    digitalWrite(RELAY2, 1);
    digitalWrite(RELAY3, 1);
    digitalWrite(RELAY4, 1);

    // Tunggu hingga terhubung dengan WiFi
    while (Blynk.connect() == false) {
        delay(1000);
        Serial.println("Mencoba terhubung ke Blynk...");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Connecting...");
    }

    // Tampilkan status koneksi di Serial Monitor dan LCD
    Serial.println("Koneksi WiFi berhasil");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Connected");

    Serial.print("Status koneksi Blynk: ");
    Serial.println(Blynk.connected() ? "Terhubung" : "Tidak terhubung");
    lcd.setCursor(0, 1);
    lcd.print(Blynk.connected() ? "Blynk: Connected" : "Blynk: Disconnected");
}

void loop() {
    Blynk.run();
}

// Fungsi untuk mengontrol relay melalui Blynk
BLYNK_WRITE(V0) {
    int pinValue = param.asInt();
    Serial.print("Mengubah status RELAY1 ke: ");
    Serial.println(pinValue);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("RELAY1: ");
    lcd.print(pinValue == 1 ? "ON" : "OFF");
    delay(500); // Tambahkan sedikit jeda waktu
    digitalWrite(RELAY1, pinValue == 1 ? LOW : HIGH);
}

BLYNK_WRITE(V1) {
    int pinValue = param.asInt();
    Serial.print("Mengubah status RELAY2 ke: ");
    Serial.println(pinValue);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("RELAY2: ");
    lcd.print(pinValue == 1 ? "ON" : "OFF");
    delay(500); // Tambahkan sedikit jeda waktu
    digitalWrite(RELAY2, pinValue == 1 ? LOW : HIGH);
}

BLYNK_WRITE(V2) {
    int pinValue = param.asInt();
    Serial.print("Mengubah status RELAY3 ke: ");
    Serial.println(pinValue);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("RELAY3: ");
    lcd.print(pinValue == 1 ? "ON" : "OFF");
    delay(500); // Tambahkan sedikit jeda waktu
    digitalWrite(RELAY3, pinValue == 1 ? LOW : HIGH);
}

BLYNK_WRITE(V3) {
    int pinValue = param.asInt();
    Serial.print("Mengubah status RELAY4 ke: ");
    Serial.println(pinValue);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("RELAY4: ");
    lcd.print(pinValue == 1 ? "ON" : "OFF");
    delay(500); // Tambahkan sedikit jeda waktu
    digitalWrite(RELAY4, pinValue == 1 ? LOW : HIGH);
}
