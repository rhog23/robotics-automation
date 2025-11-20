/*
 * Proyek Multi Sensor ESP32
 * Sensor:
 * 1. PIR Motion Sensor (dengan LED indikator)
 * 2. Sensor Infrared (IR Obstacle)
 * 3. Sensor Gas MQ2 (Digital + Analog)
 * 4. Sensor Suhu DS18B20
 * 5. Sensor DHT11 + LCD I2C (suhu & kelembapan)
 *
 * Pastikan semua library berikut sudah terpasang:
 * - DallasTemperature (oleh Miles Burton)
 * - OneWire (oleh Jim Studt)
 * - DHT sensor library (oleh Adafruit)
 * - LiquidCrystal_I2C (oleh Frank de Brabander)
 */

#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>

// ======================= DEFINISIKAN PIN ==========================
#define PIR_PIN        19   // pin sensor gerak PIR
#define LED_PIN        17   // LED indikator PIR
#define IR_PIN         18   // pin sensor infrared
#define MQ2_DO_PIN     16   // pin digital MQ2
#define MQ2_AO_PIN     36   // pin analog MQ2
#define DS18B20_PIN    27   // pin sensor DS18B20
#define DHT11_PIN      23   // pin sensor DHT11
// ===============================================================

// Objek sensor
OneWire oneWire(DS18B20_PIN);
DallasTemperature DS18B20(&oneWire);
DHT dht11(DHT11_PIN, DHT11);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Variabel PIR
int motionCurrent = LOW;
int motionPrevious = LOW;

// Variabel sensor suhu
float tempC_DS18B20 = 0;
float tempF_DS18B20 = 0;

// Waktu update (agar LCD tidak flicker)
unsigned long lastUpdate = 0;
const long updateInterval = 2000; // 2 detik

void setup() {
  Serial.begin(9600);
  Serial.println("Inisialisasi semua sensor...");

  // --- PIR Sensor ---
  pinMode(PIR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  // --- Infrared Sensor ---
  pinMode(IR_PIN, INPUT);

  // --- Gas Sensor ---
  pinMode(MQ2_DO_PIN, INPUT);
  analogSetAttenuation(ADC_11db); // pembacaan analog ~3.3V

  // --- DS18B20 ---
  DS18B20.begin();

  // --- DHT11 + LCD ---
  dht11.begin();
  lcd.init();
  lcd.backlight();

  // Pesan awal LCD
  lcd.setCursor(0, 0);
  lcd.print("ESP32 MULTI SENSOR");
  lcd.setCursor(0, 1);
  lcd.print("Inisialisasi...");
  delay(3000);
  lcd.clear();

  // Pemanasan sensor MQ2
  Serial.println("Memanaskan sensor MQ2 selama 20 detik...");
  delay(20000);
}

void loop() {
  // ---------------- PIR Sensor ----------------
  motionPrevious = motionCurrent;
  motionCurrent = digitalRead(PIR_PIN);

  if (motionPrevious == LOW && motionCurrent == HIGH) {
    Serial.println("Gerakan terdeteksi! LED menyala");
    digitalWrite(LED_PIN, HIGH);
  } else if (motionPrevious == HIGH && motionCurrent == LOW) {
    Serial.println("Gerakan berhenti! LED mati");
    digitalWrite(LED_PIN, LOW);
  }

  // ---------------- Infrared Sensor ----------------
  int irState = digitalRead(IR_PIN);
  if (irState == LOW)
    Serial.println("IR: Ada halangan di depan sensor");
  else
    Serial.println("IR: Tidak ada halangan di depan sensor");

  // ---------------- Gas Sensor MQ2 ----------------
  int gasDigital = digitalRead(MQ2_DO_PIN);
  int gasAnalog = analogRead(MQ2_AO_PIN);

  if (gasDigital == LOW)
    Serial.println("MQ2: Gas terdeteksi!");
  else
    Serial.println("MQ2: Tidak ada gas terdeteksi");

  Serial.print("MQ2 (Analog): ");
  Serial.println(gasAnalog);

  // ---------------- DS18B20 ----------------
  DS18B20.requestTemperatures();
  tempC_DS18B20 = DS18B20.getTempCByIndex(0);
  tempF_DS18B20 = tempC_DS18B20 * 9 / 5 + 32;
  Serial.print("DS18B20: ");
  Serial.print(tempC_DS18B20);
  Serial.print("°C  /  ");
  Serial.print(tempF_DS18B20);
  Serial.println("°F");

  // ---------------- DHT11 + LCD ----------------
  unsigned long currentMillis = millis();
  if (currentMillis - lastUpdate >= updateInterval) {
    lastUpdate = currentMillis;

    float humi = dht11.readHumidity();
    float tempC_DHT = dht11.readTemperature();

    lcd.clear();
    if (isnan(tempC_DHT) || isnan(humi)) {
      lcd.setCursor(0, 0);
      lcd.print("Gagal Baca DHT11");
    } else {
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(tempC_DHT);
      lcd.print("C");

      lcd.setCursor(0, 1);
      lcd.print("Humi: ");
      lcd.print(humi);
      lcd.print("%");
    }

    // Juga tampilkan di Serial Monitor
    Serial.print("DHT11: ");
    Serial.print(tempC_DHT);
    Serial.print("°C, ");
    Serial.print(humi);
    Serial.println("% RH");
  }

  // Jeda antar pembacaan
  delay(500);
}