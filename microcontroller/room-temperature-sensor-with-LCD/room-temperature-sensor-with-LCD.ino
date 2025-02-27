#include <LiquidCrystal.h>
#include "DHT.h"
int sensor_delay = 5000;

// Inisialisasi LCD
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

// Inisialisasi sensor DHT11
#define DHTPIN 2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

void setup()
{
  lcd.begin(16, 2);
  dht.begin();

  lcd.setCursor(0, 0);
  lcd.print("Suhu: ");
  lcd.setCursor(0, 1);
  lcd.print("Kelembapan: ");

  Serial.begin(9600);
}

void loop()
{
  delay(sensor_delay); // Tunggu 2 detik antara pembacaan sensor

  // Membaca suhu dan kelembapan
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  // Cek apakah pembacaan gagal
  if (isnan(h) || isnan(t))
  {
    Serial.println("Gagal membaca dari sensor DHT!");
    return;
  }

  // Menampilkan suhu dan kelembapan di Serial Monitor
  Serial.print("Suhu: ");
  Serial.print(t);
  Serial.print(" *C ");
  Serial.print("Kelembapan: ");
  Serial.print(h);
  Serial.println(" %");

  // Menampilkan suhu di LCD
  lcd.setCursor(6, 0);
  lcd.print(t);
  lcd.print(" *C  ");

  // Menampilkan kelembapan di LCD
  lcd.setCursor(12, 1);
  lcd.print(h);
  lcd.print(" %  ");

  // Mengirim data ke Serial Plotter
  Serial.print("Temperature:");
  Serial.print(t);
  Serial.print(",");
  Serial.print("Humidity:");
  Serial.println(h);
}