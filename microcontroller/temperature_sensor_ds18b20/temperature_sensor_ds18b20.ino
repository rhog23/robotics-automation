#include <OneWire.h> // pustaka untuk komunikasi 1-Wire
#include <DallasTemperature.h> // pustaka khusus untuk sensor DS18B20

#define SENSOR_PIN 17  // pin GPIO17 pada ESP32 terhubung ke pin DATA sensor DS18B20

OneWire oneWire(SENSOR_PIN);       
DallasTemperature DS18B20(&oneWire);

float tempC; // variabel untuk menyimpan suhu dalam derajat Celsius
float tempF; // variabel untuk menyimpan suhu dalam derajat Fahrenheit

void setup() {
  Serial.begin(9600); // memulai komunikasi serial dengan komputer
  DS18B20.begin();    // menginisialisasi sensor DS18B20
}

void loop() {
  DS18B20.requestTemperatures();       // meminta data suhu dari sensor
  tempC = DS18B20.getTempCByIndex(0);  // membaca suhu pertama (dalam °C)
  tempF = tempC * 9 / 5 + 32;          // mengubah suhu dari °C ke °F

  // menampilkan hasil ke Serial Monitor
  Serial.print("Suhu: ");
  Serial.print(tempC);
  Serial.print("°C");
  Serial.print("  ~  ");
  Serial.print(tempF);
  Serial.println("°F");

  delay(500); // jeda 0,5 detik sebelum membaca ulang
}
