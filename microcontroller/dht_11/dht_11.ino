#include <DHT.h>
#define DHT11_PIN  21 // ESP32 pin GPIO21 terhubung ke sensor DHT11

DHT dht11(DHT11_PIN, DHT11);

void setup() {
  Serial.begin(9600);
  dht11.begin(); // inisialisasi sensor DHT11
}

void loop() {
  // baca kelembaban
  float humi  = dht11.readHumidity();
  // baca temperatur Celsius
  float tempC = dht11.readTemperature();
  // baca temperatur Fahrenheit
  float tempF = dht11.readTemperature(true);

  // memeriksa apakah pembacaannya berhasil / tidak
  if ( isnan(tempC) || isnan(tempF) || isnan(humi)) {
    Serial.println("Failed to read from DHT11 sensor!");
  } else {
    Serial.print("Humidity: ");
    Serial.print(humi);
    Serial.print("%");

    Serial.print("  |  ");

    Serial.print("Temperature: ");
    Serial.print(tempC);
    Serial.print("°C  ~  ");
    Serial.print(tempF);
    Serial.println("°F");
  }

  // beri jeda 2 detik antar pembacaan
  delay(2000);
}