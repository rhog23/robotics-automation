#include <DHT.h>                // pustaka untuk sensor DHT11
#include <LiquidCrystal_I2C.h>  // pustaka untuk LCD I2C

#define DHT11_PIN 23  // pin GPIO23 pada ESP32 dihubungkan ke pin DATA sensor DHT11

// Membuat objek LCD dengan alamat I2C 0x27, ukuran 16 kolom x 2 baris
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Membuat objek sensor DHT11
DHT dht11(DHT11_PIN, DHT11);

void setup() {
  dht11.begin();   // inisialisasi sensor DHT11
  lcd.init();      // inisialisasi LCD
  lcd.backlight(); // menyalakan lampu belakang LCD
}

void loop() {
  // Membaca data dari sensor
  float humi  = dht11.readHumidity();    // membaca kelembapan udara (%)
  float tempC = dht11.readTemperature(); // membaca suhu udara dalam °C

  lcd.clear(); // membersihkan tampilan LCD setiap kali update

  // Mengecek apakah pembacaan data berhasil
  if (isnan(tempC) || isnan(humi)) {
    lcd.setCursor(0, 0); // posisi baris 1, kolom 1
    lcd.print("Gagal Baca!");
  } else {
    // Menampilkan suhu
    lcd.setCursor(0, 0);   // baris pertama
    lcd.print("Temp: ");
    lcd.print(tempC);
    lcd.print("°C");

    // Menampilkan kelembapan
    lcd.setCursor(0, 1);   // baris kedua
    lcd.print("Humi: ");
    lcd.print(humi);
    lcd.print("%");
  }

  delay(2000); // jeda 2 detik sebelum pembacaan berikutnya
}
