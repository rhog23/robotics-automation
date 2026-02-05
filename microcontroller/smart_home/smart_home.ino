/* ================= BLYNK CONFIG ================= */
#define BLYNK_TEMPLATE_ID "TMPL6MxW7MPMX"
#define BLYNK_TEMPLATE_NAME "Multisensor ESP32"
#define BLYNK_AUTH_TOKEN "HPvcx8jOYJHJjQyC_vQZOL5vIT66yxWv"
#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <DHT.h>
#include <BlynkSimpleEsp32.h>

/* ================= WIFI CONFIG ================= */
char ssid[] = "LAP_RES";
char pass[] = "qwertyu888";

BlynkTimer timer;

/* ================= PIN CONFIG ================= */
#define AO_PIN 36  // Pin GPIO36 pada ESP32 terhubung ke pin AO (Analog Output) dari sensor MQ2
#define DHT11_PIN 23
#define PIR_PIN 25
#define LED_PIN 27  // LED 3.3V + resistor
#define BUZZER_PIN 17

/* ================= SENSOR ================= */
DHT dht11(DHT11_PIN, DHT11);
int gasThreshold = 2000;  // ambang untuk sensor gas, bisa disesuaikan dengan hasil eksperimen

void sendToBlynk() {
  int gasValue = analogRead(AO_PIN);
  float temperature = dht11.readTemperature();
  float humidity = dht11.readHumidity();

  Blynk.virtualWrite(V0, gasValue);
  Blynk.virtualWrite(V1, temperature);
  Blynk.virtualWrite(V2, humidity);

  Serial.print("Gas: ");
  Serial.print(gasValue);
  Serial.print(" | Temp: ");
  Serial.print(temperature);
  Serial.print(" | Hum: ");
  Serial.println(humidity);

  // GAS ALERT
  if (gasValue > gasThreshold) {
    digitalWrite(BUZZER_PIN, HIGH);
    Blynk.virtualWrite(V4, "GAS TERDETEKSI");
    Blynk.logEvent("gas_alert", "Kebocoran gas terdeteksi!");
  } else {
    digitalWrite(BUZZER_PIN, LOW);
    Blynk.virtualWrite(V4, "Aman");
  }
}

void checkPIR() {
  int pirState = digitalRead(PIR_PIN);

  if (pirState == HIGH) {
    digitalWrite(LED_PIN, HIGH);
    Blynk.virtualWrite(V3, "Gerakan!");
    Serial.println("PIR: Gerakan terdeteksi");
  } else {
    digitalWrite(LED_PIN, LOW);
    Blynk.virtualWrite(V3, "Tidak ada");
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(PIR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  dht11.begin();
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  timer.setInterval(2000L, sendToBlynk);
  timer.setInterval(500L, checkPIR);      
}

void loop() {
  // put your main code here, to run repeatedly:
  Blynk.run();
  timer.run();
}
