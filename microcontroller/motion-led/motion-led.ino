#define LED_PIN 8
#define PIR_PIN 3

int pirState = LOW;

void setup()
{
    Serial.begin(9600);
    pinMode(LED_PIN, OUTPUT);
    pinMode(PIR_PIN, INPUT);
}

void loop()
{
    int pirValue = digitalRead(PIR_PIN);
    if (pirValue == HIGH && pirState == LOW)
    {
        digitalWrite(LED_PIN, HIGH);
        pirState = HIGH;
        Serial.println("Motion detected!");
        Serial.println("LED ON");
    }
    else if (pirValue == LOW && pirState == HIGH)
    {
        Serial.println("No motion detected!");
        Serial.println("LED OFF");
        digitalWrite(LED_PIN, LOW);
        pirState = LOW;
    }
    // delay(500);
}