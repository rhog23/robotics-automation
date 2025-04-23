#include <Servo.h>

#define TRIG 5  
#define ECHO 18  
#define SERVO_PIN 13  

Servo myServo;


void setup() {
  Serial.begin(115200);
	pinMode(TRIG, OUTPUT);
	pinMode(ECHO, INPUT);
	myServo.attach(SERVO_PIN);
}

long getDistance() {
	digitalWrite(TRIG, LOW);
	delayMicroseconds(2);
	digitalWrite(TRIG, HIGH);
	delayMicroseconds(10);
	digitalWrite(TRIG, LOW);

	long duration = pulseIn(ECHO, HIGH);
	return duration * 0.034 / 2;
}


void loop() {
  int positions[] = {0, 90, 180};  
	for (int i = 0; i < 3; i++) {
    	myServo.write(positions[i]);
    	delay(500);

    	long distance = getDistance();
    	Serial.print("Arah: ");
    	Serial.print(positions[i]);
    	Serial.print("°, Jarak: ");
    	Serial.print(distance);
    	Serial.println(" cm");

    	delay(1000);
	}

}
