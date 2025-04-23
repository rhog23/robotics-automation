#define TRIG 5  
#define ECHO 18

void setup() {
  Serial.begin(115200);
	pinMode(TRIG, OUTPUT);
	pinMode(ECHO, INPUT);
}

void loop() {
  digitalWrite(TRIG, LOW);
	delayMicroseconds(2);
	digitalWrite(TRIG, HIGH);
	delayMicroseconds(10);
	digitalWrite(TRIG, LOW);

	long duration = pulseIn(ECHO, HIGH);
	long distance = duration * 0.034 / 2;  

	Serial.print("Jarak: ");
	Serial.print(distance);
	Serial.println(" cm");

	delay(500);
}
