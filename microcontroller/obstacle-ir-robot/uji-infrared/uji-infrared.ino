#define IR_FRONT_LEFT  33  
#define IR_FRONT_RIGHT 32  
#define IR_BACK_LEFT   35  
#define IR_BACK_RIGHT  34


void setup() {
  Serial.begin(115200);
	pinMode(IR_FRONT_LEFT, INPUT);
	pinMode(IR_FRONT_RIGHT, INPUT);
	pinMode(IR_BACK_LEFT, INPUT);
	pinMode(IR_BACK_RIGHT, INPUT);
}

void loop() {
  int frontLeft = digitalRead(IR_FRONT_LEFT);
	int frontRight = digitalRead(IR_FRONT_RIGHT);
	int backLeft = digitalRead(IR_BACK_LEFT);
	int backRight = digitalRead(IR_BACK_RIGHT);

	Serial.print("Front Left: "); Serial.print(frontLeft);
	Serial.print(" | Front Right: "); Serial.print(frontRight);
	Serial.print(" | Back Left: "); Serial.print(backLeft);
	Serial.print(" | Back Right: "); Serial.println(backRight);

	delay(500);
}
