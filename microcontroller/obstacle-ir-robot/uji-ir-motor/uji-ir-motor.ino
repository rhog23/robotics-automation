// SENSOR INFRAMERAH
#define IR_FRONT_LEFT  33  
#define IR_FRONT_RIGHT 32  
#define IR_BACK_LEFT   35  
#define IR_BACK_RIGHT  34

// MOTOR DRIVER
#define MOTOR_LEFT_FWD  26  
#define MOTOR_LEFT_BWD  25  
#define MOTOR_RIGHT_FWD  27  
#define MOTOR_RIGHT_BWD  14 

void setup() {
  Serial.begin(115200);

  // set inframerah
	pinMode(IR_FRONT_LEFT, INPUT);
	pinMode(IR_FRONT_RIGHT, INPUT);
	pinMode(IR_BACK_LEFT, INPUT);
	pinMode(IR_BACK_RIGHT, INPUT);

  // set motor
	pinMode(MOTOR_LEFT_FWD, OUTPUT);
	pinMode(MOTOR_LEFT_BWD, OUTPUT);
	pinMode(MOTOR_RIGHT_FWD, OUTPUT);
	pinMode(MOTOR_RIGHT_BWD, OUTPUT);
}

void loop() {
  int frontLeft = digitalRead(IR_FRONT_LEFT);
	int frontRight = digitalRead(IR_FRONT_RIGHT);
	int backLeft = digitalRead(IR_BACK_LEFT);
	int backRight = digitalRead(IR_BACK_RIGHT);

	if (frontLeft == LOW || frontRight == LOW) {
    	stopMotors();
    	delay(300);
    	reverseMotors();
    	delay(500);
    	turnRight();
    	delay(500);
	}
	else if (backLeft == LOW || backRight == LOW) {
    	stopMotors();
    	delay(300);
    	moveForward();
    	delay(500);
	}
	else {
    	moveForward();
	}
}

// fungsi untuk menggerakan mobil -> maju
void moveForward() {
	digitalWrite(MOTOR_LEFT_FWD, HIGH);
	digitalWrite(MOTOR_RIGHT_FWD, HIGH);
	digitalWrite(MOTOR_LEFT_BWD, LOW);
	digitalWrite(MOTOR_RIGHT_BWD, LOW);
}

// fungsi untuk menghentikan mobil
void stopMotors() {
	digitalWrite(MOTOR_LEFT_FWD, LOW);
	digitalWrite(MOTOR_RIGHT_FWD, LOW);
	digitalWrite(MOTOR_LEFT_BWD, LOW);
	digitalWrite(MOTOR_RIGHT_BWD, LOW);
}

// fungsi untuk menggerakan mobil -> mundur
void reverseMotors() {
	digitalWrite(MOTOR_LEFT_FWD, LOW);
	digitalWrite(MOTOR_RIGHT_FWD, LOW);
	digitalWrite(MOTOR_LEFT_BWD, HIGH);
	digitalWrite(MOTOR_RIGHT_BWD, HIGH);
}

// fungsi untuk belok kanan
void turnRight() {
	digitalWrite(MOTOR_LEFT_FWD, HIGH);
	digitalWrite(MOTOR_RIGHT_FWD, LOW);
	digitalWrite(MOTOR_LEFT_BWD, LOW);
	digitalWrite(MOTOR_RIGHT_BWD, HIGH);
}