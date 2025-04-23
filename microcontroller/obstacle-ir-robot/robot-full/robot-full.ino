#include <Servo.h>

#define TRIG 5  
#define ECHO 18  
#define SERVO_PIN 13  
#define IR_FRONT_LEFT  33  
#define IR_FRONT_RIGHT 32  
#define IR_BACK_LEFT   35  
#define IR_BACK_RIGHT  34  
#define MOTOR_LEFT_FWD  26  
#define MOTOR_LEFT_BWD  25  
#define MOTOR_RIGHT_FWD  27  
#define MOTOR_RIGHT_BWD  14  

Servo myServo;

void setup() {
  Serial.begin(115200);
	pinMode(TRIG, OUTPUT);
	pinMode(ECHO, INPUT);
	pinMode(IR_FRONT_LEFT, INPUT);
	pinMode(IR_FRONT_RIGHT, INPUT);
	pinMode(IR_BACK_LEFT, INPUT);
	pinMode(IR_BACK_RIGHT, INPUT);
	pinMode(MOTOR_LEFT_FWD, OUTPUT);
	pinMode(MOTOR_LEFT_BWD, OUTPUT);
	pinMode(MOTOR_RIGHT_FWD, OUTPUT);
	pinMode(MOTOR_RIGHT_BWD, OUTPUT);
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

void moveForward() {
	digitalWrite(MOTOR_LEFT_FWD, HIGH);
	digitalWrite(MOTOR_RIGHT_FWD, HIGH);
	digitalWrite(MOTOR_LEFT_BWD, LOW);
	digitalWrite(MOTOR_RIGHT_BWD, LOW);
}

void stopMotors() {
	digitalWrite(MOTOR_LEFT_FWD, LOW);
	digitalWrite(MOTOR_RIGHT_FWD, LOW);
	digitalWrite(MOTOR_LEFT_BWD, LOW);
	digitalWrite(MOTOR_RIGHT_BWD, LOW);
}

void turnLeft() {
	digitalWrite(MOTOR_LEFT_FWD, LOW);
	digitalWrite(MOTOR_RIGHT_FWD, HIGH);
	digitalWrite(MOTOR_LEFT_BWD, HIGH);
	digitalWrite(MOTOR_RIGHT_BWD, LOW);
}

void turnRight() {
	digitalWrite(MOTOR_LEFT_FWD, HIGH);
	digitalWrite(MOTOR_RIGHT_FWD, LOW);
	digitalWrite(MOTOR_LEFT_BWD, LOW);
	digitalWrite(MOTOR_RIGHT_BWD, HIGH);
}

void loop() {
  long frontDist = getDistance();
	int frontLeft = digitalRead(IR_FRONT_LEFT);
	int frontRight = digitalRead(IR_FRONT_RIGHT);

	if (frontDist > 20 && frontLeft == HIGH && frontRight == HIGH) {
    	moveForward();
	} else {
    	stopMotors();
    	delay(300);
    	turnRight();
    	delay(500);
	}
}
