#include <Servo.h>

#define TRIG 5  
#define ECHO 18  
#define SERVO_PIN 13  

#define MOTOR_LEFT_FWD  26  
#define MOTOR_LEFT_BWD  25  
#define MOTOR_RIGHT_FWD  27  
#define MOTOR_RIGHT_BWD  14  

Servo myServo;


void setup() {
  Serial.begin(115200);
	pinMode(TRIG, OUTPUT);
	pinMode(ECHO, INPUT);
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

void reverseMotors() {
	digitalWrite(MOTOR_LEFT_FWD, LOW);
	digitalWrite(MOTOR_RIGHT_FWD, LOW);
	digitalWrite(MOTOR_LEFT_BWD, HIGH);
	digitalWrite(MOTOR_RIGHT_BWD, HIGH);
}

void turnRight() {
	digitalWrite(MOTOR_LEFT_FWD, HIGH);
	digitalWrite(MOTOR_RIGHT_FWD, LOW);
	digitalWrite(MOTOR_LEFT_BWD, LOW);
	digitalWrite(MOTOR_RIGHT_BWD, HIGH);
}

void turnLeft() {
	digitalWrite(MOTOR_LEFT_FWD, LOW);
	digitalWrite(MOTOR_RIGHT_FWD, HIGH);
	digitalWrite(MOTOR_LEFT_BWD, HIGH);
	digitalWrite(MOTOR_RIGHT_BWD, LOW);
}

void loop() {
  long frontDist = getDistance();

	if (frontDist > 20) {  
    	moveForward();
	} else {  
    	stopMotors();
    	delay(300);

    	myServo.write(0);
    	delay(500);
    	long leftDist = getDistance();

    	myServo.write(180);
    	delay(500);
    	long rightDist = getDistance();

    	if (leftDist > rightDist) {
        	turnLeft();
    	} else {
        	turnRight();
    	}
    	delay(500);
	}
}
