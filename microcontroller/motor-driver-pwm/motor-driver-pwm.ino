#include <driver/ledc.h>


// PWM Channels
const int pwmChannelA = 0;
const int pwmChannelB = 1;
const int pwmFreq = 5000;      // 5 KHz (bisa kamu sesuaikan)
const int pwmResolution = 8;   // 8-bit (0–255)

//pin roda
const int motorA_PWM = 26;
const int motorB_PWM = 27;
const int motorA_IN1 = 16;  // Motor kanan
const int motorA_IN2 = 4;
const int motorB_IN1 = 2;  // Motor kiri
const int motorB_IN2 = 15;

//pin sensor garis
const int line_sensor1 = 34; //paling kiri
const int line_sensor2 = 35;
const int line_sensor3 = 32;
const int line_sensor4 = 33;
const int line_sensor5 = 25;

void setup() {
  // put your setup code here, to run once:
  pinMode (motorA_IN1, OUTPUT);
  pinMode (motorA_IN2, OUTPUT);
  pinMode (motorB_IN1, OUTPUT);
  pinMode (motorB_IN2, OUTPUT);

  // Setup PWM
  ledcAttach(pwmChannelA, pwmFreq, pwmResolution);  // Set PWM untuk motor A
  ledcAttachPin(motorA_PWM, pwmChannelA);          // Attach pin PWM motor A

  ledcSetup(pwmChannelB, pwmFreq, pwmResolution);  // Set PWM untuk motor B
  ledcAttachPin(motorB_PWM, pwmChannelB);          // Attach pin PWM motor B


  pinMode (line_sensor1, INPUT);
  pinMode (line_sensor2, INPUT);
  pinMode (line_sensor3, INPUT);
  pinMode (line_sensor4, INPUT);
  pinMode (line_sensor5, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  int s1 = digitalRead(line_sensor1);
  int s2 = digitalRead(line_sensor2);
  int s3 = digitalRead(line_sensor3);
  int s4 = digitalRead(line_sensor4);
  int s5 = digitalRead(line_sensor5);


  if (s1 == LOW && s2 == HIGH && s3 == HIGH && s4 == HIGH && s5 == LOW) {
    // Maju
    Serial.println("Maju");
    move_motors(100,true, 100, true);
  } 
  else if (s1 == LOW && s2 == HIGH && s3 == HIGH && s4 == HIGH && s5 == HIGH) {
    // Belok kanan
    Serial.println("Belok Kanan");
    move_motors(100,false, 100, true);
  } 
  else if (s1 == HIGH && s2 == HIGH && s3 == HIGH && s4 == HIGH && s5 == LOW) {
    // Belok kiri
    Serial.println("Belok Kiri");
    move_motors(100,true, 100, false);
  } 
  else if (s1 == HIGH && s2 == HIGH && s3 == HIGH && s4 == HIGH && s5 == HIGH) {
    // Stop (semua sensor mendeteksi garis)
    Serial.println("berhenti, dipersimpangan");
    stop_motors();
  } 
  else if (s1 == LOW && s2 == LOW && s3 == LOW && s4 == LOW && s5 == LOW) {
    // Stop (semua sensor mendeteksi garis)
    Serial.println("berhenti, tidak ad garis");
    stop_motors();
  } 
  else {
    // Default: berhenti
    stop_motors();
  }

}

void stop_motors() {
  move_motors(0, true, 0, true);
}

void move_motors(int speedA, bool forwardA, int speedB, bool forwardB) {
  digitalWrite(motorA_IN1, forwardA ? HIGH : LOW);
  digitalWrite(motorA_IN2, forwardA ? LOW : HIGH);
  ledcWrite(pwmChannelA, speedA);

  digitalWrite(motorB_IN1, forwardB ? HIGH : LOW);                   
  digitalWrite(motorB_IN2, forwardB ? LOW : HIGH);
  ledcWrite(pwmChannelB, speedB);
}