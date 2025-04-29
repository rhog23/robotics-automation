#define IN1 25
#define IN2 26
#define IN3 27
#define IN4 14
#define enable1Pin 2
#define enable2Pin 4

// frekuensi PWM
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 150;


void setup() {
  // put your setup code here, to run once:
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  pinMode(enable2Pin, OUTPUT);

  ledcAttachChannel(enable1Pin, freq, resolution, pwmChannel);
  ledcAttachChannel(enable2Pin, freq, resolution, pwmChannel);

  Serial.begin(115200);

  Serial.print("Testing DC Motor...");
}

void loop() {
  // Move the DC motor forward at max speed
  Serial.println("Moving Forward");
  // Move DC motor forward with increasing speed
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  while (dutyCycle <= 255){
    ledcWrite(enable1Pin, dutyCycle);   
    ledcWrite(enable2Pin, dutyCycle);   
    Serial.print("Forward with duty cycle: ");
    Serial.println(dutyCycle);
    dutyCycle = dutyCycle + 5;
    delay(500);
  }

  dutyCycle = 150;
}
