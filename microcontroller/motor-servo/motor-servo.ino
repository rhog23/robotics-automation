#include <Servo.h>

// Servo objects
Servo servo1; // pin 3
Servo servo2; // pin 5
Servo servo3; // pin 6
Servo servo4; // pin 9

// Current positions
int angle1 = 0, angle2 = 0, angle3 = 0, angle4 = 0;

// --- Smooth synchronized movement ---
void moveSmooth(int t1, int t2, int t3, int t4, int stepDelay = 15) {
  int maxSteps = max(max(abs(t1 - angle1), abs(t2 - angle2)),
                     max(abs(t3 - angle3), abs(t4 - angle4)));

  for (int step = 0; step <= maxSteps; step++) {
    int a1 = angle1 + (t1 - angle1) * step / maxSteps;
    int a2 = angle2 + (t2 - angle2) * step / maxSteps;
    int a3 = angle3 + (t3 - angle3) * step / maxSteps;
    int a4 = angle4 + (t4 - angle4) * step / maxSteps;

    servo1.write(a1);
    servo2.write(a2);
    servo3.write(a3);
    servo4.write(a4);

    delay(stepDelay);
  }

  // Update current angles
  angle1 = t1;
  angle2 = t2;
  angle3 = t3;
  angle4 = t4;
}

void setup() {
  Serial.begin(9600);

  servo1.attach(3);
  servo2.attach(5);
  servo3.attach(6);
  servo4.attach(9);

  // Initial position
  servo1.write(angle1);
  servo2.write(angle2);
  servo3.write(angle3);
  servo4.write(angle4);

  Serial.println("=== Smooth 4-Servo Controller ===");
  Serial.println("Input: a b c d (0-180)");
  Serial.println("Example: 90 45 120 30");
}

void loop() {
  if (Serial.available() > 0) {

    int a = Serial.parseInt();
    int b = Serial.parseInt();
    int c = Serial.parseInt();
    int d = Serial.parseInt();

    if ((a >= 0 && a <= 180) &&
        (b >= 0 && b <= 180) &&
        (c >= 0 && c <= 180) &&
        (d >= 0 && d <= 180)) {

      Serial.println("Moving smoothly...");
      moveSmooth(a, b, c, d);

      Serial.print("Reached: ");
      Serial.print(a); Serial.print(", ");
      Serial.print(b); Serial.print(", ");
      Serial.print(c); Serial.print(", ");
      Serial.print(d); Serial.println(" degrees");

    } else {
      Serial.println("Invalid input! Use 0-180.");
    }

    // Clear buffer
    while (Serial.available()) Serial.read();
  }
}