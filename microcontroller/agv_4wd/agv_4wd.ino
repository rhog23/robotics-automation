#include <driver/ledc.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>

const char *ssid = "LAB TIF";
const char *password = "qwertyu8888";

WiFiServer server(80);        // Main communication port
WiFiServer statusServer(81);  // Status reporting port

// Status tracking
enum RobotState {
  IDLE,
  WAITING_FOR_COLOR,
  MOVING_TO_INTERSECTION,
  TURNING,
  GOING_TO_BASKET,
  DELIVERING,
  RETURNING
};

RobotState currentState = IDLE;
unsigned long lastMovementTime = 0;
unsigned long stateChangeTime = 0;
const unsigned long MOVEMENT_TIMEOUT = 1000; // 1 second without movement = idle

String receivedData = "";
bool DataReceived = false;
bool canReceiveData = true;
bool isTurning = false;
bool isBack = false;
bool outOfLine = false;
bool onLine = true;
int crossCount = 0;
bool onStartPoint = true;
bool goBasket = false;

// Improved motor control
const int ACCEL_STEP = 10;
const int ACCEL_DELAY = 5;
const int START_SPEED = 80;
const int MAX_SPEED = 200;
const int TURN_SPEED = 180;

int currentSpeedA = 0;
int currentSpeedB = 0;
bool isAccelerating = false;

Servo myservo;

// PWM configuration
const int pwmFreq = 5000;
const int pwmResolution = LEDC_TIMER_8_BIT;

// Motor pins
const int motorA_PWM = 22;
const int motorB_PWM = 23;
const ledc_channel_t pwmChannelA = LEDC_CHANNEL_0;
const ledc_channel_t pwmChannelB = LEDC_CHANNEL_1;

const int motorA_IN1 = 17;
const int motorA_IN2 = 16;
const int motorB_IN1 = 19;
const int motorB_IN2 = 18;

// Sensor pins
const int line_sensor1 = 27;
const int line_sensor2 = 26;
const int line_sensor3 = 25;
const int line_sensor4 = 33;
const int line_sensor5 = 32;
const int IR_SENSOR_PIN = 36;
const int buzzer = 14;

void setup() {
  delay(2000);
  Serial.begin(115200);
  
  // WiFi connection
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());
  
  server.begin();
  statusServer.begin();
  
  // Initialize pins
  setupMotorPins();
  setupSensorPins();
  
  myservo.attach(13);
  myservo.write(0); // Initial position
  
  setupPWM();
  
  currentState = WAITING_FOR_COLOR;
  stateChangeTime = millis();
  
  Serial.println("[INFO] System initialized, waiting for color detection...");
}

void loop() {
  // Handle status reporting
  handleStatusReporting();
  
  // Handle color data reception
  handleColorData();
  
  // Main state machine
  switch (currentState) {
    case WAITING_FOR_COLOR:
      handleWaitingForColor();
      break;
      
    case MOVING_TO_INTERSECTION:
      handleMovingToIntersection();
      break;
      
    case TURNING:
      handleTurning();
      break;
      
    case GOING_TO_BASKET:
      handleGoingToBasket();
      break;
      
    case DELIVERING:
      handleDelivering();
      break;
      
    case RETURNING:
      handleReturning();
      break;
      
    case IDLE:
      handleIdle();
      break;
  }
  
  // Update movement tracking
  updateMovementTracking();
}

void handleStatusReporting() {
  WiFiClient statusClient = statusServer.available();
  if (statusClient) {
    if (statusClient.connected()) {
      DynamicJsonDocument statusDoc(200);
      statusDoc["status"] = getStateString(currentState);
      statusDoc["timestamp"] = millis();
      statusDoc["crossCount"] = crossCount;
      statusDoc["canReceiveData"] = canReceiveData;
      
      String statusJson;
      serializeJson(statusDoc, statusJson);
      statusClient.println(statusJson);
    }
    statusClient.stop();
  }
}

String getStateString(RobotState state) {
  switch (state) {
    case IDLE: return "idle";
    case WAITING_FOR_COLOR: return "waiting";
    case MOVING_TO_INTERSECTION: return "moving";
    case TURNING: return "turning";
    case GOING_TO_BASKET: return "delivering";
    case DELIVERING: return "delivering";
    case RETURNING: return "returning";
    default: return "unknown";
  }
}

void handleColorData() {
  WiFiClient client = server.available();
  if (client && canReceiveData) {
    Serial.println("[INFO] Client connected for color data");
    while (client.connected()) {
      if (client.available()) {
        String rawData = client.readStringUntil('\n');
        
        // Try to parse JSON
        DynamicJsonDocument doc(200);
        DeserializationError error = deserializeJson(doc, rawData);
        
        if (!error) {
          receivedData = doc["color"].as<String>();
          DataReceived = true;
          canReceiveData = false;
          Serial.print("[INFO] Received color data: ");
          Serial.println(receivedData);
          
          changeState(MOVING_TO_INTERSECTION);
        } else {
          // Fallback for simple string data
          receivedData = rawData;
          receivedData.trim();
          DataReceived = true;
          canReceiveData = false;
          Serial.print("[INFO] Received simple data: ");
          Serial.println(receivedData);
          
          changeState(MOVING_TO_INTERSECTION);
        }
        break;
      }
    }
    client.stop();
  }
}

void handleWaitingForColor() {
  // Stay stationary and wait for color data
  stop_motors();
  
  // Blink LED or buzzer to indicate waiting
  if ((millis() - stateChangeTime) % 2000 < 100) {
    digitalWrite(buzzer, HIGH);
  } else {
    digitalWrite(buzzer, LOW);
  }
}

void handleMovingToIntersection() {
  if (!DataReceived || !isValidColor(receivedData)) {
    Serial.println("[WARNING] Invalid color data");
    changeState(WAITING_FOR_COLOR);
    canReceiveData = true;
    return;
  }
  
  int s1 = digitalRead(line_sensor1);
  int s2 = digitalRead(line_sensor2);
  int s3 = digitalRead(line_sensor3);
  int s4 = digitalRead(line_sensor4);
  int s5 = digitalRead(line_sensor5);
  
  // Check for intersection
  if (s1 == HIGH && s2 == HIGH && s3 == HIGH && s4 == HIGH && s5 == HIGH) {
    Serial.println("[INFO] Intersection detected");
    stop_motors();
    delay(500);
    
    crossCount++;
    if (shouldTurnAtIntersection()) {
      changeState(TURNING);
    } else {
      // Continue straight
      changeState(MOVING_TO_INTERSECTION);
    }
  } else {
    // Follow line
    logic_move(s1, s2, s3, s4, s5);
  }
}

void handleTurning() {
  int s1 = digitalRead(line_sensor1);
  int s2 = digitalRead(line_sensor2);
  int s3 = digitalRead(line_sensor3);
  int s4 = digitalRead(line_sensor4);
  int s5 = digitalRead(line_sensor5);
  
  // Check if turn is complete
  if (s1 == LOW && s5 == LOW && s2 == HIGH && s3 == HIGH && s4 == HIGH) {
    Serial.println("[INFO] Turn completed");
    stop_motors();
    delay(200);
    changeState(GOING_TO_BASKET);
    crossCount = 0; // Reset after successful turn
  } else {
    // Continue turning
    performTurn();
  }
}

void handleGoingToBasket() {
  int s1 = digitalRead(line_sensor1);
  int s2 = digitalRead(line_sensor2);
  int s3 = digitalRead(line_sensor3);
  int s4 = digitalRead(line_sensor4);
  int s5 = digitalRead(line_sensor5);
  
  // Check if reached basket (all sensors LOW)
  if (s1 == LOW && s2 == LOW && s3 == LOW && s4 == LOW && s5 == LOW) {
    Serial.println("[INFO] Reached basket");
    stop_motors();
    changeState(DELIVERING);
  } else {
    logic_move(s1, s2, s3, s4, s5);
  }
}

void handleDelivering() {
  Serial.println("[INFO] Delivering package");
  
  // Move servo to drop package
  for (int angle = 0; angle <= 90; angle += 2) {
    myservo.write(angle);
    delay(20);
  }
  
  delay(1000); // Wait for package to drop
  
  // Return servo to original position
  for (int angle = 90; angle >= 0; angle -= 2) {
    myservo.write(angle);
    delay(20);
  }
  
  Serial.println("[INFO] Package delivered, starting return journey");
  changeState(RETURNING);
}

void handleReturning() {
  back_logic();
  
  if (onStartPoint) {
    Serial.println("[INFO] Returned to start point");
    resetForNextDelivery();
    changeState(WAITING_FOR_COLOR);
  }
}

void handleIdle() {
  stop_motors();
  // System is idle, waiting for next command
}

bool shouldTurnAtIntersection() {
  if (crossCount == 1 && (receivedData == "me" || receivedData == "ji")) {
    return true;
  } else if (crossCount == 2 && (receivedData == "ku" || receivedData == "hi")) {
    return true;
  } else if (crossCount == 3 && (receivedData == "bi" || receivedData == "un")) {
    return true;
  }
  return false;
}

void performTurn() {
  if (receivedData == "me" || receivedData == "ku" || receivedData == "bi") {
    // Turn left
    smooth_turn(TURN_SPEED, false, TURN_SPEED + 20, true);
  } else if (receivedData == "ji" || receivedData == "hi" || receivedData == "un") {
    // Turn right  
    smooth_turn(TURN_SPEED + 20, true, TURN_SPEED, false);
  }
}

bool isValidColor(String color) {
  return (color == "me" || color == "ji" || color == "ku" || 
          color == "hi" || color == "bi" || color == "un");
}

void changeState(RobotState newState) {
  if (currentState != newState) {
    Serial.print("[INFO] State change: ");
    Serial.print(getStateString(currentState));
    Serial.print(" -> ");
    Serial.println(getStateString(newState));
    
    currentState = newState;
    stateChangeTime = millis();
  }
}

void updateMovementTracking() {
  // Check if motors are moving
  if (currentSpeedA > 0 || currentSpeedB > 0) {
    lastMovementTime = millis();
  }
  
  // Auto-transition to IDLE if no movement for timeout period
  if ((millis() - lastMovementTime) > MOVEMENT_TIMEOUT && 
      currentState != IDLE && currentState != WAITING_FOR_COLOR) {
    changeState(IDLE);
  }
}

void resetForNextDelivery() {
  receivedData = "";
  DataReceived = false;
  canReceiveData = true;
  isTurning = false;
  isBack = false;
  outOfLine = false;
  onLine = true;
  crossCount = 0;
  onStartPoint = true;
  goBasket = false;
}

// Motor control functions (improved versions of your original functions)
void setupMotorPins() {
  pinMode(motorA_IN1, OUTPUT);
  pinMode(motorA_IN2, OUTPUT);
  pinMode(motorB_IN1, OUTPUT);
  pinMode(motorB_IN2, OUTPUT);
}

void setupSensorPins() {
  pinMode(line_sensor1, INPUT);
  pinMode(line_sensor2, INPUT);
  pinMode(line_sensor3, INPUT);
  pinMode(line_sensor4, INPUT);
  pinMode(line_sensor5, INPUT);
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(buzzer, OUTPUT);
}

void setupPWM() {
  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = pwmFreq,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t ledc_channelA = {
    .gpio_num = motorA_PWM,
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .channel = pwmChannelA,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&ledc_channelA);

  ledc_channel_config_t ledc_channelB = {
    .gpio_num = motorB_PWM,
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .channel = pwmChannelB,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
    .hpoint = 0
  };
  ledc_channel_config(&ledc_channelB);
}

void smooth_turn(int speedA, bool forwardA, int speedB, bool forwardB) {
  int startSpeed = 80;
  int targetSpeedA = speedA;
  int targetSpeedB = speedB;
  
  for (int pwm = startSpeed; pwm <= max(targetSpeedA, targetSpeedB); pwm += ACCEL_STEP) {
    int currentA = min(pwm, targetSpeedA);
    int currentB = min(pwm, targetSpeedB);
    
    move_motors(currentA, forwardA, currentB, forwardB);
    delay(ACCEL_DELAY);
  }
  
  move_motors(targetSpeedA, forwardA, targetSpeedB, forwardB);
}

void smooth_forward(int targetSpeedA, int targetSpeedB) {
  if (currentSpeedA == 0 && currentSpeedB == 0) {
    for (int pwm = START_SPEED; pwm <= max(targetSpeedA, targetSpeedB); pwm += ACCEL_STEP) {
      int speedA = min(pwm, targetSpeedA);
      int speedB = min(pwm, targetSpeedB);
      
      move_motors(speedA, true, speedB, true);
      currentSpeedA = speedA;
      currentSpeedB = speedB;
      delay(ACCEL_DELAY);
    }
  }
  
  move_motors(targetSpeedA, true, targetSpeedB, true);
  currentSpeedA = targetSpeedA;
  currentSpeedB = targetSpeedB;
}

void stop_motors() {
  move_motors(0, true, 0, true);
  currentSpeedA = 0;
  currentSpeedB = 0;
}

void move_motors(int speedA, bool forwardA, int speedB, bool forwardB) {
  digitalWrite(motorA_IN1, forwardA ? HIGH : LOW);
  digitalWrite(motorA_IN2, forwardA ? LOW : HIGH);
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, pwmChannelA, speedA);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, pwmChannelA);

  digitalWrite(motorB_IN1, forwardB ? HIGH : LOW);
  digitalWrite(motorB_IN2, forwardB ? LOW : HIGH);
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, pwmChannelB, speedB);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, pwmChannelB);
}

void check_obstacle() {
  int adcValue = analogRead(IR_SENSOR_PIN);
  float voltage = adcValue * (3.3 / 4095.0);
  float distance = 61.573 * pow(voltage, -1.106) - 10;

  if (distance <= 30) {
    digitalWrite(buzzer, HIGH);
    stop_motors();
  } else {
    digitalWrite(buzzer, LOW);
  }
}

void logic_move(int s1, int s2, int s3, int s4, int s5) {
  check_obstacle();
  
  if (s1 == LOW && s2 == LOW && s3 == LOW && s4 == LOW && s5 == LOW) {
    stop_motors();
  } else if (s2 == LOW && s3 == LOW && s4 == HIGH) {
    smooth_forward(MAX_SPEED, 0);
  } else if (s2 == LOW && s3 == HIGH && s4 == LOW) {
    smooth_forward(MAX_SPEED, MAX_SPEED);
  } else if (s2 == LOW && s3 == HIGH && s4 == HIGH) {
    smooth_forward(MAX_SPEED, 150);
  } else if (s2 == HIGH && s3 == LOW && s4 == LOW) {
    smooth_forward(0, MAX_SPEED);
  } else if (s2 == HIGH && s3 == LOW && s4 == HIGH) {
    smooth_forward(MAX_SPEED, MAX_SPEED);
  } else if (s2 == HIGH && s3 == HIGH && s4 == LOW) {
    smooth_forward(150, MAX_SPEED);
  } else if (s2 == HIGH && s3 == HIGH && s4 == HIGH) {
    smooth_forward(MAX_SPEED, MAX_SPEED);
  } else if (s1 == HIGH && s2 == HIGH && s3 == LOW && s4 == LOW && s5 == LOW) {
    move_motors(MAX_SPEED, false, MAX_SPEED, true);
  } else if (s1 == LOW && s2 == LOW && s3 == LOW && s4 == HIGH && s5 == HIGH) {
    move_motors(MAX_SPEED, true, MAX_SPEED, false);
  } else {
    stop_motors();
  }
}

void back_logic() {
  Serial.println("[INFO] Starting return journey");
  changeState(RETURNING);

  // Turn around until all sensors are LOW
  while (!outOfLine) {
    move_motors(150, false, 150, false);
    delay(500);
    
    int s1 = digitalRead(line_sensor1);
    int s2 = digitalRead(line_sensor2);
    int s3 = digitalRead(line_sensor3);
    int s4 = digitalRead(line_sensor4);
    int s5 = digitalRead(line_sensor5);

    if (s1 == LOW && s2 == LOW && s3 == LOW && s4 == LOW && s5 == LOW) {
      outOfLine = true;
      onLine = false;
      stop_motors();
      delay(200);
      break;
    }

    move_motors(230, true, 255, false);
    delay(10);
  }

  // Continue turning until back on line
  while (!onLine) {
    int s2 = digitalRead(line_sensor2);
    int s3 = digitalRead(line_sensor3);
    int s4 = digitalRead(line_sensor4);

    if (s2 == HIGH && s3 == HIGH && s4 == HIGH) {
      onLine = true;
      onStartPoint = false;
      stop_motors();
      delay(200);
      break;
    }

    move_motors(230, true, 255, false);
    delay(10);
  }

  // Navigate back to start point
  while (!onStartPoint) {
    int s1 = digitalRead(line_sensor1);
    int s2 = digitalRead(line_sensor2);
    int s3 = digitalRead(line_sensor3);
    int s4 = digitalRead(line_sensor4);
    int s5 = digitalRead(line_sensor5);

    if (s5 == HIGH && s2 == HIGH && s3 == HIGH && s4 == HIGH && s1 == LOW) {
      // Turn left at intersection
      while (!(s2 == HIGH && s3 == HIGH && s4 == HIGH && s1 == LOW && s5 == LOW)) {
        move_motors(130, false, 230, true);
        delay(10);
        
        s1 = digitalRead(line_sensor1);
        s2 = digitalRead(line_sensor2);
        s3 = digitalRead(line_sensor3);
        s4 = digitalRead(line_sensor4);
        s5 = digitalRead(line_sensor5);
      }
      stop_motors();
      delay(200);

    } else if (s1 == HIGH && s2 == HIGH && s3 == HIGH && s4 == HIGH && s5 == LOW) {
      // Turn right at intersection
      while (!(s2 == HIGH && s3 == HIGH && s4 == HIGH)) {
        move_motors(230, true, 130, false);
        delay(10);
        
        s2 = digitalRead(line_sensor2);
        s3 = digitalRead(line_sensor3);
        s4 = digitalRead(line_sensor4);
      }
      stop_motors();
      delay(200);

    } else if (s1 == LOW && s2 == LOW && s3 == LOW && s4 == LOW && s5 == LOW) {
      // Reached start point
      Serial.println("[INFO] Reached start point");
      onStartPoint = true;
      isBack = false;
      outOfLine = false;
      onLine = true;
      stop_motors();
      return;
    }

    // Continue following line
    smooth_forward(MAX_SPEED, MAX_SPEED);
  }
}