#include <driver/ledc.h> // Library for LEDC (PWM) control on ESP32
#include <WiFi.h>        // Library for WiFi functionality
#include <ESP32Servo.h>  // Library for controlling servo motors
#include <ArduinoJson.h> // Library for handling JSON data

const char *ssid = "LAB TIF";         // WiFi network name (SSID)
const char *password = "qwertyu8888"; // WiFi network password

WiFiServer server(80);       // Creates a server on port 80 for main communication
WiFiServer statusServer(81); // Creates a server on port 81 for status reporting

enum RobotState // Defines possible states of the robot as an enumeration
{
  IDLE,                   // Robot is stopped and waiting
  WAITING_FOR_COLOR,      // Waiting for color data to start the task
  MOVING_TO_INTERSECTION, // Moving along the line to an intersection
  TURNING,                // Turning at an intersection
  GOING_TO_BASKET,        // Moving toward the delivery basket
  DELIVERING,             // Delivering the package
  RETURNING               // Returning to the starting point
};

RobotState currentState = IDLE;              // Initializes the robot's state to IDLE
unsigned long lastMovementTime = 0;          // Tracks the last time the robot moved (in milliseconds)
unsigned long stateChangeTime = 0;           // Tracks the time of the last state change (in milliseconds)
const unsigned long MOVEMENT_TIMEOUT = 1000; // Timeout (1 second) to revert to IDLE if no movement

String receivedData = "";   // Stores data received from the client (e.g., color info)
bool DataReceived = false;  // Flag indicating if data has been received
bool canReceiveData = true; // Flag indicating if the robot can accept new data
bool isTurning = false;     // Flag indicating if the robot is currently turning
bool isBack = false;        // Flag indicating if the robot is moving backward
bool outOfLine = false;     // Flag indicating if the robot has lost the line
bool onLine = true;         // Flag indicating if the robot is on the line
int crossCount = 0;         // Counts the number of intersections crossed
bool onStartPoint = true;   // Flag indicating if the robot is at the starting point
bool goBasket = false;      // Flag indicating if the robot should head to the basket

const int ACCEL_STEP = 10;  // Speed increment for smooth motor acceleration
const int ACCEL_DELAY = 5;  // Delay (in milliseconds) between acceleration steps
const int START_SPEED = 80; // Initial speed for motor movement
const int MAX_SPEED = 200;  // Maximum speed for motor movement
const int TURN_SPEED = 180; // Speed used during turns

int currentSpeedA = 0;       // Current speed of motor A
int currentSpeedB = 0;       // Current speed of motor B
bool isAccelerating = false; // Flag indicating if the motors are accelerating

Servo myservo; // Creates a Servo object for controlling the delivery mechanism

const int pwmFreq = 5000;                   // PWM frequency (in Hz) for motor control
const int pwmResolution = LEDC_TIMER_8_BIT; // PWM resolution (8-bit, 0-255 range)

const int motorA_PWM = 22;                         // Pin for PWM signal to motor A
const int motorB_PWM = 23;                         // Pin for PWM signal to motor B
const ledc_channel_t pwmChannelA = LEDC_CHANNEL_0; // PWM channel for motor A
const ledc_channel_t pwmChannelB = LEDC_CHANNEL_1; // PWM channel for motor B

const int motorA_IN1 = 17; // Pin for motor A direction control (forward)
const int motorA_IN2 = 16; // Pin for motor A direction control (reverse)
const int motorB_IN1 = 19; // Pin for motor B direction control (forward)
const int motorB_IN2 = 18; // Pin for motor B direction control (reverse)

const int line_sensor1 = 27;  // Pin for line sensor 1 (leftmost)
const int line_sensor2 = 26;  // Pin for line sensor 2
const int line_sensor3 = 25;  // Pin for line sensor 3 (center)
const int line_sensor4 = 33;  // Pin for line sensor 4
const int line_sensor5 = 32;  // Pin for line sensor 5 (rightmost)
const int IR_SENSOR_PIN = 36; // Pin for infrared sensor to detect obstacles
const int buzzer = 14;        // Pin for the buzzer for audible feedback

void setup() // Setup function runs once at startup
{
  delay(2000);          // Waits 2 seconds to allow hardware to stabilize
  Serial.begin(115200); // Starts serial communication at 115200 baud rate

  WiFi.begin(ssid, password);           // Initiates WiFi connection with the specified SSID and password
  while (WiFi.status() != WL_CONNECTED) // Loops until WiFi is connected
  {
    delay(500);        // Waits 500ms between connection attempts
    Serial.print("."); // Prints a dot to indicate connection progress
  }
  Serial.print("ESP32 IP Address: "); // Prints a label for the IP address
  Serial.println(WiFi.localIP());     // Prints the assigned IP address

  server.begin();       // Starts the main server on port 80
  statusServer.begin(); // Starts the status server on port 81

  setupMotorPins();  // Configures motor pins as outputs
  setupSensorPins(); // Configures sensor pins as inputs and buzzer as output

  myservo.attach(13); // Attaches the servo to pin 13
  myservo.write(0);   // Sets the servo to its initial position (0 degrees)

  setupPWM(); // Configures PWM settings for motor control

  currentState = WAITING_FOR_COLOR;                                            // Sets the initial state to wait for color data
  stateChangeTime = millis();                                                  // Records the time of the state change
  Serial.println("[INFO] System initialized, waiting for color detection..."); // Logs initialization
}

void loop() // Main loop runs continuously after setup
{
  handleStatusReporting(); // Sends status updates to clients on port 81
  handleColorData();       // Processes incoming color data from clients on port 80
  switch (currentState)    // Switches behavior based on the current state
  {
  case WAITING_FOR_COLOR:         // If waiting for color data
    handleWaitingForColor();      // Calls function to handle this state
    break;                        // Exits the switch case
  case MOVING_TO_INTERSECTION:    // If moving toward an intersection
    handleMovingToIntersection(); // Calls function to handle this state
    break;                        // Exits the switch case
  case TURNING:                   // If turning at an intersection
    handleTurning();              // Calls function to handle this state
    break;                        // Exits the switch case
  case GOING_TO_BASKET:           // If moving toward the basket
    handleGoingToBasket();        // Calls function to handle this state
    break;                        // Exits the switch case
  case DELIVERING:                // If delivering the package
    handleDelivering();           // Calls function to handle this state
    break;                        // Exits the switch case
  case RETURNING:                 // If returning to the start
    handleReturning();            // Calls function to handle this state
    break;                        // Exits the switch case
  case IDLE:                      // If idle
    handleIdle();                 // Calls function to handle this state
    break;                        // Exits the switch case
  }
  updateMovementTracking(); // Updates the movement timeout tracking
}

void handleStatusReporting() // Sends status updates to clients
{
  WiFiClient statusClient = statusServer.available(); // Checks for a client on port 81
  if (statusClient)                                   // If a client is connected
  {
    if (statusClient.connected()) // If the client connection is active
    {
      DynamicJsonDocument statusDoc(200);                 // Creates a JSON document for status
      statusDoc["status"] = getStateString(currentState); // Adds current state as a string
      statusDoc["timestamp"] = millis();                  // Adds current time
      statusDoc["crossCount"] = crossCount;               // Adds intersection count
      statusDoc["canReceiveData"] = canReceiveData;       // Adds data reception status
      String statusJson;                                  // String to hold serialized JSON
      serializeJson(statusDoc, statusJson);               // Serializes JSON document
      statusClient.println(statusJson);                   // Sends JSON to the client
    }
    statusClient.stop(); // Closes the client connection
  }
}

String getStateString(RobotState state) // Converts robot state to a string
{
  switch (state) // Switches based on the state
  {
  case IDLE:                   // IDLE state
    return "idle";             // Returns "idle" (corrected from "moving")
  case WAITING_FOR_COLOR:      // WAITING_FOR_COLOR state
    return "waiting";          // Returns "waiting"
  case MOVING_TO_INTERSECTION: // MOVING_TO_INTERSECTION state
    return "moving";           // Returns "moving"
  case TURNING:                // TURNING state
    return "turning";          // Returns "turning"
  case GOING_TO_BASKET:        // GOING_TO_BASKET state
    return "delivering";       // Returns "delivering"
  case DELIVERING:             // DELIVERING state
    return "delivering";       // Returns "delivering"
  case RETURNING:              // RETURNING state
    return "returning";        // Returns "returning"
  default:                     // Unknown state
    return "unknown";          // Returns "unknown"
  }
}

void handleColorData() // Processes incoming color data
{
  WiFiClient client = server.available(); // Checks for a client on port 80
  if (client && canReceiveData)           // If a client is connected and data can be received
  {
    Serial.println("[INFO] Client connected for color data"); // Logs client connection
    while (client.connected())                                // While the client is connected
    {
      if (client.available()) // If data is available from the client
      {
        String rawData = client.readStringUntil('\n');              // Reads data until newline
        DynamicJsonDocument doc(200);                               // Creates a JSON document for parsing
        DeserializationError error = deserializeJson(doc, rawData); // Attempts to parse JSON
        if (!error)                                                 // If JSON parsing succeeds
        {
          receivedData = doc["color"].as<String>();     // Extracts color data
          DataReceived = true;                          // Sets data received flag
          canReceiveData = false;                       // Prevents further data reception
          Serial.print("[INFO] Received color data: "); // Logs received data
          Serial.println(receivedData);                 // Prints the color data
          changeState(MOVING_TO_INTERSECTION);          // Transitions to MOVING_TO_INTERSECTION
        }
        else // If JSON parsing fails
        {
          receivedData = rawData;                        // Treats data as a simple string
          receivedData.trim();                           // Removes whitespace
          DataReceived = true;                           // Sets data received flag
          canReceiveData = false;                        // Prevents further data reception
          Serial.print("[INFO] Received simple data: "); // Logs received data
          Serial.println(receivedData);                  // Prints the data
          changeState(MOVING_TO_INTERSECTION);           // Transitions to MOVING_TO_INTERSECTION
        }
        break; // Exits the loop after processing data
      }
    }
    client.stop(); // Closes the client connection
  }
}

void handleWaitingForColor() // Handles the WAITING_FOR_COLOR state
{
  stop_motors();                                 // Stops all motors
  if ((millis() - stateChangeTime) % 2000 < 100) // Every 2 seconds, for 100ms
  {
    digitalWrite(buzzer, HIGH); // Activates buzzer to indicate waiting
  }
  else // For the remaining time
  {
    digitalWrite(buzzer, LOW); // Deactivates buzzer
  }
}

void handleMovingToIntersection() // Handles the MOVING_TO_INTERSECTION state
{
  if (!DataReceived || !isValidColor(receivedData)) // Checks for valid color data
  {
    Serial.println("[WARNING] Invalid color data"); // Logs warning
    changeState(WAITING_FOR_COLOR);                 // Reverts to WAITING_FOR_COLOR
    canReceiveData = true;                          // Allows new data reception
    return;                                         // Exits the function
  }
  int s1 = digitalRead(line_sensor1);                                     // Reads sensor 1 (leftmost)
  int s2 = digitalRead(line_sensor2);                                     // Reads sensor 2
  int s3 = digitalRead(line_sensor3);                                     // Reads sensor 3 (center)
  int s4 = digitalRead(line_sensor4);                                     // Reads sensor 4
  int s5 = digitalRead(line_sensor5);                                     // Reads sensor 5 (rightmost)
  if (s1 == HIGH && s2 == HIGH && s3 == HIGH && s4 == HIGH && s5 == HIGH) // Detects intersection
  {
    Serial.println("[INFO] Intersection detected"); // Logs intersection
    stop_motors();                                  // Stops motors
    delay(500);                                     // Waits 500ms to stabilize
    crossCount++;                                   // Increments intersection counter
    if (shouldTurnAtIntersection())                 // Checks if a turn is needed
    {
      changeState(TURNING); // Transitions to TURNING
    }
    else // If no turn is needed
    {
      changeState(MOVING_TO_INTERSECTION); // Continues moving
    }
  }
  else // If not at an intersection
  {
    logic_move(s1, s2, s3, s4, s5); // Follows the line
  }
}

void handleTurning() // Handles the TURNING state
{
  Serial.println("[INFO] Starting turn sequence");                          // Logs start of turn
  if (receivedData == "me" || receivedData == "ku" || receivedData == "bi") // Checks for left turn colors
  {
    turnUntilLineFound(true); // Executes left turn
  }
  else if (receivedData == "ji" || receivedData == "hi" || receivedData == "un") // Checks for right turn colors
  {
    turnUntilLineFound(false); // Executes right turn
  }
  Serial.println("[INFO] Turn completed"); // Logs turn completion
  stop_motors();                           // Stops motors
  delay(200);                              // Brief pause to stabilize
  changeState(DELIVERING);                 // Transitions to DELIVERING (modified to skip GOING_TO_BASKET)
  crossCount = 0;                          // Resets intersection counter
}

const int TURN_SPEED_OUTER = 200;               // Reduced outer wheel speed for turning (from 200)
const int TURN_SPEED_INNER = 150;               // Reduced inner wheel speed for turning (from 150)
const int STRAIGHT_LINE_THRESHOLD = 5;          // Number of consecutive (01110) detections required
const unsigned long MAX_TURN_TIME = 10000;      // Maximum turn time (10 seconds)
const unsigned long SENSOR_CHECK_INTERVAL = 10; // Sensor check interval (10ms)

void turnUntilLineFound(bool turnLeft) // Executes a turn until the line is found
{
  unsigned long turnStartTime = millis();      // Records start time of turn
  unsigned long lastSensorCheck = 0;           // Tracks last sensor check time
  int straightLineCount = 0;                   // Counts consecutive straight line detections
  Serial.print("[INFO] Turning ");             // Logs turn direction
  Serial.println(turnLeft ? "LEFT" : "RIGHT"); // Specifies left or right turn
  while (true)                                 // Continues until line is found or timeout
  {
    if (turnLeft) // If turning left
    {
      move_motors(TURN_SPEED_OUTER, true, TURN_SPEED_INNER, false); // Left turn: outer forward, inner reverse
    }
    else // If turning right
    {
      move_motors(TURN_SPEED_INNER, false, TURN_SPEED_OUTER, true); // Right turn: inner reverse, outer forward
    }
    if (millis() - lastSensorCheck >= SENSOR_CHECK_INTERVAL) // Checks sensors every 10ms
    {
      lastSensorCheck = millis(); // Updates last check time
      if (isOnStraightLine())     // If straight line pattern (01110) is detected
      {
        straightLineCount++;                                                                   // Increments detection counter
        Serial.println("[DEBUG] Straight line detected, count: " + String(straightLineCount)); // Logs detection
        if (straightLineCount >= STRAIGHT_LINE_THRESHOLD)                                      // If threshold reached
        {
          Serial.println("[INFO] Straight line confirmed!"); // Logs confirmation
          stop_motors();                                     // Stops motors
          delay(100);                                        // Brief pause to stabilize
          return;                                            // Exits function
        }
      }
      else // If not on straight line
      {
        straightLineCount = 0; // Resets counter
      }
      if ((millis() - turnStartTime) % 100 == 0) // Logs sensors every 100ms
      {
        int s1 = digitalRead(line_sensor1); // Reads sensor 1
        int s2 = digitalRead(line_sensor2); // Reads sensor 2
        int s3 = digitalRead(line_sensor3); // Reads sensor 3
        int s4 = digitalRead(line_sensor4); // Reads sensor 4
        int s5 = digitalRead(line_sensor5); // Reads sensor 5
        Serial.print("[DEBUG] Sensors: ");  // Logs sensor readings
        Serial.print(s1);                   // Prints sensor 1
        Serial.print(" ");                  // Space separator
        Serial.print(s2);                   // Prints sensor 2
        Serial.print(" ");                  // Space separator
        Serial.print(s3);                   // Prints sensor 3
        Serial.print(" ");                  // Space separator
        Serial.print(s4);                   // Prints sensor 4
        Serial.print(" ");                  // Space separator
        Serial.println(s5);                 // Prints sensor 5
      }
    }
    if (millis() - turnStartTime > MAX_TURN_TIME) // Checks for turn timeout
    {
      Serial.println("[ERROR] Turn timeout - line not found"); // Logs timeout error
      stop_motors();                                           // Stops motors
      changeState(IDLE);                                       // Transitions to IDLE
      return;                                                  // Exits function
    }
    delay(1); // Small delay to prevent overwhelming CPU
  }
}

bool isOnStraightLine() // Checks if robot is centered on the line
{
  int s1 = digitalRead(line_sensor1);                                        // Reads sensor 1 (leftmost)
  int s2 = digitalRead(line_sensor2);                                        // Reads sensor 2
  int s3 = digitalRead(line_sensor3);                                        // Reads sensor 3 (center)
  int s4 = digitalRead(line_sensor4);                                        // Reads sensor 4
  int s5 = digitalRead(line_sensor5);                                        // Reads sensor 5 (rightmost)
  return (s2 == HIGH && s3 == HIGH && s4 == HIGH && s1 == LOW && s5 == LOW); // Returns true for (01110)
}

void handleGoingToBasket() // Handles the GOING_TO_BASKET state
{
  int s1 = digitalRead(line_sensor1);                                // Reads sensor 1
  int s2 = digitalRead(line_sensor2);                                // Reads sensor 2
  int s3 = digitalRead(line_sensor3);                                // Reads sensor 3
  int s4 = digitalRead(line_sensor4);                                // Reads sensor 4
  int s5 = digitalRead(line_sensor5);                                // Reads sensor 5
  if (s1 == LOW && s2 == LOW && s3 == LOW && s4 == LOW && s5 == LOW) // If all sensors off (basket reached)
  {
    Serial.println("[INFO] Reached basket"); // Logs basket reached
    stop_motors();                           // Stops motors
    changeState(DELIVERING);                 // Transitions to DELIVERING
  }
  else // If still on line
  {
    logic_move(s1, s2, s3, s4, s5); // Follows the line
  }
}

void handleDelivering() // Handles the DELIVERING state
{
  Serial.println("[INFO] Delivering package"); // Logs delivery start
  for (int angle = 0; angle <= 90; angle += 2) // Gradually moves servo to 90 degrees
  {
    myservo.write(angle); // Sets servo position
    delay(20);            // Waits 20ms per step
  }
  delay(3000);                                 // Waits 3 seconds for package drop
  for (int angle = 90; angle >= 0; angle -= 2) // Returns servo to 0 degrees
  {
    myservo.write(angle); // Sets servo position
    delay(20);            // Waits 20ms per step
  }
  move_motors(150, false, 150, false);                                 // Moves backward at speed 150
  delay(500);                                                          // Moves back for 500ms
  stop_motors();                                                       // Stops motors
  delay(200);                                                          // Brief pause to stabilize
  Serial.println("[INFO] Package delivered, starting return journey"); // Logs delivery completion
  changeState(RETURNING);                                              // Transitions to RETURNING
}

void handleReturning() // Handles the RETURNING state
{
  back_logic();     // Executes return logic
  if (onStartPoint) // If start point reached
  {
    Serial.println("[INFO] Returned to start point"); // Logs return
    resetForNextDelivery();                           // Resets variables for next cycle
    changeState(WAITING_FOR_COLOR);                   // Transitions to WAITING_FOR_COLOR
  }
}

void handleIdle() // Handles the IDLE state
{
  stop_motors(); // Stops all motors
}

bool shouldTurnAtIntersection() // Determines if a turn is needed at the current intersection
{
  if (crossCount == 1 && (receivedData == "me" || receivedData == "ji")) // Turn at first intersection for "me" or "ji"
  {
    return true; // Indicates turn needed
  }
  else if (crossCount == 2 && (receivedData == "ku" || receivedData == "hi")) // Turn at second intersection for "ku" or "hi"
  {
    return true; // Indicates turn needed
  }
  else if (crossCount == 3 && (receivedData == "bi" || receivedData == "un")) // Turn at third intersection for "bi" or "un"
  {
    return true; // Indicates turn needed
  }
  return false; // No turn needed
}

bool isValidColor(String color) // Validates received color data
{
  return (color == "me" || color == "ji" || color == "ku" || // Checks valid colors
          color == "hi" || color == "bi" || color == "un");  // Returns true if valid
}

void changeState(RobotState newState) // Changes the robot's state
{
  if (currentState != newState) // If state is different
  {
    Serial.print("[INFO] State change: ");      // Logs state change
    Serial.print(getStateString(currentState)); // Prints current state
    Serial.print(" -> ");                       // Separator
    Serial.println(getStateString(newState));   // Prints new state
    currentState = newState;                    // Updates current state
    stateChangeTime = millis();                 // Records time of change
  }
}

void updateMovementTracking() // Tracks movement to detect idle conditions
{
  if (currentSpeedA > 0 || currentSpeedB > 0) // If motors are moving
  {
    lastMovementTime = millis(); // Updates last movement time
  }
  if ((millis() - lastMovementTime) > MOVEMENT_TIMEOUT &&        // If no movement for 1 second
      currentState != IDLE && currentState != WAITING_FOR_COLOR) // And not in IDLE or WAITING
  {
    changeState(IDLE); // Transitions to IDLE
  }
}

void resetForNextDelivery() // Resets variables for the next delivery cycle
{
  receivedData = "";     // Clears received data
  DataReceived = false;  // Resets data received flag
  canReceiveData = true; // Allows new data reception
  isTurning = false;     // Resets turning flag
  isBack = false;        // Resets backward flag
  outOfLine = false;     // Resets off-line flag
  onLine = true;         // Sets on-line flag
  crossCount = 0;        // Resets intersection counter
  onStartPoint = true;   // Sets start point flag
  goBasket = false;      // Resets basket flag
}

void setupMotorPins() // Configures motor control pins
{
  pinMode(motorA_IN1, OUTPUT); // Sets motor A forward pin as output
  pinMode(motorA_IN2, OUTPUT); // Sets motor A reverse pin as output
  pinMode(motorB_IN1, OUTPUT); // Sets motor B forward pin as output
  pinMode(motorB_IN2, OUTPUT); // Sets motor B reverse pin as output
}

void setupSensorPins() // Configures sensor and buzzer pins
{
  pinMode(line_sensor1, INPUT);  // Sets sensor 1 as input
  pinMode(line_sensor2, INPUT);  // Sets sensor 2 as input
  pinMode(line_sensor3, INPUT);  // Sets sensor 3 as input
  pinMode(line_sensor4, INPUT);  // Sets sensor 4 as input
  pinMode(line_sensor5, INPUT);  // Sets sensor 5 as input
  pinMode(IR_SENSOR_PIN, INPUT); // Sets IR sensor as input
  pinMode(buzzer, OUTPUT);       // Sets buzzer as output
}

void setupPWM() // Configures PWM for motor control
{
  ledc_timer_config_t ledc_timer = {
      // Defines PWM timer configuration
      .speed_mode = LEDC_HIGH_SPEED_MODE,  // High-speed mode
      .duty_resolution = LEDC_TIMER_8_BIT, // 8-bit resolution
      .timer_num = LEDC_TIMER_0,           // Timer 0
      .freq_hz = pwmFreq,                  // 5kHz frequency
      .clk_cfg = LEDC_AUTO_CLK             // Automatic clock selection
  };
  ledc_timer_config(&ledc_timer); // Applies timer configuration

  ledc_channel_config_t ledc_channelA = {
      // Defines PWM channel A configuration
      .gpio_num = motorA_PWM,             // Motor A PWM pin
      .speed_mode = LEDC_HIGH_SPEED_MODE, // High-speed mode
      .channel = pwmChannelA,             // Channel 0
      .intr_type = LEDC_INTR_DISABLE,     // No interrupts
      .timer_sel = LEDC_TIMER_0,          // Uses timer 0
      .duty = 0,                          // Initial duty cycle 0
      .hpoint = 0                         // High point 0
  };
  ledc_channel_config(&ledc_channelA); // Applies channel A configuration

  ledc_channel_config_t ledc_channelB = {
      // Defines PWM channel B configuration
      .gpio_num = motorB_PWM,             // Motor B PWM pin
      .speed_mode = LEDC_HIGH_SPEED_MODE, // High-speed mode
      .channel = pwmChannelB,             // Channel 1
      .intr_type = LEDC_INTR_DISABLE,     // No interrupts
      .timer_sel = LEDC_TIMER_0,          // Uses timer 0
      .duty = 0,                          // Initial duty cycle 0
      .hpoint = 0                         // High point 0
  };
  ledc_channel_config(&ledc_channelB); // Applies channel B configuration
}

void smooth_turn(int speedA, bool forwardA, int speedB, bool forwardB) // Smoothly accelerates for turns
{
  int startSpeed = 80;                                                                  // Starting speed for acceleration
  int targetSpeedA = speedA;                                                            // Target speed for motor A
  int targetSpeedB = speedB;                                                            // Target speed for motor B
  for (int pwm = startSpeed; pwm <= max(targetSpeedA, targetSpeedB); pwm += ACCEL_STEP) // Ramps up speed
  {
    int currentA = min(pwm, targetSpeedA);               // Limits speed A to target
    int currentB = min(pwm, targetSpeedB);               // Limits speed B to target
    move_motors(currentA, forwardA, currentB, forwardB); // Applies motor speeds
    delay(ACCEL_DELAY);                                  // Waits between steps
  }
  move_motors(targetSpeedA, forwardA, targetSpeedB, forwardB); // Sets final speeds
}

void smooth_forward(int targetSpeedA, int targetSpeedB) // Smoothly accelerates forward
{
  if (currentSpeedA == 0 && currentSpeedB == 0) // If motors are stopped
  {
    for (int pwm = START_SPEED; pwm <= max(targetSpeedA, targetSpeedB); pwm += ACCEL_STEP) // Ramps up speed
    {
      int speedA = min(pwm, targetSpeedA);     // Limits speed A
      int speedB = min(pwm, targetSpeedB);     // Limits speed B
      move_motors(speedA, true, speedB, true); // Moves forward
      currentSpeedA = speedA;                  // Updates current speed A
      currentSpeedB = speedB;                  // Updates current speed B
      delay(ACCEL_DELAY);                      // Waits between steps
    }
  }
  move_motors(targetSpeedA, true, targetSpeedB, true); // Sets final forward speeds
  currentSpeedA = targetSpeedA;                        // Updates current speed A
  currentSpeedB = targetSpeedB;                        // Updates current speed B
}

void stop_motors() // Stops all motors
{
  move_motors(0, true, 0, true); // Sets speed to 0
  currentSpeedA = 0;             // Resets speed A
  currentSpeedB = 0;             // Resets speed B
}

void move_motors(int speedA, bool forwardA, int speedB, bool forwardB) // Controls motor speed and direction
{
  digitalWrite(motorA_IN1, forwardA ? HIGH : LOW);          // Sets motor A forward direction
  digitalWrite(motorA_IN2, forwardA ? LOW : HIGH);          // Sets motor A reverse direction
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, pwmChannelA, speedA); // Sets motor A PWM duty
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, pwmChannelA);      // Updates motor A PWM
  digitalWrite(motorB_IN1, forwardB ? HIGH : LOW);          // Sets motor B forward direction
  digitalWrite(motorB_IN2, forwardB ? LOW : HIGH);          // Sets motor B reverse direction
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, pwmChannelB, speedB); // Sets motor B PWM duty
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, pwmChannelB);      // Updates motor B PWM
}

void check_obstacle() // Checks for obstacles using IR sensor
{
  int adcValue = analogRead(IR_SENSOR_PIN);            // Reads IR sensor value
  float voltage = adcValue * (3.3 / 4095.0);           // Converts to voltage
  float distance = 61.573 * pow(voltage, -1.106) - 10; // Calculates distance
  if (distance <= 30)                                  // If obstacle within 30cm
  {
    digitalWrite(buzzer, HIGH); // Activates buzzer
    stop_motors();              // Stops motors
  }
  else // If no obstacle
  {
    digitalWrite(buzzer, LOW); // Deactivates buzzer
  }
}

void logic_move(int s1, int s2, int s3, int s4, int s5) // Line-following logic
{
  check_obstacle();                                                  // Checks for obstacles before moving
  if (s1 == LOW && s2 == LOW && s3 == LOW && s4 == LOW && s5 == LOW) // All sensors off line
  {
    stop_motors(); // Stops motors
  }
  else if (s2 == LOW && s3 == LOW && s4 == HIGH) // Right sensor on line
  {
    smooth_forward(MAX_SPEED, 0); // Turns left by stopping right motor
  }
  else if (s2 == LOW && s3 == HIGH && s4 == LOW) // Center sensor on line
  {
    smooth_forward(MAX_SPEED, MAX_SPEED); // Moves straight
  }
  else if (s2 == LOW && s3 == HIGH && s4 == HIGH) // Center and right sensors on line
  {
    smooth_forward(MAX_SPEED, 150); // Adjusts left slightly
  }
  else if (s2 == HIGH && s3 == LOW && s4 == LOW) // Left sensor on line
  {
    smooth_forward(0, MAX_SPEED); // Turns right by stopping left motor
  }
  else if (s2 == HIGH && s3 == LOW && s4 == HIGH) // Left and right sensors on line
  {
    smooth_forward(MAX_SPEED, MAX_SPEED); // Moves straight
  }
  else if (s2 == HIGH && s3 == HIGH && s4 == LOW) // Left and center sensors on line
  {
    smooth_forward(150, MAX_SPEED); // Adjusts right slightly
  }
  else if (s2 == HIGH && s3 == HIGH && s4 == HIGH) // Middle three sensors on line
  {
    smooth_forward(MAX_SPEED, MAX_SPEED); // Moves straight
  }
  else if (s1 == HIGH && s2 == HIGH && s3 == LOW && s4 == LOW && s5 == LOW) // Left sensors on line
  {
    move_motors(MAX_SPEED, false, MAX_SPEED, true); // Sharp left turn
  }
  else if (s1 == LOW && s2 == LOW && s3 == LOW && s4 == HIGH && s5 == HIGH) // Right sensors on line
  {
    move_motors(MAX_SPEED, true, MAX_SPEED, false); // Sharp right turn
  }
  else // Unknown sensor pattern
  {
    stop_motors(); // Stops motors
  }
}

void back_logic() // Handles return journey to start point
{
  Serial.println("[INFO] Starting return journey"); // Logs return start
  const int ALIGN_THRESHOLD = 5;                    // Number of consecutive line detections for alignment
  int alignCount = 0;                               // Counts consecutive line detections

  while (!outOfLine) // Until robot is off the line
  {
    move_motors(150, false, 150, false);                               // Moves backward
    delay(500);                                                        // Moves back for 500ms
    int s1 = digitalRead(line_sensor1);                                // Reads sensor 1
    int s2 = digitalRead(line_sensor2);                                // Reads sensor 2
    int s3 = digitalRead(line_sensor3);                                // Reads sensor 3
    int s4 = digitalRead(line_sensor4);                                // Reads sensor 4
    int s5 = digitalRead(line_sensor5);                                // Reads sensor 5
    if (s1 == LOW && s2 == LOW && s3 == LOW && s4 == LOW && s5 == LOW) // All sensors off
    {
      outOfLine = true; // Sets off-line flag
      onLine = false;   // Clears on-line flag
      stop_motors();    // Stops motors
      delay(200);       // Brief pause
      break;            // Exits loop
    }
    move_motors(230, true, 255, false); // Turns to find line
    delay(10);                          // Short delay per iteration
  }

  while (!onLine) // Until robot is back on line
  {
    int s2 = digitalRead(line_sensor2);         // Reads sensor 2
    int s3 = digitalRead(line_sensor3);         // Reads sensor 3
    int s4 = digitalRead(line_sensor4);         // Reads sensor 4
    if (s2 == HIGH && s3 == HIGH && s4 == HIGH) // Middle sensors on line
    {
      alignCount++;                      // Increments alignment counter
      if (alignCount >= ALIGN_THRESHOLD) // If threshold reached
      {
        onLine = true;        // Sets on-line flag
        onStartPoint = false; // Clears start point flag
        stop_motors();        // Stops motors
        delay(200);           // Brief pause
        break;                // Exits loop
      }
    }
    else // If not aligned
    {
      alignCount = 0; // Resets counter
    }
    move_motors(230, true, 255, false); // Continues turning
    delay(10);                          // Short delay per iteration
  }

  while (!onStartPoint) // Until start point is reached
  {
    int s1 = digitalRead(line_sensor1);                                    // Reads sensor 1
    int s2 = digitalRead(line_sensor2);                                    // Reads sensor 2
    int s3 = digitalRead(line_sensor3);                                    // Reads sensor 3
    int s4 = digitalRead(line_sensor4);                                    // Reads sensor 4
    int s5 = digitalRead(line_sensor5);                                    // Reads sensor 5
    if (s5 == HIGH && s2 == HIGH && s3 == HIGH && s4 == HIGH && s1 == LOW) // Left intersection
    {
      while (!(s2 == HIGH && s3 == HIGH && s4 == HIGH && s1 == LOW && s5 == LOW)) // Until aligned
      {
        move_motors(130, false, 230, true); // Turns left
        delay(10);                          // Short delay
        s1 = digitalRead(line_sensor1);     // Updates sensor 1
        s2 = digitalRead(line_sensor2);     // Updates sensor 2
        s3 = digitalRead(line_sensor3);     // Updates sensor 3
        s4 = digitalRead(line_sensor4);     // Updates sensor 4
        s5 = digitalRead(line_sensor5);     // Updates sensor 5
      }
      stop_motors(); // Stops motors
      delay(200);    // Brief pause
    }
    else if (s1 == HIGH && s2 == HIGH && s3 == HIGH && s4 == HIGH && s5 == LOW) // Right intersection
    {
      while (!(s2 == HIGH && s3 == HIGH && s4 == HIGH)) // Until aligned
      {
        move_motors(230, true, 130, false); // Turns right
        delay(10);                          // Short delay
        s2 = digitalRead(line_sensor2);     // Updates sensor 2
        s3 = digitalRead(line_sensor3);     // Updates sensor 3
        s4 = digitalRead(line_sensor4);     // Updates sensor 4
      }
      stop_motors(); // Stops motors
      delay(200);    // Brief pause
    }
    else if (s1 == LOW && s2 == LOW && s3 == LOW && s4 == LOW && s5 == LOW) // Start point reached
    {
      Serial.println("[INFO] Reached start point"); // Logs arrival
      onStartPoint = true;                          // Sets start point flag
      isBack = false;                               // Clears backward flag
      outOfLine = false;                            // Clears off-line flag
      onLine = true;                                // Sets on-line flag
      stop_motors();                                // Stops motors
      return;                                       // Exits function
    }
    smooth_forward(MAX_SPEED, MAX_SPEED); // Continues moving forward
  }
}