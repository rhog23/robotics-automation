#define SENSOR_ON_LINE  HIGH

#define ENA   5    // left motor speed  (PWM)
#define IN1   7    // left motor dir A
#define IN2   8    // left motor dir B
#define ENB   6    // right motor speed (PWM)
#define IN3   9    // right motor dir A
#define IN4   10   // right motor dir B

// ------------------------------------------------------------
// IR SENSOR PINS
// ------------------------------------------------------------
#define IR_LEFT   A0
#define IR_RIGHT  A1

// ------------------------------------------------------------
// SPEED TUNING (duty 0~255)
// ------------------------------------------------------------
#define BASE_SPEED     150   // straight line
#define TURN_OUTER     170   // soft turn: outer wheel
#define TURN_INNER      90   // soft turn: inner wheel (still forward)
#define SPIN_OUTER     160   // hard turn: outer wheel forward
#define SPIN_INNER     130   // hard turn: inner wheel REVERSE

#define MIN_DUTY        80

#define LEFT_TRIM        0
#define RIGHT_TRIM       0

#define KICK_DUTY      220
#define KICK_MS         25

#define HARD_TURN_MS   120

#define LOST_TIMEOUT_MS  0

#define START_DELAY_MS 1000

#define DEBUG            1

// ============================================================
// MOTOR LAYER
// ============================================================
struct Motor {
  uint8_t en, inA, inB;
  int trim;
  int8_t dir;                 // -1 reverse, 0 stopped, 1 forward
  unsigned long kickStart;
};

Motor leftMotor  = {ENA, IN1, IN2, LEFT_TRIM,  0, 0};
Motor rightMotor = {ENB, IN3, IN4, RIGHT_TRIM, 0, 0};

void setWheel(Motor &m, int speed) {
  int8_t dir = (speed > 0) - (speed < 0);

  if (dir == 0) {
    digitalWrite(m.inA, LOW);
    digitalWrite(m.inB, LOW);
    analogWrite(m.en, 255);
    m.dir = 0;
    return;
  }

  int duty = constrain(abs(speed) + m.trim, MIN_DUTY, 255);

  if (dir != m.dir) {
    analogWrite(m.en, 0);
    m.kickStart = millis();
    m.dir = dir;
  }
  if (millis() - m.kickStart < KICK_MS) {
    duty = max(duty, KICK_DUTY);
  }

  digitalWrite(m.inA, dir > 0 ? HIGH : LOW);
  digitalWrite(m.inB, dir > 0 ? LOW  : HIGH);
  analogWrite(m.en, duty);
}

void drive(int left, int right) {
  setWheel(leftMotor,  left);
  setWheel(rightMotor, right);
}

void steer(int8_t dir, bool hard) {
  int outer = hard ? SPIN_OUTER  : TURN_OUTER;
  int inner = hard ? -SPIN_INNER : TURN_INNER;
  if (dir < 0) drive(inner, outer);   // turning left: left wheel is inner
  else         drive(outer, inner);   // turning right: right wheel is inner
}

// ============================================================
// STATE
// ============================================================
enum State { ST_FORWARD, ST_LEFT, ST_RIGHT, ST_RECOVER };

State         state      = ST_FORWARD;
int8_t        lastDir    = 0;   // -1 left, 0 straight, 1 right
unsigned long stateSince = 0;

#if DEBUG
const char *stateName(State s) {
  switch (s) {
    case ST_FORWARD: return "FORWARD";
    case ST_LEFT:    return "LEFT";
    case ST_RIGHT:   return "RIGHT";
    default:         return "RECOVER";
  }
}
#endif

// ============================================================
// SETUP
// ============================================================
void setup() {
#if DEBUG
  Serial.begin(115200);
  Serial.println(F("[LINE FOLLOWER] Init"));
#endif

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(IR_LEFT,  INPUT);
  pinMode(IR_RIGHT, INPUT);

  drive(0, 0);
  delay(START_DELAY_MS);
  stateSince = millis();

#if DEBUG
  Serial.println(F("[LINE FOLLOWER] Go"));
#endif
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  bool L = (digitalRead(IR_LEFT)  == SENSOR_ON_LINE);
  bool R = (digitalRead(IR_RIGHT) == SENSOR_ON_LINE);

  State next;
  if      (!L && !R) next = ST_FORWARD;
  else if ( L && !R) next = ST_LEFT;
  else if (!L &&  R) next = ST_RIGHT;
  else               next = ST_RECOVER;

  unsigned long now = millis();
  if (next != state) {
    state = next;
    stateSince = now;
#if DEBUG
    Serial.print(F("L:")); Serial.print(L);
    Serial.print(F(" R:")); Serial.print(R);
    Serial.print(F(" -> ")); Serial.println(stateName(state));
#endif
  }
  unsigned long inState = now - stateSince;

  switch (state) {
    case ST_FORWARD:
      drive(BASE_SPEED, BASE_SPEED);
      lastDir = 0;
      break;

    case ST_LEFT:                       // line under LEFT sensor
      steer(-1, inState > HARD_TURN_MS);
      lastDir = -1;
      break;

    case ST_RIGHT:                      // line under RIGHT sensor
      steer(+1, inState > HARD_TURN_MS);
      lastDir = 1;
      break;

    case ST_RECOVER:                    // both on black
      if (LOST_TIMEOUT_MS > 0 && inState >= LOST_TIMEOUT_MS) {
        drive(0, 0);
      } else if (lastDir == 0) {
        drive(BASE_SPEED, BASE_SPEED);  // came straight in: crossing
      } else {
        steer(lastDir, true);           // sharp curve: keep spinning
      }
      break;
  }
}