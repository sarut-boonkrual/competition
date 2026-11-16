// ---------- Pin ----------
const int L_PWM = 5, L_IN1 = 4, L_IN2 = 7;
const int R_PWM = 6, R_IN1 = 8, R_IN2 = 9;

const int TRIG_F = 10, ECHO_F = 11;
const int TRIG_L = 12, ECHO_L = 13;
const int TRIG_R = A1, ECHO_R = A2;
const int TAPE_PIN = A0;

// ---------- Calibration ----------
const int SPEED         = 150;
const int MS_PER_BLOCK  = 700;   // Time to move 1 square (20 cm)
const int MS_TURN_90    = 420;
const int WALL_CM       = 15;    // closer than this = there's a wall
const int FRONT_STOP_CM = 6;     // Too close to the front wall to stop
const int TAPE_THRESH   = 600;   // analog value consider black
const int PARK_MS       = 600;   // continue moving after found tape to enter Parking zone
const float KP          = 6.0;   // gain helps keep it centered in the channel

// ---------- Structure ----------
struct Sensors {
  int front;
  int left;
  int right;
  bool tape;
};

// ---------- Ultrasonic + median filter (array) ----------
int readUltrasonicOnce(int trig, int echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long us = pulseIn(echo, HIGH, 20000);
  if (us == 0) return 200;           // no echo = clear
  return (int)(us * 0.034 / 2);
}

int readUltrasonic(int trig, int echo) {
  int v[3];                           // array collect 3 value
  for (int i = 0; i < 3; i++) {
    v[i] = readUltrasonicOnce(trig, echo);
    delay(5);
  }
  // median of 3 value
  if ((v[0] <= v[1] && v[1] <= v[2]) || (v[2] <= v[1] && v[1] <= v[0])) return v[1];
  if ((v[1] <= v[0] && v[0] <= v[2]) || (v[2] <= v[0] && v[0] <= v[1])) return v[0];
  return v[2];
}

// read all the sensor from pointer
void readSensors(Sensors *s) {
  s->front = readUltrasonic(TRIG_F, ECHO_F);
  s->left  = readUltrasonic(TRIG_L, ECHO_L);
  s->right = readUltrasonic(TRIG_R, ECHO_R);
  s->tape  = analogRead(TAPE_PIN) > TAPE_THRESH;
}

// ---------- Motor ----------
void setMotor(int pwm, int in1, int in2, int speed) {
  if (speed > 0)      { digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  }
  else if (speed < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else                { digitalWrite(in1, LOW);  digitalWrite(in2, LOW);  }
  analogWrite(pwm, constrain(abs(speed), 0, 255));
}

void drive(int left, int right) {
  setMotor(L_PWM, L_IN1, L_IN2, left);
  setMotor(R_PWM, R_IN1, R_IN2, right);
}

void stopMotors() { drive(0, 0); }

void turnLeft90()  { drive(-SPEED, SPEED); delay(MS_TURN_90); stopMotors(); delay(120); }
void turnRight90() { drive(SPEED, -SPEED); delay(MS_TURN_90); stopMotors(); delay(120); }
void turnAround()  { turnLeft90(); turnLeft90(); }

// ---------- Move Forward 1 Box and helps keep it centered and check the black tape ----------
// return true if black tape is on the way
bool forwardOneBlock() {
  Sensors s;
  unsigned long start = millis();

  while (millis() - start < (unsigned long)MS_PER_BLOCK) {
    readSensors(&s);

    if (s.tape) { stopMotors(); return true; }
    if (s.front < FRONT_STOP_CM) break;

    // try to make it stays in the middle with nearby wall
    float error = 0;
    if (s.left < WALL_CM && s.right < WALL_CM) error = s.left - s.right;
    else if (s.left < WALL_CM)  error = s.left - 10;
    else if (s.right < WALL_CM) error = 10 - s.right;

    int corr = (int)(KP * error);
    drive(SPEED - corr, SPEED + corr);   // error > 0 = keep to the right -> Turn Left
  }
  stopMotors();
  delay(120);
  return false;
}

// ---------- Enter Parking zone and pause ----------
void parkAndStop() {
  drive(SPEED, SPEED);
  delay(PARK_MS);
  stopMotors();
}

void setup() {
  pinMode(L_PWM, OUTPUT); pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_PWM, OUTPUT); pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  delay(2000);
}

// ---------- Left-hand wall following ----------
void loop() {
  Sensors s;
  readSensors(&s);

  if (s.tape) { parkAndStop(); while (true); }

  bool leftOpen  = s.left  > WALL_CM;
  bool frontOpen = s.front > WALL_CM;
  bool rightOpen = s.right > WALL_CM;

  if (leftOpen)       turnLeft90();
  else if (frontOpen) { /* go straight */ }
  else if (rightOpen) turnRight90();
  else                turnAround();

  if (forwardOneBlock()) {      // Found Black Tape
    parkAndStop();
    while (true);               // End
  }
}

