/*
 * competition/auto/auto.ino
 * Autonomous maze robot: sensor-only (3x HC-SR04), no hardcoded path.
 * Left-hand wall following. Stops at a dead end after enough blocks.
 * Uses: function, structure, loop, array, pointer
 *
 * wire connect (through Sensor Shield v5):
 *   motor L298N : ENA=D5, IN1=D7, IN2=D8, ENB=D6, IN3=D9, IN4=D10
 *   HC-SR04 front  : TRIG=A0, ECHO=A1
 *   HC-SR04 left  : TRIG=A2, ECHO=A3
 *   HC-SR04 right   : TRIG=A4, ECHO=A5
 */


const bool SENSOR_TEST_MODE = false;
const int  MIN_BLOCKS_BEFORE_STOP = 4;
const unsigned long MAX_RUN_MS = 90000UL;


const int L_PWM = 5, L_IN1 = 7, L_IN2 = 8;
const int R_PWM = 6, R_IN1 = 9, R_IN2 = 10;
const int TRIG_F = A0, ECHO_F = A1;
const int TRIG_L = A2, ECHO_L = A3;
const int TRIG_R = A4, ECHO_R = A5;


const int L_DIR = 1;
const int R_DIR = 1;


const int   SPEED               = 150;
const int   MS_PER_BLOCK        = 700;
const int   MS_TURN_90          = 420;
const int   WALL_CM             = 12;
const int   FRONT_OPEN_CM       = 15;
const int   FRONT_STOP_CM       = 5;
const int   TURN_FRONT_CLEAR_CM = 4;
const int   SIDE_TARGET_CM      = 4;
const int   BACKUP_MS           = 150;
const float KP                  = 6.0;
const int   MAX_CORR            = 60;


struct Sensors {
  int front;
  int left;
  int right;
};


int readUltrasonicOnce(int trig, int echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long us = pulseIn(echo, HIGH, 20000);
  if (us == 0) return 200;
  return (int)(us * 0.034 / 2);
}

int readUltrasonic(int trig, int echo) {
  int v[3];
  for (int i = 0; i < 3; i++) {
    v[i] = readUltrasonicOnce(trig, echo);
    delay(5);
  }
  if ((v[0] <= v[1] && v[1] <= v[2]) || (v[2] <= v[1] && v[1] <= v[0])) return v[1];
  if ((v[1] <= v[0] && v[0] <= v[2]) || (v[2] <= v[0] && v[0] <= v[1])) return v[0];
  return v[2];
}


void readSensors(Sensors *s) {
  s->front = readUltrasonic(TRIG_F, ECHO_F);
  s->left  = readUltrasonic(TRIG_L, ECHO_L);
  s->right = readUltrasonic(TRIG_R, ECHO_R);
}


void setMotor(int pwm, int in1, int in2, int speed) {
  if (speed > 0)      { digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  }
  else if (speed < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else                { digitalWrite(in1, LOW);  digitalWrite(in2, LOW);  }
  analogWrite(pwm, constrain(abs(speed), 0, 255));
}

void drive(int left, int right) {
  setMotor(L_PWM, L_IN1, L_IN2, left  * L_DIR);
  setMotor(R_PWM, R_IN1, R_IN2, right * R_DIR);
}

void stopMotors() { drive(0, 0); }

void turnLeft90()  { drive(-SPEED, SPEED); delay(MS_TURN_90); stopMotors(); delay(120); }
void turnRight90() { drive(SPEED, -SPEED); delay(MS_TURN_90); stopMotors(); delay(120); }
void turnAround()  { turnLeft90(); turnLeft90(); }

void backUp(int ms) {
  drive(-SPEED, -SPEED);
  delay(ms);
  stopMotors();
  delay(100);
}


void makeRoomToTurn() {
  Sensors s;
  for (int i = 0; i < 8; i++) {
    readSensors(&s);
    if (s.front >= TURN_FRONT_CLEAR_CM) return;
    backUp(BACKUP_MS);
  }
}


void forwardOneBlock() {
  Sensors s;
  unsigned long start = millis();

  while (millis() - start < (unsigned long)MS_PER_BLOCK) {
    readSensors(&s);
    if (s.front < FRONT_STOP_CM) break;

    float error = 0;
    if (s.left < WALL_CM && s.right < WALL_CM) error = s.left - s.right;
    else if (s.left < WALL_CM)  error = s.left - SIDE_TARGET_CM;
    else if (s.right < WALL_CM) error = SIDE_TARGET_CM - s.right;

    int corr = constrain((int)(KP * error), -MAX_CORR, MAX_CORR);
    drive(SPEED - corr, SPEED + corr);
  }
  stopMotors();
  delay(120);
}


void backOutOfDeadEnd() {
  Sensors s;
  for (int i = 0; i < 8; i++) {
    backUp(MS_PER_BLOCK);
    readSensors(&s);
    if (s.left > WALL_CM || s.right > WALL_CM) break;
  }
  turnAround();
}

void finish() {
  stopMotors();
  while (true);
}


int blocksMoved = 0;
unsigned long startTime = 0;

void setup() {
  Serial.begin(9600);
  pinMode(L_PWM, OUTPUT); pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
  pinMode(R_PWM, OUTPUT); pinMode(R_IN1, OUTPUT); pinMode(R_IN2, OUTPUT);
  pinMode(TRIG_F, OUTPUT); pinMode(ECHO_F, INPUT);
  pinMode(TRIG_L, OUTPUT); pinMode(ECHO_L, INPUT);
  pinMode(TRIG_R, OUTPUT); pinMode(ECHO_R, INPUT);
  stopMotors();
  delay(2000);
  startTime = millis();
}


void loop() {
  Sensors s;

  if (SENSOR_TEST_MODE) {
    readSensors(&s);
    Serial.print("front="); Serial.print(s.front);
    Serial.print("  left="); Serial.print(s.left);
    Serial.print("  right="); Serial.println(s.right);
    delay(300);
    return;
  }

  if (millis() - startTime > MAX_RUN_MS) finish();

  readSensors(&s);

  bool leftOpen  = s.left  > WALL_CM;
  bool frontOpen = s.front > FRONT_OPEN_CM;
  bool rightOpen = s.right > WALL_CM;

  if (leftOpen) {
    makeRoomToTurn();
    turnLeft90();
  } else if (frontOpen) {
    // move forward
  } else if (rightOpen) {
    makeRoomToTurn();
    turnRight90();
  } else {
    
    if (blocksMoved >= MIN_BLOCKS_BEFORE_STOP) finish();
    backOutOfDeadEnd();
  }

  forwardOneBlock();
  blocksMoved++;
}

