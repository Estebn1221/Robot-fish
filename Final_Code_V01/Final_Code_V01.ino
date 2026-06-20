// ============================================================
//  DC Motor Speed Control + Gyroscope Head Stabilization
//
//  DC Motor:  PI controller maintaining constant speed (TARGET_FREQ)
//             Hardware: Arduino Uno + DRV8871 driver
//             Encoder:  11 PPR, 34:1 gearbox → 1496 counts/rev (4x)
//
//  Gyroscope: MPU-9250 reads rotation on X-axis (gyro only, no accel)
//             Maps angle to servo to stabilize/correct head position
//
//  Serial commands (Serial Monitor @ 115200):
//             'r' → reset gyro angle reference to current position
// ============================================================

#include <util/atomic.h>
#include <Servo.h>
#include <Wire.h>
#include <MPU9250_asukiaaa.h>


// ============================================================
//  PIN DEFINITIONS
// ============================================================
#define ENCA       2   // Encoder channel A — hardware interrupt INT0
#define ENCB       3   // Encoder channel B — hardware interrupt INT1
#define IN1        5   // Motor driver PWM forward
#define IN2        6   // Motor driver PWM reverse
#define SERVO_PIN  10  // Servo signal pin


// ============================================================
//  OBJECTS
// ============================================================
Servo            headServo;
MPU9250_asukiaaa imu;


// ============================================================
//  DC MOTOR — SETTINGS & STATE
// ============================================================
const float TARGET_FREQ = 3.5f;                  // Target speed in rev/s
float       TARGET_RPM  = TARGET_FREQ * 60.0f;   // Converted to RPM

// Encoder ISR shared variables (volatile = modified inside interrupt)
volatile int  pos_i   = 0;
volatile long prevT_i = 0;

// Main loop timing
long  prevT_motor = 0;
int   posPrev     = 0;

// Low-pass filter state (25 Hz cutoff, 2nd-order IIR)
float v1Filt = 0.0f;
float v1Prev = 0.0f;

// PI controller state
float eintegral = 0.0f;

// Serial print throttle timestamp
long printT = 0;


// ============================================================
//  SERVO — SETTINGS
// ============================================================
int servoCenter = 71;   // Neutral position (degrees)
int servoMin    = 56;   // Minimum travel limit (degrees)
int servoMax    = 86;   // Maximum travel limit (degrees)


// ============================================================
//  GYROSCOPE — SETTINGS & STATE
// ============================================================
// Gyro angle range that maps to full servo travel
const int GYRO_MIN = -15;   // degrees
const int GYRO_MAX =  15;   // degrees

// Angle tracking
float         angleX        = 0.0f;
float         angleInitial  = 0.0f;
bool          firstReading  = true;
unsigned long prevT_gyro    = 0;

// Calibration
float       gyroX_offset    = 0.0f;
const int   CAL_SAMPLES     = 500;    // Samples averaged during calibration


// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  // --- Motor driver pins ---
  pinMode(ENCA, INPUT_PULLUP);   // Pull-up avoids floating pin noise
  pinMode(ENCB, INPUT_PULLUP);
  pinMode(IN1,  OUTPUT);
  pinMode(IN2,  OUTPUT);

  // --- Encoder interrupts (4x quadrature mode) ---
  attachInterrupt(digitalPinToInterrupt(ENCA), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB), isrB, CHANGE);

  // --- Servo ---
  headServo.attach(SERVO_PIN);
  headServo.write(servoCenter);   // Start at neutral position

  // --- IMU (I2C) ---
  Wire.begin();
  imu.setWire(&Wire);
  imu.beginGyro();

  delay(100);   // Let sensor stabilize before calibration

  // --- Gyroscope calibration ---
  calibrateGyro();

  // --- Initialize timers ---
  prevT_i     = micros();
  prevT_motor = micros();
  prevT_gyro  = millis();
}


// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  runMotorControl();
  runGyroServo();
}


// ============================================================
//  DC MOTOR — PI SPEED CONTROLLER
//  Reads encoder, estimates RPM, applies PI output to motor
// ============================================================
void runMotorControl() {
  // Read encoder position atomically (prevent ISR corruption)
  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = pos_i;
  }

  // Compute elapsed time since last loop
  long  currT  = micros();
  float deltaT = (float)(currT - prevT_motor) / 1.0e6f;   // seconds

  if (deltaT <= 0.0f) return;   // Guard against division by zero

  // Velocity from position difference (counts/s → RPM)
  float velocity = (float)(pos - posPrev) / deltaT;       // counts/s
  float v1       = velocity / 1496.0f * 60.0f;            // RPM

  posPrev     = pos;
  prevT_motor = currT;

  // Low-pass filter — smooths out encoder noise (25 Hz cutoff)
  v1Filt = 0.854f * v1Filt + 0.0728f * v1 + 0.0728f * v1Prev;
  v1Prev = v1;

  // PI controller
  float kp = 1.0f;
  float ki = 0.5f;

  float error  = TARGET_RPM - v1Filt;
  eintegral   += error * deltaT;
  eintegral    = constrain(eintegral, -200.0f, 200.0f);   // Anti-windup clamp

  float u = kp * error + ki * eintegral;

  // Map controller output to direction + PWM
  int dir = (u >= 0) ? -1 : 1;
  int pwr = constrain((int)fabs(u), 0, 255);

  setMotor(dir, pwr, IN1, IN2);

  // Print RPM to Serial Plotter (throttled to every 50 ms)
  if ((currT - printT) >= 50000L) {
    Serial.print("RPM:");
    Serial.print(v1Filt, 1);
    Serial.print(",Target:");
    Serial.println(TARGET_RPM, 1);
    printT = currT;
  }
}


// ============================================================
//  GYROSCOPE + SERVO — ANGLE TRACKING & CORRECTION
//  Integrates gyro X-axis to get angle, maps to servo position
// ============================================================
void runGyroServo() {
  imu.gyroUpdate();

  // Delta time in seconds
  unsigned long now = millis();
  float dt = (now - prevT_gyro) / 1000.0f;
  prevT_gyro = now;

  // Read gyro X and remove calibration offset
  float gx = imu.gyroX() - gyroX_offset;

  // Dead-zone: ignore readings below noise floor (< 0.3 °/s)
  if (fabs(gx) < 0.3f) gx = 0.0f;

  // Integrate angular velocity → angle (gyro only, no accel correction)
  // Note: accel cannot correct yaw on the gravity-aligned axis
  angleX += gx * dt;

  // Latch initial angle so output always starts at 0°
  if (firstReading) {
    angleInitial = angleX;
    firstReading = false;
  }

  // Angle relative to starting position
  float relAngle = angleX - angleInitial;

  // Wrap to [-180°, +180°]
  if (relAngle >  180.0f) relAngle -= 360.0f;
  if (relAngle < -180.0f) relAngle += 360.0f;

  // Map gyro angle range to servo range (inverted: servo mounted reversed)
  float clampedAngle = constrain(relAngle, GYRO_MIN, GYRO_MAX);
  int   servoPos     = map(clampedAngle, GYRO_MIN, GYRO_MAX, servoMax, servoMin);
  headServo.write(servoPos);

  // Serial command: send 'r' to reset angle reference
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'r') {
      angleInitial = angleX;
      Serial.println("Gyro angle reset.");
    }
  }

  delay(10);   // ~100 Hz update rate
}


// ============================================================
//  MOTOR DRIVER — DRV8871
//    dir  1  → forward  (PWM on IN1, IN2 LOW)
//    dir -1  → reverse  (IN1 LOW, PWM on IN2)
//    else    → coast    (both LOW)
// ============================================================
void setMotor(int dir, int pwmVal, int in1, int in2) {
  if (dir == 1) {
    analogWrite(in1,  pwmVal);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    analogWrite(in2,  pwmVal);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}


// ============================================================
//  GYROSCOPE CALIBRATION
//  Averages CAL_SAMPLES readings at rest to determine offset.
//  Keep the sensor completely still during this phase.
// ============================================================
void calibrateGyro() {
  Serial.println("Calibrating gyroscope — keep sensor still...");
  float sum = 0.0f;

  for (int i = 0; i < CAL_SAMPLES; i++) {
    imu.gyroUpdate();
    sum += imu.gyroX();
    delay(4);
  }

  gyroX_offset = sum / CAL_SAMPLES;

  Serial.print("Gyro X offset: ");
  Serial.println(gyroX_offset, 4);
  Serial.println("Calibration complete.\n");
  delay(500);
}


// ============================================================
//  ENCODER ISRs — 4x QUADRATURE MODE
//  Both channels trigger on CHANGE for maximum resolution.
//  Direction determined by comparing A vs B state.
// ============================================================
void isrA() {
  // On channel A change: A == B → forward, A != B → reverse
  if (digitalRead(ENCA) != digitalRead(ENCB)) pos_i++;
  else                                          pos_i--;
}

void isrB() {
  // On channel B change: A == B → forward, A != B → reverse
  if (digitalRead(ENCA) == digitalRead(ENCB)) pos_i++;
  else                                         pos_i--;
}
