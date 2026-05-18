#include <util/atomic.h>
#include <Servo.h>
#include <BMI160Gen.h>

// Pin definitions
#define ENCA 2 //Encoder channel A — hardware interrupt INT0
#define ENCB 3

// PWM capable pins
#define IN1 5
#define IN2 6

// Motor driver
#define SERVO_PIN 9 

// Accelerator - 0 = X,  1 = Y,  2 = Z ????
#define INCLINATION_AXIS  1
const float ACCEL_OFFSET = 0.0f; //calibration

//Servo constraints ?????
const int SERVO_CENTER   = 90;   // Straight-ahead position (degrees)
const int SERVO_MAX_DEG  = 30;   // Maximum deflection either side
const int SERVO_MIN      = SERVO_CENTER - SERVO_MAX_DEG;  // 60°
const int SERVO_MAX      = SERVO_CENTER + SERVO_MAX_DEG;  // 120°

// TIMING — non-blocking millis() intervals
const unsigned long SERVO_IMU_INTERVAL_MS = 20;   // Update servo & IMU @ 50 Hz
unsigned long lastServoImuUpdate = 0;

//PI control globals -----------------------------------
volatile int pos_i = 0;  //stores the # of counts read by the encoder
volatile float velocity_i = 0;
volatile long prevT_i = 0;

long prevT = 0;
int posPrev = 0;
float v1Filt = 0, v1Prev = 0;
float v2Filt = 0, v2Prev = 0;
float eintegral = 0;

//????????????????
//Peak motor speed during each tail stroke (RPM).
// Stay below no-load speed (350 RPM). Recommended range: 100–250.
const float SWIM_SPEED_RPM = 200.0f; //180.0F
// Duration of each half-stroke in microseconds.
// 1,000,000 µs = 1 second ON, then 1 second OFF (≈ 0.5 Hz tail beat).
// Lower value = faster tail beat. Try 500,000 for a quicker stroke.
const float STROKE_PERIOD_US = 1.0e6f; //7.0e5f

void setup() {
  Serial.begin(115200);

  // DC motor & Encoder pins
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);  //each time the interrupt is triggered it will call the readEncoder func
  
  // Servo 
  headServo.attach(SERVO_PIN);
  headServo.write(SERVO_CENTER);  

  // --- BMI160 IMU (I2C) ---
  // ????Default I2C address is 0x69; use 0x68 if SDO pin is pulled LOW.
  if (!BMI160.begin(BMI160GenClass::I2C_MODE)) {
    Serial.println("BMI160 init failed! Check wiring (SDA→A4, SCL→A5).");
    while (1);  // Halt — IMU is required
  }
  // Set accelerometer range to ±2g for best resolution at small tilt angles.
  // Options: 2, 4, 8, 16 (g). ???Increase if fish experiences strong vibrations.
  BMI160.setAccelerometerRange(2);
  Serial.println("BMI160 ready.");
}

void loop() {
  //---READ ENCODER----the position in an atomic block (preven ISR corruption)
  int pos = 0;
  float velocity2 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = pos_i;
    velocity2 = velocity_i;
  }

  //----DC MOTOR — velocity method 1 (position delta / deltaT)----
  long currT = micros();                            //measure the time elapsed using the micros function
  float deltaT = ((float)(currT - prevT)) / 1.0e6f;  //delta = time elapsed - the last time measurment
  float velocity1 = (pos - posPrev) / deltaT;       //difference of current - previous encoder count / deltaT
  posPrev = pos;
  prevT = currT;

  //Convert count/s to RPM: PPR x 4 (quadrature)
  // 150 PPR encoder → 150 × 4 = 600. Ours is 11 PPR x 32(ratio) x 4→ use 192.
  float v1 = velocity1 / 1496.0f * 60.0f;
  float v2 = velocity2 / 1496.0f * 60.0f;

  // low-pass filter (25 Hz cutoff)
  v1Filt = 0.854f * v1Filt + 0.0728f * v1 + 0.0728f * v1Prev;
  v1Prev = v1;
  v2Filt = 0.854f * v2Filt + 0.0728f * v2 + 0.0728f * v2Prev;
  v2Prev = v2;

  //------PI SPEED CONTROL----------
  //Set a target speed ?????
  float vt = SWIM_SPEED_RPM * (sin(currT / STROKE_PERIOD_US) > 0);

  // Compute the control signal u
  float kp = 1.0f; // 0 0.5 1.0 2.0
  float ki = 0.5f; // 0 ------------> 0.1 0.3 1.0
  //(1.5f, 0.3f)

  float e = vt - v1Filt;
  eintegral = eintegral + e * deltaT;
  eintegral = constrain(eintegral, -200.0f, 200.0f); //anti-windup (claude's recomendation)
  float u = kp * e + ki * eintegral;

  // Set the motor speed and direction
  int dir = 1;
  if (u < 0) {
    dir = -1;
  }
  int pwr = (int)fabs(u);
  if (pwr > 255) {
    pwr = 255;
  }
  setMotor(dir, pwr, IN1, IN2);

  //----NON-BLOCKING IMU + SERVO UPDATE (runs at SERVO_IMU_INTERVAL_MS Hz) (claude's)
  unsigned long now = millis();
  if (now - lastServoImuUpdate >= SERVO_IMU_INTERVAL_MS) {
    lastServoImuUpdate = now;
    updateHeadServo();
  }

  //----Serial printing
  Serial.print(v1);
  Serial.println(" ");
  Serial.print(v1Filt);
  Serial.println();
  delay(1);
}

void updateHeadServo() { // Reads IMU, computes inclination on the configured axis, maps it to the
//  servo's constrained 60°–120° range
// --- Read raw accelerometer data (all three axes in one call) ---
  int axRaw, ayRaw, azRaw;
  BMI160.readAccelerometer(axRaw, ayRaw, azRaw);

  // --- Select the axis that represents left/right tilt (roll) ---
  // This is controlled by INCLINATION_AXIS at the top of the file.
  float axisRaw;
  switch (INCLINATION_AXIS) {
    case 0:  axisRaw = (float)axRaw;  break;  // X axis
    case 1:  axisRaw = (float)ayRaw;  break;  // Y axis
    default: axisRaw = (float)azRaw;  break;  // Z axis
  }

  // --- Apply calibration offset ---
  // Subtract the resting bias so the fish reads 0 when level.
  float axisCalibrated = axisRaw - ACCEL_OFFSET;

  // --- Convert raw counts to g-force ---
  // Clamp to ±1g to avoid asin() domain errors from noise.
  float accel_g = constrain(axisCalibrated / 16384.0f, -1.0f, 1.0f);

  // --- Calculate tilt angle in degrees using arcsin ---
  // Result range: -90° (full tilt one way) to +90° (full tilt other way).
  float tiltDeg = asin(accel_g) * (180.0f / PI);

  // --- Map tilt angle to servo position ---
  // We only care about ±SERVO_MAX_DEG (±30°) of physical tilt.
  // Any tilt beyond ±30° saturates the servo at its limit.
  //
  //  tiltDeg  -30° → servoAngle  60°  (full left)
  //  tiltDeg    0° → servoAngle  90°  (center)
  //  tiltDeg  +30° → servoAngle 120°  (full right)
  //
  // map() uses integer math; we cast and constrain for safety.
  float tiltClamped  = constrain(tiltDeg, -(float)SERVO_MAX_DEG, (float)SERVO_MAX_DEG);
  int   servoAngle   = (int)map(
                          (long)(tiltClamped * 100),          // scale up for integer map
                          (long)(-SERVO_MAX_DEG * 100),       // input min
                          (long)( SERVO_MAX_DEG * 100),       // input max
                          SERVO_MIN,                           // output min  = 60°
                          SERVO_MAX                            // output max  = 120°
                       );

  // Final safety clamp — belt and suspenders
  servoAngle = constrain(servoAngle, SERVO_MIN, SERVO_MAX);
  headServo.write(servoAngle);
}


void setMotor(int dir, int pwmVal, int in1, int in2) {
  if (dir == 1) {
    analogWrite(in1, pwmVal);  // DRV8871: dir =  1 → forward  (IN1 = PWM, IN2 = LOW)
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    analogWrite(in2, pwmVal);  // DRV8871: dir = -1 → reverse  (IN1 = LOW, IN2 = PWM)
    else {
    digitalWrite(in1, LOW);  // stop  (IN1 = LOW, IN2 = LOW)
    digitalWrite(in2, LOW);
  }
}

void readEncoder() {
  int b = digitalRead(ENCB);
  int increment = 0;
  if (b > 0) {
    increment = 1;
  } else {
    increment = -1;
  }
  pos_i = pos_i + increment;

  //compute velocity with method 2
  long currT = micros();                              //measure the current time
  float deltaT = ((float)(currT - prevT_i)) / 1.0e6;  //deltaT = currT - prevT
  velocity_i = increment / deltaT;                    //speed = +- 1 / deltaT
  prevT_i = currT;
}