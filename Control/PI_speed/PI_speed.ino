#include <util/atomic.h>
#include <Servo.h>
#include <MPU9250_asukiaaa.h>


// Pin definitions
#define ENCA 2  //Encoder channel A — hardware interrupt INT0
#define ENCB 3

// PWM capable pins
#define IN1 5
#define IN2 6

// Motor driver
#define SERVO_PIN 9
Servo headServo;

// Accelerator - 0 = X,  1 = Y,  2 = Z ????
MPU9250_asukiaaa sensor;

//Servo constraints ?????
const int SERVO_CENTER = 72;  // Straight-ahead position (degrees)
const int SERVO_MIN = 56;
const int SERVO_MAX = 86;
float anguloZ = 0.0;
float anguloInicial = 0.0;
bool primeraLectura = true;
unsigned long tiempoAnterior = 0;

// --- Calibración ---
float gyroZ_offset = 0.0;
const int NUM_MUESTRAS_CAL = 500;
// Gyroscope angle
const int GYRO_MIN = -90;  // Ángulo del giroscopio que mapea al mínimo
const int GYRO_MAX = 90;   // Ángulo del giroscopio que mapea al máximo


// TIMING — non-blocking millis() intervals
const unsigned long SERVO_IMU_INTERVAL_MS = 20;  // Update servo & IMU @ 50 Hz
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
const float SWIM_SPEED_RPM = 120.0f;  //180.0F
// Duration of each half-stroke in microseconds.
// 1,000,000 µs = 1 second ON, then 1 second OFF (≈ 0.5 Hz tail beat).
// Lower value = faster tail beat. Try 500,000 for a quicker stroke.
const float STROKE_PERIOD_US = 1.0e6f;  //7.0e5f    1.0e6f

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
  // Wire.begin();
  // if (!mpu.setup(0x68)) {  // ???0x68 is default I2C address; try 0x69 if it fails
  //   Serial.println("ERROR: MPU-9250 not found! Check wiring (SDA→A4, SCL→A5).");
  //   while (1)
  //     ;
  // }
  Wire.begin();
  sensor.setWire(&Wire);
  sensor.beginGyro();
  sensor.beginAccel();
  delay(100);
  calibrarGiroscopio();  // your calibration function
  tiempoAnterior = millis();
  // Optional but recommended: calibrate accelerometer on startup.
  // Keep the fish STILL and LEVEL for ~5 seconds after powering on.
  Serial.println("Keep sensor still for calibration...");
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
  long currT = micros();                             //measure the time elapsed using the micros function
  float deltaT = ((float)(currT - prevT)) / 1.0e6f;  //delta = time elapsed - the last time measurment
  float velocity1 = (pos - posPrev) / deltaT;        //difference of current - previous encoder count / deltaT
  posPrev = pos;
  prevT = currT;

  //Convert count/s to RPM: PPR x 4 (quadrature)
  // 150 PPR encoder → 150 × 4 = 600. Ours is 11 PPR x 34(ratio) x 4→ use 1496.
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
  float kp = 1.0f;  // 0 0.5 1.0 2.0
  float ki = 0.5f;  // 0 ------------> 0.1 0.3 1.0
  //(1.5f, 0.3f)

  float e = vt - v1Filt;
  eintegral = eintegral + e * deltaT;
  eintegral = constrain(eintegral, -200.0f, 200.0f);  //anti-windup (claude's recomendation)
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
  Serial.print("RPM:");
  Serial.print(v1Filt);
  Serial.print(",");
  Serial.print("Target:");
  Serial.println(vt);
}

void updateHeadServo() {
  sensor.gyroUpdate();
  sensor.accelUpdate();

  unsigned long ahora = millis();
  float dt = (ahora - tiempoAnterior) / 1000.0f;
  tiempoAnterior = ahora;

  float gz = sensor.gyroZ() - gyroZ_offset;
  float ax = sensor.accelX();
  float ay = sensor.accelY();
  float anguloAcel = atan2(ay, ax) * 180.0f / PI;

  float alpha = 0.75f;
  anguloZ = alpha * (anguloZ + gz * dt) + (1.0f - alpha) * anguloAcel;

  if (primeraLectura) {
    anguloInicial = anguloZ;
    primeraLectura = false;
  }

  float anguloRelativo = anguloZ - anguloInicial;
  if (anguloRelativo > 180.0f) anguloRelativo -= 360.0f;
  if (anguloRelativo < -180.0f) anguloRelativo += 360.0f;

  float anguloConstrained = constrain(anguloRelativo, GYRO_MIN, GYRO_MAX);
  int posServo = (int)map((long)anguloConstrained, GYRO_MIN, GYRO_MAX, SERVO_MIN, SERVO_MAX);
  posServo = constrain(posServo, SERVO_MIN, SERVO_MAX);
  headServo.write(posServo);
}


void setMotor(int dir, int pwmVal, int in1, int in2) {
  if (dir == 1) {
    analogWrite(in1, pwmVal);  // DRV8871: dir =  1 → forward  (IN1 = PWM, IN2 = LOW)
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    analogWrite(in2, pwmVal);  // DRV8871: dir = -1 → reverse  (IN1 = LOW, IN2 = PWM)
  } else {
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

void calibrarGiroscopio() {
  Serial.println("Calibrando... No muevas el sensor.");
  float suma = 0;
  for (int i = 0; i < NUM_MUESTRAS_CAL; i++) {
    sensor.gyroUpdate();
    suma += sensor.gyroZ();
    delay(4);
  }
  gyroZ_offset = suma / NUM_MUESTRAS_CAL;
  Serial.println("Calibracion completa.");
  delay(500);
}