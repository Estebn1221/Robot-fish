#include <util/atomic.h>

// Pin definitions
#define ENCA 2
#define ENCB 3
// PWM capable pins
#define IN1 5
#define IN2 6

//globals
long prevT = 0;
int posPrev = 0;
volatile int pos_i = 0;  //stores the # of counts read by the encoder
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;

float eintegral = 0;

void setup() {
  Serial.begin(115200);

  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);  //each time the interrupt is triggered it will call the readEncoder func
}

void loop() {
  // read the position in an atomic block
  // to avoid potential misreads
  int pos = 0;
  float velocity2 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = pos_i;
    velocity2 = velocity_i;
  }

  // Compute velocity with method 1
  long currT = micros();                            //measure the time elapsed using the micros function
  float deltaT = ((float)(currT - prevT)) / 1.0e6;  //delta = time elapsed - the last time measurment
  float velocity1 = (pos - posPrev) / deltaT;       //difference of current - previous encoder count / deltaT
  posPrev = pos;
  prevT = currT;

  //Convert count/s to RPM
  float v1 = velocity1 / 600.0 * 60.0;
  float v2 = velocity2 / 600.0 * 60.0;

  // low-pass filter (25 Hz cutoff)
  v1Filt = 0.854 * v1Filt + 0.0728 * v1 + 0.0728 * v1Prev;
  v1Prev = v1;
  v2Filt = 0.854 * v2Filt + 0.0728 * v2 + 0.0728 * v2Prev;
  v2Prev = v2;

  //Set a target speed
  float vt = 100 * (sin(currT / 1e6) > 0);

  // Compute the control signal u
  float kp = 1;
  float ki = 1;
  float e = vt - v1Filt;
  eintegral = eintegral + e * deltaT;
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

  Serial.print(v1);
  Serial.println(" ");
  Serial.print(v1Filt);
  Serial.println();
  delay(1);
}

void setMotor(int dir, int pwmVal, int in1, int in2) {
  if (dir == 1) {
    analogWrite(in1, pwmVal);  // DRV8871: PWM on IN1 = forward
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    analogWrite(in2, pwmVal);  // DRV8871: PWM on IN2 = reverse
  } else {
    digitalWrite(in1, LOW);  // coast / stop
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