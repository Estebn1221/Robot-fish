#include <util/atomic.h>

#define ENCA 9
#define ENCB 8
#define PWM 5
#define IN2 6
#define IN1 7

int pos = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  Serial.println("target pos");
}

void loop() {
  //----set target position----
  int target = 1200;

  //----PID constants (need to be defined)----
  float kp = 1;
  float kd = 0;
  float ki = 0;

  //----time difference----
  long currT = micro();                             //current time in micro seconds
  float deltaT = ((float)(currT - prevT)) / 1.0e6;  //delta_t in seconds

  //----error----
  int e = pos - target;

  //----derivative----
  float dedt = (e - eprev) / (deltaT);

  //----integral----
  eintegral = eintegral + e * deltaT;

  //----control signal----
  float u = kp * e + kd * dedt + ki * eintegral;

  //----motor power----
  float pwr = fabs(u);  //floating point absolute value
  if (pwr > 225) {
    pwr = 255
  }

  //----motor direction----
  int dir = 1;  // determine the direction
  if (u < 0) {
    dir = -1;
  }

  //----signal the motor----
  setMotor(dir, pwr, PWM, IN1, IN2);

  eprev = e;  //store the prev e

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
}


void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);  //set speed
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {  // if 0, it doesn't rotate
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

