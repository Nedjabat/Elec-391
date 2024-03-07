#include <stdio.h>
#include <util/atomic.h>

//encoder pins
#define ENCA_M1 2  //M1 ENC YELLOW, GPIO16/RX2
#define ENCB_M1 3  //M1 ENC WHITE, GPIO17/TX2

#define ENCA_M2 4  //M2 ENC YELLOW, GPIO18/D18
#define ENCB_M2 5  //M2 ENC WHITE, GPIO19/D19

//driver pins
#define PWM_M1  6  //M1 ENABLE/PWM, GPIO21/D21
#define CW_M1   7  //M1 CLOCKWISE/RIGHT, GPIO22/D22
#define CCW_M1  8  //M1 COUNTERCLOCKWISE/LEFT, GPIO23/23

#define PWM_M2  9  //M2 ENABLE/PWM, GPIO25/D25
#define CW_M2   10 //M2 CLOCKWISE/UP, GPIO26/D26

#define CCW_M2  11 //M2 COUNTERCLOCKWISE/DOWN, GPIO27/D27

#define LASER   12 //Laser pin, GPIO32/D32
//GPIOs 20, 24, 28, 29, 30, and 31 no accessible.

// globals
volatile int pos_M1 = 0;
int pos = 0;

//int pos_M1 = 0;
long prevT_M1 = 0;
float eprev_M1 = 0;
float eintegral_M1 = 0;

int pos_M2 = 0;
long prevT_M2 = 0;
float eprev_M2 = 0;
float eintegral_M2 = 0;

//PID gain values
float kp_M1 = 30;   //60
float ki_M1 = 600;  //600
float kd_M1 = 0;  //0.1

float kp_M2 = 1;   //60
float ki_M2 = 0;  //600
float kd_M2 = 0;  //0.1

//function declarations
void setMotor();

//136 encoder counts / degrees
void setup() {
  Serial.begin(9600);
  pinMode(ENCA_M1, INPUT);
  pinMode(ENCB_M1, INPUT);

  pinMode(ENCA_M2, INPUT);
  pinMode(ENCB_M2, INPUT);

  pinMode(PWM_M1, OUTPUT);
  pinMode(CW_M1, OUTPUT);
  pinMode(CCW_M1, OUTPUT);

  pinMode(PWM_M2, OUTPUT);
  pinMode(CW_M2, OUTPUT);
  pinMode(CCW_M2, OUTPUT);

  pinMode(LASER, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA_M1),readEncoderM1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_M2),readEncoderM2, RISING);
}

void loop() {

  int target_M1 = 1200;

  long currT_M1 = micros();
  float deltaT_M1 = ((float)(currT_M1-prevT_M1))/(1.0e6); //time difference
  prevT_M1 = currT_M1;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { //atomic block to ensure no misread
    pos = pos_M1;
  }

  int e_M1 = target_M1 - pos; //error calculation, might need to switch sign
  float dedt_M1 = (e_M1-eprev_M1)/(deltaT_M1);  //derivative
  eintegral_M1 = eintegral_M1 + e_M1*deltaT_M1; //integral

  float u_M1 = kp_M1*e_M1 + kd_M1*dedt_M1 + ki_M1*eintegral_M1; //control signal

  float pwr_M1 = fabs(u_M1);  //motor power                              
  if(pwr_M1 > 255){
    pwr_M1 = 255;
  }

  int dir_M1 = 1;  //motor direction     
  if(u_M1<0){      //u_M1<0
    dir_M1 = -1;
  }
  
  setMotor(dir_M1, PWM_M1, pwr_M1, CW_M1, CCW_M1);  //send signal to the motor

  eprev_M1 = e_M1;    //store previous error

  Serial.print("target: ");
  Serial.print(target_M1);
  Serial.print(" | ");
  Serial.print("position: ");
  Serial.print(pos_M1);
  Serial.print(" | ");
  Serial.print("pwm: ");
  Serial.print(pwr_M1);
  Serial.print(" | ");
  Serial.print("u_M1:");
  Serial.print(u_M1);
  Serial.print(" | ");
  Serial.print("dir_M1:");
  Serial.print(dir_M1);
  Serial.println();
 
}

void setMotor(int dir, int pwm, int pwmVal, int in1, int in2){
  analogWrite(pwm, pwmVal); //implement speed control using driver

  if(dir == 1){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if(dir == -1){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else{
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void readEncoderM1(){
  if(digitalRead(ENCB_M1)>0){
    pos_M1++;
  }
  else{
    pos_M1--;
  }
}

void readEncoderM2(){
  if(digitalRead(ENCB_M2)>0){
    pos_M2++;
  }
  else{
    pos_M2--;
  }
}