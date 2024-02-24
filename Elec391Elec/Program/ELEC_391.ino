#include <util/atomic.h>

#define ENCA_M1 2  //M1 ENC YELLOW
#define ENCB_M1 3  //M1 ENC WHITE

#define ENCA_M2 4  //M2 ENC YELLOW
#define ENCB_M2 5  //M2 ENC WHITE

#define PWM_M1  6  //M1 ENABLE/PWM
#define CW_M1   7  //M1 CLOCKWISE/RIGHT
#define CCW_M1  8  //M1 COUNTERCLOCKWISE/LEFT
  
#define PWM_M2  9  //M2 ENABLE/PWM
#define CW_M2   10 //M2 CLOCKWISE/UP
#define CCW_M2  11 //M2 COUNTERCLOCKWISE/DOWN

// globals
int pos_M1 = 0;
int pos_M2 = 0;

//encoder resolution: (1 motor shaft rot/12 encouder counts) * (1 output shaft rot/34 motor shaft rot)
// = 1 output shaft rotation / 408 encoder counts
// to degrees: 408/360 = 1.133... encounder counts per degree

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

  attachInterrupt(digitalPinToInterrupt(ENCA_M1),readEncoderM1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_M2),readEncoderM2, RISING);
}

void loop() {
  //int pwm = 50; //motor speed
  //int dir = 1;

  setMotor(dir, PWM_M1, pwm, CW_M1, CCW_M1);
  delay(1000);
  Serial.println(pos);
  
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
  int b = digitalRead(ENCB_M1);
  if(b>0){
    pos_M1++;
  }
  else{
    po_M1--;
  }
}

void readEncoderM2(){
  int b = digitalRead(ENCB_M2);
  if(b>0){
    pos_M2++;
  }
  else{
    pos_M2--;
  }
}