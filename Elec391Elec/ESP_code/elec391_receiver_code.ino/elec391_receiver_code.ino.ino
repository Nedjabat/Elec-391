#include <esp_now.h>
#include <WiFi.h>
#include "esp32/rom/ets_sys.h"
#include "Arduino.h"

#define PWM_M1 5  //M1 PWM pin, GPIO5
#define CW_M1  18 //M1 Clockwise rotation pin, GPIO18
#define CCW_M1 4  //M1 Counterclockwise rotation pin, GPIO4

#define PWM_M2 14 //M2 PWM pin, GPIO14
#define CW_M2  13 //M2 Clockwise rotation pin, GPIO13
#define CCW_M2 12 //M2 Counterclockwise rotation pin, GPIO12

//volatile int end_time = 0;

typedef struct struct_message{
  int encoder_M1;
  int encoder_M2;
}struct_message;

struct_message encoderData;

int target_M1 = 120;
int target_M2 = 360;

volatile int pos_M1 = 0;
volatile int pos_M2 = 0;

volatile long prevTime = 0;
volatile double ePrev_M1 = 0;
volatile double ePrev_M2 = 0;
volatile double integral_M1 = 0.0;
volatile double integral_M2 = 0.0;
volatile double integralPrev_M1 = 0.0;
volatile double integralPrev_M2 = 0.0;
double controlSignal_M1 = 0.0;
double controlSignal_M2 = 0.0;

hw_timer_t *Timer0_Cfg = NULL;

void IRAM_ATTR Timer0_ISR(){
  //int start = micros();

  //update motor position in degrees 1:134
  pos_M1 = encoderData.encoder_M1/134;
  pos_M2 = encoderData.encoder_M2/134;

  //update control signal
  pidController(pos_M1, pos_M2, &controlSignal_M1, &controlSignal_M2);

  //send control signal to motors
  moveMotor(CW_M1, CCW_M1, PWM_M1, controlSignal_M1);
  moveMotor(CW_M2, CCW_M2, PWM_M2, controlSignal_M2);

  //end_time = micros() - start;
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  
  pinMode(PWM_M1,OUTPUT);
  pinMode(CW_M1,OUTPUT);
  pinMode(CCW_M1,OUTPUT);

  pinMode(PWM_M2,OUTPUT);
  pinMode(CW_M2,OUTPUT);
  pinMode(CCW_M2,OUTPUT);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } 

  //register to receive package
  esp_now_register_recv_cb(OnDataRecv);

  //initialize timer, 130us period (80us ISR runtime): 61.5% duty cycle
  Timer0_Cfg = timerBegin(0, 80, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, 1500, true);
  timerAlarmEnable(Timer0_Cfg);
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy( &encoderData, incomingData, sizeof(encoderData));

  /*
  Serial.print("encoder_M1:");
  Serial.print(encoderData.encoder_M1/134);
  Serial.print(" | ");
  Serial.print("encoder_M2:");
  Serial.println(encoderData.encoder_M2/134);
  */
}

void pidController(int pos_M1, int pos_M2, volatile double *u_M1, volatile double *u_M2){
  //PID gain values
  double T = 0.0015;  //sampling time of discrete controller

  double kp_M1 = 2.0;
  double ki_M1 = 0.2;
  double kd_M1 = 0.1;

  double kp_M2 = 2.0;
  double ki_M2 = 0.2;
  double kd_M2 = 0.1;

  //Time difference
  long currentTime = micros();
  double deltaT = ((double)(currentTime-(prevTime)))/1.0e6;

  //Compute error for proportional control
  double e_M1 = (pos_M1 - target_M1);
  double e_M2 = (pos_M2 - target_M2);

  //Compute proportional term
  double proportional_M1 = e_M1*kp_M1;
  double proportional_M2 = e_M2*kp_M2;

  //Compute integral term
  //eIntegral_M1 = eIntegral_M1 + e_M1*deltaT;
  double integral_M1 = (ki_M1*T*0.5)*(e_M1+ePrev_M1)+integralPrev_M1;
  double integral_M2 = (ki_M2*T*0.5)*(e_M2+ePrev_M2)+integralPrev_M2;

  /*
  //reset integral term if motor position moves from above to below from target or vice versa
  if ((e_M1 > 0 & integralPrev_M1 < 0) | (e_M1 < 0 & integralPrev_M1 > 0)){
    integralPrev_M1 = 0.0;
  } 

  //integral clamping
  if (integral_M1 > 255){
    integral_M1 = 255.0;
  } else if (integral_M1 < -255){
    integral_M1 = -255.0;
  }

  if (integral_M2 > 255){
    integral_M2 = 255.0;
  } else if (integral_M2 < -255){
    integral_M2 = -255.0;
  }
  */

  //Compute derivative term
  double eDerivative_M1 = (e_M1-(ePrev_M1))/(deltaT);
  double eDerivative_M2 = (e_M2-(ePrev_M2))/(deltaT);

  //update variables for next iteration
  prevTime = currentTime;
  ePrev_M1 = e_M1;
  ePrev_M2 = e_M2;
  integralPrev_M1 = integral_M1;
  integralPrev_M2 = integral_M2;

  *u_M1 = (proportional_M1) + (integral_M1) + (kd_M1*eDerivative_M1);
  *u_M2 = (proportional_M2) + (integral_M2) + (kd_M2*eDerivative_M2);
}

void moveMotor(int in1, int in2, int pwm_pin, double u){
  double speed = fabs(u);  //motor power                              
  if(speed > 255){
    speed = 255;
  }

  int direction = 1;  //motor direction     
  if(u<0){            //u<0
    direction = -1;
  }

  analogWrite(pwm_pin, speed); //implement speed control using driver     

  if(direction == 1){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if(direction == -1){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else{
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void loop() {
  
  Serial.print("target_M1:");
  Serial.print(target_M1);
  Serial.print(" | ");
  Serial.print("pos_M1:");
  Serial.print(pos_M1);
  Serial.print(" | ");
  Serial.print("controlSignal_M1:");
  Serial.print(controlSignal_M1);

  Serial.print(" | ");
  Serial.print("target_M2:");
  Serial.print(target_M2);
  Serial.print(" | ");
  Serial.print("pos_M2:");
  Serial.print(pos_M2);
  Serial.print(" | ");
  Serial.print("controlSignal_M2:");
  Serial.println(controlSignal_M2);
  

  /*
  Serial.print("end_time:");
  Serial.print(end_time);
  Serial.println("us");
  */
}
