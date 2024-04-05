#include <esp_now.h>
#include <WiFi.h>
#include "esp32/rom/ets_sys.h"
#include "Arduino.h"

#define PWM_M1 21  //M1 PWM pin, GPIO21/D21
#define CW_M1  22  //M1 Clockwise rotation pin, GPIO22/D22
#define CCW_M1 23  //M1 Counterclockwise rotation pin, GPIO23/23

#define PWM_M2 25  //M2 PWM pin, GPIO25/D25
#define CW_M2  26 //M2 Clockwise rotation pin, GPIO26/D26
#define CCW_M2 27 //M2 Counterclockwise rotation pin, GPIO27/D27
#define LED 18

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

int target_M1 = 120;

int temp_encoder_M1 = 0;
int pos_M1 = 0;

double controlSignal_M1 = 0.0;
volatile long previousTime = 0;
volatile long ePrevious_M1 = 0;
volatile double eIntegral_M1 = 0.0;

hw_timer_t *Timer0_Cfg = NULL;

void IRAM_ATTR Timer0_ISR(){
  portENTER_CRITICAL_ISR(&mux);
    pos_M1 = temp_encoder_M1/134;
  portEXIT_CRITICAL_ISR(&mux); 

  pidController(pos_M1, &controlSignal_M1);
  moveMotor(CW_M1, CCW_M1, PWM_M1, controlSignal_M1);
  //moveMotor(CW_M2, CCW_M2, PWM_M2, controlSignal_M2);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  
  pinMode(PWM_M1,OUTPUT);
  pinMode(CW_M1,OUTPUT);
  pinMode(CCW_M1,OUTPUT);

  pinMode(LED, OUTPUT);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } 

  //register to receive package
  esp_now_register_recv_cb(OnDataRecv);

  //initialize timer, 1000hz/1ms
  Timer0_Cfg = timerBegin(0, 80, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, 10000, true);
  timerAlarmEnable(Timer0_Cfg);
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy( &temp_encoder_M1, incomingData, sizeof(temp_encoder_M1));
  //Serial.print("temp_encoder_M1:");
  //Serial.println(temp_encoder_M1);
}

void pidController(int pos_M1, volatile double *u_M1){
  //PID gain values
  double kp_M1 = 3.0;
  double ki_M1 = 0.5; //0.01;  
  double kd_M1 = 0.1;

  //Time difference
  long currentTime = micros();
  double deltaT = ((double)(currentTime-(previousTime)))/1.0e6;

  //Compute error for proportional control
  double e_M1 = (pos_M1 - target_M1);

  //Compute integral term
  eIntegral_M1 = eIntegral_M1 + e_M1*deltaT;

  //Compute derivative term
  double eDerivative_M1 = (e_M1-(ePrevious_M1))/(deltaT);

  //update variables for next iteration
  previousTime = currentTime;
  ePrevious_M1 = e_M1;

  *u_M1 = (kp_M1*e_M1) + (ki_M1*eIntegral_M1) + (kd_M1*eDerivative_M1);
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
  Serial.println(controlSignal_M1);

}
