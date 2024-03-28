#include <esp_now.h>
#include <WiFi.h>


#define PWM_M1 21  //M1 PWM pin, GPIO21/D21
#define CW_M1  22  //M1 Clockwise rotation pin, GPIO22/D22
#define CCW_M1 23  //M1 Counterclockwise rotation pin, GPIO23/23

#define PWM_M2 25  //M2 PWM pin, GPIO25/D25
#define CW_M2  26 //M2 Clockwise rotation pin, GPIO26/D26
#define CCW_M2 27 //M2 Counterclockwise rotation pin, GPIO27/D27

hw_timer_t *Timer0_Cfg = NULL;

long encoder_M1 = 0;
long temp_encoder_M1 = 0;
long encoder_M2 = 0;

volatile long pos_M1 = 0;
volatile long pos_M2 = 0;

//variables for testing purposes
int target_M1 = 120;
int target_M2 = 90;
//volatile int ISR_time = 0;

//PID
 float u_M1 = 0;
 float u_M2 = 0;

float kp_M1 = 10;   //60
float ki_M1 = 0.01; //600
float kd_M1 = -0.1;  //0.1

float kp_M2 = 10;   //60
float ki_M2 = 0.01; //600
float kd_M2 = 0.1;  //0.1


volatile long previousTime = 0;
volatile float ePrevious_M1 = 0;
volatile float ePrevious_M2 = 0;
volatile float eIntegral_M1 = 0;
volatile float eIntegral_M2 = 0;
volatile float eDerivativeBuffer_M1[10] = {0.0};
volatile float eDerivativeBuffer_M2[10] = {0.0};
const int eDerivativeSamples = 10;

void IRAM_ATTR Timer0_ISR()
{
  encoder_M1 = temp_encoder_M1;
  //int start = micros();
  //convert encoder pulses to degrees, 136 pulses per degree
  pos_M1 = encoder_M1/102;
  //pos_M2 = encoder_M2/102;

 

  //PID control signal calculation
  //pidController(target_M1, target_M2, pos_M1, pos_M2, kp_M1, kp_M2, ki_M1, ki_M2, kd_M1, kd_M2, u_M1, u_M2);

  //Convert control signal into associated PWM value and direction, and send to motor
  //moveMotor(CW_M1, CCW_M1, PWM_M1, -250);
  //moveMotor(CW_M2, CCW_M2, PWM_M2, u_M2);

  //ISR_time = micros() - start;
   
  //Serial.print(ISR_time);
  
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  
  
  pinMode(PWM_M1,OUTPUT);
  pinMode(CW_M1,OUTPUT);
  pinMode(CCW_M1,OUTPUT);

  
 
  Timer0_Cfg = timerBegin(0, 80, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, 1500, true);
  timerAlarmEnable(Timer0_Cfg);

   



  if (esp_now_init() != ESP_OK) {
  Serial.println("Error initializing ESP-NOW");
  return;

  
}

 esp_now_register_recv_cb(OnDataRecv);

}


void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
memcpy( &temp_encoder_M1, incomingData, sizeof(temp_encoder_M1));
 //Serial.print("Encoder_M1:");
  //Serial.println(encoder_M1);
}


void loop() {
 
 pidController(target_M1, target_M2, pos_M1, pos_M2, kp_M1, kp_M2, ki_M1, ki_M2, kd_M1, kd_M2, u_M1, u_M2);

  moveMotor(CW_M1, CCW_M1, PWM_M1, u_M1);
 
 //erial.println("us");
  Serial.print("target_M1:");
  Serial.print(target_M1);
  Serial.print(" | ");
  Serial.print("Encoder_M1:");
  Serial.print(encoder_M1);
  Serial.print(" | ");
  Serial.print("Pos_M1:");
  Serial.print(pos_M1);
  Serial.print("u_m1:");
  Serial.print(e_M1);
  Serial.println(" ");

}



void pidController(int target_M1, int target_M2, int pos_M1,  int pos_M2, float kp_M1, float kp_M2, float ki_M1, float ki_M2, float kd_M1, float kd_M2, float u_M1, float u_M2){
  //time difference
  long currentTime = micros();
  float deltaT = ((float)(currentTime-previousTime))/1.0e6;

  //compute error for proportional control
  long e_M1 = pos_M1 - target_M1;
  long e_M2 = pos_M2 - target_M2;

  //compute integral error
  eIntegral_M1 = eIntegral_M1 + e_M1*deltaT;
  eIntegral_M2 = eIntegral_M2 + e_M2*deltaT;

  //compute derivative term using weighted sum filters
  //shift the values in eDerivativeBuffer down and update the latest value to the last index
  for (int i = 0; i<eDerivativeSamples-1; i++){
    eDerivativeBuffer_M1[i] = eDerivativeBuffer_M1[i+1];
  }
  eDerivativeBuffer_M1[eDerivativeSamples-1] = (e_M1-ePrevious_M1)/(deltaT);

  for (int i = 0; i<eDerivativeSamples-1; i++){
    eDerivativeBuffer_M2[i] = eDerivativeBuffer_M2[i+1];
  }
  eDerivativeBuffer_M2[eDerivativeSamples-1] = (e_M2-ePrevious_M2)/(deltaT);

  //apply weights to eDerivative terms
  float eDerivative_M1 = 0.0;
  float eDerivative_M2 = 0.0;
  float weight_M1 = 1.0;
  float weight_M2 = 1.0;
  float weightSum = 0.0;
  
  for (int i=0; i<eDerivativeSamples; i++){
    eDerivative_M1 += eDerivativeBuffer_M1[i]*weight_M1;
    weightSum += weight_M1;
    weight_M1 *= 1.2;
  }

  for (int i=0; i<eDerivativeSamples; i++){
    eDerivative_M2 += eDerivativeBuffer_M2[i]*weight_M2;
    weight_M2 *= 1.2;
  }

  //normalize eDerivative
  eDerivative_M1 /= weightSum;
  eDerivative_M2 /= weightSum;

  //output PID control signal
  u_M1 = (kp_M1*e_M1) + (ki_M1*eIntegral_M1) + (kd_M1*eDerivative_M1);
  u_M2 = (kp_M2*e_M2) + (ki_M2*eIntegral_M2) + (kd_M2*eDerivative_M2);

  //update variables for next iteration
  ePrevious_M1 = e_M1;
  ePrevious_M2 = e_M2;
  previousTime = currentTime;
}

void moveMotor(int in1, int in2, int pwm_pin, float u){
  float speed = fabs(u);  //motor power                              
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

