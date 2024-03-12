#define ENCA_M1 2 //YELLOW, M1 (Yaw) encoder intterupt pin, GPIO16/RX2
#define ENCA_M2 3 //YELLOW, M2 (Pitch) encoder intterupt pin, GPIO17/TX2
#define ENCB_M1 4 //WHITE, M1 other encoder pin, GPIO18/D18
#define ENCB_M2 5 //WHITE, M2 other encoder pin, GPIO19/D19

#define PWM_M1 6  //M1 PWM pin, GPIO21/D21
#define CW_M1  7  //M1 Clockwise rotation pin, GPIO22/D22
#define CCW_M1 8  //M1 Counterclockwise rotation pin, GPIO23/23

#define PWM_M2 9  //M2 PWM pin, GPIO25/D25
#define CW_M2  10 //M2 Clockwise rotation pin, GPIO26/D26
#define CCW_M2 11 //M2 Counterclockwise rotation pin, GPIO27/D27

#define LASER 12  //Laser pin, GPIO32/D32

//encoder counts
volatile long encoder_M1 = 0;
volatile long encoder_M2 = 0;

volatile long pos_M1 = 0;
volatile long pos_M2 = 0;

//variables for testing purposes
int target_M1 = 120;
int target_M2 = 90;
//volatile int ISR_time = 0;

//PID
volatile float u_M1 = 0;
volatile float u_M2 = 0;

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

void setup() {
  Serial.begin(115200);

  //Initialize interrupt timer
  cli();                // Disable interrupts
  TCCR1A = 0;           // Init Timer1
  TCCR1B = 0;           // Init Timer1
  TCCR1B |= B00000011;  // Prescalar = 64
  OCR1A = 375;          // Timer CompareA Register
  TIMSK1 |= B00000010;  // Enable Timer COMPA Interrupt
  sei();                // Enable interrupts

  pinMode(ENCA_M1,INPUT);
  pinMode(ENCB_M1,INPUT);
  pinMode(ENCA_M2,INPUT);
  pinMode(ENCB_M2,INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA_M1),readEncoderM1,RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA_M2),readEncoderM2,RISING);

  pinMode(PWM_M1,OUTPUT);
  pinMode(CW_M1,OUTPUT);
  pinMode(CCW_M1,OUTPUT);
  pinMode(PWM_M2,OUTPUT);
  pinMode(CW_M2,OUTPUT);
  pinMode(CCW_M2,OUTPUT);
}

void loop() {
  //Serial.print("ISR_Time:");
  //Serial.print(ISR_time);
  //erial.println("us");
  Serial.print("target_M1:");
  Serial.print(target_M1);
  Serial.print(" | ");
  Serial.print("Encoder_M1:");
  Serial.print(encoder_M1);
  Serial.print(" | ");
  Serial.print("Pos_M1:");
  Serial.println(pos_M1);
}

ISR(TIMER1_COMPA_vect){
  // Advance The COMPA Register, handle The 200us/5000hz Timer Interrupt
  OCR1A += 375;

  //int start = micros();

  //convert encoder pulses to degrees, 136 pulses per degree
  pos_M1 = encoder_M1/102;
  pos_M2 = encoder_M2/102;

  //PID control signal calculation
  pidController(target_M1, target_M2, pos_M1, pos_M2, kp_M1, kp_M2, ki_M1, ki_M2, kd_M1, kd_M2, &u_M1, &u_M2);

  //Convert control signal into associated PWM value and direction, and send to motor
  moveMotor(CW_M1, CCW_M1, PWM_M1, u_M1);
  moveMotor(CW_M2, CCW_M2, PWM_M2, u_M2);

  //ISR_time = micros() - start;
}

void pidController(int target_M1, int target_M2, int pos_M1,  int pos_M2, float kp_M1, float kp_M2, float ki_M1, float ki_M2, float kd_M1, float kd_M2, float* u_M1, float* u_M2){
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
  *u_M1 = (kp_M1*e_M1) + (ki_M1*eIntegral_M1) + (kd_M1*eDerivative_M1);
  *u_M2 = (kp_M2*e_M2) + (ki_M2*eIntegral_M2) + (kd_M2*eDerivative_M2);

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

void readEncoderM1(){
  if(digitalRead(ENCB_M1) > 0){
    encoder_M1++; //counter clockwise rotation
  }
  else{
    encoder_M1--; //clockwise rotation
  }
}

void readEncoderM2(){
  if(digitalRead(ENCB_M2) > 0){
    encoder_M2++; //counter clockwise rotation
  }
  else{
    encoder_M2--; //clockwise rotation
  }
}