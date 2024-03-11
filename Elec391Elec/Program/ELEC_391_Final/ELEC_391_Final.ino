#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define PWM_M1 5
#define CW_M1 6
#define CCW_M1 7

//encoder counts
volatile long encoder_M1 = 0;
volatile int pos = 0;

//variables for testing purposes
int target_M1 = 120;
//volatile int ISR_time = 0;

//PID
volatile float u = 0;
volatile long previousTime = 0;
volatile float ePrevious = 0;
volatile float eIntegral = 0;
volatile float eDerivativeBuffer[10] = {0.0};
const int eDerivativeSamples = 10;

float kp_M1 = 10;      //60
float ki_M1 = 0.01;   //600
float kd_M1 = 0.1;      //0.1

void setup() {
  Serial.begin(115200);

  //Initialize interrupt timer
  cli();                // Disable interrupts
  TCCR1A = 0;           // Init Timer1
  TCCR1B = 0;           // Init Timer1
  TCCR1B |= B00000011;  // Prescalar = 64
  OCR1A = 225;           // Timer CompareA Register
  TIMSK1 |= B00000010;  // Enable Timer COMPA Interrupt
  sei();                // Enable interrupts

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM_M1,OUTPUT);
  pinMode(CW_M1,OUTPUT);
  pinMode(CCW_M1,OUTPUT);
}

void loop() {
  Serial.print("target_M1:");
  Serial.print(target_M1);
  Serial.print(" | ");
  Serial.print("Pos_M1:");
  Serial.println(pos);
}

ISR(TIMER1_COMPA_vect){
  // Advance The COMPA Register, handle The 200us/5000hz Timer Interrupt
  OCR1A += 225;

  //int start = micros();

  //convert encoder pulses to degrees
  pos = encoder_M1/136;
  
  //PID control signal calculation
  u = pidController(target_M1, pos, kp_M1, ki_M1, kd_M1);

  //Convert control signal into associated PWM value and direction, and send to motor
  moveMotor(CW_M1, CCW_M1, PWM_M1, u);
  
  //ISR_time = micros() - start;
}

float pidController(int target, int pos, float kp, float ki, float kd){
  long currentTime = micros();
  float deltaT = ((float)(currentTime-previousTime))/1.0e6; //time difference

  //compute error for proportional control
  int e = pos - target;

  //compute integral error
  eIntegral = eIntegral + e*deltaT;

  //compute derivative term using weighted sum filters
  //shift the values in eDerivativeBuffer down and update the latest value to the last index
  for (int i = 0; i<eDerivativeSamples-1; i++){
    eDerivativeBuffer[i] = eDerivativeBuffer[i+1];
  }
  eDerivativeBuffer[eDerivativeSamples-1] = (e-ePrevious)/(deltaT);
  
  //apply weights to eDerivative terms
  float eDerivative = 0.0;
  float weightSum = 0.0;
  float weight = 1.0;
  
  for (int i=0; i<eDerivativeSamples; i++){
    eDerivative += eDerivativeBuffer[i]*weight;
    weightSum += weight;
    weight *= 1.2;
  }

  //normalize eDerivative
  eDerivative /= weightSum;

  //output PID control signal
  float u = (kp*e) + (ki*eIntegral) + (kd*eDerivative);

  //update variables for next iteration
  ePrevious = e;
  previousTime = currentTime;

  return u;
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

void readEncoder(){
  if(digitalRead(ENCB) > 0){
    encoder_M1++; //counter clockwise rotation
  }
  else{
    encoder_M1--; //clockwise rotation
  }
}