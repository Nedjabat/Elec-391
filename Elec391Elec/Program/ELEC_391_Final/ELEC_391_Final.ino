#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define PWM_M1 5
#define CW_M1 6
#define CCW_M1 7

volatile long encoder_M1 = 0;

int target_M1 = 120;

//PID
volatile float u = 0;
volatile long previousTime = 0;
volatile float ePrevious = 0;
volatile float eIntegral = 0;

float kp_M1 = 2;   //60
float ki_M1 = 0.02;  //600
float kd_M1 = 0;  //0.1

void setup() {
  Serial.begin(115200);

  //Initialize interrupt timer
  cli();                // Disable interrupts
  TCCR1A = 0;           // Init Timer1
  TCCR1B = 0;           // Init Timer1
  TCCR1B |= B00000011;  // Prescalar = 64
  OCR1A = 60;           // Timer CompareA Register
  TIMSK1 |= B00000010;  // Enable Timer COMPA Interrupt
  sei();                // Enable interrupts

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM_M1,OUTPUT);
  pinMode(CW_M1,OUTPUT);
  pinMode(CCW_M1,OUTPUT);
  
  Serial.println(target_M1);
}

ISR(TIMER1_COMPA_vect){
  // Advance The COMPA Register, handle The 200us/5000hz Timer Interrupt
  OCR1A += 60;

  //int start = micros();

  //convert encoder pulses to degrees
  int pos = encoder_M1/136;
  
  //PID control signal calculation
  u = pidController(target_M1, pos, kp_M1, ki_M1, kd_M1);

  //Convert control signal into associated PWM value and direction, and send to motor
  moveMotor(CW_M1, CCW_M1, PWM_M1, u);
  
  Serial.print("pos:");
  Serial.println(pos);

  //int end = micros();
  //ISR_time = end - start;
}

void loop() {
}

float pidController(int target, int pos, float kp, float ki, float kd){
  long currentTime = micros();
  float deltaT = ((float)(currentTime-previousTime))/1.0e6; //time difference

  //compute error, derivative and integral
  int e = pos - target;
  float eDerivative = (e-ePrevious)/(deltaT);
  eIntegral = eIntegral + e*deltaT;

  float u = (kp*e) + (kd*eDerivative) + (ki*eIntegral); //control signal

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