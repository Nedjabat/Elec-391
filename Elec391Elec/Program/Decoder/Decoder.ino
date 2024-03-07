#include <stdlib.h>

#define RST_M1  2 //M1 DEC RESET (active low)
#define OE_M1   3 //M1 DEC OE (active low)
#define SEL1_M1 4 //M1 DEC SEL1
#define SEL2_M1 5 //M1 DEC SEL2

#define D0_M1   6 //M1 DEC D0
#define D1_M1   7 //M1 DEC D1
#define D2_M1   8 //M1 DEC D2
#define D3_M1   9 //M1 DEC D3
#define D4_M1   10 //M1 DEC D4
#define D5_M1   11 //M1 DEC D5
#define D6_M1   12 //M1 DEC D6
#define D7_M1   13 //M1 DEC D7

//Global variables
volatile long countData = 0; //32-bit count
volatile long angle_M1 = 0;   //M1 angle in degrees

void setup() {
  Serial.begin(9600);

  pinMode(RST_M1, OUTPUT);
  digitalWrite(RST_M1, LOW);   //Reset the internal counter
  delay(10);
  digitalWrite(RST_M1, HIGH);  //Stay high/off for the rest of the run

  pinMode(OE_M1, OUTPUT);
  pinMode(SEL1_M1, OUTPUT);
  pinMode(SEL2_M1, OUTPUT);

  pinMode(D0_M1, INPUT);
  pinMode(D1_M1, INPUT);
  pinMode(D2_M1, INPUT);
  pinMode(D3_M1, INPUT);
  pinMode(D4_M1, INPUT);
  pinMode(D5_M1, INPUT);
  pinMode(D6_M1, INPUT);
  pinMode(D7_M1, INPUT);
}

void loop() {

  digitalWrite(OE_M1, HIGH); // Set OE to HIGH (disable)
  delay(10);                 // Need a better way
  
  digitalWrite(SEL1_M1, LOW);
  digitalWrite(SEL2_M1, HIGH); // SEL1 = 0 and SEL2 = 1
  
  digitalWrite(OE_M1, LOW); // Set OE to LOW (enable)
  byte B1_result = getB1();
  
  digitalWrite(SEL1_M1, HIGH);
  digitalWrite(SEL2_M1, HIGH); // SEL1 = 1 and SEL2 = 1
  byte B2_result = getB2();
  
  digitalWrite(SEL1_M1, LOW);
  digitalWrite(SEL2_M1, LOW); // SEL1 = 0 and SEL2 = 0
  byte B3_result = getB3();
  
  digitalWrite(SEL1_M1, HIGH);
  digitalWrite(SEL2_M1, LOW); // SEL1 = 1 and SEL2 = 0
  byte B4_result = getB4();

  digitalWrite(OE_M1, HIGH); // Set OE to HIGH (disable)
  delay(10);
  
  countData = mergeBytes(B1_result, B2_result, B3_result, B4_result);\

  //12pulses/rev * 12/3 = 136pulses/rev
  angle_M1 = countData/136;
  
  Serial.print("Counter: ");
  Serial.print(countData);
  Serial.print(" | ");
  Serial.print("Angle M1: ");
  Serial.println(angle_M1);
}

byte getB1(){
  //Get stable data for MSB of countData
  byte B1_old = 0;
  byte B1_new = 0;

  for (int i=6; i<=13; i++){
    bitWrite(B1_old, i-6, digitalRead(i));
  }

  for (int i=6; i<=13; i++){
    bitWrite(B1_new, i-6, digitalRead(i));  //repeat for stable data
  }
  
  if (B1_new = B1_old){
    byte B1_result = B1_new;
    return B1_result;
  }
}

byte getB2(){
  //Get stable data for MSB of countData
  byte B2_old = 0;
  byte B2_new = 0;

  for (int i=6; i<=13; i++){
    bitWrite(B2_old, i-6, digitalRead(i));
  }

  for (int i=6; i<=13; i++){
    bitWrite(B2_new, i-6, digitalRead(i));  //repeat for stable data
  }
  
  if (B2_new = B2_old){
    byte B2_result = B2_new;
    return B2_result;
  }
}

byte getB3(){
  //Get stable data for MSB of countData
  byte B3_old = 0;
  byte B3_new = 0;

  for (int i=6; i<=13; i++){
    bitWrite(B3_old, i-6, digitalRead(i));
  }

  for (int i=6; i<=13; i++){
    bitWrite(B3_new, i-6, digitalRead(i));  //repeat for stable data
  }
  
  if (B3_new = B3_old){
    byte B3_result = B3_new;
    return B3_result;
  }
}

byte getB4(){
  //Get stable data for MSB of countData
  byte B4_old = 0;
  byte B4_new = 0;

  for (int i=6; i<=13; i++){
    bitWrite(B4_old, i-6, digitalRead(i));
  }

  for (int i=6; i<=13; i++){
    bitWrite(B4_new, i-6, digitalRead(i));  //repeat for stable data
  }  

  if (B4_new == B4_old){
    byte B4_result = B4_new;
    return B4_result;
  }
}

long mergeBytes(byte b1, byte b2, byte b3, byte b4){
/*Merges the 4 bytes returning one 32-bit variable called countData*/

  return (((long)b1 << 24) | ((long)b2 << 16) | ((long)b3 << 8) | ((long)b4));
}
