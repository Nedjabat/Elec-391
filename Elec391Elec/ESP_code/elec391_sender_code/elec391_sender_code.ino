#include <esp_now.h>
#include <WiFi.h>

#define ENCA_M1 15 //YELLOW, M1 (Yaw) encoder interrupt pin, GPIO16
#define ENCB_M1 18 //WHITE, M1 other encoder pin, GPIO18
#define ENCA_M2 19 //YELLOW, M2 (Pitch) encoder interrupt pin, GPIO19
#define ENCB_M2 21 //WHITE, M2 other encoder pin, GPIO21

#define readA_M1 bitRead(GPIO.in,ENCA_M1)  //faster than digitalRead
#define readB_M1 bitRead(GPIO.in,ENCB_M1)
#define readA_M2 bitRead(GPIO.in,ENCA_M2)
#define readB_M2 bitRead(GPIO.in,ENCB_M2)

typedef struct struct_message{
  volatile long encoder_M1;
  volatile long encoder_M2;
} struct_message;

//broadcast to receiver MAC address
uint8_t broadcastAddress[] = {0x40, 0x91, 0x51, 0xFD, 0x26, 0x94};

esp_now_peer_info_t peerInfo;

struct_message encoderData;

void IRAM_ATTR readEncoderA_M1(){
  if(readB_M1 != readA_M1){
    encoderData.encoder_M1++; //clockwise rotation
  }
  else{
    encoderData.encoder_M1--; //counter clockwise rotation
  }
}

void IRAM_ATTR readEncoderB_M1(){
  if(readA_M1 == readB_M1){
    encoderData.encoder_M1++; //clockwise rotation
  }
  else{
    encoderData.encoder_M1--; //counter clockwise rotation
  }
}

void IRAM_ATTR readEncoderA_M2(){
  if(readB_M2 != readA_M2){
    encoderData.encoder_M2++; //clockwise rotation
  }
  else{
    encoderData.encoder_M2--; //counter clockwise rotation
  }
}

void IRAM_ATTR readEncoderB_M2(){
  if(readA_M2 == readB_M2){
    encoderData.encoder_M2++; //clockwise rotation
  }
  else{
    encoderData.encoder_M2--; //counter clockwise rotation
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
  Serial.println("Error initializing ESP-NOW");
  return;
  }

  pinMode(ENCA_M1,INPUT);
  pinMode(ENCB_M1,INPUT);
  pinMode(ENCA_M2,INPUT);
  pinMode(ENCB_M2,INPUT);
  
  //register to get the status of the transmitted package
  esp_now_register_send_cb(OnDataSent);

  //Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  //Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  attachInterrupt(digitalPinToInterrupt(ENCA_M1), readEncoderA_M1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_M1), readEncoderB_M1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA_M2), readEncoderA_M2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_M2), readEncoderB_M2, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &encoderData, sizeof(encoderData));

  /*
  if (result == ESP_OK) {
  Serial.println("Sent with success");
  }
  else {
  Serial.println("Error sending the data");
  }
  */

  /*
  Serial.print("ENCA_M1:");
  Serial.println(digitalRead(ENCA_M1));
  Serial.print("ENCB_M1:");
  Serial.println(digitalRead(ENCB_M1));
  */

  Serial.print("encoder_M1: ");
  Serial.print(encoderData.encoder_M1/134);
  Serial.print(" | ");
  Serial.print("encoder_M2: ");
  Serial.println(encoderData.encoder_M2/134);
}