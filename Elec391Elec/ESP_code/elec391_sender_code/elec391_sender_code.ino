//white = B
//yellow = A
#include <esp_now.h>
#include <WiFi.h>


#define ENCA_M1 16 //YELLOW, M1 (Yaw) encoder intterupt pin, GPIO16/RX2
#define ENCB_M1 18 //WHITE, M1 other encoder pin, GPIO18/D18


uint8_t broadcastAddress[] = {0x40, 0x91, 0x51, 0xFD, 0x26, 0x94};

volatile long encoder_M1 = 0;


esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
 // Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
  Serial.println("Error initializing ESP-NOW");
  return;
  }
  pinMode(ENCA_M1,INPUT);
  pinMode(ENCB_M1,INPUT);
  

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

  attachInterrupt(digitalPinToInterrupt(ENCA_M1),readEncoderM1,RISING);

}

void loop() {
  // put your main code here, to run repeatedly:

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &encoder_M1, sizeof(encoder_M1));

 /*if (result == ESP_OK) {
  Serial.println("Sent with success");
}
else {
  Serial.println("Error sending the data");
}*/
Serial.println(encoder_M1);

}



void readEncoderM1(){
  if(digitalRead(ENCB_M1) > 0){
    encoder_M1++; //counter clockwise rotation
  }
  else{
    encoder_M1--; //clockwise rotation
  }
}