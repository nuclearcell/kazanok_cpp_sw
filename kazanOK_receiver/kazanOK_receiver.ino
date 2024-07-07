#include <SFE_BMP180.h>

#include "LoRa.h"
#include "../../../../Sketches/conversations_defines.h"
#include <SPI.h>
 
#define IMG_BUFFER_SIZE 3200
#define RECEIVER_TIMEOUT 10000

unsigned long last_received = 0;
bool is_in_awaiting_packet = false;

#define ss 15
#define rst 16
#define dio0 4
//#define dio0 2 d2=gpio4 d4=gpio2
 
volatile bool packetAvaible = false;

void setup() {
  
  Serial.begin(115200);
  //while (!Serial);
 
  Serial.println("LoRa Receiver Callback");
 
  LoRa.setPins(ss, rst, dio0);
 
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1) delay(10);
  }
  //pinMode(dio0, INPUT);
  //attachInterrupt(digitalPinToInterrupt(dio0), onPacketReceived, RISING);

  //LoRa.beginPacket();
  //LoRa.println("get_img");
  //LoRa.endPacket();

  // register the receive callback
  //LoRa.onReceive(onReceive);
 
  // put the radio into receive mode
  //LoRa.receive();
}

String inString = ""; 
String prevPrompt = "";
size_t counter = 0;
size_t offset = 0;
char img_buffer[IMG_BUFFER_SIZE];
void loop() {
  if (millis() - last_received >= RECEIVER_TIMEOUT) {
    LoRa.beginPacket();
    LoRa.write(SYNC_PACKET);
    LoRa.endPacket(true);
    Serial.println("Connection lost, awaiting....");
    last_received = millis();
  }

  long start1 = millis();
  int packetSize = LoRa.parsePacket();
  if (packetSize) { 
    //Serial.println(packetSize);
    long start2 = millis();
    inString = LoRa.readString();
    Serial.println(inString);
    Serial.println("Time1:" + String(millis()-start1));
    Serial.println("Time2:" + String(millis()-start2));
    Serial.print("LEN:");
    Serial.println(inString.length());
    //Serial.println(offset);
    Serial.println(counter);
    //inString = "";   
    offset += packetSize; 
    counter++; 
    //Serial.println(LoRa.packetRssi());    
    LoRa.beginPacket();
    LoRa.write(SYNC_PACKET);
    LoRa.endPacket(true);
    last_received = millis();
  } 
  
  
  if (Serial.available()) {
    offset = 0;
    prevPrompt = Serial.readStringUntil('\n');
    LoRa.beginPacket();
    LoRa.println(prevPrompt);
    LoRa.endPacket();
  }
  
  //Serial.println(val);
  
}

void readBuffer(){
  int packetSize = LoRa.parsePacket();
  LoRa.readBytes(img_buffer, packetSize);
  //Serial.println(LoRa.readString());
  Serial.println(counter);
  counter++;
}

void onPacketReceived(){
  packetAvaible = true;
}

 
void onReceive(int packetSize) {
  // received a packet
  Serial.print("Received packet '");
 
  // read packet
  for (int i = 0; i < packetSize; i++) {
    Serial.print((char)LoRa.read());
  }
 
  // print RSSI of packet
  Serial.print("' with RSSI ");
  Serial.println(LoRa.packetRssi());
}
