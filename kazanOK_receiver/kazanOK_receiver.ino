#include <SFE_BMP180.h>

#include "LoRa.h"
#include "../../../../Sketches/conversations_defines.h"
#include <SPI.h>
 
#define IMG_BUFFER_SIZE 3200

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
    while (1);
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
  //if(packetAvaible){
  //  packetAvaible = false;
  //  readBuffer();
  //}
  // do nothing
  /*
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.println(LoRa.readString());
  }
  if (Serial.available()){

  }
  */
  
  int packetSize = LoRa.parsePacket();
  if (packetSize /*&& prevPrompt.startsWith("get_img")*/) { 
    //Serial.println(packetSize);
    long start = millis();
    if(offset+packetSize >= IMG_BUFFER_SIZE){
      //Serial.println("ERR: Image Buffer Overflow");
      //Serial.println("Rewriting...");
      offset = 0;
    }
    // read packet 
    /*while (LoRa.available())
    {
      //LoRa.readString()
      int inChar = LoRa.read();
      inString += (char)inChar;
      //inString += String(inChar, HEX);
      //val = inString.toInt();       
    }*/
    inString = LoRa.readString();
    //LoRa.readBytes(img_buffer+offset, packetSize);
    //Serial.write(inString.c_str(), inString.length());
    //Serial.availableForWrite()
    Serial.println(inString);
    Serial.println("Time:" + String(millis()-start));
    Serial.print("LEN:");
    Serial.println(inString.length());
    //Serial.println(offset);
    Serial.println(counter);
    //inString = "";   
    offset += packetSize; 
    counter++; 
    //Serial.println(LoRa.packetRssi());    
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
