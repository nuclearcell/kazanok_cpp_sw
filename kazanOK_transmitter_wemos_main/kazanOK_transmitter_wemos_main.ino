#pragma once // SI_CA 鹿 // TODO❗❗❗: increase bandwidth of lora!!!

#include <SPI.h>              // include libraries
#include "LoRa.h"
#include "../../../../Sketches/conversations_defines.h"

#include "I2Cdev.h"
#include <MPU6050.h>
#include "Wire.h"
//https://github.com/ElectronicCats/mpu6050

#include <SFE_BMP180.h>
#include <BMP180I2C.h>
#define I2C_ADDRESS 0x77
BMP180I2C bmp180(I2C_ADDRESS);
//https://bitbucket.org/christandlg/bmp180mi/src/master/src/BMP180MI.cpp

#include <SoftwareSerial.h> 
#include <TinyGPS++.h>
#include <TinyGPSPlus.h>

//static const int RXPin = A0, TXPin = D4;
static const int RXPin = D4, TXPin = D3;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
unsigned long lastGPSTime = 0UL;
//https://forum.arduino.cc/t/changing-baudrate-of-a-gps-module/640668
const char UBLOX_INIT[] = {
  0x24,0x45,0x49,0x47,0x50,0x51,0x2c,0x5a,0x44,0x41,0x2a,0x33,0x39,0x0d,0x0a,0xb5,0x62,0x06,0x01,0x03,0x00,0xf0,0x08,0x01,0x03,0x20
};

const char RESET_GPS[] = {
  0xB5,0x62,0x06,0x04,0x04,0x00,0xFF,0x87,0x01,0x00,0x95,0xF7
};

double latitude, longitude, altitude;

#define cs 15
#define rst 16
#define dio0 2

//https://github.com/centaq/arduino-async-bmp180wrapper
SFE_BMP180 presstemp;
double T, P;
unsigned long lastPresstempTime = 0UL;

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
unsigned long lastAccelgyroTime = 0UL;

unsigned long lastLoopUpdate = 0UL;

byte img_buffer[256];
size_t img_len;
bool img_updated = false;


bool receiver_avaible = false;//❎

bool data_pack_await = false;
unsigned long data_pack_timer = 0UL;
unsigned long data_sending_delay = 3000;//1000

unsigned long lora_send_await_start = 0;
unsigned long lora_send_await_delay = 0;
float lora_send_await_quotient = 1.1; //0.1

void setup() {
  Serial.begin(115200);
  ss.begin(9600);

  //Serial.println(Serial.getRxBufferSize());
  //Serial.setRxBufferSize(IMG_BUFFER_SIZE);


  //📡📡📡LORA📡📡📡
  LoRa.setPins(cs, rst, -1);

  if (!LoRa.begin(433E6)) {
    Serial.write(DATA_PACKET);
    Serial.print("LoRa init failed. Check your connections.~");
    while (1)delay(10);                       // if failed, do nothing
  }
  Serial.write(DATA_PACKET);
  Serial.print("LoRa init succeeded.~");

  LoRa.beginPacket();
  LoRa.println("hello from other plate!");
  LoRa.endPacket();
  //📡📡📡LORA📡📡📡

  // 📟📟📟I2C📟📟📟
  Wire.begin();
  scan_i2c_devices();

  Serial.write(DATA_PACKET);
  Serial.print("Initializing I2C devices...~");
  accelgyro.initialize();

  Serial.write(DATA_PACKET);
  Serial.print("Testing device connections...~");
  Serial.write(DATA_PACKET);
  Serial.print(accelgyro.testConnection() ? "MPU6050 connection successful~" : "MPU6050 connection failed~");


  //accelgyro.resetGyroscopePath()
  accelgyro.setSleepEnabled(false);
  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  //❗❗❗Calibration
  //accelgyro.CalibrateAccel();
  //accelgyro.CalibrateGyro();
  //getAccelGyro();
  if (bmp180.begin()){
    Serial.write(DATA_PACKET);
    Serial.print("BMP180 init success~");
    bmp180.resetToDefaults();
	  bmp180.setSamplingMode(BMP180MI::MODE_HR);
    //bmp180_loop();
  }
  else
  {
    Serial.write(DATA_PACKET);
    Serial.print("BMP180 init fail (disconnected?)\n\n~");
    while(1) delay(10); // Pause forever.
  }
  // 📟📟📟I2C📟📟📟

  //🛰🛰🛰GPS🛰🛰🛰
  gps_setup();
  start_bmp();
  LoRa.beginPacket();
  LoRa.println("Systems setup complete! \n Awaiting camera...");
  LoRa.endPacket(true);
  while (true) {
    if(Serial.available()){
      String answer = Serial.readStringUntil('\n');
      if(answer.indexOf("ready") != -1)
        break;
    }
    delay(100);
  }
  Serial.write(IMG_PACKET);

  LoRa.beginPacket();
  LoRa.println("Starting main loop");
  LoRa.endPacket(true);
}

int inline min(int a, int b){
  return ((a < b) ? a : b);
}


// Collects all data //❗🛑🛑 TODO data optimization later
bool send_data_packet_async(){
  if(LoRa.beginPacket() == 0) return false;
  unsigned long transmission_start = millis();
  LoRa.write(DATA_PACKET);
  //LoRa.write((uint8_t*)&transmission_start, sizeof(transmission_start));
  String bmp_packet = String(P) + "," + String(T) + ";";
  String mpu_packet = String(ax) + "," + String(ay) + "," + String(az) + ";";
  String mpu_packet2 = String(gx) + "," + String(gy) + "," + String(gz) + ";";
  LoRa.print(bmp_packet+mpu_packet+mpu_packet2);

  LoRa.print(latitude, 6);
  LoRa.print(",");
  LoRa.print(longitude, 6);
  LoRa.print(";");

  return LoRa.endPacket(true);
}

size_t i_counter = 0;

bool send_image_packet_async(){
  if(LoRa.beginPacket() == 0) return false;
  //unsigned long transmission_start = millis();
  // ❗❗❗DEBUG
  //Serial.write(DATA_PACKET);
  //Serial.write(img_buffer, img_len);
  //Serial.write('~');


  LoRa.write(IMG_PACKET);
  //LoRa.print(min(i_counter,999));
  //LoRa.print('|');
  LoRa.write(img_buffer, img_len);
  i_counter++;
  //if(img_len < IMG_PACKET_MAX_SIZE) i_counter = 0;
  return LoRa.endPacket(true);
}

//byte img_pack_type;

void loop() {
  if(millis() - data_pack_timer > data_sending_delay){
    data_pack_await = true;
    data_pack_timer = millis();
  }


  size_t packetSize = LoRa.parsePacket();
  if(packetSize){
    byte req_code = LoRa.read();
    String output = LoRa.readString();
    if (req_code == SYNC_PACKET){
      receiver_avaible = true;
    }
    else if (req_code == DATA_PACKET){
      Serial.write(DATA_PACKET);
      Serial.print("LoRa ");
      Serial.print(output);
      Serial.write('~');
    }
    
  }

  if(Serial.available()){
    byte req_code = Serial.read();
    if(req_code == IMG_PACKET){
      img_len = Serial.read();
      Serial.readBytes(img_buffer, img_len);
      img_updated = true;
    }
  }
  update_sensors();
  if(receiver_avaible){
    if(data_pack_await){
      if(send_data_packet_async()){
        data_pack_await = false;
      }
    }
    else {
      if(img_updated){
        if(send_image_packet_async()){
          Serial.write(IMG_PACKET);
          img_updated = false;
        }
      }
    }
    receiver_avaible = false;
  }
  //Serial.write(DATA_PACKET);
  //Serial.print(millis()-lastLoopUpdate);
  //Serial.write('~');
  lastLoopUpdate = millis();
}


enum last_bmp_measurement_t {
  TEMP,
  PRES
};
last_bmp_measurement_t last_bmp_measurement = PRES;


void start_bmp(){
  if(last_bmp_measurement == PRES){
    bmp180.measureTemperature();
  }
  else{
    bmp180.measurePressure();
  }
}

void update_bmp(){
  if(bmp180.hasValue()){
    if(last_bmp_measurement == PRES){
      T = bmp180.getTemperature();
      last_bmp_measurement = TEMP;
      bmp180.measurePressure();
      //Serial.write(DATA_PACKET);
      //Serial.print(millis());
      //Serial.print("->");
      //Serial.print(T);
      //Serial.write('~');

    }
    else{
      P = bmp180.getPressure();
      last_bmp_measurement = PRES;
      bmp180.measureTemperature();
      //Serial.write(DATA_PACKET);
      //Serial.print(millis());
      //Serial.print("->");
      //Serial.print(P);
      //Serial.write('~');
    }
  }
}

void update_sensors(){
  update_bmp();
  //bmp180.measureTemperature();
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  if(millis() - lastAccelgyroTime > data_sending_delay){
    Serial.write(DATA_PACKET);
    Serial.print(millis());
    Serial.print("->");
    String mpu_packet = String(ax) + "," + String(ay) + "," + String(az) + ";";
    String mpu_packet2 = String(gx) + "," + String(gy) + "," + String(gz) + ";";
    Serial.print(mpu_packet+mpu_packet2);
    //Serial.write('~');
    Serial.print(T);
    Serial.write(',');
    Serial.print(P);
    Serial.write('~');  
    lastAccelgyroTime = millis();
  }
  update_gps();

  update_bmp();
}


void gps_setup(){
  while (!ss.available()){
    delay(100);
    Serial.write(DATA_PACKET);
    Serial.print("Awaiting GPS...~");
  }
}


void update_gps()
{
  // Dispatch incoming characters
  while (ss.available() > 0)
    gps.encode(ss.read());

  if (gps.location.isUpdated())
  {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    Serial.write(DATA_PACKET);
    Serial.print(F("Lat="));
    Serial.print(latitude, 6);
    Serial.print(F("Long="));
    Serial.print(longitude, 6);
    Serial.write('~');
  }

  else if (gps.altitude.isUpdated())
  {
    altitude = gps.altitude.meters();
    Serial.write(DATA_PACKET);
    Serial.print(F("Alt="));
    Serial.print(altitude, 6);
    Serial.write('~');

  }

}

void scan_i2c_devices() {
  byte error, address;
  int nDevices;
  Serial.write(DATA_PACKET);
  Serial.print("Scanning...~");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.write(DATA_PACKET);
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.print(address,HEX);
      Serial.print('~');
      nDevices++;
    }
    else if (error==4) {
      Serial.write(DATA_PACKET);
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.print(address,HEX);
      Serial.print('~');
    }    
  }
  if (nDevices == 0) {
    Serial.write(DATA_PACKET);
    Serial.print("No I2C devices found\n~");
  }
  else {
    Serial.write(DATA_PACKET);
    Serial.print("done\n~");
  }
  delay(5000);          
}



/*void update_gps()
{
  // Dispatch incoming characters
  while (ss.available() > 0)
    gps.encode(ss.read());

  if (gps.location.isUpdated())
  {
    Serial.write(DATA_PACKET);
    Serial.print(F("LOCATION   Fix Age="));
    Serial.print(gps.location.age());
    Serial.print(F("ms Raw Lat="));
    Serial.print(gps.location.rawLat().negative ? "-" : "+");
    Serial.print(gps.location.rawLat().deg);
    Serial.print("[+");
    Serial.print(gps.location.rawLat().billionths);
    Serial.print(F(" billionths],  Raw Long="));
    Serial.print(gps.location.rawLng().negative ? "-" : "+");
    Serial.print(gps.location.rawLng().deg);
    Serial.print("[+");
    Serial.print(gps.location.rawLng().billionths);
    Serial.print(F(" billionths],  Lat="));
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(" Long="));
    Serial.println(gps.location.lng(), 6);
  }

  else if (gps.date.isUpdated())
  {
    Serial.print(F("DATE       Fix Age="));
    Serial.print(gps.date.age());
    Serial.print(F("ms Raw="));
    Serial.print(gps.date.value());
    Serial.print(F(" Year="));
    Serial.print(gps.date.year());
    Serial.print(F(" Month="));
    Serial.print(gps.date.month());
    Serial.print(F(" Day="));
    Serial.println(gps.date.day());
  }

  else if (gps.time.isUpdated())
  {
    Serial.print(F("TIME       Fix Age="));
    Serial.print(gps.time.age());
    Serial.print(F("ms Raw="));
    Serial.print(gps.time.value());
    Serial.print(F(" Hour="));
    Serial.print(gps.time.hour());
    Serial.print(F(" Minute="));
    Serial.print(gps.time.minute());
    Serial.print(F(" Second="));
    Serial.print(gps.time.second());
    Serial.print(F(" Hundredths="));
    Serial.println(gps.time.centisecond());
  }

  else if (gps.speed.isUpdated())
  {
    Serial.print(F("SPEED      Fix Age="));
    Serial.print(gps.speed.age());
    Serial.print(F("ms Raw="));
    Serial.print(gps.speed.value());
    Serial.print(F(" Knots="));
    Serial.print(gps.speed.knots());
    Serial.print(F(" MPH="));
    Serial.print(gps.speed.mph());
    Serial.print(F(" m/s="));
    Serial.print(gps.speed.mps());
    Serial.print(F(" km/h="));
    Serial.println(gps.speed.kmph());
  }

  else if (gps.course.isUpdated())
  {
    Serial.print(F("COURSE     Fix Age="));
    Serial.print(gps.course.age());
    Serial.print(F("ms Raw="));
    Serial.print(gps.course.value());
    Serial.print(F(" Deg="));
    Serial.println(gps.course.deg());
  }

  else if (gps.altitude.isUpdated())
  {
    Serial.print(F("ALTITUDE   Fix Age="));
    Serial.print(gps.altitude.age());
    Serial.print(F("ms Raw="));
    Serial.print(gps.altitude.value());
    Serial.print(F(" Meters="));
    Serial.print(gps.altitude.meters());
    Serial.print(F(" Miles="));
    Serial.print(gps.altitude.miles());
    Serial.print(F(" KM="));
    Serial.print(gps.altitude.kilometers());
    Serial.print(F(" Feet="));
    Serial.println(gps.altitude.feet());
  }

  else if (gps.satellites.isUpdated())
  {
    Serial.print(F("SATELLITES Fix Age="));
    Serial.print(gps.satellites.age());
    Serial.print(F("ms Value="));
    Serial.println(gps.satellites.value());
  }

  else if (gps.hdop.isUpdated())
  {
    Serial.print(F("HDOP       Fix Age="));
    Serial.print(gps.hdop.age());
    Serial.print(F("ms raw="));
    Serial.print(gps.hdop.value());
    Serial.print(F(" hdop="));
    Serial.println(gps.hdop.hdop());
  }

  else if (millis() - lastGPSTime > 5000)
  {
    Serial.println();
    if (gps.location.isValid())
    {
      static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
      double distanceToLondon =
        TinyGPSPlus::distanceBetween(
          gps.location.lat(),
          gps.location.lng(),
          LONDON_LAT, 
          LONDON_LON);
      double courseToLondon =
        TinyGPSPlus::courseTo(
          gps.location.lat(),
          gps.location.lng(),
          LONDON_LAT, 
          LONDON_LON);

      Serial.print(F("LONDON     Distance="));
      Serial.print(distanceToLondon/1000, 6);
      Serial.print(F(" km Course-to="));
      Serial.print(courseToLondon, 6);
      Serial.print(F(" degrees ["));
      Serial.print(TinyGPSPlus::cardinal(courseToLondon));
      Serial.println(F("]"));
    }

    Serial.print(F("DIAGS      Chars="));
    Serial.print(gps.charsProcessed());
    Serial.print(F(" Sentences-with-Fix="));
    Serial.print(gps.sentencesWithFix());
    Serial.print(F(" Failed-checksum="));
    Serial.print(gps.failedChecksum());
    Serial.print(F(" Passed-checksum="));
    Serial.println(gps.passedChecksum());

    if (gps.charsProcessed() < 10)
      Serial.println(F("WARNING: No GPS data.  Check wiring."));

    lastGPSTime = millis();
    Serial.println();
  }
}*/