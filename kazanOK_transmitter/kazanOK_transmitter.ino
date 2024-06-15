#include <SPI.h>
//#include "SD.h"
//#include "LoRa.h"
#include "FS.h"                // SD Card ESP32
#include "SD.h"
#include "SD_MMC.h"
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"

//#include "../../../../Sketches/conversations_defines.h"
#include "../conversations_defines.h"
//#define LORA_RST 4
//#define LORA_DIO0 2
//#define LORA_SS 16
//
//#define LORA_MOSI 13
//#define LORA_MISO 12
//#define LORA_SCLK 14


//Some forum variant
//#define LORA_RST 16
//#define LORA_DIO0 2
//#define LORA_SS 13
//
//#define LORA_MOSI 12
//#define LORA_MISO 2
//#define LORA_SCLK 14


// LoRa setup

//Stuart Robinson
//#define LORA_RST 4
//#define LORA_DIO0 12
//#define LORA_SS 13
//
//#define LORA_MOSI 2
//#define LORA_MISO 15
//#define LORA_SCLK 14

//SD card setup
// #define LORA_RST 12
// #define LORA_DIO0 16
// #define LORA_SS 4
// 
// #define LORA_MOSI 15
// #define LORA_MISO 2
// #define LORA_SCLK 14
// 
// SPIClass* hspi;
// ------ LoRa -----

// ------ Camera -----
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// support IDF 5.x
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

#include "esp_camera.h"


#define CAM_PIN_PWDN 32
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

//❗❗❗ image quality
static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QQVGA,    //QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

    //Def.jpeg_quality = 12, //0-63, for OV series camera sensors, lower number means higher quality
    .jpeg_quality = 32, //❗❗❗0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 1,       //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

static esp_err_t init_camera(void)
{
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }

    return ESP_OK;
}

static const char *get_cam_format_name(pixformat_t pixel_format)
{
    switch (pixel_format) {
    case PIXFORMAT_JPEG: return "JPEG";
    case PIXFORMAT_RGB565: return "RGB565";
    case PIXFORMAT_RGB888: return "RGB888";
    case PIXFORMAT_YUV422: return "YUV422";
    default:
        break;
    }
    return "UNKNOWN";
}

byte img_buffer[3072]; // 3/4*4096
size_t img_buf_offset=0;
size_t img_len;
bool img_buf_emptied = true;

esp_err_t camera_capture(){
    //acquire a frame
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera Capture Failed");
        return ESP_FAIL;
    }
    //replace this with your own function
    process_image(fb->width, fb->height, fb->format, fb->buf, fb->len);
  
    //return the frame buffer back to the driver for reuse
    esp_camera_fb_return(fb);
    return ESP_OK;
}
// ------ Camera -----

void process_image(size_t width, size_t height, pixformat_t form, uint8_t* img, size_t len){
  if(img_buf_emptied){
    memcpy(img_buffer, img, len);
    img_len = len;
    img_buf_offset = 0;
    img_buf_emptied = false;
  }
  writeFile(SD_MMC, ("/images/img_" + String(millis()) + ".jpeg").c_str(), img, len);
}



void initMicroSDCard() {
  // Start the MicroSD card
 
  //Serial.println("Mounting MicroSD Card");
  if (!SD_MMC.begin()) {
    //Serial.println("MicroSD Card Mount Failed");
    return;
  }
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    //Serial.println("No MicroSD Card found");
    return;
  }
 
}

void writeFile(fs::FS &fs, const char *path, uint8_t *data, size_t data_len) {
  //Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    //Serial.println("Failed to open file for writing");
    return;
  }
  
  if (file.write(data, data_len)) {
    //Serial.println("File written");
  } else {
    //Serial.println("Write failed");
  }
  file.close();
}

void writeFile(fs::FS &fs, const char *path, const char *message) {
  //Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    //Serial.println("Failed to open file for writing");
    return;
  }

  if (file.print(message)) {
    //Serial.println("File written");
  } else {
    //Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  //Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    //Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    //Serial.println("Message appended");
  } else {
    //Serial.println("Append failed");
  }
}

void createDir(fs::FS &fs, const char *path) {
  //Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    //Serial.println("Dir created");
  } else {
    //Serial.println("mkdir failed");
  }
}

void setup(){
  Serial.begin(115200);

  init_camera();
  //Serial.println(init_camera() == ESP_OK);

  initMicroSDCard();

  writeFile(SD_MMC, "/hello.txt", "Hello ");

  createDir(SD_MMC, "/images");
  camera_capture();

  Serial.println("ready");
}

int min(int a, int b){
  return ((a<b) ? a : b);
}

bool end_camera = true;

void loop(){
  while(Serial.available()){
    byte req_code = Serial.read();
    switch (req_code) {
      case DATA_PACKET: //Write data to sd card
      {
        String answer = Serial.readStringUntil('~') + '\n';
        appendFile(SD_MMC, "/log.txt", answer.c_str());
        break;
      }
      case IMG_PACKET: // Send part of image IMG_PACKET_MAX_SIZE
      {
        byte pckt_size = (uint8_t)min(img_len - img_buf_offset, IMG_PACKET_MAX_SIZE);
        Serial.write(IMG_PACKET);
        Serial.write((uint8_t)pckt_size);
        Serial.write(img_buffer + img_buf_offset, pckt_size);
        img_buf_offset += pckt_size;
        if(img_buf_offset >= img_len){
          img_buf_emptied = true;
          camera_capture();
        }
        break;
      }
      case COMMAND_PACKET: // Nothing to do yet
        {end_camera = !end_camera; break;}
      default: //Rubbish out
        {break;}
    }
  }
  if(!end_camera)
    camera_capture();
    //writeFile(SD_MMC, ("/" + String(millis()) + ".txt").c_str(), answer.c_str());
    //Serial.println("platecho: " + answer);
}

/*

void process_image(size_t width, size_t height, pixformat_t format, uint8_t* buf, size_t len){
  const bool async = false;
  const size_t packet_size = 255;
  long start = millis();
  // LoRa.beginPacket();
  // LoRa.print("Image(");
  // LoRa.print(width);
  // LoRa.print("x");
  // LoRa.print(height);
  // LoRa.print("f");
  // LoRa.print(get_cam_format_name(format));
  // LoRa.print("l");
  // LoRa.print(len);
  // LoRa.println(")\n");
  // LoRa.println("\n");
  // LoRa.endPacket(async);
  //Serial.println("pacakage 1 sent");
  for(size_t i = 0, j = 0; i < len; i+=packet_size, j++){
    //delay(20);
    Serial.println(i);
    LoRa.beginPacket();
    //LoRa.println(i);
    LoRa.write(buf + i,((len - i) < packet_size) ? (len - i) : packet_size);
    Serial.println(LoRa.endPacket(async));
  }
  //Serial.println("loop ended in");
  //delay(20);
  //LoRa.beginPacket();
  //LoRa.print("\nend in");
  //LoRa.println(millis() - start);
  //LoRa.endPacket(async);
  Serial.println(len);
  Serial.println(millis() - start);
}



#define REASSIGN_PINS
SPIClass* sdspi;
int sck = 14;
int miso = 2;
int mosi = 15;
int cs = 13;

//uint32_t initial_freq = 400000;
uint32_t initial_freq = 8E6;

bool check_sd_card(){
  sdspi = new SPIClass();
  //pinMode(miso, INPUT_PULLUP);
  sdspi->begin(sck, miso, mosi, cs);
  sdspi->setDataMode(SPI_MODE0);
  pinMode(cs, OUTPUT);
  digitalWrite(LORA_SS, HIGH);
  digitalWrite(cs, LOW);
  if(!SD.begin(cs, *sdspi, initial_freq)){
      Serial.println("Card Mount Failed");
      return false;
  }
  Serial.println("Card Mount success!");
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
      Serial.println("No SD_MMC card attached");
      return true;
  }

  Serial.print("SD_MMC Card Type: ");
  if(cardType == CARD_MMC){
      Serial.println("MMC");
  } else if(cardType == CARD_SD){
      Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
      Serial.println("SDHC");
  } else {
      Serial.println("UNKNOWN");
  }
  return true;
}

#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#define EXAMPLE_MAX_CHAR_SIZE    64

static const char *TAG = "example";

#define MOUNT_POINT "/sdcard"

// Pin assignments can be set in menuconfig, see "SD SPI Example Configuration" menu.
// You can also change the pin assignments here by changing the following 4 lines.
#define PIN_NUM_MISO  GPIO_NUM_2
#define PIN_NUM_MOSI  GPIO_NUM_15
#define PIN_NUM_CLK   GPIO_NUM_14
#define PIN_NUM_CS    GPIO_NUM_13




// ------ SD card ------
// void check_sd_card_main(){
//   //SD_MMC.setPins();
//   if(!SD_MMC.begin("/sdcard", true)){
//       Serial.println("Card Mount Failed");
//       return;
//   }
//   uint8_t cardType = SD_MMC.cardType();
// 
//   if(cardType == CARD_NONE){
//       Serial.println("No SD_MMC card attached");
//       return;
//   }
// 
//   Serial.print("SD_MMC Card Type: ");
//   if(cardType == CARD_MMC){
//       Serial.println("MMC");
//   } else if(cardType == CARD_SD){
//       Serial.println("SDSC");
//   } else if(cardType == CARD_SDHC){
//       Serial.println("SDHC");
//   } else {
//       Serial.println("UNKNOWN");
//   }
// }
// ------ SD card ------




void setup() {
  //Debug only
  Serial.begin(115200);

  Serial.println(init_camera());

  pinMode(LORA_SS, OUTPUT);
  pinMode(cs, OUTPUT);

  //check_sd_card();

  //while(!check_sd_card())delay(100);
  //sdspi->setFrequency(8E6);

  hspi = new SPIClass();

  hspi->begin(LORA_SCLK, LORA_MISO, LORA_MOSI, LORA_SS);
  pinMode(LORA_SS, OUTPUT);
  //hspi->setFrequency(1E6);
  //hspi.setFrequency(uint32_t freq)

  LoRa.setSPI(*hspi);
  //LoRa.setSPIFrequency(2E6);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1) delay(10);
  }
  Serial.println("Starting LoRa success!");

  
  
  
  //LoRa.beginPacket();
  //LoRa.write((uint8_t*)"Hello from other plate", 22);
  //LoRa.endPacket();
  
  
  //LoRa.setSignalBandwidth(signalBandwidth);
  //SPI.begin(mosi=,miso=,sck=,ss=);

  //spi.begin(sck=, miso=, mosi=, ss=);
  // put your setup code here, to run once:
}

String inString = ""; 
void loop() {
  long start = millis();
  int packetSize = LoRa.parsePacket();
  if (packetSize) { 
    // read packet    
    while (LoRa.available())
    {
      //LoRa.readString()
      int inChar = LoRa.read();
      inString += (char)inChar;
      //val = inString.toInt();       
    }
    Serial.println(inString);

    if (!inString.startsWith("get_img")){
      LoRa.beginPacket();  
      LoRa.println("answer:");
      LoRa.print(inString);
      LoRa.print("returned in ");
      LoRa.println(millis() - start);
      LoRa.endPacket();
    }
    else {
      camera_capture();
    }


    inString = "";     
    //Serial.println(LoRa.packetRssi());    
  }
  // put your main code here, to run repeatedly:
  //LoRa.beginPacket();  
  //LoRa.println("Hello from other plate");
  //LoRa.endPacket();
  //delay(50);
}
*/