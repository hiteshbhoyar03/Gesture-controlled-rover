/**
 * ______________________________________________________________________________________________________
 * @author		:		HITESH BHOYAR
 * @file    	:		transmitter.ino
 * @brief   	:		This file includes the rover code and its functions
 * ______________________________________________________________________________________________________
 */

/*
        TRANSMITTER CODE
*/
#include <SPI.h>                // FOR SPI COMMUNICATION OF NRF24L01 WITH ARDUINO
#include <nRF24L01.h>           // FOR NRF24L01
#include <RF24.h>               // FOR NRF24L01
#include <Wire.h>               // FOR I2C COMMUNICATION with OLED AND ACCELEROMETER
#include <Adafruit_SH1106.h>    // FOR OLED
#include <Adafruit_GFX.h>       // FOR OLED
 
RF24 radio(9, 10);              // nRF24L01 (CE, CSN)
const byte addresses[][6] = {"00001", "00002"};  //PIPE ADDRESS
 
#define j1 4                   // JOYSTICK BUTTON
#define j2 5
#define t1 7                    // TOGGLE SWITCH 
 
const int MPU = 0x68;           // MPU6050 I2C ADDRESS
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY;
float angleX, angleY;
float elapsedTime, currentTime, previousTime;
int c = 0;
 
#define OLED_RESET -1             // ADAFRUIT OLED 
Adafruit_SH1106 display(OLED_RESET);
 
unsigned long time0 = 0;
unsigned long time1 = 0;
 
struct data_package               // MAX SIZE OF THIS STRUCK IS 32 BYTES - NRF24L01 BUFFER LIMIT
{
  byte joy1X;
  byte joy1Y;
  byte j1button;
  
  byte joy2X;
  byte joy2Y;
  byte j2button;
 
  byte toggle1;
};
data_package data;                 // VARIABLE IS data
 
struct gps_package                 // STRUCTURE FOR GPS DATA MAX 32 BYTES
{
  float latitude;
  float longitude;
  float altitude;
  float speed;
  byte  satellite;
  byte  abc;
};
gps_package gpsdata;               // VARIABLE IS gpsdata
 
void setup() 
{
  Serial.begin(9600);
 
  initialize_MPU6050();                     // Initialize interface to the MPU6050
 
  radio.begin();                            // SET RADIO
  radio.openWritingPipe(addresses[1]);      // 00002
  radio.openReadingPipe(1, addresses[0]);   // 00001
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  
  pinMode(j1, INPUT_PULLUP);                // Activate the Arduino internal pull-up resistors
  pinMode(j2, INPUT_PULLUP);
  pinMode(t1, INPUT_PULLUP);
 
 
  set_default();                            // SET INITIAL DEFAULT VATUES OF INPUTS
 
  display.begin(SH1106_SWITCHCAPVCC, 0x3C); // INITIALIZE DISPLAY WITH ADDRESS
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println("hello");
  display.println("world");
  display.display();
}
 
void loop() 
{
  radio.stopListening();                              // SET AS TRANSMITTER
  // READ ANALOG INPUTS
  data.joy1X = map(analogRead(A0), 1023, 0, 0, 255);  // Convert the analog read value from 0 to 1023 into a BYTE value from 0 to 255
  data.joy1Y = map(analogRead(A1), 1023, 0, 0, 255);
  data.joy2X = map(analogRead(A2), 0, 1023, 0, 255);
  data.joy2Y = map(analogRead(A3), 0, 1023, 0, 255);
 
  // READ DIGITAL INPUTS
  data.j1button = digitalRead(j1);
  data.j2button = digitalRead(j2);
  data.toggle1  = digitalRead(t1);
 
 
  if (digitalRead(t1) == 0) 
  {  read_IMU(); }
 
  radio.write( &data, sizeof(data_package) );          // Send the data from the structure to the receiver
 
  delay(5);
 
  radio.startListening();                              // SET AS RECEIVER FOR OLED
  if ( radio.available() ) 
  {
  radio.read( &gpsdata, sizeof(gps_package) );
  time0 = millis();
  }
  time1 = millis();
 
    Serial.print(data.joy1X);  
    Serial.print("  |  ");
    Serial.print(data.joy1Y);  
    Serial.print("  |  ");
    Serial.print(data.joy2X); 
    Serial.print("  |  ");
    Serial.println(data.joy2Y);  
  
  /*
  // SERIAL PRINT GPS RECEIVED DATA
    Serial.print("LATI: ");  
    Serial.print( gpsdata.latitude );
    Serial.print("  |  ");
    Serial.print("LONG: ");
    Serial.print( gpsdata.longitude );
    Serial.print("  |  ");
    Serial.print("ALTI: ");
    Serial.print( gpsdata.altitude );
    Serial.print("  |  ");
    Serial.print("Speed: ");
    Serial.print( gpsdata.speed );
    Serial.print("Kmph");
    Serial.print("  |  ");
    Serial.print("SAT: ");
    Serial.print( gpsdata.satellite );
    Serial.print("  |  ");
    Serial.print("receiver data: ");
    Serial.println( gpsdata.abc );
  //
  */
  if ( time1 - time0 < 30000 )
  {  
    display.clearDisplay();                   // DISPLAY CO-ORDINATES
    display.setTextSize(1);
    display.setCursor(0, 0);
  
    display.println("GPS LOCATION");
 
    display.print("LATI: ");                  // LATITUDE
    display.println(gpsdata.latitude,  6 );
 
    display.print("LONG: ");                  // LONGITUDE
    display.println(gpsdata.longitude, 6 );
 
    display.print("ALTI:");                   // ALTITUDE
    display.println(gpsdata.altitude,  3 );
 
    display.print("Speed:");                  // SPEED KMPH
    display.print(gpsdata.speed, 3 );
    display.println(" Kmph");
  
    display.print("SAT : ");                    // NUM OF SATELLITE TRACKING
    display.println(gpsdata.satellite );
 
    display.display();
  }
  else
  {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0,0);
    display.println("GPS");
    display.println("CONNECTION");
    display.println("LOST");
    display.display();
  }
  /*
  if( gpsdata.abc==1 )                        // IF GPS MODULE IS NOT GETTING DATA
  {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println("GPS");
    display.println("INITIALIZING");
    display.display();
  }
  gpsdata.abc=0;
  */
  gpsdata.abc = 0;               // receiver data check
  delay(5);
}
void set_default()
{
  data.joy1X    = 127;
  data.joy1Y    = 127;
  data.j1button = 1;
 
  data.joy2X    = 127;
  data.joy2Y    = 127;
  data.j2button = 1;
 
  data.toggle1 = 1;
 
}
 
void initialize_MPU6050() 
{
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  // Configure Accelerometer
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(true);
}
 
void read_IMU() 
{
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-8g, we need to divide the raw values by 4096, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 4096.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 4096.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 4096.0; // Z-axis value
 
  // Calculating angle values using
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) + 1.15; // AccErrorX ~(-1.15) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - 0.52; // AccErrorX ~(0.5)
 
  // === Read gyro data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000;   // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 4, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 32.8; // For a 1000dps range we have to divide first the raw value by 32.8, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 32.8;
  GyroX = GyroX + 1.85; //// GyroErrorX ~(-1.85)
  GyroY = GyroY - 0.15; // GyroErrorY ~(0.15)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = GyroX * elapsedTime;
  gyroAngleY = GyroY * elapsedTime;
 
  // Complementary filter - combine acceleromter and gyro angle values
  angleX = 0.98 * (angleX + gyroAngleX) + 0.02 * accAngleX;
  angleY = 0.98 * (angleY + gyroAngleY) + 0.02 * accAngleY;
  // Map the angle values from -90deg to +90 deg into values from 0 to 255, like the values we are getting from the Joystick
  data.joy1X = map(angleX, -90, +90, 255, 0);
  data.joy1Y = map(angleY, -90, +90, 255, 0);
}

3.3 ESP32 Camera Code: -

#include "esp_camera.h"
#include <WiFi.h>
 
#define CAMERA_MODEL_AI_THINKER // Has PSRAM   // JUST USE THE CAMERAWEBSERVER EXAMPLE
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
 
// ===========================
// Enter your WiFi credentials
// ===========================
const char* ssid = "**********";
const char* password = "**********";
 
void startCameraServer();
void setupLedFlash(int pin);
 
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
 
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if(config.pixel_format == PIXFORMAT_JPEG){
    if(psramFound()){
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }
 
#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif
 
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
 
  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  // drop down frame size for higher initial frame rate
  if(config.pixel_format == PIXFORMAT_JPEG){
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif
 
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
 
  startCameraServer();
 
  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}
 
void loop() {
  // Do nothing. Everything is done in another task by the web server
  delay(10000);
}
