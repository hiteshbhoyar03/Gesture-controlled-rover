/**
 * ______________________________________________________________________________________________________
 * @author		:		HITESH BHOYAR
 * @file    	:		transmitter.ino
 * @brief   	:		This file includes the rover code and its functions
 * ______________________________________________________________________________________________________
 */

#include <SPI.h>             // FOR SPI COMMUNICATION OF NRF24L01 WITH ARDUINO
#include <nRF24L01.h>        // FOR NRF24L01
#include <RF24.h>            // FOR NRF24L01
#include <AFMotor.h>         // FOR MOTORS
#include <TinyGPS++.h>       // FOR GPS
#include <Servo.h>           // FOR SERVO 
#include <NewPing.h>         // FOR ULTRASONIC SENSORS
 
RF24 radio(48, 49);                  // nRF24L01 (CE, CSN)
const byte addresses[][6] = {"00001", "00002"};  //PIPE ADDRESS
 
unsigned long time0 = 0;
unsigned long time1 = 0;
 
#define toggle 16
int flag_front=1 , flag_back=1 , flag_left=1 , flag_right=1;
 
 
AF_DCMotor motor1(1);                // FOR MOTORS
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);
 
Servo servoX;                        // CREATE SERVO USING THIS FUNCTION 
Servo servoY;
int angleX = 180;
int angleY = 180;
 
#define neogps Serial1               // SET SERIAL PORT1 AS GPS RECEIVING PORT
TinyGPSPlus gps;                     // TINYGPS set gps as function
 
#define maxdist 200                  // FOR ULTRASONIC SENSORS
 
NewPing sonar_front (22 , 23 , maxdist); // Ultrasonic sensor trig, echo, maxdist
NewPing sonar_back  (24 , 25 , maxdist);
NewPing sonar_left  (26 , 27 , maxdist);
NewPing sonar_right (28 , 29 , maxdist);
 
struct data_package                  // STRUCTURE FOR CONTROLLER
  {
  byte joy1X;
  byte joy1Y;
  byte j1button;
  
  byte joy2X;
  byte joy2Y;
  byte j2button;
  
  byte toggle1;
  };
data_package data;                   // VARIABLE IS data
 
struct gps_package                   // STRUCTURE FOR GPS DATA
  {
  float latitude;
  float longitude;
  float altitude;
  float speed;
  byte  satellite;
  byte  abc;
  };
gps_package gpsdata;                 // VARIABLE IS gpsdata
 
void setup() 
{ 
  Serial.begin(9600);
  neogps.begin(9600);                        // BEGIN SERIAL1 COMMUNICATION WITH Neo6mGPS
  gpsdata.abc=0;
 
  radio.begin();                             // SET RADIO 
  radio.openWritingPipe(   addresses[0]);    // 00001
  radio.openReadingPipe(1, addresses[1]);    // 00002
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
 
  pinMode(toggle, INPUT_PULLUP);
 
  motor1.setSpeed(255);                      // SET MOTOR SPEED
  motor2.setSpeed(255);
  motor3.setSpeed(255);
  motor4.setSpeed(255);
  motor1.run(RELEASE);                       // SET MOTOR TO RELEASE(STOP)
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
 
  servoX.attach(9);                          // ATTACH SERVO TO PINS
  servoY.attach(10);
 
  resetdata();
}
void loop() 
{
  radio.startListening();                    //  SET AS RECEIVER
  if (radio.available()) 
  {
    radio.read(&data, sizeof(data_package));   
    time0 = millis();
  }
  time1 = millis(); 
  if ( time1 - time0 > 1000 ) 
  { resetdata(); } 
 
  serial_print_data();                       // SERIAL PRINT RECEIVED DATA
 
  int toggle_switch = digitalRead(toggle);
 
  if( toggle_switch == 0 )
  {
  if( sonar_front.ping_cm() < 40 )
  flag_front = 0;
  if( sonar_back.ping_cm () < 40 )
  flag_back  = 0;
  if( sonar_left.ping_cm () < 40 )
  flag_left  = 0;
  if( sonar_right.ping_cm() < 40 )
  flag_right = 0;
  }
 
  // MOVEMENTS OF THE ROVER
 
    if (data.joy1Y > 150 && flag_front == 1 )            // FORWARD
    {
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
    }
    else if (data.joy1Y < 104 && flag_back  == 1 )       // BACKWARD 
    {
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);
    }
    else if (data.joy1X > 150 && flag_left  == 1 )       // LEFT
    {
    motor1.run(BACKWARD);
    motor2.run(FORWARD);
    motor3.run(FORWARD);
    motor4.run(BACKWARD);
    }
    else if (data.joy1X < 104 && flag_right == 1 )       // RIGHT
    {
    motor1.run(FORWARD);
    motor2.run(BACKWARD);
    motor3.run(BACKWARD);
    motor4.run(FORWARD);
    }
    else                             // STOP MOVING
    release();
 
    if ( data.joy1Y > 150 && data.joy1X > 150 && flag_front == 1 && flag_left  == 1 )       // FORWARD LEFT
    {
    motor1.run(RELEASE);
    motor2.run(FORWARD);
    motor3.run(FORWARD);
    motor4.run(RELEASE);
    }
    else if (data.joy1Y > 150 && data.joy1X < 104 && flag_front == 1 && flag_right == 1 )    // FORWARD RIGHT
    {
    motor1.run(FORWARD);
    motor2.run(RELEASE);
    motor3.run(RELEASE);
    motor4.run(FORWARD);
    }
    else if (data.joy1Y < 104 && data.joy1X > 150 && flag_back  == 1 && flag_left  == 1 )    // BACKWARD LEFT
    { 
    motor1.run(BACKWARD);
    motor2.run(RELEASE);
    motor3.run(RELEASE);
    motor4.run(BACKWARD);
    }
    else if (data.joy1Y < 104 && data.joy1X < 104 && flag_back  == 1 && flag_right == 1 )    // BACKWARD RIGHT
    {
    motor1.run(RELEASE);
    motor2.run(BACKWARD);
    motor3.run(BACKWARD);
    motor4.run(RELEASE);
    }
    else                               //STOP MOVING
    release();
 
    if (data.j1button == 1)            // ROTATE LEFT OR ANTI-CLOCKWISE
    {
    motor1.run(BACKWARD);
    motor2.run(FORWARD);
    motor3.run(BACKWARD);
    motor4.run(FORWARD);
    }
    else if (data.j2button == 1)       // ROTATE RIGHT OR CLOCKWISE
    {
    motor1.run(FORWARD);
    motor2.run(BACKWARD);
    motor3.run(FORWARD);
    motor4.run(BACKWARD);
    }
    else                               //STOP MOVING
    release();
 
  flag_front = 1;
  flag_back  = 1;
  flag_left  = 1;
  flag_right = 1;
  //
  
  // MOVEMENTS OF THE SERVO                  // SERVO MOVE UNTIL THE JOYSTICK IS MOVED THEN STOPS WHEN JOYSTICK IS RELEASED
  
    if (data.joy2X > 150 && angleX < 360)         // SERVO X AXIS
    {
    angleX = angleX + 1;
    servoX.write(angleX);
    }
 
    if(data.joy2X < 104 && angleX > 0 )
    {
    angleX = angleX - 1;
    servoX.write(angleX);
    }
    
    if(data.joy2Y > 150 && angleY < 360)   // SERVO Y AXIS
    {
    angleY = angleY + 1;
    servoY.write(angleY);  
    } 
    
    if(data.joy2Y < 104 && angleY > 0)
    {
    angleY = angleY - 1;    
    servoY.write(angleY);
    }
  //  
 
  delay(5);
 
  radio.stopListening();                     // SET AS TRANSMITTER
 
    while (neogps.available() > 0)           // GET GPS DATA 
    if (gps.encode(neogps.read()))
    {  
    if (  gps.location.isValid() == 1  )
    {
    gpsdata.latitude  = gps.location.lat();
    gpsdata.longitude = gps.location.lng();
    gpsdata.altitude  = gps.altitude.meters();
    gpsdata.speed     = gps.speed.kmph();
    gpsdata.satellite = gps.satellites.value();  
    }
    //else
    //{ gpsdata.abc = 1; }
  }
  gpsdata.abc = 1;
  radio.write( &gpsdata, sizeof(gps_package) );
  
  delay(5);
}
 
void release()
{
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}
void serial_print_data()
{
  Serial.print("joy1 = ");
  Serial.print(data.joy1X);
  Serial.print("  |  ");
  Serial.print(data.joy1Y);
  Serial.print("  |  ");
  Serial.print(data.j1button);
  Serial.print("     |     ");
  Serial.print("joy2 = ");
  Serial.print(data.joy2X);
  Serial.print("  |  ");
  Serial.print(data.joy2Y);
  Serial.print("  |  ");
  Serial.print(data.j2button);
  Serial.print("     |     ");
  Serial.print("toggle = ");
  Serial.println(data.toggle1);
}
 
void resetdata()                             // RESET DATA PACAKAGE OF TRANSMITTER
{
  data.joy1X = 127;
  data.joy1Y = 127;
  data.j1button = 1;
  
  data.joy2X = 127;
  data.joy2X = 127;
  data.j2button = 1;
    
  data.toggle1 = 1;
}

