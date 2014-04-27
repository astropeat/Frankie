#include <Protocol.h>
#include <Servo.h>
#include <Wire.h>
#include "TinyGPS++.h"
TinyGPSPlus gps;

Protocol btooth(Serial1); // create a protocol object called 'link'

Servo motor;
Servo steering;

int HMC6352Address = 0x42;
int slaveAddress;
int led = 13;
boolean ledState = false;
byte headingData[2];
int i, headingValue;
Packet compass_data('C', 4); // create a packet with type C -compass- and maximum size 4
Packet gps_data('G', 12);
int desired_heading = 90;
double Blue_Post_Lat = 37.428467;    // defines latitude & longitude of blue post in engineering quad, Stanford.
double Blue_Post_Long = -122.17465;

uint8_t deadman = 0;
uint8_t autonomous = 0;

#define MOTOR_NEUTRAL 93
#define STEER_NEUTRAL 88

void linkCallback(Packet &p) {
  switch(p.type) {
    case 'V': 
    //Speed & Direction - Vector
      {
        int8_t spee = p.reads8();
        int8_t dir = p.reads8();
        if( deadman != 0 && autonomous == 0 ) {
          motor.write( spee + MOTOR_NEUTRAL );
          steering.write( dir + STEER_NEUTRAL );
        }
      }
      break;
    
    case 'A':
    //Autonomous mode
      {
       autonomous = p.reads8();
      }
      break;
     
    case 'D': 
    //dead man's switch
      {
        deadman = p.reads8();
        if (deadman == 0)
        {
          motor.write(MOTOR_NEUTRAL);
          steering.write(STEER_NEUTRAL);
        }
      }
      break;
        
      /*
    case 'R': 
    //route- latitude, longitude, radius of each step in route
    {
      int8_t step = p.reads8();
      for (int i=0; i < step; i++)
      {
      int32_t latitude = p.reads32();
      int32_t longitude = p.reads32();
      int8_t radius = p.reads8();
      }
    }
    */
  }
}

void setup() {
  // Attach bluetooth, motor, steering, serial
  btooth.setCallback(linkCallback);
  motor.attach(3);
  steering.attach(2);
  motor.write( MOTOR_NEUTRAL );
  steering.write( STEER_NEUTRAL );
  Serial1.begin(57600);  //bluetooth
    
  // Attach compass!
  // Shift the device's documented slave address (0x42) 1 bit right
  // This compensates for how the TWI library only wants the
  // 7 most significant bits (with the high bit padded with 0)
  slaveAddress = HMC6352Address >> 1;   // This results in 0x21 as the address to pass to TWI
  Serial.begin(9600);    //debug
  pinMode(led, OUTPUT);      // Set the LED pin as output
  Wire.begin();
  
  //Attach GPS
  Serial3.begin(4800);   //gps
}

void loop() {
  // Bluetooth
  btooth.poll();
  
  //Compass
  Wire.beginTransmission(slaveAddress);
  Wire.write("A");              // The "Get Data" command
  Wire.endTransmission();
  delay(1);                   // The HMC6352 needs at least a 70us (microsecond) delay
  // after this command.  Using 10ms just makes it safe
  // Read the 2 heading bytes, MSB first
  // The resulting 16bit word is the compass heading in 10th's of a degree
  // For example: a heading of 1345 would be 134.5 degrees
  Wire.requestFrom(slaveAddress, 2);        // Request the 2 byte heading (MSB comes first)
  i = 0;
  while(Wire.available() && i < 2)
  { 
    headingData[i] = Wire.read();
    i++;
  }
  headingValue = headingData[0]*256 + headingData[1];  // Put the MSB and LSB together
  
  compass_data.append(headingValue); // adds heading value to our packet
  btooth.send(compass_data); // sends 'compass_data'
   
  Serial.print("Current heading: ");
  Serial.print(int (headingValue / 10));     // The whole number part of the heading
  Serial.print(".");
  Serial.print(int (headingValue % 10));     // The fractional part of the heading
  Serial.println(" degrees");
  delay(10);
  
  
  Serial.print("Speed: ");
  Serial.println(motor.read());
  
  //GPS
  while (Serial3.available() > 0)
    gps.encode(Serial3.read());
    
  if (gps.location.isUpdated()){
    gps_data.append((int)(gps.location.lat() * 1000000)); // Adds Latitude
    gps_data.append((int)(gps.location.lng() * 1000000)); // Adds Longitude
    gps_data.append(gps.satellites.value()); // Adds # satellites

    btooth.send(gps_data); // sends 'gps_data'
        
    Serial.print("LAT=");  Serial.println(gps.location.lat());
    Serial.print("LONG="); Serial.println(gps.location.lng());
    Serial.print("NUM_SAT=");  Serial.println(gps.satellites.value());
    /*
   double distance = gps.distance(
      gps.location.lat(),
      gps.location.lng(),
      EIFFEL_TOWER_LAT,
      EIFFEL_TOWER_LNG)*/
    
    double courseTo = gps.courseTo(
        gps.location.lat(),               //This here reads current location and writes a course to Blue Post.
        gps.location.lng(),
        Blue_Post_Lat,
        Blue_Post_Long);
        
    desired_heading = courseTo;
    
    Serial.println(desired_heading); 
    }
  
  
  //DO A THING
  //GO NORTH
   if (autonomous != 0 && deadman != 0) {
     
     int headingComputed = (headingValue/10);
     headingComputed -= desired_heading;
     
      if (headingComputed < 180 && headingComputed > 0) {
        // turn left
        if (headingComputed >= 24) {
          steering.write( STEER_NEUTRAL - 24 );
        } else {
          steering.write( STEER_NEUTRAL - headingComputed );
        }
        motor.write( MOTOR_NEUTRAL + 8 );
      } else {
        headingComputed = headingComputed - 360;
        
        // turn right
        if (headingComputed < -24) {
          steering.write( STEER_NEUTRAL + 24 );
          motor.write( MOTOR_NEUTRAL + 8 );
        } else {
          steering.write( STEER_NEUTRAL - headingComputed );
          motor.write( MOTOR_NEUTRAL + 8);
        }
      }
      
   }
}
