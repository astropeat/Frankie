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
double distance = 0;
//double Blue_Post_Lat = 37.428467;    // defines latitude & longitude of blue post in engineering quad, Stanford.
//double Blue_Post_Long = -122.17465;
int current_waypoint = 0;
struct Waypoint {
  double latitude;
  double longitude;
};
/*
Waypoint moffet_park[] = {
  { 37.401132, -122.073164 },
  { 37.401231, -122.073020 },
  { 37.401387, -122.073115 },
  { 37.401237, -122.073303 },
  { 37.401132, -122.073164 },
};
*/

Waypoint sparkfun_2014[] = {
  //{ 40.071374969556928, -105.22978898137808 },   // start/finish line, SF given
  { 40.071381, -105.229795 }, //start finish mine
  //{ 40.071258964017034, -105.23002602159977 },   // first corner, SF given
  { 40.071268, -105.230119}, // first turn mine
  { 40.07075596600771,  -105.22971798665822 },   // second corner, SF given
  //{ 40.076743, -105.229799}, // second turn mine
  { 40.070829978212714, -105.22953098639846 },   // hoop, SF given
  //{ 40.070976996794343, -105.22919101640582 },   // third corner, SF given
  { 40.070954, -105.229212}, // third turn mine
  { 40.071081016212702, -105.22919897921383 },   // ramp, SF given
  { 40.071284, -105.229419 },
  //{ 40.071331970393658, -105.22946602664888 },   // fourth corner, SF given
  { 40.071440, -105.229496}, //fourth turn mine
  //{ 40.071374969556928, -105.22978898137808 },   // start/finish line
  { 40.071381, -105.229795 }, //start finish mine

};
  
//double target_lat = moffet_park[0].latitude;
//double target_long = moffet_park[0].longitude;

double target_lat = sparkfun_2014[0].latitude;
double target_long = sparkfun_2014[0].longitude;


uint8_t deadman = 0;
long last_deadman = 0;
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
        last_deadman = millis();
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
  Serial.begin(115200);    //debug
  pinMode(led, OUTPUT);      // Set the LED pin as output
  Wire.begin();
  
  //Attach GPS
  uint8_t cmd[14] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};

  Serial3.begin(9600);   //gps
  Serial3.write(cmd, 14);
 

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
  
  if( i < 2 ) Serial.println("Failed to read compass");
  else {
    Serial.print("Current heading: ");
    Serial.print(int (headingValue / 10));     // The whole number part of the heading
    Serial.print(".");
    Serial.print(int (headingValue % 10));     // The fractional part of the heading
    Serial.println(" degrees");
    //delay(10);
  }
  
  /*
  Serial.print("Speed: ");
  Serial.println(motor.read());
  */
  
  //GPS
  while (Serial3.available() > 0) {
    int c = Serial3.read();
    //Serial.print((char)c);
    gps.encode(c);
  }
    
  if (gps.location.isUpdated()){
    gps_data.append((int)(gps.location.lat() * 1000000)); // Adds Latitude
    gps_data.append((int)(gps.location.lng() * 1000000)); // Adds Longitude
    gps_data.append(gps.satellites.value()); // Adds # satellites

    btooth.send(gps_data); // sends 'gps_data'
        
    Serial.print("LAT=");  Serial.println(gps.location.lat()*1000000);
    Serial.print("LONG="); Serial.println(gps.location.lng()*1000000);
    //Serial.print("NUM_SAT=");  Serial.println(gps.satellites.value());   
    
    distance = gps.distanceBetween(
      gps.location.lat(),                  //Distance between current location and desired location (Blue Post).
      gps.location.lng(),
      target_lat,
      target_long);
    if (distance <= 2.5){
      if (current_waypoint < 9){  //while position in array less than 5 go to next waypoint.
        current_waypoint++;
      }
      //target_lat = moffet_park[current_waypoint].latitude;
      //target_long = moffet_park[current_waypoint].longitude;
      target_lat = sparkfun_2014[current_waypoint].latitude;
      target_long = sparkfun_2014[current_waypoint].longitude;

      distance = gps.distanceBetween(
        gps.location.lat(),                  //Distance between current location and desired location (Blue Post).
        gps.location.lng(),
        target_lat,
        target_long);
    }
    
    double courseTo = gps.courseTo(
        gps.location.lat(),               //This here reads current location and writes a course to desired location (Blue Post).
        gps.location.lng(),
        target_lat,
        target_long);
        
    desired_heading = courseTo;
    
    Serial.println(desired_heading); 
  }
  
  /*
  if( millis() - last_deadman > 500 ) {
    deadman = 0;
    motor.write(MOTOR_NEUTRAL);
    steering.write(STEER_NEUTRAL);
  }*/
  
  //DO A THING
  //GO FORTH
   if (autonomous != 0 && deadman != 0) {
     
     int headingComputed = (headingValue/10); // range 0 to 360
     headingComputed -= desired_heading; // we expect this to be in the range 0-360 to 360+360
     headingComputed += (360 + 180); // add 360 so we're positive, 180 for bias
     headingComputed %= 360; // modulo 360 brings us back into the 0 to 360 range
     headingComputed -= 180; // remove the bias we added earlier, brings us back to -180 to 180
     
      //if ( headingComputed < 180 && headingComputed > 0 ) {
      if ( headingComputed > 0 ) {
        // turn left
        if (headingComputed >= 24) {
          steering.write( STEER_NEUTRAL - 24 );
        } else {
          steering.write( STEER_NEUTRAL - headingComputed );
        }
      } else {
        /*
        if( headingComputed > 0 ){
          headingComputed = headingComputed - 360;
        }*/
        
        // turn right
        if (headingComputed < -24) {
          steering.write( STEER_NEUTRAL + 24 );
        } else {
          steering.write( STEER_NEUTRAL - headingComputed );
        }
      }

      if //(distance <= 3){
        //motor.write(MOTOR_NEUTRAL); // stop
      //} else if 
      (distance >3 && distance <= 8){
        motor.write(MOTOR_NEUTRAL + 8);
      } else {
        motor.write( MOTOR_NEUTRAL + 10);  //go
      }
   }
}
