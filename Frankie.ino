#include <Protocol.h>
#include <Servo.h>

Protocol link(Serial); // create a protocol object called 'link'

Servo motor;
Servo steering;

int led = 13;

#define MOTOR_NEUTRAL 92
#define STEER_NEUTRAL 88

void linkCallback(Packet &p) {
  switch((char)p.type) {
    case 'V': 
    //Speed & Direction - Vector
      {
        digitalWrite(led, HIGH);
        int8_t spee = p.reads8();
        int8_t dir = p.reads8();
        motor.write( spee + MOTOR_NEUTRAL );
        steering.write( dir + STEER_NEUTRAL ); 
      }
      break;
      /*
    case 'D': 
    //dead man's switch
      {
        int8_t deadman = p.reads8();
        if (deadman == 0)
        {
          motor.write(MOTOR_NEUTRAL);
          steering.write(STEER_NEUTRAL);
        }
      }
      break;
      */
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
  link.setCallback(linkCallback);
  motor.attach(11);
  steering.attach(10);
  Serial.begin(57600);
  
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  delay(500);
  digitalWrite(led, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  link.poll();
}
