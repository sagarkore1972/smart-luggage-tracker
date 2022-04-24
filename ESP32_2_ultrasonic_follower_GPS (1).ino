#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

float latitude , longitude;
String  latitude_string , longitude_string;

TinyGPSPlus gps;
HardwareSerial SerialGPS(2);

#define a   27
#define b   26
#define c   25
#define d   32

#define trigPin_Left    22
#define echoPin_Left    23
#define trigPin_Right   18
#define echoPin_Right   19


long duration_Left, distance_Left;
long duration_Right, distance_Right;


void forward()
  {
    digitalWrite(a,LOW);
    digitalWrite(b,HIGH);
    digitalWrite(c,LOW);
    digitalWrite(d,HIGH);
  }


void backward()
  {
    digitalWrite(a,HIGH);
    digitalWrite(b,LOW);
    digitalWrite(c,HIGH);
    digitalWrite(d,LOW);
  }
  

void left()
  {
    digitalWrite(a,LOW);
    digitalWrite(b,HIGH);
    digitalWrite(c,HIGH);
    digitalWrite(d,LOW);
  }


void right()
  {
    digitalWrite(a,HIGH);
    digitalWrite(b,LOW);
    digitalWrite(c,LOW);
    digitalWrite(d,HIGH);
  }


void stops()
  {
    digitalWrite(a,LOW);
    digitalWrite(b,LOW);
    digitalWrite(c,LOW);
    digitalWrite(d,LOW);
  }


void get_location()
  {
    while (SerialGPS.available() > 0) 
    {
      if (gps.encode(SerialGPS.read()))
        {
          if (gps.location.isValid())
            {
              latitude = gps.location.lat();
              latitude_string = String(latitude , 6);
              longitude = gps.location.lng();
              longitude_string = String(longitude , 6);
              
              Serial.print("Latitude = ");
              Serial.println(latitude_string);
              Serial.print("Longitude = ");
              Serial.println(longitude_string);
              Serial.println();
            }       
        }
    }  
  }


void get_command()
  {
    if(SerialBT.available())
    {
      String BT_data = SerialBT.readString();

      if(BT_data == "getloc")
        {
          SerialBT.print("Lat= ");
          SerialBT.print(latitude_string);
          SerialBT.print(",Lon= ");
          SerialBT.print(longitude_string);
          SerialBT.print(",");
        }
    }
  }


void get_distance_Left()
  {
      delay(20);                                         // waits for 5 milli-seconds
      digitalWrite(trigPin_Left, LOW);                       // make Trigger Pin of Ultrasonic sensor LOW
      delayMicroseconds(2);                             // waits for 2 micro-seconds
    
      digitalWrite(trigPin_Left, HIGH);                      // make Trigger Pin of Ultrasonic sensor HIGH
      delayMicroseconds(10);                            // waits for 10 micro-seconds
      digitalWrite(trigPin_Left, LOW);                       // make Trigger Pin of Ultrasonic sensor LOW
    
      duration_Left = pulseIn(echoPin_Left, HIGH);                // make Echo Pin of Ultrasonic sensor HIGH
      distance_Left = ((duration_Left / 2) / 29.1);               // calculate the distance as function of time
  }



void get_distance_Right()
  {
      delay(20);                                         // waits for 5 milli-seconds
      digitalWrite(trigPin_Right, LOW);                       // make Trigger Pin of Ultrasonic sensor LOW
      delayMicroseconds(2);                             // waits for 2 micro-seconds
    
      digitalWrite(trigPin_Right, HIGH);                      // make Trigger Pin of Ultrasonic sensor HIGH
      delayMicroseconds(10);                            // waits for 10 micro-seconds
      digitalWrite(trigPin_Right, LOW);                       // make Trigger Pin of Ultrasonic sensor LOW
    
      duration_Right = pulseIn(echoPin_Right, HIGH);                // make Echo Pin of Ultrasonic sensor HIGH
      distance_Right = ((duration_Right / 2) / 29.1);               // calculate the distance as function of time
  }
  

void setup() 
{
  Serial.begin(9600);
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);
  SerialBT.begin("Follower Robot");
  
  pinMode(trigPin_Left, OUTPUT);
  pinMode(echoPin_Left, INPUT);
  pinMode(trigPin_Right, OUTPUT);
  pinMode(echoPin_Right, INPUT);

  pinMode(a, OUTPUT);
  pinMode(b, OUTPUT);
  pinMode(c, OUTPUT);
  pinMode(d, OUTPUT);  
}

void loop() 
{
  get_distance_Left();
  get_distance_Right();

  Serial.print(distance_Left);
  Serial.print(" ");
  Serial.print(distance_Right);
  Serial.print(" ");

  if((distance_Left < 20) && (distance_Right > 20))
    {
      left();
      Serial.println("Left");
    }

  if((distance_Left > 20) && (distance_Right < 20))
    {
      right();
      Serial.println("Right");
    }

  if((distance_Left < 20) && (distance_Right < 20))
    {
      forward();
      Serial.println("Forward");
    }

  if((distance_Left > 20) && (distance_Right > 20))
    {
      stops();
      Serial.println("Stop");
    }

  get_location();
  get_command();
}
