/*
  Author:

  Learning Intention:
  The students will learn how to connect and control a servo motor.

  Success Criteria:
    1.  I understand how to connect the servo motor
    2.  I can manually write different degrees of movement to the servo
    3.  I can map a potentiometer to a servo and control its movement
    4.  I understand that a 180deg servo angle of movement is set by a
        frequency signal sent from the microcontroller

  Student Notes: 

  Documentation:
    https://www.sparkfun.com/servos
    https://github.com/arduino-libraries/Servo <-- We are still using this library

  Schematic:
    https://www.tinkercad.com/things/lQ9RyYJRoLn?sharecode=MKlN0A7R0WGodkdTRKkPJO7I8PeI5L_GCR7pCclQ0qM
    https://github.com/TempeHS/TempeHS_Ardunio_Bootcamp/blob/main/10.servoMotor/Bootcamp-servoMotor.png
*/

#include<Arduino.h>
#include<U8g2lib.h>
#include<SPI.h>
#include<Wire.h>

#include <Servo.h>
#include "Ultrasonic.h"

unsigned static int servoPin = 6;
unsigned static int usPin = 5;


Servo myservo;  
Ultrasonic us_sensor(usPin);

int potpin = A1; 
int val;


void setup() {
  myservo.attach(servoPin);
  Serial.begin(9600);
  Serial.println("Baud 9600");
  Serial.println("-------");

}

void loop() {

  unsigned long range_in_cm;
  range_in_cm = us_sensor.distanceRead();
  range_in_cm = map(range_in_cm, 0, 1023, 0, 180);

  val = analogRead(potpin);
  val = map(val, 0, 1023, 0, 180);

-

  

}