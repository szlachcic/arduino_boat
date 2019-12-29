#include "Arduino.h"
#include "motor.h"


void MOTOR::begin()
{
    pinMode(13, OUTPUT); 
    pinMode(12, OUTPUT);
  
}


void MOTOR::drive(int dir, int speed)
{
    digitalWrite(13, dir);
    analogWrite(12, speed); 
}


void MOTOR::stop()
{
    digitalWrite(13, 0);
    analogWrite(12, 0); 
}