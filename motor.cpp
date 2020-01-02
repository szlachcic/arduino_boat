#include "Arduino.h"
#include "motor.h"


void MOTOR::begin()
{
    pinMode(13, OUTPUT); 
    pinMode(12, OUTPUT);
    pinMode(11, OUTPUT); 
    pinMode(10, OUTPUT);
  
}


void MOTOR::drive(char motor, int dir, int speed)
{
    if (motor=='L')
    {
        digitalWrite(13, dir);
        analogWrite(12, speed); 
    }
    else if (motor=='R')
    {
        digitalWrite(11, dir);
        analogWrite(10, speed);  
    }
  
}


void MOTOR::stop()
{
    digitalWrite(13, 0);
    analogWrite(12, 0); 
    digitalWrite(11, 0);
    analogWrite(10, 0);
}