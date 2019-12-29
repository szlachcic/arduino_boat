#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "Arduino.h"

class MOTOR
{
public:
    int speedL = 0;
    int speedP = 0;


    void stop();
    void begin();
    void drive(int dir, int speed);
    
};

#endif