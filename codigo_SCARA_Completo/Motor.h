//
// Created by thano on 28.10.2022.
//

#ifndef CODIGO_SCARA_COMPLETO_MOTOR_H
#define CODIGO_SCARA_COMPLETO_MOTOR_H

#include "Arduino.h"

class Motor
{
public:
    Motor(int dir1, int dir2, int pwm);
    void init();
    void setFwd();
    void setBack();
    void setFree();
    void setStop();
    void setPWM(int level);
private:
    int _dir1, _dir2, _pwm;
};

#endif //CODIGO_SCARA_COMPLETO_MOTOR_H
