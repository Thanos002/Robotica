//
// Created by thano on 28.10.2022.
//

#ifndef CODIGO_SCARA_COMPLETO_GIRO_H
#define CODIGO_SCARA_COMPLETO_GIRO_H

#include <Servo.h>

class Giro {

public:

    int angle;


    Giro();
    void move(int);
    int getAngle();
    void init(int);

private:
    int giro_pin;
    Servo giro;

};

#endif //CODIGO_SCARA_COMPLETO_GIRO_H
