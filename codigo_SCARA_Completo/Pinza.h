//
// Created by thano on 28.10.2022.
//

#ifndef CODIGO_SCARA_COMPLETO_PINZA_H
#define CODIGO_SCARA_COMPLETO_PINZA_H

#include "Servo.h"

class Pinza {

public:

    bool estado; //Estado 1= Cerrado, Estado 2= Abierto


    Pinza();
    void move();
    int getStado();
    void setStado(int);
    void init(int);

private:
    int pinza_pin;
    Servo pinza;

};


#endif //CODIGO_SCARA_COMPLETO_PINZA_H
