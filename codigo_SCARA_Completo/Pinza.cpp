//
// Created by thano on 28.10.2022.
//

#include "Pinza.h"
#include "Pinout.h"
#include <arduino.h>
#include <Servo.h>

Pinza::Pinza() {

    estado=false;
}

void Pinza::init(int servo_Pin){

    pinza.attach(servo_Pin);
}

void Pinza::move() {

    if(estado == 1)
    {
        pinza.write(ANG_MAX);
        estado = 0;
    }                  //Estado en 0, abierto
    else
    {
        pinza.write(ANG_MIN);
        estado = 1;
    }                         //Estado en 1, cerrado
}

int Pinza::getStado() { return estado; }

void Pinza::setStado(int stat) { estado=stat; }