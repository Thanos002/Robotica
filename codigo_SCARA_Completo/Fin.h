//
// Created by thano on 28.10.2022.
//

#ifndef CODIGO_SCARA_COMPLETO_FIN_H
#define CODIGO_SCARA_COMPLETO_FIN_H

#include<Arduino.h>

class Fin
{
protected:
    int mypin;
public:
    void init();
    Endstop(int pin) { mypin = pin; };
    bool pressed();
};

#endif //CODIGO_SCARA_COMPLETO_FIN_H
