//
// Created by thano on 28.10.2022.
//

#ifndef CODIGO_SCARA_COMPLETO_ENCODER_H
#define CODIGO_SCARA_COMPLETO_ENCODER_H

#include <Arduino.h>
//#include <arm.h>

#include "SimplyAtomic.h"

class Encoder_p
{
private:
    int pin_canal_A;
    int pin_canal_B;
    volatile int posicion_tics;
    float posicion_grados;
    float grados_por_tic;

public:
    Encoder_p(int, int, float);
    void init();
    void actualizar_posicion(); //Llamar con interrupcion en canal A.
    float getPosicionGrados();
    int getTics();
    void resetPosicion(); //Pone la posicion a 0;
    void setPosicionGrados(int);

};


#endif //CODIGO_SCARA_COMPLETO_ENCODER_H
