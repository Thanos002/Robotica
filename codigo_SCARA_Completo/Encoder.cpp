//
// Created by thano on 28.10.2022.
//

#include "Encoder.h"
#include "Pinout.h"


Encoder_p::Encoder_p(int canal_A, int canal_B, float tics_vuelta)
{
    pin_canal_A = canal_A;
    pin_canal_B = canal_B;
    grados_por_tic = 360.0/(tics_vuelta);
    posicion_tics = 0;

}

void Encoder_p::init()
{
    pinMode(pin_canal_A, INPUT_PULLUP);
    pinMode(pin_canal_B, INPUT_PULLUP);
}

void Encoder_p::actualizar_posicion()
{
    if(digitalRead(pin_canal_A) == HIGH)
    {
        if(digitalRead(pin_canal_B) == LOW)
            posicion_tics++;
        else
            posicion_tics--;
    }

}


float Encoder_p::getPosicionGrados()
{
    return (grados_por_tic*getTics());
}

int Encoder_p::getTics()
{
    int pos;
    ATOMIC()
    {
        pos = posicion_tics;
    }
    return posicion_tics;
}

void Encoder_p::resetPosicion()
{
    posicion_tics = 0;
}

void Encoder_p::setPosicionGrados(int grados)
{
    posicion_tics = grados/grados_por_tic;
}