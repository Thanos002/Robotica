//
// Created by thano on 28.10.2022.
//

#ifndef CODIGO_SCARA_COMPLETO_PIDMOTOR_H
#define CODIGO_SCARA_COMPLETO_PIDMOTOR_H
#include<Arduino.h>
#include"Motor.h"
#include"Encoder.h"
#include"Pinout.h"
#include "PID_v1.h"

class PIDMotor
{
private:
    float kp;
    float kd;
    float ki;
    double in, out, set;
    long tiempo_previo; //para calcular deltaT
    int referencia_tics; //referencia para el PID en pulsos
    float error_acumulado; //Se va sumando el error
    float error_previo;
    float grados_por_tic;
    bool motorApagado; //senal digital que habilita o deshabilita el PID();
    Motor* myMotor;
    Encoder_p* myEncoder;
    PID* myPID;
public:
    Controlposicion(Motor*, Encoder_p*, float);
    void setPosicionTics(int); //LLeva al motor a una posicion en tics;
    void setPosicionGrados(float); //Lleva al motor a una posicion en grados;
    void apagarMotor(); //deja libre el motor;
    void setGains(int, int, int); //Para dar las ganancias kp, ki y kd
    void control_logic(); // Donde se actualizan los valores del PID();
    float getError(){return error_previo;};
    PID& getPID(){return *myPID;};
};

#endif //CODIGO_SCARA_COMPLETO_PIDMOTOR_H
