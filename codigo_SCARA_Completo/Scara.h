//
// Created by thano on 28.10.2022.
//

#ifndef CODIGO_SCARA_COMPLETO_SCARA_H
#define CODIGO_SCARA_COMPLETO_SCARA_H

#include "Motor.h"
#include "Encoder.h"
#include "PIDMotor.h"
#include "FlexyStepper.h"
#include "Fin.h"
#include "Pinza.h"
#include "Giro.h"
#include <Arduino.h>

class Scara
{
private:
    Fin* MyFin[2];
    PIDMotor* MyControl[2];
    Motor* MyMotor[2];
    Motor* Motor_A;
    Encoder_p* MyEncoder[1];
    FlexyStepper* MyStepper;
    //stepper
    Pinza* MyPinza;
    Giro* MyGiro;

    bool pidStatus;
    bool homing; // variable que indica a logic que debe estar en homing
    bool setfree; // variable que indica a logic que debe estar en free
    bool setlock; // variable que indica a logic que debe estar en lock
    bool a,b,c; //variable que indica si endstop esta en ON
public:
    Scara();
    void init();

    Motor& getMotor(int);
    Encoder_p& getEncoder(int);
    PIDMotor& getControlposicion(int);
    FlexyStepper& getStepper();
    Fin& getEndstop(int);
    Pinza& getPinza();
    Giro& getGiro();

    void setPosicionBrazos(float, float, float, float, int);
    void setPosicionBrazos_pulsos(int,int,int,float, int);

    void goHome(); //Hace homing al robot
    void setFree();
    void setLock(); //apaga los motores

    void SerialPrintPosicionTics();
    void SerialPrintErrores();
    void RobotLogic(); //Maquina de estados del robot


};

#endif //CODIGO_SCARA_COMPLETO_SCARA_H
