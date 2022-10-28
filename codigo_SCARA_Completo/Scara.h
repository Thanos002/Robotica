//
// Created by thano on 28.10.2022.
//

#ifndef CODIGO_SCARA_COMPLETO_SCARA_H
#define CODIGO_SCARA_COMPLETO_SCARA_H

#include "Motor.h"
#include "Encoder.h"
#include "PIDMotor.h"
#include "Stepper.h"
#include "Endstop.h"
#include "Pinza.h"
#include <Arduino.h>

class Pullup
{
private:
    Endstop* misEndstops[4];
    Controlposicion* misControles[3];
    Motor* misMotores[3];
    Motor* Motor_A;
    Encoder_p* misEncoders[3];
    Stepper* miStepper;
    //stepper
    Pinza* miPinza;

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
    Controlposicion& getControlposicion(int);
    Stepper& getStepper();
    Fin& getEndstop(int);
    Pinza& getPinza();
    Giro& getGiro();

    void setPosicionArticulares(float, float, float, float, int, int);
    void setPosicionArticulares_tics(int,int,int,float, int, int);

    void goHome(); //Hace homing al robot
    void setFree();
    void setLock(); //apaga los motores

    void RobotLogic(); //Maquina de estados del robot


};

#endif //CODIGO_SCARA_COMPLETO_SCARA_H
