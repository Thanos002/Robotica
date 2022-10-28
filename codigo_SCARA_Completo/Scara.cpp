//
// Created by thano on 28.10.2022.
//

#include "Scara.h"
#include "Pinout.h"
#include "FlexyStepper.h"
#include "Pinza.h"
#include "Giro.h"
#include <PID_v1.h>

Scara::Scara()
{
    MyMotor[D] = new Motor(PIN_MOTORA_IN1, PIN_MOTORA_IN2, PIN_MOTORA_PWM);
    MyMotor[I] = new Motor(PIN_MOTORB_IN1, PIN_MOTORB_IN2, PIN_MOTORB_PWM);
    MyMotor[Z] = new Motor(PIN_MOTORC_IN1, PIN_MOTORC_IN2, PIN_MOTORC_PWM);

    MyEncoder[D] = new Encoder_p(PIN_MOTORA_CANALA, PIN_MOTORA_CANALB, TICS_A);
    MyEncoder[I] = new Encoder_p(PIN_MOTORB_CANALA, PIN_MOTORB_CANALB, TICS_B);

    MyFin[D] = new Endstop(PIN_ENDSTOPA);
    MyFin[I] = new Endstop(PIN_ENDSTOPB);
    MyFin[Z] = new Endstop(PIN_ENDSTOPC);

    MyControl[D] = new PIDMotor(MyMotor[D], MyEncoder[D], TICS_D);
    MyControl[I] = new PIDMotor(MyMotor[I], MyEncoder[I], TICS_I);

    MyStepper = new FlexyStepper(PIN_STEP,PIN_DIR,RESET,SLEEP);
    MyPinza = new Pinza(PINZA);
    MyGiro = new Giro(GIRO);

    pidStatus =true;
    homing = false;
    setfree = false;
    setlock = false;
    this->setPosicionBrazos(0,0,0,0);
}

void Scara::init()
{
    MyMotor[D]->init();
    MyMotor[I]->init();

    MyEncoder[D]->init();
    MyEncoder[I]->init();

    MyFin[D]->init();
    MyFin[I]->init();
    MyFin[Z]->init();

    MyControl[D]->init();
    MyControl[I]->init();

    FlexyStepper->connectToPins(STEP, DIR,  MS1, MS2, MS3, SLEEP, SRESET, ENAPIN);
    MyPinza->init(PINZA);
    MyGiro->init(GIRO);
}

Motor& Scara::getMotor(int which)
{
    return *misMotores[which];
}

Encoder_p& Scara::getEncoder(int which)
{
    return *misEncoders[which];
}

Controlposicion& Scara::getControlposicion(int which)
{
    return *misControles[which];
}

Stepper& Scara::getStepper()
{
    return *MyStepper;
}

Pinza& Scara::getPinza()
{
    return *MyPinza;
}

void Scara::setPosicionBrazos(float gradosD, float gradosI, float mmstepper, float giro, int pinza)
{

    misControles[D]->setPosicionGrados(gradosD);
    delay(400);
    misControles[I]->setPosicionGrados(gradosI);
    delay(400);

    pidStatus = true;
    setfree = false;
    homing = false;
    setlock = false;

    myStepper->moveToPositionInMillimeters(mmstepper);
    myGiro->move(giro);
    if(myPinza->getStado!=pinza){
        myPinza->move();
    }
}

void Scara::setPosicionBrazos_pulsos(int ticsA, int ticsB, float setps)
{
    misControles[A]->getPID().SetOutputLimits(-155,155);
    misControles[B]->getPID().SetOutputLimits(-155,155);
    misControles[C]->getPID().SetOutputLimits(-155,155);

    misControles[A]->setPosicionTics(ticsA);
    delay(400);
    misControles[B]->setPosicionTics(ticsB);
    delay(400);
    misControles[C]->setPosicionTics(ticsC);
    pidStatus = true;
    setfree = false;
    homing = false;
    setlock = false;
    miStepper->moveToPositionInMillimeters(mmstepper);
}

void Scara::RobotLogic()
{
    if(pidStatus == true) //Enciende o apaga el PID
    {
        MyControl[D]->control_logic();
        MyControl[I]->control_logic();
    };

    if(homing == true)
    {
        this -> goHome();
    }

    if(setfree == true)
    {
        this -> setFree();
    }

    if(setlock == true)
    {
        this->setLock();
    }

    if((MyFin[D]->pressed() && (MyFin[I]->pressed())))) digitalWrite(LED_BUILTIN, HIGH);
    else digitalWrite(LED_BUILTIN, 0x0);

    MyStepper->moveToPositionInMillimeters(0);
    MyPinza->move();
    
}