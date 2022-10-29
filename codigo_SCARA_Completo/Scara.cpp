//
// Created by thano on 28.10.2022.
//

#include "Scara.h"
#include "Pinout.h"
#include "FlexyStepper.h"
#include "Pinza.h"
#include "Giro.h"
#include "PID_v1.h"

Scara::Scara()
{
    MyMotor[D] = new Motor(DCD_1, DCD_2, PWMD);
    MyMotor[I] = new Motor(DCI_1, DCI_2, PWMI);

    MyEncoder[D] = new Encoder_p(ENCDA, ENCDB, FACTOR);
    MyEncoder[I] = new Encoder_p(ENCIA, ENCIB, FACTOR);

    MyFin[D] = new Fin(FIND);
    MyFin[I] = new Fin(FINI);
    MyFin[Z] = new Fin(FINZ);

    MyControl[D] = new PIDMotor(MyMotor[D], MyEncoder[D], RATIO);
    MyControl[I] = new PIDMotor(MyMotor[I], MyEncoder[I], RATIO);

    MyStepper = new FlexyStepper();
    MyPinza = new Pinza();
    MyGiro = new Giro();

    pidStatus =true;
    homing = false;
    setfree = false;
    setlock = false;
    this->setPosicionBrazos(0,0,0,0,0);

    Serial.println("Constr done");
}

void Scara::init()
{
    MyMotor[D]->init();
    MyMotor[I]->init();

    MyEncoder[D]->init();
    MyEncoder[I]->init();

    MyFin[D]->init(FIND);
    MyFin[I]->init(FINI);
    MyFin[Z]->init(FINZ);

    MyStepper->connectToPins(STEP, DIR,  MS1, MS2, MS3, SLEEP, SRESET, ENAPIN);
    MyPinza->init(PINZA);
    MyGiro->init(GIRO);

    Serial.print("Scara init");
}

Motor& Scara::getMotor(int which)
{
    return *MyMotor[which];
}

Encoder_p& Scara::getEncoder(int which)
{
    return *MyEncoder[which];
}

PIDMotor& Scara::getControlposicion(int which)
{
    return *MyControl[which];
}

FlexyStepper& Scara::getStepper()
{
    return *MyStepper;
}

Pinza& Scara::getPinza()
{
    return *MyPinza;
}

Giro& Scara::getGiro()
{
    return *MyGiro;
}

void Scara::setPosicionBrazos(float gradosD, float gradosI, float mmstepper, float giro, int pinza)
{

    MyControl[D]->setPosicionGrados(gradosD);
    delay(400);
    MyControl[I]->setPosicionGrados(gradosI);
    delay(400);

    pidStatus = true;
    setfree = false;
    homing = false;
    setlock = false;

    MyStepper->moveToPositionInMillimeters(mmstepper);
    MyGiro->move(giro);
    if(MyPinza->getStado()!=pinza){
        MyPinza->move();
    }
}

void Scara::setPosicionBrazos_pulsos(int ticsA, int ticsB, int steps, float giro, int pinza)
{
    MyControl[D]->getPID().SetOutputLimits(-155,155);
    MyControl[I]->getPID().SetOutputLimits(-155,155);

    MyControl[D]->setPosicionTics(ticsA);
    delay(400);
    MyControl[I]->setPosicionTics(ticsB);
    delay(400);
    pidStatus = true;
    setfree = false;
    homing = false;
    setlock = false;
    MyStepper->moveToPositionInSteps(steps);
    MyGiro->move(giro);
    if(MyPinza->getStado()!=pinza){
        MyPinza->move();
    }
}

void Scara::RobotLogic()
{
    if(pidStatus == true) //Enciende o apaga el PID
    {
        MyControl[D]->control_logic();
        MyControl[I]->control_logic();
    }

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

    if((MyFin[D]->pressed() && (MyFin[I]->pressed()))){
      digitalWrite(LED_BUILTIN, HIGH);}
    else digitalWrite(LED_BUILTIN, 0x0);

    MyStepper->moveToPositionInMillimeters(0);
    MyPinza->move();
    
}

void Scara::SerialPrintPosicionTics()
{
    Serial.println(MyEncoder[D]->getTics());  
    Serial.println(MyEncoder[I]->getTics());
    Serial.println(MyEncoder[Z]->getTics()); 
}

void Scara::SerialPrintErrores()
{
    Serial.println(MyControl[D]->getError());
    Serial.println(MyControl[I]->getError());
    Serial.println(MyControl[Z]->getError());
}