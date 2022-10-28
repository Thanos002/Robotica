//
// Created by thano on 28.10.2022.
//

#include "PIDMotor.h"

PIDMotor::PIDMotor(Motor* Motor, Encoder_p* Encoder_p, float tics_vuelta)
{
    myMotor = Motor;
    myEncoder = Encoder_p;
    kp = 1;
    ki = 0;
    kd = 0;
    myMotor ->setFree();
    tiempo_previo = 0;
    referencia_tics = 0;
    error_acumulado = 0;
    error_previo = 0;
    grados_por_tic = 360.0/ tics_vuelta;
    //motorApagado = true;
    myPID = new PID(&in,&out,&set,kp,ki,kd,DIRECT);
    myPID->SetMode(AUTOMATIC);
    myPID->SetSampleTime(1);
    myPID->SetOutputLimits(-155,155);
    in = 0;
    out = 0;
    set = 0;
}

void PIDMotor::setGains(int kp_, int ki_, int kd_)
{
    kp = kp_;
    ki = ki_;
    kd = kd_;
}

void PIDMotor::setPosicionTics(int ref)
{
    referencia_tics = ref;
    motorApagado = false;
}

void PIDMotor::setPosicionGrados(float ref)
{
    referencia_tics = ref/ grados_por_tic;
    motorApagado = false;
}

void PIDMotor::apagarMotor()
{
    motorApagado = true;
}
void PIDMotor::control_logic()
{
    set = referencia_tics;

    in = myEncoder->getTics();
    myPID->Compute();
    if(out > 0)
    {
        myMotor ->setFwd();
        myMotor ->setPWM(out);
    }
    else
    {
        myMotor->setBack();
        myMotor->setPWM(abs(out));
    }
}