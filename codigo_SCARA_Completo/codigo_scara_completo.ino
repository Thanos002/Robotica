#include <Arduino.h>
#include <SimplyAtomic.h>
//#include <util/atomic.h>  // For the ATOMIC_BLOCK macro
#include <arm.h>
#include "PID_v1.h"
#include "Scara.h"

Scara myScara;

void handler_encoderD()
{
    myScara.getEncoder(D).actualizar_posicion();
}
void handler_encoderI()
{
    myScara.getEncoder(I).actualizar_posicion();
}

void setup() {
    myScara.init();

    Serial.begin(9600);

    attachInterrupt(digitalPinToInterrupt(ENCDA), handler_encoderD, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCIA), handler_encoderI, CHANGE);

    myScara.getControlposicion(D).setGains(KP_D, KI_D, KD_D);
    myScara.getControlposicion(I).setGains(KP_I, KI_I, KI_D);
}

int dird, diri, gradd, gradi, alto, pinza, pulsosd, pulsosi;

void loop() {

    // wait until serial available
    if (Serial.available()) {
        // opcode syntax (int dird, int diri, int gradd, int gradi, int alto, , int giro, int pinza)
        dird = (int)Serial.readStringUntil(',').toInt();
        diri = (int)Serial.readStringUntil(',').toInt();
        gradd = (int)Serial.readStringUntil(',').toInt();
        gradi = (int)Serial.readStringUntil(',').toInt();
        alto = (int)Serial.readStringUntil(',').toInt();
        giro = (int)Serial.readStringUntil(',').toInt();
        pinza = (int)Serial.readStringUntil('\n').toInt();
        pulsosd = signCorrection(dird, gradd);
        pulsosi = signCorrection(diri, gradi);
    }

    myScara.setPosicionArticulares(pulsosd, pulsosi, alto, grio, pinza);
}

int signCorrection(int dir, int grad) {
    if (grad >= 0 && grad <= 360) {
        pulsos = (int)((float)grad * (float)((float)ratio / (float)360);
    }
    if (dir == 0) {
        pulsos = -pulsos
    } else {
        //Serial.print("Valor por encima de 360º");
        return 0;
    }
    return pulsos;
}