//#include <SimplyAtomic.h>
//#include <util/atomic.h>  // For the ATOMIC_BLOCK macro
//#include <arm.h>
//#include "PID_v1.h"

#include "Scara.h"
#include "Pinout.h"

Scara myScara;

void handler_encoderD() {
  myScara.getEncoder(D).actualizar_posicion();
}
void handler_encoderI() {
  myScara.getEncoder(I).actualizar_posicion();
}

void timer_handler()
{
  myScara.RobotLogic();
}

void setup() {
  Serial.begin(9600);
  Serial.println("m");


  myScara.init();

  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);

  attachInterrupt(digitalPinToInterrupt(ENCDA), handler_encoderD, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCIA), handler_encoderI, CHANGE);

  myScara.getControlposicion(D).setGains(KP_D, KI_D, KD_D);
  myScara.getControlposicion(I).setGains(KP_I, KI_I, KI_D);

  Serial.println("Setup done");
}

float gradd, gradi, alto, pinza, pulsosd, pulsosi, giro;

void loop() {
  Serial.println("Waiting...");
  // wait until serial available
  if (Serial.available()) {
    // opcode syntax (int dird, int diri, int gradd, int gradi, int alto, , int giro, int pinza)
    gradd = (float)Serial.readStringUntil(',').toInt();
    gradi = (float)Serial.readStringUntil(',').toInt();
    alto = (float)Serial.readStringUntil(',').toInt();
    giro = (float)Serial.readStringUntil(',').toInt();
    pinza = (float)Serial.readStringUntil('\n').toInt();
    Serial.println("Received");

    myScara.setPosicionBrazos(gradd, gradi, alto, giro, pinza);
    
  }

  delay(1000);
  Serial.println("Waiting...");
  myScara.setPosicionBrazos(1, 1, 1, 0, 0);

#ifdef DEBUGGING_
  myScara.printMovidas();
  myScara.printGrados();
  myScara.SerialPrintPosicionTics();
#endif
}
