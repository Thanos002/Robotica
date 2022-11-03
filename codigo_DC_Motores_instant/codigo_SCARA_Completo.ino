#include <Arduino.h>

// homing functions

//  if (initialized == false)
//  {
//    homed(DCD_1, DCD_2, FINI, PWMD, posid, DPULSOSMAX); // derecho - interno
  //  homei(DCI_1, DCI_2, FINE, PWMI, posii, IPULSOSMAX); // iz - ext.
  //  homestepper(DIR, STEP, FINZ);
//      delay(5000);
//    Serial.println("INIT END");
//    initialized = true;
//  }
  
float homed(int motorpin1, int motorpin2, int finpin, int PWMpin, int posi, int DPULSOSMAX) {
  analogWrite(PWMpin, 100);
  posprev = posi;
  while (digitalRead(finpin) == 0) {
    digitalWrite(motorpin1, LOW);
    digitalWrite(motorpin2, HIGH);
  }
  digitalWrite(motorpin1, LOW);
  digitalWrite(motorpin2, LOW);

  float delta = fabs(posprev - posi);
  posi = -DPULSOSMAX / 2;
}

float homei(int motorpin1, int motorpin2, int finpin, int PWMpin, int posi, int DPULSOSMAX) {
  analogWrite(PWMpin, 100);
  posprev = posi;
  while (digitalRead(finpin) == 0) {
    digitalWrite(motorpin1, HIGH);
    digitalWrite(motorpin2, LOW);
  }
  digitalWrite(motorpin1, LOW);
  digitalWrite(motorpin2, LOW);

  float delta = fabs(posprev - posi);
  posi = -IPULSOSMAX / 2;
}

float homestepper(int dirpin, int steppin, int finpin) {
  digitalWrite(SLEEP, HIGH);
  delay(1000);
  while (digitalRead(finpin) == 1) {
    digitalWrite(dirpin, LOW);
    digitalWrite(steppin, HIGH);
    delayMicroseconds(3000);
    digitalWrite(steppin, LOW);
    delayMicroseconds(3000);
  }
  digitalWrite(steppin, LOW);
  digitalWrite(SLEEP, LOW);

  alto = 0;
}
