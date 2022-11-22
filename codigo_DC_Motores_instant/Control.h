#ifndef Control
#define Control

#include "Control.h"
#include "math.h"
#include <Arduino.h>

#define DLENGTH 91.61   //primer brazo
#define ILENGTH 105.92  //segundo brazo
#define RAD2DEG 57.2957795
#define DEG2RAD 0.01745329252

float lawOfCosines(float a, float b, float c) {
  return acosf((a * a + b * b - c * c) / (2.0f * a * b));
}

float lawOfCosines2(float a, float b, float c) {
  return acosf((a * a - b * b - c * c) / (2.0f * b * c));
}

float distance(float x, float y) {
  return sqrt(x * x + y * y);
}

void cinematicaInversa(float x, float y, float *Q1, float *Q2) {
  *Q1 = *Q1 + 90;  // adjust zero position for calcs
  float r;
  float aux1, aux2;
  r = distance(x, y);

  if (y < 0 || r < ILENGTH/2) {  // fuera de campo de trabajo
    Serial.print(4);
    Serial.print(",");
    Serial.print(5);
    Serial.print(",");
    Serial.print(x);
    Serial.print(",");
    Serial.println(y);
  } else {
    if (r > (DLENGTH + ILENGTH)) {
      r = (DLENGTH + ILENGTH) - 0.001f;
      Serial.print(4);
      Serial.print(",,");
      Serial.print(5);
      Serial.print(",");
      Serial.print(x);
      Serial.print(",");
      Serial.println(y);
    }

    if (x > 0) {
      aux1 = (atan2(y, x)) * RAD2DEG;                        //beta
      aux2 = (lawOfCosines(r, DLENGTH, ILENGTH)) * RAD2DEG;  //alpha
      *Q1 = (aux1 - aux2);
      *Q2 = (lawOfCosines2(r, DLENGTH, ILENGTH)) * RAD2DEG;
    }
    if (x == 0) {
      x = x + 0.001f;
      aux1 = (atan2(y, x)) * RAD2DEG;                        //beta
      aux2 = (lawOfCosines(r, DLENGTH, ILENGTH)) * RAD2DEG;  //alpha
      *Q1 = (aux1 - aux2);
      *Q2 = (lawOfCosines2(r, DLENGTH, ILENGTH)) * RAD2DEG;
    }
    if (x < 0) {
      aux1 = (atan2(-y, -x)) * RAD2DEG;                      //beta
      aux2 = (lawOfCosines(r, DLENGTH, ILENGTH)) * RAD2DEG;  //alpha
      *Q1 = 180 + (aux1 - aux2);                              // 180 + un numero negativo
      *Q2 = (lawOfCosines2(r, DLENGTH, ILENGTH)) * RAD2DEG;
    }
  }
  *Q1 = *Q1 - 90;  // adjust zero position for rest of code
}

void jacobiana_inversa(float q1, float q2, float Vx, float Vy) {
  q1 = q1*DEG2RAD;
  q2 = q2*DEG2RAD;
  float w1, w2;
  float L1 = 1, L2 = 1;
  if (cos(q1) * sin(q1 + q2) - sin(q1) * cos(q1 + q2) == 0) {
    Serial.print(4);
    Serial.print(",");
    Serial.print(4);
    Serial.print(",");
    Serial.print(q1);
    Serial.print(",");
    Serial.println(q2);
  } else {
    w1 = ((-cos(q1 + q2) / (L1 * cos(q1 + q2) * sin(q1) - L1 * sin(q1 + q2) * cos(q1))) * Vx + (-sin(q1 + q2) / (L1 * cos(q1 + q2) * sin(q1) - L1 * sin(q1 + q2) * cos(q1))) * Vy)*1;
    w2 = (((L2 * cos(q1 + q2) + L1 * cos(q1)) / (L1 * L2 * cos(q1 + q2) * sin(q1) - L1 * L2 * sin(q1 + q2) * cos(q1))) * Vx + ((L2 * sin(q1 + q2) + L1 * sin(q1)) / (L1 * L2 * cos(q1 + q2) * sin(q1) - L1 * L2 * sin(q1 + q2) * cos(q1))) * Vy)*1;
  }
}
// saves angular velocities in degrees/sec en vx/vy_jac
void jacobiana_directa(float q1, float q2, float w1, float w2) {
  float vx, vy;
  float L1 = 1, L2 = 1;
  if (q2 == 0) {
    Serial.print(4);
    Serial.print(",");
    Serial.print(4);
    Serial.print(",");
    Serial.print(q1);
    Serial.print(",");
    Serial.println(q2);
  } else {
    vx = ((-L1 * sin(q1) - L2 * sin(q1 + q2)) * w1 + (-L2 * sin(q1 + q2)) * w2);
    vy = ((L1 * cos(q1) + L2 * cos(q1 + q2)) * w1 + (L2 * cos(q1 + q2)) * w2);

  }
}
#endif