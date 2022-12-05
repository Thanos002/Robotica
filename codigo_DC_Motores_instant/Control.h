#ifndef Control
#define Control

#include "Config.h"
#include <math.h>
#include <Arduino.h>

/**
#define DLENGTH 91.61   //primer brazo
#define ILENGTH 105.92  //segundo brazo
#define RAD2DEG 57.2957795
#define DEG2RAD 0.01745329252 **/

float lawOfCosines(float a, float b, float c) {
  return acosf((a * a + b * b - c * c) / (2.0f * a * b));
}

float lawOfCosines2(float a, float b, float c) {
  return acosf((a * a - b * b - c * c) / (2.0f * b * c));
}

float distance(float x, float y) {
  return sqrt(x * x + y * y);
}

void cinematicaInversa_old(float x, float y, float *Q1, float *Q2) {
  *Q1 = *Q1 + 90;  // adjust zero position for calcs
  *Q2 = -*Q2;
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
  *Q2 = -*Q2;
  Serial.println("endkin");
  Serial.println(*Q1);
  Serial.println(*Q2);
}

// CINEMATICA DIRECTA
void cinematicaDirecta(float q1, float q2,float *x, float*y){
    q1=q1*DEG2RAD;
    q2=q2*DEG2RAD;
    *x=DLENGTH*sin(q1) + ILENGTH*sin(q1+q2);
    *y=DLENGTH*cos(q1) + ILENGTH*cos(q1+q2);
}


// CINEMATICA INVERSA
void cinematicaInversa2(float x, float y, float *q1, float *q2){
    if((x*x + y*y - DLENGTH*DLENGTH - ILENGTH*ILENGTH)/(2*DLENGTH*ILENGTH)<-1 || (x*x + y*y - DLENGTH*DLENGTH - ILENGTH*ILENGTH)/(2*DLENGTH*ILENGTH)>1){
        *q2=0;
    }
    else {
        *q2=acos((x*x + y*y - DLENGTH*DLENGTH - ILENGTH*ILENGTH)/(2*DLENGTH*ILENGTH));
    }
    *q1=atan2(y,x) - atan2(ILENGTH*sin(*q2),DLENGTH+ILENGTH*cos(*q2));
    
    *q1=*q1*RAD2DEG;
    *q2=*q2*RAD2DEG;
    
    *q1=90 - *q1;
    *q2= - *q2;
}

// velocidades positivas derechas
// w1, w2 en deg/s
// target_vel en RPM

// JACOBIANA
void jacobianaDirecta(float q1, float q2, float w1, float w2, float *Vx, float *Vy){
    if(q2==0){
        //cout<<"Singularidad\n";
    }
    else{
        *Vx=(DLENGTH*cos(q1)+ILENGTH*cos(q1+q2))*w1 + (ILENGTH*cos(q1+q2))*w2;
        *Vy=(-DLENGTH*sin(q1)-ILENGTH*sin(q1+q2))*w1 + (-ILENGTH*sin(q1+q2))*w2;
    }
}

void jacobianaInversa(float q1, float q2, float Vx, float Vy, float *w1, float *w2){
    if(cos(q1)*sin(q1+q2)-sin(q1)*cos(q1+q2)==0){
        //cout<<"Singularidad\n";
    }
    else{
        *w1=(-sin(q1 + q2)/(DLENGTH*cos(q1 + q2)*sin(q1) - DLENGTH*sin(q1 + q2)*cos(q1)))*Vx + (-cos(q1 + q2)/(DLENGTH*cos(q1 + q2)*sin(q1) - DLENGTH*sin(q1 + q2)*cos(q1)))*Vy;
        *w2=((ILENGTH*sin(q1 + q2) + DLENGTH*sin(q1))/(DLENGTH*ILENGTH*cos(q1 + q2)*sin(q1) - DLENGTH*ILENGTH*sin(q1 + q2)*cos(q1)))*Vx + 
        ((ILENGTH*cos(q1 + q2) + DLENGTH*cos(q1))/(DLENGTH*ILENGTH*cos(q1 + q2)*sin(q1) - DLENGTH*ILENGTH*sin(q1 + q2)*cos(q1)))*Vy;
    }
    *w1 = RAD2DEG* (*w1);
    *w2 = RAD2DEG * (*w2);
}

void cinematicaInversa(float x, float y, float *Q1, float *Q2) {
  *Q1 = *Q1 + 90;  // adjust zero position for calcs
  float r;
  float aux1, aux2;
  r = distance(x, y);

  if (y < 0 || r < DLENGTH) {  // fuera de campo de trabajo
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
      Serial.print(6);
      Serial.print(",");
      Serial.print(x);
      Serial.print(",");
      Serial.println(y);
    }

    if (x > 0) {
      aux1 = (atan2(y, x)) * RAD2DEG;                        //beta
      aux2 = (lawOfCosines(r, DLENGTH, ILENGTH)) * RAD2DEG;  //alpha
      *Q1 = 90-(aux1 - aux2);
      *Q2 =- (lawOfCosines2(r, DLENGTH, ILENGTH)) * RAD2DEG;
            if(*Q1 > 90){
                *Q1 = 180 - 2*atan2(y,x)* RAD2DEG - *Q1;
                *Q2 = -*Q2;
            }
     
    }
    if (x == 0) {
      x = x + 0.001f;
      aux1 = (atan2(y, x)) * RAD2DEG;                        //beta
      aux2 = (lawOfCosines(r, DLENGTH, ILENGTH)) * RAD2DEG;  //alpha
      *Q1 = 90-(aux1 - aux2);
      *Q2 = -(lawOfCosines2(r, DLENGTH, ILENGTH)) * RAD2DEG;
      
    }
    if (x < 0) {
      aux1 = (atan2(y, x)) * RAD2DEG;                      //beta
      aux2 = (lawOfCosines(r, DLENGTH, ILENGTH)) * RAD2DEG;  //alpha
      *Q1 =  90-(aux1 - aux2);                              // 180 + un numero negativo
      *Q2 = -(lawOfCosines2(r, DLENGTH, ILENGTH)) * RAD2DEG;
     
    }
  }
  //*Q1 = *Q1 - 90;  // adjust zero position for rest of code
  //*Q2 = -*Q2;
}

// returns absolute distance between two points (A and B)
float getDistance(float xA, float yA, float xB, float yB) {
  return sqrt(pow((xB - xA), 2) + pow((yB - yA), 2));
}

//returns time of trajectory from A to B with constant velocity v
float getTrayTime(float xA, float xB, float yA, float yB, float v){
  float result = getDistance(xA, xB, yA, yB)/(float)v;
  return result;
}

float getTrapezVel(float vmax){
  float result = vmax * (1-accelp);
}

// return x or y velocity vectors (v1d) given two points and totaltime
float getV(float time, float a, float b){
  float result = (a-b)/time;
  return result; 
}

//returns x or y velocity depending on trapezoidal position
float getCurrentVelTrap(float timenow, float totaltime, float vmax){
  float result;
  if (timenow<accelp*totaltime){
    result = vmax*(timenow/(accelp*totaltime));
  }
  else if (timenow > totaltime*(1-accelp)){
    result = vmax - vmax*(timenow - totaltime*(1-accelp)/(totaltime));
  }
  else{
    result = vmax;
  }
  return result;
}

float getVel(float v1d, float t0, float tnow, float totaltime){
  float result = v1d * (t)/accelp;
}

#define DPS2RPM_D 0.1667*FACTOR_A
#define DPS2RPM_I 0.1667*FACTOR_B

float DdegPerSec2RPM(float degPs){ //converts deg per seconds of extremo to actual motor speed
  float result = 0.1667*FACTOR_A*degPs;
  return result;
}

float IdegPerSec2RPM(float degPs){ //converts deg per seconds of interior to actual motor speed
  float result = 0.1667*FACTOR_B*degPs;
  return result;
}

void setMotorSpeed(float speed, float *pwm){  // maps RPM value to PWM value
  *pwm = map(speed, 0, 70, 0, 255);
}

#endif