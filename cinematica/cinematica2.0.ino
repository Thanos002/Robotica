// CINEMATICA DIRECTA
void cinematicaDirecta(float theta1, float theta2) {
  float theta1F = theta1 * PI / 180;   // grados a radianes
  float theta2F = theta2 * PI / 180;
  xP = round(L1 * cos(theta1F) + L2 * cos(theta1F + theta2F));
  yP = round(L1 * sin(theta1F) + L2 * sin(theta1F + theta2F));
}

// CINEMATICA INVERSA
float[] cinematicaInversa(float x, float y) {
  theta2 = acos((sq(x) + sq(y) - sq(L1) - sq(L2)) / (2 * L1 * L2));
  if (x < 0 & y < 0) {
    theta2 = (-1) * theta2;
  }
  
  theta1 = atan(x / y) - atan((L2 * sin(theta2)) / (L1 + L2 * cos(theta2)));
  
  theta2 = (-1) * theta2 * 180 / PI;
  theta1 = theta1 * 180 / PI;

 // Ajuste de angulos dependiendo del cuadrante en el que la coordenada final x,y esta
  if (x >= 0 & y >= 0) {       // 1er cuadrante
    theta1 = 90 - theta1;
  }
  if (x < 0 & y > 0) {       // 2o cuadrante
    theta1 = 90 - theta1;
  }
  if (x < 0 & y < 0) {       // 3er cuadrante
    theta1 = 270 - theta1;
    phi = 270 - theta1 - theta2;
    phi = (-1) * phi;
  }
  if (x > 0 & y < 0) {       // 4o cuadrante
    theta1 = -90 - theta1;
  }
  if (x < 0 & y == 0) {
    theta1 = 270 + theta1;
  }
  
  // Calcular el angulo "phi" de forma que la pinza sea paralela al eje X
  phi = 90 + theta1 + theta2;
  phi = (-1) * phi;

  // Ajuste de angulos dependiendo del cuadrante en el que la coordenada final x,y esta
  if (x < 0 & y < 0) {       // 3er cuadrante
    phi = 270 - theta1 - theta2;
  }
  if (abs(phi) > 165) {
    phi = 180 + phi;
  }

  theta1=round(theta1);
  theta2=round(theta2);
  phi=round(phi);
  
  float[] result = [theta1, theta2, phi];
  return result;
  
}

//Trayectoria recta -> Calcula las velocidades angulares w1,w2 para cada instante de tiempo en función de los ángulos q1,q2 que toma en cada instante
//Se necesitan las coordenadas del punto A de partida y las del B de llegada, además del tiempo t en el que se desea llegar de A hasta B.
//Pasar los ángulos en grados, tiempo en segundos, velocidad en grados/segundo
int[] recta(float theta1, float theta2, float Xa, float Ya, float Xb, float Yb, float t){
  q1=theta1*PI/180;   //Transformación de grados a radianes
  q2=theta2*PI/180;
  w1=(-cos(q1 + q2)/(L1*cos(q1 + q2)*sin(q1) - L1*sin(q1 + q2)*cos(q1)))*(Xb-Xa)/t                        + (-sin(q1 + q2)/(L1*cos(q1 + q2)*sin(q1) - L1*sin(q1 + q2)*cos(q1)))*(Yb-Ya)/t;
  w2=((L2*cos(q1 + q2) + L1*cos(q1))/(L1*L2*cos(q1 + q2)*sin(q1) - L1*L2*sin(q1 + q2)*cos(q1)))*(Xb-Xa)/t + ((L2*sin(q1 + q2) + L1*sin(q1))/(L1*L2*cos(q1 + q2)*sin(q1) - L1*L2*sin(q1 + q2)*cos(q1)))*(Yb-Ya)/t;
  w1=round(w1);
  w2=round(w2);
  vz=round((z-alto)/t);
  int[] result = [w1, w2, vz];
  if(w1>70 || w2 >70 || vz>30){  //limit speed output to 70, if not increase time
    return recta(theta1, theta2, Xa, Ya, Xb, Yb, t+1);
  }
  else{
    return result;
  }
}

float[] DdegPerSec2RPM(float degPs){ //converts deg per seconds of extremo to actual motor speed
  result = 0.1667*FACTOR_A*degPs;
  return result;
}

float[] IdegPerSec2RPM(float degPs){ //converts deg per seconds of interior to actual motor speed
  result = 0.1667*FACTOR_B*degPs;
  return result;
}

void setMotorSpeed(int[] speeds){
  vd=(speeds[0], 0, 70, 0, 255);  // 70 rps corresponds to PWM 255
  vi=(speeds[1], 0, 70, 0, 255);
  stepper.setSpeedInMillimetersPerSecond(speeds[2]);
}

