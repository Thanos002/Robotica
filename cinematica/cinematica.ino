// CINEMATICA DIRECTA
void cinematicaDirecta() {
  float theta1F = theta1 * PI / 180;   // grados a radianes
  float theta2F = theta2 * PI / 180;
  xP = round(L1 * cos(theta1F) + L2 * cos(theta1F + theta2F));
  yP = round(L1 * sin(theta1F) + L2 * sin(theta1F + theta2F));
}

// CINEMATICA INVERSA
void cinematicaInversa(float x, float y) {
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
  
}