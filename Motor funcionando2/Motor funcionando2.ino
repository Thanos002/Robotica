#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define IN1 6    // Input3 conectada al pin 5 DC1
#define IN2 7    // Input4 conectada al pin 4 DC2
#define ENCA 2    // ENA conectada al pin 3 de Arduino
#define ENCB 4    //ENB
#define PWM1 5     // PWM Velocidad

String operacion;
String valor;
int pulsos = 0;
int valor_pulsos = 0;

int contador = 0;
int contador2 = 0;
int rounds = 0;
int limit;
int ratio = 1580;
int operacion_int = 1;

void setup()
{
  Serial.begin(9600);
  pinMode (ENCA, INPUT_PULLUP);
  pinMode (ENCB, INPUT_PULLUP);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), interrupcion, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCB), interrupcion2, RISING);
}

void loop()
{
  if (Serial.available()) {
    operacion = Serial.readStringUntil(',');
    valor = Serial.readStringUntil('\n');
    int operacion_int;
    operacion_int = operacion.toInt();
    limit = valor.toInt();
    limit = limit * (ratio / 360);

  if (!isdigit(operacion_int) || !isdigit(limit)) {
    Serial.println("Numero invalido");
  }
  }
  switch (operacion_int) {
    case 0:
      Serial.println("Giro hacia la izquierda");
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      break;
    case 1:
      Serial.println("Giro hacia la derecha");
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      break;
    default: Serial.println("Operación no reconocida");

      //Preparamos la salida para que el motor gire en un sentido
  }
  float kp = 1;

  int pos = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      pos = contador;
    }

  float e = contador - limit;
  float u = kp * e;

  float pwr = fabs(u);
    if ( pwr > 255 ) {
      pwr = 255;
    }
    int dir = 1;
    if (u < 0) {
      dir = -1;
    }
  setMotor(dir, pwr, PWM1, IN1, IN2);  

  
  while (abs(contador) < limit) {
    delay(1);
  }
  Serial.println("beforeif");
  if (abs(contador) >= limit) {
    Serial.println("if");
    digitalWrite (IN1, LOW);
    digitalWrite (IN2, LOW);
    Serial.println("Completo");
    Serial.println(contador);
    Serial.println(rounds);
    rounds++;
    delay(1000);
    contador = 0;
  }
}


void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  //set direction and speed of motor
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    /*pulsos, dir = 0;*/
  }
}
/*analogWrite(ENA,255);

  Serial.print("pulsos: ");
  Serial.println(contador);
  analogWrite(ENA,0);*/


/* // Aplicamos PWM al pin ENB, haciendo girar el motor, cada 2 seg aumenta la velocidad
  analogWrite(ENA,55);
  delay(2000);
  analogWrite(ENA,105);
  delay(2000);
  analogWrite(ENA,255);
  delay(2000);
  // Apagamos el motor y esperamos 5 seg
  analogWrite(ENA,0);
  delay(2000);*/

void interrupcion() {
  if (digitalRead(ENCA) ==  digitalRead(ENCB))   contador++;
  else contador--;
}

void interrupcion2() {
  if (digitalRead(ENCA) ==  digitalRead(ENCB))   contador--;
  else contador++;
}
