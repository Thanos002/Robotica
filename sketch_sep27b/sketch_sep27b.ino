int IN1 = 6;
int IN2 = 7;
#define ENC1 2
#define ENC2 4
String operacion;
String valor;
int pulsos=0;
int valor_pulsos=0;

int contador = 0;
int contador2 = 0;
int rounds = 0;
int limit = 1490 * 1;
int operacion_int = 1;

void setup()
{
  pinMode (ENC1, INPUT_PULLUP);
  pinMode (ENC2, INPUT_PULLUP);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(ENC1), interrupcion, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2), interrupcion2, RISING);
}

void loop()
{

  if (Serial.available()) {
    operacion = Serial.readStringUntil(',');
    valor = Serial.readStringUntil('\n');
    int operacion_int, valor_int;
    operacion_int = operacion.toInt();
    limit = valor.toInt();
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
    while(abs(contador)<limit){
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
}

void interrupcion() {
  if (digitalRead(ENC1) ==  digitalRead(ENC2))   contador++;
  else contador--;
}

void interrupcion2() {
  if (digitalRead(ENC1) ==  digitalRead(ENC2))   contador--;
  else contador++;
}
