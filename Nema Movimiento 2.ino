// Definimos los números de los pines
#define stepPin 3
#define dirPin 4

int dir;
int step;

void setup() {
  Serial.begin(9600);
  // Definimos ambos pines como salidas
  pinMode(stepPin,OUTPUT);  // Pin que define el numero de pasos 
  pinMode(dirPin,OUTPUT);  // Pin que define la dirección del motor
}
void loop() {
  if (Serial.available()) {
    dir = (int)Serial.readStringUntil(',').toInt();
    // 200 pasos equivalen a un giro de 360º del stepper
    step = (int)Serial.readStringUntil('\n').toInt();
    switch (dir) {
      case 0:
        //Serial.println("Giro hacia la izquierda");
        digitalWrite(dirPin,LOW);
        break;
      case 1:
        //Serial.println("Giro hacia la derecha");
        digitalWrite(dirPin,HIGH);
        break;
      default:
        //Serial.println("Operación no reconocida");
        return;
    }
  
  // Bucle para contar el número de pasos que le introducimos y una vez que los realice se pare
  
  for(int x = 0; x < step; x++) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(500); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(500); 
  }
  delay(1000); // Un segundo de delay hasta la proxima interacción

  }
}