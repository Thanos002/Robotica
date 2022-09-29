int IN1 = 5;    // Input3 conectada al pin 5
int IN2 = 4;    // Input4 conectada al pin 4 
int ENC1 = 3;    // ENA conectada al pin 3 de Arduino

int contador=0;

void setup()
{
 pinMode (ENC1, INPUT); 
 pinMode (IN1, OUTPUT);
 pinMode (IN2, OUTPUT);
 Serial.begin(9600);
attachInterrupt(1,interrupcion,RISING);
}

void loop()
{
  //Preparamos la salida para que el motor gire en un sentido
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  if(contador>1450){
    digitalWrite (IN1, LOW);
  }
  /*analogWrite(ENA,255);*/
  
  Serial.print("pulsos: ");
  Serial.println(contador);
  /*analogWrite(ENA,0);*/
  

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

void interrupcion(){
  contador++;
}