//La amplitud de la garra es de 0º (abierta) hasta 36º(cerrada)
//No forzar el motor ni su posición de origen

#include <Servo.h>
Servo myservo2;

void setup() {
  // put your setup code here, to run once:
  myservo2.attach(9);

}

void loop() {
  // put your main code here, to run repeatedly:
  myservo2.write(0);
delay(2000);
myservo2.write(36); 
delay(2000);
}
