//La amplitud de la garra es de 0º (abierta) hasta 36º(cerrada)
//No forzar el motor ni su posición de origen

#include <Servo.h>
Servo myservo1;
Servo myservo2;
 int ps = 0;
void setup() {
  // put your setup code here, to run once:
  myservo1.attach(9); //PWM
  myservo2.attach(10); //PWM
 

}

void loop() {
  // put your main code here, to run repeatedly:
  //Servo 1:
  myservo1.write(10);
  delay(2000);
  myservo1.write(90); 
  delay(2000);


 /* for (ps = 0; ps<180; ps+=1){
    myservo1.write(ps);
    delay(15);
      }
  myservo1.write(10);
  delay(5000);*/

  //Servo 2:
  myservo2.write(0);
  delay(2000);
  myservo2.write(36); 
  delay(2000);
}

//Posibles casos auxiliares:

/*
------------------------------------------------------1----------------
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-servo-motor


#include <Servo.h>

Servo myServo;
unsigned long MOVING_TIME = 1000; // moving time is 3 seconds
unsigned long moveStartTime;
int startAngle = 0; // 30°
int stopAngle  = 10; // 90°

void setup() {
  myServo.attach(9);
  moveStartTime = millis(); // start moving

  // TODO: other code
}

void loop() {
  unsigned long progress = millis() - moveStartTime;

  if (progress <= MOVING_TIME) {
    long angle = map(progress, 0, MOVING_TIME, startAngle, stopAngle);
    myServo.write(angle); 
  }

  // TODO: other code
} 

-------------------------------------------------------2
Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 https://www.arduino.cc/en/Tutorial/LibraryExamples/Sweep


#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
}

*/