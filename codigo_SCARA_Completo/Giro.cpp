//
// Created by thano on 28.10.2022.
//

#include "Giro.h"
#include "Pinout.h"
#include <arduino.h>
#include <Servo.h>

Giro::Giro() {

    angle=0;
}

void Giro::init(int servo_Pin){

    giro.attach(servo_Pin);
}

void Giro::move(int angle) {

        giro.write(angle);

}

int Giro::getAngle() { return angle; }
