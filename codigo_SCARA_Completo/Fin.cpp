//
// Created by thano on 28.10.2022.
//

#include "Fin.h"

void Fin::init()
{
    pinMode(mypin, INPUT);

}

bool Fin::pressed()
{
    if (digitalRead(mypin)) return false;
    else return true;
}