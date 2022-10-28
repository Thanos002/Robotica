//
// Created by thano on 28.10.2022.
//

#include <Arduino.h>
#include <stdio.h>

#ifndef CODIGO_SCARA_COMPLETO_PINOUT_H
#define CODIGO_SCARA_COMPLETO_PINOUT_H

//motor derecho
#define DCD_1 22
#define DCD_2 23

//motor izquierdo
#define DCI_1 53
#define DCI_2 52

// encoder pins (p.e. ENCDA - encoder del motor derecho pin A)
#define ENCDA 49
#define ENCDB 48
#define ENCIA 51
#define ENCIB 50

// pins del stepper motor
#define STEP 25
#define DIR 27

// pins del stepper driver
#define SRESET 31  // resets internal logic and step table, defualt HIGH during operation
#define SLEEP 29   // low power mode, needs 1ms to reactivate, HIGH during operation
#define ENAPIN 39  // Enables outputs, defualt inactive/low during operation

// para micropasos del stepper:
#define MS1 37
#define MS2 35
#define MS3 33

//pins de los servos
#define GIRO 9   // servo girar
#define PINZA 10  // servo abrir

// pins de fin de carrera (p.e. FINDE fin de motor del extremo (izquierdo) a la posicion derecha)
#define FINE 40
#define FINI 44
#define FINZ 42


//pins pwm motores DC
#define PWMD 2  // PWM Velocidad
#define PWMI 3

#define KP_I 4.5
#define KI_I 0.3
#define KD_I 0.1
#define KP_D 4.5
#define KI_D 0.3
#define KD_D 0.1

#define ANG_MIN 0
#define ANG_MAX 36

#define GRADOS_MAX_D 190
#define GRADOS_MAX_I 280
#define ZMAX 270

#define GRADOS_POR_TIC 4.3861
#define GRADOS_A_PULSOS(x) (x)*4.3861
#define PULSOS_A_GRADOS(x) (x)*4.3861

#define RATIO 1579 // NUMBER OF PULSOS PER REVOLUTION DC MOTOR

#endif //CODIGO_SCARA_COMPLETO_PINOUT_H
