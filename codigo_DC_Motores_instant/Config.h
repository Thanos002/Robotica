// Pinout para el Arduino Mega 2540

#ifndef CONFIG
#define CONFIG

#include <Arduino.h>

//motor derecho
#define DCD_1 22
#define DCD_2 23

//motor izquierdo
#define DCI_1 53
#define DCI_2 52

// encoder pins (p.e. ENCDA - encoder del motor derecho pin A)
#define ENCDA 18
#define ENCDB 19
#define ENCIA 20
#define ENCIB 21

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
#define SERVO1 9   // servo girar
#define SERVO2 10  // servo abrir

// pins de fin de carrera
#define FINE 40
#define FINI 44
#define FINZ 42


//pins pwm motores DC
#define PWMD 2  // PWM Velocidad
#define PWMI 3

// motor izquierda: brazo extremo
// motor derecha: brazo interior

int posprev;
int dird;
int gradd;
int diri;
int gradi;
int pulsosd = 0;
int pulsosi = 0;
const float ratio = 1579;  //1579*4
int iterations = 0;

int pos_d;
int pos_i;

// numero de pulsos/grados maximos EN UNA DIRRECION 
int DGRADOSMAX = 95;
int IGRADOSMAX = 140;
#define DGRAD2PULSOS 19.7375 // 1579/360*4.5
#define IGRAD2PULSOS 31.9322  //1579/360*7.2803
#define DPULSOS2GRAD 1/DGRAD2PULSOS
#define IPULSOS2GRAD 1/IGRAD2PULSOS
int DPULSOSMAX = (int)((float)DGRADOSMAX*(float)DGRAD2PULSOS);
int IPULSOSMAX = (int)((float)IGRADOSMAX * (float)IGRAD2PULSOS);
int ZMMMAX = 270;
int ZPULSOSMAX = 270;

//int contador = 0;
//int contador2 = 0;
int rounds = 0;
int limit;
float alto;
float prevalto;
int pinza;
bool initialized = false;
bool communicating = false;

// numero maximo de pasos/pulsos

volatile int posid = 0;  // specify posi as volatile
long prevT = 0;
float eprevd = 0;
float eintegrald = 0;
volatile int posii = 0;  // specify posi as volatile
float eprevi = 0;
float eintegrali = 0;
int giro;

#endif 