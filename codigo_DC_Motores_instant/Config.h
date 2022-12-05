// Pinout para el Arduino Mega 2540

#ifndef Config
#define Config

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
float gradd;
int diri;
float gradi;
int pulsosd = 0;
int pulsosi = 0;
const float ratio = 1579;  //1579  //1450
long iterations = 0;

int pos_d;
int pos_i;

// numero de pulsos/grados maximos EN UNA DIRECCION
int DGRADOSMAX = 120;
int IGRADOSMAX = 139;
#define FACTOR_A 3.6                                    // =72/20
#define FACTOR_B 5.82424                                // =62/20 * 62/33
#define DGRAD2PULSOS ((float)ratio / 360.0) * FACTOR_A  // 1579/360*3.6
#define IGRAD2PULSOS ((float)ratio / 360.0) * FACTOR_B  //1579/360*5.82424
#define DPULSOS2GRAD 1.0 / (float)DGRAD2PULSOS
#define IPULSOS2GRAD 1.0 / (float)IGRAD2PULSOS
#define DPULSOSMAX ((float)DGRADOSMAX * (float)DGRAD2PULSOS)
#define IPULSOSMAX ((float)IGRADOSMAX * (float)IGRAD2PULSOS)
#define DSWITCHPULSOS 1830
#define ISWITCHPULSOS -3480
#define ZMMMAX 270
#define ZPULSOSMAX 270

#define DLENGTH 92   //primer brazo  91.62
#define ILENGTH 122  //segundo brazo 106
#define PENLENGTH 122

//#define X_Y_CORR FACTOR_A/FACTOR_B
#define X_Y_CORR 0.9

#define D_I_CORR 33.0 / 62.0

#define MAXRPM 70.0
#define RPM2RADpS 0.10472

#define RAD2DEG 57.2957795
#define DEG2RAD 0.01745329252

// velocidades maximas de los brazos en rad/s
#define DMAXSPEED MAXRPM* RPM2RADpS / FACTOR_A
#define IMAXSPEED MAXRPM* RPM2RADpS / FACTOR_B

//#define MINRADIUS sqrt(DLENGTH+ILEGNTH*cos(IGRADOSMAX)^2+ILENGTH*sin(IGRADOSMAX)^2)

float robot_limits_min[] = { -140, -140, -140, -140, -140, -140, -140, -130, -120, -110, -95, -88 };
float robot_limits_max[] = { 88, 95, 110, 120, 130, 140, 140, 140, 140, 140, 140, 140 };
int limits_index;

float xA, xB, yA, yB, dx, dy, dt, tprev, t, w1, w2, x_line, y_line, y_real, x_real, v_y, v_x, x, y;
int puntos, line_increment, timer, totalTime;

float ed, ei, dedtd, dedti, ud, ui, pwrd, pwri, vd, vi, w1_jac, w2_jac, v1_jac, v2_jac, c_inputi, c_inputd;
int mode;

float timenow, target_vel, target_v1, target_v2, D_pwm, I_pwm, q1_now, q2_now;

// PID constants
float kp, kd, ki;

float accelp = 0.1;

//int contador = 0;
//int contador2 = 0;
int rounds = 0;
int limit;
float alto;
float prevalto;
int pinza;
bool initialized = false;
bool communicating = false;

float v_cart_max_d, v_cart_max_i, vX, vY, vZ, initialtime, timeLeft, x_now, y_now, pwm_max_d, pwm_max_i;
float v_art_max_d = 70; // in RPM
float v_art_max_i = 70;

float vel_buffer_d, vel_buffer_i;

// numero maximo de pasos/pulsos

volatile float posid = 0;  // specify posi as volatile
long prevT = 0;
long currT;
float deltaT;
float eprevd = 0;
float eintegrald = 0;
volatile float posii = 0;  // specify posi as volatile
float eprevi = 0;
float eintegrali = 0;
int giro;

#endif