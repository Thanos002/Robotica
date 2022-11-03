#include <FlexyStepper.h>

#include <util/atomic.h>  // For the ATOMIC_BLOCK macro
#include <Servo.h>

// Pinout para el Arduino Mega 2540

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

// pins de fin de carrera (p.e. FINDE fin de motor del extremo (izquierdo) a la posicion derecha)
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

// numero de pulsos/grados maximos
const int DGRADOSMAX = 190;
const int IGRADOSMAX = 280;
const int DPULSOSMAX = round((float)DGRADOSMAX * (float)(ratio / 360.0) * 3.5);       //15,3513
#define DGRAD2PULSOS 15.3513
#define IGRAD2PULSOS 24.1236
#define DPULSOS2GRAD 1/DGRAD2PULSOS
#define IPULSOS2GRAD 1/IGRAD2PULSOS
const int IPULSOSMAX = round((float)IGRADOSMAX * (float)(ratio / 360.0) * 5.5);  // 24,1236
const int ZMMMAX = 270;
const int ZPULSOSMAX = 270;

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

// init servos
Servo girar;
Servo abrir;

FlexyStepper stepper;


// BEGIN OF SETUP

void setup() {
  Serial.begin(9600);
  pinMode(ENCDA, INPUT);
  pinMode(ENCDB, INPUT);
  pinMode(ENCDA, INPUT);
  pinMode(ENCDB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCDA), handler_encoderD, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCIA), handler_encoderI, CHANGE);

  pinMode(PWMD, OUTPUT);
  pinMode(PWMI, OUTPUT);
  pinMode(DCD_1, OUTPUT);
  pinMode(DCD_2, OUTPUT);
  pinMode(DCI_1, OUTPUT);
  pinMode(DCI_2, OUTPUT);

  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);
  pinMode(SRESET, OUTPUT);
  pinMode(SLEEP, OUTPUT);
  pinMode(ENAPIN, OUTPUT);

  girar.attach(SERVO1);
  abrir.attach(SERVO2);
  setPinza(20,30);

  pinMode(FINE, INPUT_PULLUP);
  pinMode(FINI, INPUT_PULLUP);
  pinMode(FINZ, INPUT_PULLUP);

  digitalWrite(SLEEP, HIGH);
  digitalWrite(SRESET, HIGH);
  digitalWrite(ENAPIN, LOW);
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(MS3, LOW);
  digitalWrite(SERVO1, LOW);
  digitalWrite(SERVO2, LOW);

  stepper.connectToPins(STEP, DIR);
  stepper.setStepsPerRevolution(200);
  stepper.setStepsPerMillimeter(25);
  stepper.setSpeedInStepsPerSecond(400);
  stepper.setAccelerationInStepsPerSecondPerSecond(200);
  stepper.setSpeedInStepsPerSecond(600);
  stepper.setAccelerationInStepsPerSecondPerSecond(200);

  initialized = false;

}

void loop() {

  // wait until serial available
  if (Serial.available()) {
    communicating = true;
    // opcode syntax (int dird, int diri, int gradd, int gradi, int alto, boolean pinza)
    gradd = (int)Serial.readStringUntil(',').toInt();
    gradi = (int)Serial.readStringUntil(',').toInt();
    alto = (float)Serial.readStringUntil(',').toInt();
    giro = (int)Serial.readStringUntil(',').toInt();
    pinza = (int)Serial.readStringUntil('\n').toInt();
    pulsosd = Dgrados2pulsos(gradd);
    pulsosi = Igrados2pulsos(gradi);

    stepper.moveToPositionInMillimeters(-alto);
    setPinza(giro, pinza);
  }

  // PID constants
  float kp = 5;
  float kd = 0.1;
  float ki = 0.1;

  // time difference
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / (1.0e6);
  prevT = currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  int pos_d = 0;
  int pos_i = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos_d = posid;
    pos_i = posii;
  }
  if ((iterations % 200 == 0)) {
    Serial.print(1);
    Serial.print(",");
    Serial.print(millis());
    Serial.print(",");
    Serial.print((int)(pulsosd*DPULSOS2GRAD));
    Serial.print(",");
    Serial.println((int)(pos_d*DPULSOS2GRAD));

    Serial.print(2);
    Serial.print(",");
    Serial.print(millis());
    Serial.print(",");
    Serial.print((int)(pulsosi*IPULSOS2GRAD));
    Serial.print(",");
    Serial.println((int)(pos_i*IPULSOS2GRAD));

    Serial.print(3);
    Serial.print(",");
    Serial.print(millis());
    Serial.print(",");
    Serial.print((int)alto);
    Serial.print(",");
    Serial.println((int)alto);
  }
  iterations = iterations + 1;

  // error

  float ed = pos_d - pulsosd;
  float ei = pos_i - pulsosi;

  // derivative
  float dedtd = (ed - eprevd) / (deltaT);
  float dedti = (ei - eprevi) / (deltaT);

  // integral
  eintegrald = eintegrald + ed * deltaT;
  eintegrali = eintegrali + ei * deltaT;

  // control signal
  float ud = kp * ed + kd * dedtd + ki * eintegrald;
  float ui = kp * ei + kd * dedti + ki * eintegrali;

  // motor power
  float pwrd = setPower(ud);
  float pwri = setPower(ui);

  // motor direction
  int dird = setDir(dird, ud);
  int diri = setDir(diri, ui);

  // signal the motor
  setMotor(dird, pwrd, PWMD, DCD_1, DCD_2);
  setMotor(diri, pwri, PWMI, DCI_1, DCI_2);

  // store previous errors
  eprevd = ed;
  eprevi = ei;
}

// replace opcode with sign
int signCorrection(int dir, int grad) {
  if (dir == 0) {
    grad = -grad;
    return grad;
  }
}

// function to set motor for PID, argument direction and pwm
void setMotor(int dir, int pwmVal, int pwm, int pin1, int pin2) {
  //set direction and speed of motor
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
  } else if (dir == -1) {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
  } else {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
    if ((int)pwm - (int)PWMD == 0) {
      dird = 0;
      pulsosd = 0;
    } else {
      diri = 0;
      pulsosi = 0;
    }
  }
}

void setPinza(int grad, int abierto) {
  grad = map(grad, 0, 90, 20, 90) ;
  girar.write(grad);
  abrir.write(abierto == 0 ? 30 : 90);
}

float setPower(float u) {
  float pwr = fabs(u);
  if (pwr > 200) {
    pwr = 200;
  }
  return pwr;
}


int setDir(int dir, float u) {
  dir = 1;
  if (u < 0) {
    dir = -1;
  }
  return dir;
}

void handler_encoderD() {
  if (digitalRead(ENCDA) == HIGH) {
    if (digitalRead(ENCDB) == LOW)
      posid++;
    else
      posid--;
  }

}
void handler_encoderI() {
  if (digitalRead(ENCIA) == HIGH) {
    if (digitalRead(ENCIB) == LOW)
      posii++;
    else
      posii--;
  }

}

// conversion de grados a pulsos del motor derecho /brazo interno
// comprobar si el punto esta dentro el campo de trabajo
float Dgrados2pulsos(float grados) {
  float result = (float)grados * (float)(ratio / 360.0) * 3.5;
  if ((result < DPULSOSMAX / 2 - posid) && (result > posid - DPULSOSMAX / 2)) {
    return result;
  } else {
    Serial.print(4);
    Serial.print(",");
    Serial.print(1);
    Serial.print(",");
    Serial.print(result);
    Serial.print(",");
    Serial.println(posid);
    return 0;
  }
}

// lo igual, para el motor izquierdo
float Igrados2pulsos(float grados) {
  float result = (float)grados * (float)(ratio / 360.0) * 5.5;
  if ((result < IPULSOSMAX / 2 - posii) && (result > posii - IPULSOSMAX / 2)) {
    return result;
  } else {
    Serial.print(4);
    Serial.print(",");
    Serial.print(2);
    Serial.print(",");
    Serial.print(result);
    Serial.print(",");
    Serial.println(posii);
    return 0;
  }
}

// stepper conversion
float mm2Pulsos(float mm) {
  float result = (float)mm * 100;
  if (result < 0 or result > 270) {
    return result;
  } else {
    Serial.print(4);
    Serial.print(",");
    Serial.print(3);
    Serial.print(",");
    Serial.print(result);
    Serial.print(",");
    Serial.println(alto);
    return 0;
  }
}
