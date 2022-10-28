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
const int ratio = 6316;  //1579*4
int iterations = 0;

// numero de pulsos/grados maximos
const int DGRADOSMAX = 190;
const int IGRADOSMAX = 280;
const int DPULSOSMAX = round((float)DGRADOSMAX * (float)(ratio / 360) * 4.5);       //15000
const int IPULSOSMAX = round((float)IGRADOSMAX * (float)(ratio / 360) * 7.281125);  //22106
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

// homing functions
float homed(int motorpin1, int motorpin2, int finpin, int PWMpin, int posi, int DPULSOSMAX) {
  analogWrite(PWMpin, 100);
  posprev = posi;
  while (digitalRead(finpin) == 0) {
    digitalWrite(motorpin1, LOW);
    digitalWrite(motorpin2, HIGH);
  }
  digitalWrite(motorpin1, LOW);
  digitalWrite(motorpin2, LOW);

  float delta = fabs(posprev - posi);
  posi = -DPULSOSMAX / 2;
}

float homei(int motorpin1, int motorpin2, int finpin, int PWMpin, int posi, int DPULSOSMAX) {
  analogWrite(PWMpin, 100);
  posprev = posi;
  while (digitalRead(finpin) == 0) {
    digitalWrite(motorpin1, HIGH);
    digitalWrite(motorpin2, LOW);
  }
  digitalWrite(motorpin1, LOW);
  digitalWrite(motorpin2, LOW);

  float delta = fabs(posprev - posi);
  posi = -IPULSOSMAX / 2;
}

float homestepper(int dirpin, int steppin, int finpin) {
  digitalWrite(SLEEP, HIGH);
  delay(1000);
  while (digitalRead(finpin) == 1) {
    digitalWrite(dirpin, LOW);
    digitalWrite(steppin, HIGH);
    delayMicroseconds(3000);
    digitalWrite(steppin, LOW);
    delayMicroseconds(3000);
  }
  digitalWrite(steppin, LOW);
  digitalWrite(SLEEP, LOW);

  alto = 0;
}

// BEGIN OF SETUP

void setup() {
  Serial.begin(9600);
  pinMode(ENCDA, INPUT);
  pinMode(ENCDB, INPUT);
  pinMode(ENCDA, INPUT);
  pinMode(ENCDB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCDA), readEncoderDAF, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCIA), readEncoderIAF, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCDB), readEncoderDBR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCIB), readEncoderIBR, RISING);

  attachInterrupt(digitalPinToInterrupt(ENCDA), readEncoderDAR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCIA), readEncoderIAR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCDB), readEncoderDBF, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCIB), readEncoderIBF, FALLING);

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

  initialized = false;

}

void loop() {

  if (initialized == false)
  {
    homed(DCD_1, DCD_2, FINI, PWMD, posid, DPULSOSMAX); // derecho - interno
    homei(DCI_1, DCI_2, FINE, PWMI, posii, IPULSOSMAX); // iz - ext.
    homestepper(DIR, STEP, FINZ);
    delay(5000);
    Serial.println("INIT END");
    initialized = true;
  }
  // wait until serial available
  if (Serial.available()) {
    // opcode syntax (int dird, int diri, int gradd, int gradi, int alto, boolean pinza)
    dird = (int)Serial.readStringUntil(',').toInt();
    diri = (int)Serial.readStringUntil(',').toInt();
    gradd = (int)Serial.readStringUntil(',').toInt();
    gradi = (int)Serial.readStringUntil(',').toInt();
    alto = (int)Serial.readStringUntil(',').toInt();
    giro = (int)Serial.readStringUntil(',').toInt();
    pinza = (int)Serial.readStringUntil('\n').toInt();
    pulsosd = Dgrados2pulsos(signCorrection(dird, gradd));
    pulsosi = Igrados2pulsos(signCorrection(diri, gradi));

    setStepper(alto);
    setPinza(giro, pinza);
  }

  // PID constants
  float kp = 4.5;
  float kd = 0.3;
  float ki = 0.2;

  // time difference
  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / (1.0e6);
  prevT = currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  int posd_atomic = 0;
  int posi_atomic = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    posd_atomic = posid;
    posi_atomic = posii;
  }
  if (iterations % 200 == 0) {
    Serial.print(1);
    Serial.print(",");
    Serial.print(millis());
    Serial.print(",");
    Serial.print(pulsosd);
    Serial.print(",");
    Serial.println(posid);

    Serial.print(2);
    Serial.print(",");
    Serial.print(millis());
    Serial.print(",");
    Serial.print(pulsosi);
    Serial.print(",");
    Serial.println(posii);

    Serial.print(3);
    Serial.print(",");
    Serial.print(millis());
    Serial.print(",");
    Serial.print(alto);
    Serial.print(",");
    Serial.println(alto);
  }
  iterations = iterations + 1;

  // error

  float ed = posid - pulsosd;
  float ei = posii - pulsosi;

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
  int diri = setDir(diri, ud);

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
      Serial.println("Update derecha");
    } else {
      diri = 0;
      pulsosi = 0;
      Serial.println("Update izquierda");
    }
  }
}

void setStepper(int steps) {
  digitalWrite(SLEEP, HIGH);
  delay(1);
  if (steps - prevalto > 0) {
    digitalWrite(DIR, HIGH);
  } else {
    digitalWrite(DIR, LOW);
  }
  for (int x = 0; x < steps; x++) {
    digitalWrite(STEP, HIGH);
    delayMicroseconds(3000);
    digitalWrite(STEP, LOW);
    delayMicroseconds(3000);
    Serial.println("Stepperloop");
  }
  delay(500);
  digitalWrite(SLEEP, LOW);
  prevalto = steps;
}

void setPinza(int grad, int abierto) {
  girar.write(grad);
  if (abierto == 0) {
    abrir.write(36); // pinza cerrada, open = abierta
  } else {
    abrir.write(0);
  }
}

float setPower(float u) {
  float pwr = fabs(u);
  if (pwr > 255) {
    pwr = 255;
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

void readEncoderDAF() {  //when A Falling or B Rising, pass posi var and opposite enc
  int b = digitalRead(ENCDB);
  if (b > 0) {  //determine whether A or B interrupted first -> direction
    posid++;
  } else {
    posid--;
  }
}

void readEncoderIAF() {  //when A Falling or B Rising, pass posi var and opposite enc
  int b = digitalRead(ENCIB);
  if (b > 0) {  //determine whether A or B interrupted first -> direction
    posii++;
  } else {
    posii--;
  }
}

void readEncoderDBR() {  //when A Falling or B Rising, pass posi var and opposite enc
  int b = digitalRead(ENCDA);
  if (b > 0) {  //determine whether A or B interrupted first -> direction
    posid++;
  } else {
    posid--;
  }
}

void readEncoderIBR() {  //when A Falling or B Rising, pass posi var and opposite enc
  int b = digitalRead(ENCIA);
  if (b > 0) {  //determine whether A or B interrupted first -> direction
    posii++;
  } else {
    posii--;
  }
}

void readEncoderDAR() {  //when A Rising or B Falling
  int b = digitalRead(ENCDB);
  if (b > 0) {  //determine whether A or B interrupted first -> direction
    posid--;
  } else {
    posid++;
  }
}

void readEncoderIAR() {  //when A Rising or B Falling
  int b = digitalRead(ENCIB);
  if (b > 0) {  //determine whether A or B interrupted first -> direction
    posii--;
  } else {
    posii++;
  }
}

void readEncoderDBF() {  //when A Rising or B Falling
  int b = digitalRead(ENCDA);
  if (b > 0) {  //determine whether A or B interrupted first -> direction
    posid--;
  } else {
    posid++;
  }
}

void readEncoderIBF() {  //when A Rising or B Falling
  int b = digitalRead(ENCIA);
  if (b > 0) {  //determine whether A or B interrupted first -> direction
    posii--;
  } else {
    posii++;
  }
}

// conversion de grados a pulsos del motor derecho /brazo interno
// comprobar si el punto esta dentro el campo de trabajo
float Dgrados2pulsos(float grados) {
  float result = (float)grados * (float)(ratio / 360) * 4.5;
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
  float result = (float)grados * (float)(ratio / 360) * 7.281125;
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
