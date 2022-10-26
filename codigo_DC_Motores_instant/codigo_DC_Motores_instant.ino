#include <util/atomic.h>  // For the ATOMIC_BLOCK macro
#include <Servo.h>

// Pinout para el Arduino Mega 2540

#define DCD_1 22
#define DCD_2 23

#define DCI_1 24
#define DCI_2 25

#define ENCDA 26
#define ENCDB 27
#define ENCIA 28
#define ENCIB 29

#define STEP 30
#define DIR 31

#define SRESET 32    // resets internal logic and step table, defualt inactive/low during operation
#define SLEEP 33  // low power mode, needs 1ms to reactivate
#define ENAPIN 34    // Enables outputs, defualt inactive/low during operation

// para micropasos:
#define MS1 35
#define MS2 36
#define MS3 37

#define SERVO1 38
#define SERVO2 39

#define FIN1 40
#define FIN2 41
#define FIN3 42

#define PWMD 2  // PWM Velocidad
#define PWMI 3

// motor izquierda: brazo extremo
// motor derecha: brazo interior

int dird;
int gradd;
int diri;
int gradi;
int pulsosd = 0;
int pulsosi = 0;
const int ratio = 6316;  //1579*4
int iterations = 0;

//int contador = 0;
//int contador2 = 0;
int rounds = 0;
int limit;
float alto;
float prevalto;
bool pinza;

// numero maximo de pasos/pulsos
int maxD;
int maxR;
int maxS;

volatile int posid = 0;  // specify posi as volatile
long prevT = 0;
float eprevd = 0;
float eintegrald = 0;
volatile int posii = 0;  // specify posi as volatile
float eprevi = 0;
float eintegrali = 0;
int giro;

Servo girar;
Servo abrir;

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

  pinMode(FIN1, INPUT);
 
}

void loop() {

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
    pulsosd = signCorrection(dird, gradd);
    pulsosi = signCorrection(diri, gradi);

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
  int posd = 0;
  int posi = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    posd = posid;
    posi = posii;
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

  // store previous error
  eprevd = ed;
  eprevi = ei;
}

int signCorrection(int dir, int grad) {
  int pulsos;
  if (grad >= 0 && grad <= 360) {
    pulsos = (int)((float)grad * (float)((float)ratio / (float)360));
  }
  if (dir == 0) {
    pulsos = -pulsos;
  } else {
    //Serial.print("Valor por encima de 360º");
    return 0;
  }
  return pulsos;
}

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
    if(pwm == PWMD){
      dird = 0;
      pulsosd = 0;
    }
    else{
      diri = 0;
      pulsosi = 0;
    }

  }
}

void setStepper(int steps){
  digitalWrite(SLEEP,0);
  delay(1);
  if(steps-prevalto>0){
    digitalWrite(DIR, HIGH);
  }
  else{
    digitalWrite(DIR,LOW);
  }
  for(int x = 0; x < steps; x++) {
    digitalWrite(STEP,HIGH); 
    delayMicroseconds(500); 
    digitalWrite(STEP,LOW); 
    delayMicroseconds(500); 
  }
  delay(500);
  digitalWrite(SLEEP,0);
  prevalto=steps;
}

void setPinza(int grad, bool open){
  girar.write(grad);
  if (open==0){
    abrir.write(36);
  }
  else{
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

//conversion de grados a pulsos del motor derecho /brazo interno
float brazoDgrados(float grados){
  return (float)grados*(float)(ratio/360)*4.5;
}

// motor izquierdo
float brazoIgrados(float grados){
  return (float)grados*(float)(ratio/360)*7.281125;
}

// stepper
float mmToPulsos(float mm){
  return (float)mm*100;
}
