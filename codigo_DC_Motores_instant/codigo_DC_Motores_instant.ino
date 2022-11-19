#include <FlexyStepper.h>

#include <util/atomic.h>  // For the ATOMIC_BLOCK macro
#include <Servo.h>
#include "Config.h"


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
  setPinza(20, 30);

  pinMode(FINE, INPUT_PULLUP);  // fin de carrera brazo exterior izquierdo
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
  stepper.setSpeedInStepsPerSecond(600);
  stepper.setAccelerationInStepsPerSecondPerSecond(200);

  initialized = true;
}

void loop() {

  if (initialized == false) {

    delay(5000);
    while (digitalRead(FINI) == HIGH) {
      setMotor(-1, 100, PWMD, DCD_1, DCD_2);
      delay(1);
    }
    setMotor(0, 0, PWMD, DCD_1, DCD_2);                 // stop motor
    posid, pulsosd, pos_d = DGRAD2PULSOS * DGRADOSMAX;  //set posi vars to current posi

    delay(2000);

    while (digitalRead(FINE) == HIGH) {
      setMotor(1, 120, PWMI, DCI_1, DCI_2);
      delay(1);
    }
    setMotor(0, 0, PWMI, DCI_1, DCI_2);
    posii, pulsosi, pos_i = IGRAD2PULSOS * IGRADOSMAX;  //set posi vars to current posi

    delay(5000);
    stepper.moveToHomeInMillimeters(1, 10, 270, FINZ);  // normally closed!!!
    alto = 0;

    initialized = true;
    stepper.moveToPositionInMillimeters(-20);
    delay(500);
    pulsosd, pulsosi = 0;  // goto home
  }


  // wait until serial available
  if (Serial.available()) {
    communicating = true;
    // opcode syntax (int gradd, int gradi, int alto, boolean pinza)
    mode = (int)Serial.readStringUntil(',').toInt();  //0 cin directa, 1 cin inversa, 2 trayctoria, ...
    gradd = (int)Serial.readStringUntil(',').toInt();
    gradi = (int)Serial.readStringUntil(',').toInt();
    alto = (float)Serial.readStringUntil(',').toInt();
    giro = (int)Serial.readStringUntil(',').toInt();
    pinza = (int)Serial.readStringUntil('\n').toInt();
    pulsosd = Dgrados2pulsos(gradd);
    pulsosi = Igrados2pulsos(gradi);

    stepper.setTargetPositionInMillimeters(-alto);
    setPinza(giro, pinza);
  }


  // time difference
  currT = micros();
  deltaT = ((float)(currT - prevT)) / (1.0e6);
  prevT = currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  pos_d = 0;
  pos_i = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos_d = posid;
    pos_i = posii;
  }
  if ((iterations % 200 == 0)) {
    Serial.print(1);
    Serial.print(",");
    Serial.print(millis());
    Serial.print(",");
    Serial.print((int)(pulsosd * DPULSOS2GRAD));
    Serial.print(",");
    Serial.println((int)(pos_d * DPULSOS2GRAD));

    Serial.print(2);
    Serial.print(",");
    Serial.print(millis());
    Serial.print(",");
    Serial.print((int)(pulsosi * IPULSOS2GRAD));
    Serial.print(",");
    Serial.println((int)(pos_i * IPULSOS2GRAD));

    Serial.print(3);
    Serial.print(",");
    Serial.print(millis());
    Serial.print(",");
    Serial.print((int)alto);
    Serial.print(",");
    Serial.println((int)alto);
  }
  iterations = iterations + 1;

  ed = pos_d - pulsosd;
  ei = pos_i - pulsosi;

  // derivative
  dedtd = (ed - eprevd) / (deltaT);
  dedti = (ei - eprevi) / (deltaT);

  // integral
  eintegrald = eintegrald + ed * deltaT;
  eintegrali = eintegrali + ei * deltaT;

  // control signal
  ud = kp * ed + kd * dedtd + ki * eintegrald;
  ui = kp * ei + kd * dedti + ki * eintegrali;

  // motor power
  pwrd = setPower(ud, vd);
  pwri = setPower(ui, vi);

  // motor direction
  dird = setDir(dird, ud);
  diri = setDir(diri, ui);

  // signal the motor
  setMotor(dird, pwrd, PWMD, DCD_1, DCD_2);
  setMotor(diri, pwri, PWMI, DCI_1, DCI_2);
  stepper.processMovement();

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
  grad = map(grad, 0, 90, 20, 90);
  girar.write(grad);
  abrir.write(abierto == 0 ? 30 : 90);
}

float setPower(float u, float v) {  // anti windup and speed limit
  float pwr = fabs(u);
  if (pwr > v) {
    pwr = v;
  }
  return pwr;
}

// To-do: avoid pwr under 10, as motor might not be moving (set min speed and error acceptance level)

int setDir(int dir, float u) {
  dir = 1;
  if (u < 0) {
    dir = -1;
  }
  return dir;
}

void handler_encoderD() {
  if (digitalRead(ENCDA) == HIGH) {
    if (digitalRead(ENCDB) == LOW) {
      posid++;
      posii -= 0.53;
    } else {
      posid--;
      posii += 0.53;
    }
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
  float result = (float)grados * (float)DGRAD2PULSOS;
  if ((result < DPULSOSMAX) && (result > -DPULSOSMAX)) {
    return result;
  } else {
    Serial.print(4);
    Serial.print(",");
    Serial.print(1);
    Serial.print(",");
    Serial.print(result);
    Serial.print(",");
    Serial.println(posid);
    return posid;
  }
}

// lo igual, para el motor izquierdo
float Igrados2pulsos(float grados) {
  float result = (float)grados * (float)IGRAD2PULSOS;
  if ((result < IPULSOSMAX) && (result > -IPULSOSMAX)) {
    return result;
  } else {
    Serial.print(4);
    Serial.print(",");
    Serial.print(2);
    Serial.print(",");
    Serial.print(result);
    Serial.print(",");
    Serial.println(posii);
    return posii;
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
    return alto;
  }
}