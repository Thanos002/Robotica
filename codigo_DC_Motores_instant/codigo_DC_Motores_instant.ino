#include <FlexyStepper.h>

#include <util/atomic.h>  // For the ATOMIC_BLOCK macro
#include <Servo.h>
#include "Config.h"
#include "Control.h"

//Todos los valores en mm, grados!

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
  setPinza(0, 0);

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
  stepper.setAccelerationInStepsPerSecondPerSecond(300);

  initialized = true;
}

void loop() {

  if (initialized == false) {

    delay(5000);
    while (digitalRead(FINI) == HIGH) {
      setMotor(-1, 130, PWMD, DCD_1, DCD_2);
      delay(1);
    }
    setMotor(0, 0, PWMD, DCD_1, DCD_2);                 // stop motor
    posid, pulsosd, pos_d = DGRAD2PULSOS * DGRADOSMAX;  //set posi vars to current posi

    delay(2000);

    while (digitalRead(FINE) == HIGH) {
      setMotor(1, 130, PWMI, DCI_1, DCI_2);
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
    gradd = (float)Serial.readStringUntil(',').toFloat();
    gradi = (float)Serial.readStringUntil(',').toFloat();
    alto = (float)Serial.readStringUntil(',').toFloat();
    giro = (int)Serial.readStringUntil(',').toFloat();
    pinza = (int)Serial.readStringUntil(',').toInt();
    xA = (float)Serial.readStringUntil(',').toFloat();  //punto A, x
    yA = (float)Serial.readStringUntil(',').toFloat();
    xB = (float)Serial.readStringUntil(',').toFloat();
    yB = (float)Serial.readStringUntil(',').toFloat();
    vX = (float)Serial.readStringUntil(',').toFloat();
    vY = (float)Serial.readStringUntil(',').toFloat();
    vZ = (float)Serial.readStringUntil('\n').toFloat();

    stepper.setTargetPositionInMillimeters(-alto);
    setPinza(giro, pinza);

    switch (mode) {
      case 0:  // cin directa code per arrival
        pulsosd = Dgrados2pulsos(gradd);
        pulsosi = Igrados2pulsos(gradi);

        break;
      case 1:  // cin inversa code per arrival
        cinematicaInversa(xA, yA, &gradd, &gradi);
        pulsosd = Dgrados2pulsos(gradd);
        pulsosi = Igrados2pulsos(gradi);
        break;
      case 2:  // trayectoria code per arrival
        line_increment = 0;
        cinematicaInversa(xA, yA, &gradd, &gradi);
        pulsosd = Dgrados2pulsos(gradd);
        pulsosi = Igrados2pulsos(gradi);
        puntos = (int)getDistance(xA, yA, xB, yB);
        //dx = (xA - xB) / puntos;
        //dy = (yB - yA) / puntos;
        //timer = millis();
        //dt = getDistance(xA, yA, xB, yB) * 100 / puntos;
        //tprev, w1, w2 = 0;
        //vx = (xB - xA) / totalTime;
        //vy = (xB - xA) / totalTime;
        break;
      case 3:  // solo jacobiana
        target_vel=min(min(v_cart_max_d, v_cart_max_i),10);  //in mm/s
        break;
      case 4:  // jacobiana con linea recta

        break;
      case 5:  // velocidades articulares received
        v_art_max_d = vX*0.7;
        v_art_max_i = vY*0.7;
        stepper.setSpeedInStepsPerSecond(6*vZ);
        break;
      case 6:  // velocidades cartesianas received
        v_cart_max_d = vX*0.7;
        v_cart_max_i = vY*0.7;
        stepper.setSpeedInStepsPerSecond(6*vZ);
        break;
    }
  }

  if (mode == 1) {
    //cinematica inversa code in PID loop
  }

  if (mode == 2) {
    //trayectoria code in PID loop
    //timer = millis();
    x_line = xA + (xB - xA) * line_increment / puntos;
    y_line = yA + (yB - yA) * line_increment / puntos;
    //cinematica();
    //dx = x_line - x_real;
    //dy = y_line - y_real;
    //jacobiana_inversa(gradd, gradi, vx, vy);
    //wd = map(w1_jac,0,70,0,255);
    //wi = map(w2_jac,0,70,0,255);
    //wd, wi = 70;

    c_inputd = posid;
    c_inputi = posii;
    cinematicaInversa(x_line, y_line, &c_inputd, &c_inputi);
    //Serial.println("inversa call:");
    //Serial.print(x_line);
    //Serial.print(y_line);
    //Serial.println(c_inputd);
    //Serial.println(c_inputi);

    pulsosd = Dgrados2pulsos(c_inputd);
    pulsosi = Igrados2pulsos(c_inputi);

    //diri_lin = setDir(diri_lin, wi_jac);
    //dird_lin = setDir(dird_lin, wd_jac);
    //setMotor(diri_lin, 100, PWMD, DCD_1, DCD_2);
    //setMotor(diri_lin, 100, PWMI, DCI_1, DCI_2);
    //w1 = Dgrados2pulsos(w1_jac);
    //w2 = Igrados2pulsos(w2_jac);
  }

  if (mode == 3) {  // Jacobiana
    do {
      timenow = millis();
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        pos_d = posid;
        pos_i = posii;
      }
      cinematicaInversa(xA, yA, &c_inputd, &c_inputi);
      totalTime = getTrayTime(xA, xB, yA, yB, target_vel);
      v_x = getV(totalTime, xA, xB);
      v_y = getV(totalTime, yA, yB);
      q1_now = pos_d * DPULSOS2GRAD;
      q2_now = pos_i * IPULSOS2GRAD;
      jacobianaInversa(q1_now, q2_now, v_x, v_y, &target_v1, &target_v2);
      setMotorSpeed(target_v1, &D_pwm);
      setMotorSpeed(target_v2, &I_pwm);
      setMotor(setDir(dird, target_v1), D_pwm, PWMD, DCD_1, DCD_2);
      setMotor(setDir(diri, target_v2), I_pwm, PWMI, DCI_1, DCI_2);
    } while (timenow < totalTime);
    setMotor(0, 0, PWMD, DCD_1, DCD_2);
    setMotor(0, 0, PWMI, DCI_1, DCI_2);
    pulsosd = Dgrados2pulsos(pos_d);
    pulsosi = Igrados2pulsos(pos_i);
  }

  // error is used instead (see below)
  /**
  if (timer - tprev > 0.9 * dt) {
    tprev = millis();
    line_increment++;
  }
**/

  // time difference PID
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
  if ((iterations % 500 == 0)) {
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

    cinematicaDirecta(pos_d * DPULSOS2GRAD, pos_i * DPULSOS2GRAD, &x, &y);

    Serial.print(6);
    Serial.print(",");
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.print(",");
    Serial.println(distance(x, y));
    // revisar usa de distance()
  }
  iterations = iterations + 1;

  ed = pos_d - pulsosd;
  ei = pos_i - pulsosi;

  if ((ed < 8) && (ei < 8) && (line_increment <= puntos)) {
    line_increment++;
    //Serial.println("line loop");
    //Serial.println(line_increment);
  }


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
  pwrd = setPower(ud, v_art_max_d);
  pwri = setPower(ui, v_art_max_i);

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
  grad = map(grad, 0, 90, 15, 92);
  girar.write(grad);
  abrir.write(abierto == 0 ? 25 : 0);
  //abrir.write(abierto);
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
      posii -= X_Y_CORR;
    } else {
      posid--;
      posii += X_Y_CORR;
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
float Dgrados2pulsos(float gradosd) {
  //gradosd = constrain(gradosd, -DGRADOSMAX, DGRADOSMAX);
  float result = (float)gradosd * (float)DGRAD2PULSOS;
  if ((gradosd > DPULSOSMAX) || (gradosd < -DPULSOSMAX)) {
    Serial.print(4);
    Serial.print(",");
    Serial.print(1);
    Serial.print(",");
    Serial.print(result);
    Serial.print(",");
    Serial.println(posid);
    return posid;
  }
  //result = constrain(result, -DGRADOSMAX, DGRADOSMAX);
  return result;
}

// lo igual, para el motor izquierdo
float Igrados2pulsos(float gradosi) {
  float result = (float)gradosi * (float)IGRAD2PULSOS;
  if ((gradosi > IPULSOSMAX) || (gradosi < -IPULSOSMAX)) {
    Serial.print(4);
    Serial.print(",");
    Serial.print(2);
    Serial.print(",");
    Serial.print(result);
    Serial.print(",");
    Serial.println(posii);
    return posii;
  }
  //result = constrain(result, -IGRADOSMAX, IGRADOSMAX);
  //limits_index = int((pulsosd - DGRADOSMAX) / -20.0);  // assign pulsosd to limits index
  //limits_index = constrain(limits_index, 0, 11);       // limit position depending of pulsosd
  //gradosi = constrain(gradosi, robot_limits_min[limits_index], robot_limits_max[limits_index]);
  return result;
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