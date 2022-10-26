#include <Arduino.h>

#include <util/atomic.h>  // For the ATOMIC_BLOCK macro

// Pinout para el Arduino Mega 2540

#define DCD1 22
#define DCD2 23

#define DCI1 24
#define DCI2 25

#define ENCDA 26
#define ENCDB 27
#define ENCIA 28
#define ENCIB 29

#define STEP 30
#define DIR 31

#define SRESET 32    // resets internal logic and step table, defualt inactive/low during operation
#define SLEEPPIN 33  // low power mode, needs 1ms to reactivate
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

volatile int posid = 0;  // specify posi as volatile
long prevTd = 0;
float eprevd = 0;
float eintegrald = 0;
volatile int posii = 0;  // specify posi as volatile
long prevTi = 0;
float eprevi = 0;
float eintegrali = 0;


void setup() {
    Serial.begin(9600);
    pinMode(ENCDA, INPUT);
    pinMode(ENCDB, INPUT);
    pinMode(ENCDA, INPUT);
    pinMode(ENCDB, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCDA), readEncoderAFBR(posid, ENCDB), FALLING);
    attachInterrupt(digitalPinToInterrupt(ENCIA), readEncoderAFBR(posii, ENCIB), FALLING);
    attachInterrupt(digitalPinToInterrupt(ENCDB), readEncoderAFBR(posid, ENCDA), RISING);
    attachInterrupt(digitalPinToInterrupt(ENCIB), readEncoderAFBR(posii, ENCIA), RISING);

    attachInterrupt(digitalPinToInterrupt(ENCDA), readEncoderARBF(posid, ENCDB), RISING);
    attachInterrupt(digitalPinToInterrupt(ENCIA), readEncoderARBF(posii, ENCIB), RISING);
    attachInterrupt(digitalPinToInterrupt(ENCDB), readEncoderARBF(posid, ENCDA), FALLING);
    attachInterrupt(digitalPinToInterrupt(ENCIB), readEncoderARBF(posii, ENCIA), FALLING);

    pinMode(PWMD, OUTPUT);
    pinMode(PWMI, OUTPUT);
    pinMode(DCD1, OUTPUT);
    pinMode(DCD2, OUTPUT);
    pinMode(DCI1, OUTPUT);
    pinMode(DCI2, OUTPUT);
    pinMode(SERVO1, OUTPUT);
    pinMode(SERVO2, OUTPUT);
    pinMode(STEP, OUTPUT);
    pinMode(DIR, OUTPUT);
    pinMode(MS1, OUTPUT);
    pinMode(MS2, OUTPUT);
    pinMode(MS3, OUTPUT);
    pinMode(SRESET, OUTPUT);
    pinMode(SLEEPPIN, OUTPUT);
    pinMode(ENAPIN, OUTPUT);

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
        pinza = (int)Serial.readStringUntil('\n').toInt();
        pulsosd = signCorrection(dird, gradd);
        pulsosi = signCorrection(diri, gradi);
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
        Serial.print(",")
        Serial.print(millis());
        Serial.print(",");
        Serial.print(steps);
        Serial.print(",");
        Serial.println(steps);
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
    int dird = setDir(dird);
    int diri = setDir(diri);

    // signal the motor
    setMotor(dird, pwrd, PWMD, DCD1, DCD2);
    setMotor(diri, pwri, PWMI, DCI1, DCI2);

    // store previous error
    eprevd = ed;
    eprevi = ei;
}

int signCorrection(int dir, int grad) {
    if (grad >= 0 && grad <= 360) {
        pulsos = (int)((float)grad * (float)((float)ratio / (float)360);
    }
    if (dir == 0) {
        pulsos = -pulsos
    } else {
        //Serial.print("Valor por encima de 360º");
        return 0;
    }
    return pulsos;
}

void setMotor(int dir, int pwmVal, int pwm, int DCD1, int DCD2) {
//set direction and speed of motor
analogWrite(pwm, pwmVal);
if (dir == 1) {
digitalWrite(DCD1, HIGH);
digitalWrite(DCD2, LOW);
} else if (dir == -1) {
digitalWrite(DCD1, LOW);
digitalWrite(DCD2, HIGH);
} else {
digitalWrite(DCD1, LOW);
digitalWrite(DCD2, LOW);
dir = 0;
pulsos = 0;
}
}

float setPower(float u) {
    pwr = fabs(u);
    if (pwr > 255) {
        pwr = 255;
    }
    return pwr;
}


int setDir(int dir) {
    int dir = 1;
    if (u < 0) {
        dir = -1;
    }
    return dir;
}

void readEncoderAFBR(int posi, int enc) {  //when A Falling or B Rising, pass posi var and opposite enc
    int b = digitalRead(enc);
    if (b > 0) {  //determine whether A or B interrupted first -> direction
        posi++;
    } else {
        posi--;
    }
}

void readEncoderARBF(int posi, int enc) {  //when A Rising or B Falling
    int b = digitalRead(enc);
    if (b > 0) {  //determine whether A or B interrupted first -> direction
        posi--;
    } else {
        posi++;
    }
}