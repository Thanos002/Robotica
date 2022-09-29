#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define IN1 6    // Input3 conectada al pin 5 DC1
#define IN2 7    // Input4 conectada al pin 4 DC2
#define ENCA 2    // ENA conectada al pin 3 de Arduino
#define ENCB 4    //ENB
#define PWM1 5     // PWM Velocidad
int ENCA_DATA;
int ENCB_DATA;

long dir;
int grad;
int pulsos = 0;
int valor_pulsos = 0;
const int ratio = 1580;

//int contador = 0;
//int contador2 = 0;
int rounds = 0;
int limit = 1490 * 1;
int operacion_int = 1;

volatile int posi = 0; // specify posi as volatile
long prevT = 0;
float eprev = 0;
float eintegral = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  // attachInterrupt(digitalPinToInterrupt(ENCB), readEncoder, RISING);

  pinMode(PWM1, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  Serial.println("grad pos");
}


void loop() {
  // wait until serial available
  while (!Serial.available()) {
    Serial.println("Esperando operación");
    Serial.println("Operación: dir, valor");
    Serial.println("dir: izquierda 0, derecha 1");
  }
  // Convert String input to int
  dir = (int)Serial.readStringUntil(',').toInt();
  grad = (int)Serial.readStringUntil('\n').toInt();

  // check if input is digit
  if (!isdigit(dir) || !isdigit(grad)) {
    Serial.println("Numero invalido");
    return;
  }
  // convert grad to pulsos
  if (grad >= 0 && grad <= 360) {
    pulsos = grad * (ratio / 360);
  }
  else {
    Serial.print("Valor por encima de 360º");
  }

  //switch for opcode handling
  switch (operacion_int) {
    case 0:
      Serial.println("Giro hacia la izquierda");
      grad = -grad;
    case 1:
      Serial.println("Giro hacia la derecha");
    default: Serial.println("Operación no reconocida");
  }

  // PID constants
  float kp = 1;
  float kd = 0.025;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
  prevT = currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }

  // error
  int e = pos - grad;

  // derivative
  float dedt = (e - eprev) / (deltaT);

  // integral
  eintegral = eintegral + e * deltaT;

  // control signal
  float u = kp * e + kd * dedt + ki * eintegral;

  // motor power
  float pwr = fabs(u);
  if ( pwr > 255 ) {
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if (u < 0) {
    dir = -1;
  }

  // signal the motor
  setMotor(dir, pwr, PWM1, IN1, IN2);


  // store previous error
  eprev = e;

  Serial.print(grad);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  //set direction and speed of motor
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void readEncoder() { //run when A triggers interrupts
  int b = digitalRead(ENCB);
  if (b > 0) { //determine whether A or B interrupted first -> direction
    posi++;
  }
  else {
    posi--;
  }
}
