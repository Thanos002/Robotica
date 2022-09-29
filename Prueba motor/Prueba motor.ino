#define IN1 7    // Input3 conectada al pin 5 DC1
#define IN2 6    // Input4 conectada al pin 4 DC2
#define SENSOR1 3 //encoder 1
#define SENSOR2 4  //encoder 2

String operacion;
String valor;
int pulsos=0;
int valor_pulsos=0;

void	setup()	{
		pinMode(SENSOR1,	INPUT_PULLUP);
		pinMode(SENSOR2,	INPUT_PULLUP);
		attachInterrupt(digitalPinToInterrupt(SENSOR1),	Sensor1,	CHANGE);
		attachInterrupt(digitalPinToInterrupt(SENSOR2),	Sensor2,	CHANGE);
}
void Sensor1(){																				//	Interrupción	del	encoder	1
		if(digitalRead(SENSOR1)	==	digitalRead(SENSOR2))		pulsos++;
		else	pulsos--;
}
void	Sensor2(){																				//	Interrupción	del	encoder	2
			if(digitalRead(SENSOR1)	==	digitalRead(SENSOR2))		pulsos--;
		else	pulsos++;
}


void loop(){
  Serial.begin(9600);
  if(Serial.available()){
    operacion=Serial.readStringUntil(',');
    valor=Serial.readStringUntil('\n');
    int operacion_int, valor_int;
    operacion_int=operacion.toInt();
    valor_int=valor.toInt();
    switch (operacion_int){
      case 0:
      Serial.println("Giro hacia la izquierda");
      digitalWrite(IN1,HIGH);
      digitalWrite(IN2,LOW);

      case 1:
      Serial.println("Giro hacia la derecha");
      digitalWrite(IN1,LOW);
      digitalWrite(IN2,HIGH);

      default: Serial.println("Operación no reconocida");
       }
  if (valor_int>0 && valor_int <360){
  valor_pulsos=valor_int*(1448/360);
  if (pulsos<valor_pulsos && pulsos>0){
      digitalWrite(IN1,HIGH);
      digitalWrite(IN2,LOW);    
  }
  }
  else Serial.print("Valor por encima de 360º");
  }
}