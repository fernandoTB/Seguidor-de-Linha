#include <Ultrasonic.h>

Ultrasonic ultrasonic3(2, 10);
int sensor1,sensor2,sensor3;
int objeto=1;
int cont=0;

#define echoPin1 A2 //Pino ECHO do sensor no pino analógica A1
#define trigPin1 A3
#define echoPin2 A1 //Pino ECHO do sensor no pino analógica A1
#define trigPin2 A0 //Pino TRIG do sensor no pino analógica A0
#define echoPin3 A5 //Pino ECHO do sensor no pino analógica A1
#define trigPin3 A4
long duracao1;
long duracao2;
long duracao3;


// kp=4,ki=0.01,kd=1000;
// compensação do motor 1=130, 2=120;
int d1;
int d2;
int d3;
int d4;
int d5;
int linha;
double erro=0;
double erroant=0;
double P;
double I;
double D;
double      kP=2.5;
double      kI=0.0001;
double      kD=400;
double PID=0;
double pwm1;
double pwm2;
unsigned long t;
unsigned long tant=0;

void setup() {
pinMode(trigPin1, OUTPUT);
pinMode(echoPin1, INPUT);
pinMode(trigPin2, OUTPUT);
pinMode(echoPin2, INPUT);
pinMode(trigPin3, OUTPUT);
pinMode(echoPin3, INPUT);
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  sensor1=Leitura1();
  analogWrite(3,0);
  analogWrite(11,0);
    
  while (objeto){
    sensor1=Leitura1();
    if (sensor1<=15){
      cont=cont+1;
      }
    else {
      cont=0;
      }
    if (cont > 1){
      objeto = 0;
      }
    
    erro=Erro();

      PID=calculaPID(erro,erroant,kP,kI,kD);
      erroant=erro;
      pwm1=150-PID;
      pwm2=170+PID;
      if(pwm1>255){
        pwm1=255;
        }
        else if(pwm1<0){
          pwm1=0;
          }
      if(pwm2>255){
        pwm2=255;
        }
        else if(pwm2<0){
          pwm2=0;
          }
    analogWrite(3,pwm1);
    analogWrite(11,pwm2);
    Serial.println(PID);
    }
    
    analogWrite(3,0);
    analogWrite(11,0);
    delay(1000);
    sensor1=Leitura1();
    if (sensor1<=15){
      DesviaObj();
    }
    objeto=1;
    erro=0;
    erroant=erro;
    d1=digitalRead(4);
    d2=digitalRead(7);
    d3=digitalRead(8);
    d4=digitalRead(12);
    d5=digitalRead(13);
    if ((d1+d2+d3+d4+d5>=3)||(d1)){
      analogWrite(3,0);
      analogWrite(11,140);
      delay(800);
      d3=digitalRead(8);
      while (!d3){
      d3=digitalRead(8);
        }
      analogWrite(3,0);
      analogWrite(11,0);
      }
}

int Leitura1(){
  digitalWrite(trigPin1, LOW); //não envia som
  delayMicroseconds(2);
  digitalWrite(trigPin1,HIGH); //envia som 
  delayMicroseconds(10);
  digitalWrite(trigPin1,LOW); //não envia o som e espera o retorno do som enviado
  duracao1 = pulseIn(echoPin1,HIGH); //Captura a duração em tempo do retorno do som.
  sensor1 = duracao1/56; //Calcula a distância
  return sensor1; //Exibe na Serial a distância
  }
int Leitura2(){
  digitalWrite(trigPin2, LOW); //não envia som
  delayMicroseconds(2);
  digitalWrite(trigPin2,HIGH); //envia som 
  delayMicroseconds(10);
  digitalWrite(trigPin2,LOW); //não envia o som e espera o retorno do som enviado
  duracao2 = pulseIn(echoPin2,HIGH); //Captura a duração em tempo do retorno do som.
  sensor2 = duracao2/56; //Calcula a distância
  return sensor2; //Exibe na Serial a distância
}
int Leitura3(){
  digitalWrite(trigPin3, LOW); //não envia som
  delayMicroseconds(2);
  digitalWrite(trigPin3,HIGH); //envia som 
  delayMicroseconds(10);
  digitalWrite(trigPin3,LOW); //não envia o som e espera o retorno do som enviado
  duracao3 = pulseIn(echoPin3,HIGH); //Captura a duração em tempo do retorno do som.
  sensor3 = duracao3/56; //Calcula a distância
  return sensor3; //Exibe na Serial a distância
  }

double calculaPID(double erro,double erroant,double kP,double kI,double kD){
      t = millis();
      P=erro;
      I=I+erro;
      D=erro-erroant;
      erroant=erro;
      PID=kP*P+kI*I*(t-tant)+(kD*D)/(t-tant);
      tant = t;
      return PID;
  }

double Erro(){
  d1=digitalRead(4);
  d2=digitalRead(7);
  d3=digitalRead(8);
  d4=digitalRead(12);
  d5=digitalRead(13);
  if(d1==1 && d2==0 && d3==0 && d4==0 && d5==0){
    erro=50;
    }
  else if(d1==1 && d2==1 && d3==0 && d4==0 && d5==0){
    erro=40;
    }
  else if(d1==0 && d2==1 && d3==0 && d4==0 && d5==0){
    erro=30;
    }
  else if(d1==0 && d2==1 && d3==1 && d4==0 && d5==0){
    erro=20;
    }
  else if(d1==0 && d2==0 && d3==1 && d4==0 && d5==0){
    erro=0;
    }
  else if(d1==0 && d2==0 && d3==1 && d4==1 && d5==0){
    erro=-20;
    }
  else if(d1==0 && d2==0 && d3==0 && d4==1 && d5==0){
    erro=-30;
    }
  else if(d1==0 && d2==0 && d3==0 && d4==1 && d5==1){
    erro=-40;
    }
  else if(d1==0 && d2==0 && d3==0 && d4==0 && d5==1){
    erro=-50;
    }
    return erro;
  }

void DesviaObj(){
  int lateral2=1;
  int cont2=0;
  sensor2=Leitura2();
  while (lateral2){
    sensor2=Leitura2();
    analogWrite(3,0);
    analogWrite(11,130);
    if (sensor2 < 20){
      cont2=cont2+1;
      }
    else{
      cont2=0;
      }
    if (cont2>1){
      lateral2=0;
      }
    }
  analogWrite(3,0);
  analogWrite(11,0);
  delay(1000);

  int lateral3=1;
  int cont3=0;
  sensor3=Leitura3();
  while (lateral3){
    sensor3=Leitura3();
    analogWrite(3,140);
    analogWrite(11,140);
    if (sensor3 < 20){
      cont3=cont3+1;
      }
    else{
      cont3=0;
      }
    if (cont3>1){
      lateral3=0;
      }
    }
  analogWrite(3,0);
  analogWrite(11,0);
  delay(1000);

  lateral2=1;
  cont2=0;
  sensor2=Leitura2();
  while (lateral2){
    d1=digitalRead(4);
    d2=digitalRead(7);
    d3=digitalRead(8);
    linha=(d1||d2||d3);
    if (linha){
      lateral2=0;
      analogWrite(3,0);
      analogWrite(11,0);
      }
      else{
    sensor2=Leitura2();
    analogWrite(3,130);
    analogWrite(11,0);
    if (sensor2 < 10){
      cont2=cont2+1;
      }
    else{
      cont2=0;
      }
    if (cont2>1){
      lateral2=0;
      }
     }
    }
  analogWrite(3,0);
  analogWrite(11,0);
  delay(1000);

  lateral3=1;
  cont3=0;
  sensor3=Leitura3();
  while (lateral3){
    d1=digitalRead(4);
    d2=digitalRead(7);
    d3=digitalRead(8);
    linha=(d1||d2||d3);
    if (linha){
      lateral3=0;
      analogWrite(3,0);
      analogWrite(11,0);
      }
    else{
      sensor3=Leitura3();
      analogWrite(3,100);
      analogWrite(11,120);
      if (sensor3 < 20){
        cont3=cont3+1;
        }
      else{
        cont3=0;
        }
      if (cont3>1 ){
        lateral3=0;
        }
      }
    }
  analogWrite(3,0);
  analogWrite(11,0);
  delay(1000);
  if (linha){
    lateral2=0;
    }
  else{
    lateral2=1;
  }
  cont2=0;
  sensor2=Leitura2();
  while (lateral2){
  d1=digitalRead(4);
  d2=digitalRead(7);
  d3=digitalRead(8);
  linha=(d1||d2||d3);
  if(linha){
    analogWrite(3,0);
    analogWrite(11,0);
    lateral2=0;
    }
  else{  
    sensor2=Leitura2();
    analogWrite(3,140);
    analogWrite(11,0);
    if (sensor2 < 20 || linha){
      cont2=cont2+1;
      }
    else{
      cont2=0;
      }
    if (cont2>1){
      lateral2=0;
      }
    }
  }
    analogWrite(3,0);
    analogWrite(11,0);
    delay(1000);
    while(!linha){
      d1=digitalRead(4);
      d2=digitalRead(7);
      d3=digitalRead(8);
      linha=(d1||d2||d3);
      analogWrite(3,100);
      analogWrite(11,120);
      }
    analogWrite(3,0);
    analogWrite(11,0);
  }

