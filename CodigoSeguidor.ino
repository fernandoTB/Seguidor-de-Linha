#include <Ultrasonic.h>//inclui biblioteca dos sensores ultrassonicos

int sensor1,sensor2,sensor3;//variaveis interas que indicam os sensores1 2 e 3
int objeto=1; // variavel que indica a presença do objeto
int cont=0; //contador

#define echoPin1 A2 //Pino ECHO do sensor1 no pino analógica A2
#define trigPin1 A3 //Pino TRIG do sensor1 no pino analógica A3
#define echoPin2 A1 //Pino ECHO do sensor2 no pino analógica A1
#define trigPin2 A0 //Pino TRIG do sensor2 no pino analógica A0
#define echoPin3 A5 //Pino ECHO do sensor3 no pino analógica A5
#define trigPin3 A4 //Pino TRIG do sensor3 no pino analógica A4
long duracao1; //Variável criada para salvar a duração entre o sinal sonoro e o echo do mesmo, para o sensor1 
long duracao2; //Variável criada para salvar a duração entre o sinal sonoro e o echo do mesmo, para o sensor2
long duracao3; //Variável criada para salvar a duração entre o sinal sonoro e o echo do mesmo, para o sensor3


// kp=4,ki=0.01,kd=1000;
// compensação do motor 1=130, 2=120;
int d1; //Valor digital correspondente ao sensor infravermelho digital 1
int d2; //Valor digital correspondente ao sensor infravermelho digital 2
int d3; //Valor digital correspondente ao sensor infravermelho digital 3
int d4; //Valor digital correspondente ao sensor infravermelho digital 4
int d5; //Valor digital correspondente ao sensor infravermelho digital 5
int linha; //Variável que indica a presença da linha
double erro=0; //Variável que indica o erro correspondente a posição do veículo
double erroant=0; //Variável que indica o erro anterior correspondente a posição do veículo
double P; //Variável proporcional do PID
double I; //Variável integral do PID
double D; //Variável derivativa do PID
double      kP=2.5; //Ganho proporcional
double      kI=0.0001; //Ganho integral
double      kD=400; //Ganho derivativo
double PID=0; //Valor de PID calculado
double pwm1; //Valor da onda PWM para motor Esquerdo
double pwm2; //Valor da onda PWM para motor Direito
unsigned long t; //Variável que salva o tempo de código
unsigned long tant=0; //Variavel que salva o tempo anteriror para calculo de variação de tempo

void setup() {
pinMode(trigPin1, OUTPUT); //Pino trigger1 setado como saida
pinMode(echoPin1, INPUT);  //Pino Echo1 setado como entrada
pinMode(trigPin2, OUTPUT); //Pino trigger2 setado como saida
pinMode(echoPin2, INPUT);  //Pino Echo2 setado como entrada
pinMode(trigPin3, OUTPUT); //Pino trigger3 setado como saida
pinMode(echoPin3, INPUT);  //Pino Echo3 setado como entrada
Serial.begin(9600); //Inicia comunicação serial para avaliação das leituras
}

void loop() {
  sensor1=Leitura1(); //chama a função para leitura do sensor ultrassonico1
  analogWrite(3,0); //Para o motor esquerdo
  analogWrite(11,0); //Para o motor direito
    
  while (objeto){ //Roda loop enquanto não for reconhecido objeto a frente do veículo. Este loop corresponde a tarefa de "Seguir a Linha".
    sensor1=Leitura1(); //chama a função para leitura do sensor ultrassonico1
    if (sensor1<=15){ //Se o sensor1 detectar algo a 15 cm ou menos deverá contabilzar o valor "1" na variavel cont.
      cont=cont+1;
      }
    else { //Caso o sensor1 nao detectar nada a variavel cont voltará a 0.
      cont=0;
      }
    if (cont > 1){ //O objeto só será reconhecido quando o sensor1 da frente fizer uma leitura menor ou igual a 15cm duas vezes, para filtrar sinal.
      objeto = 0;
      }
    
    erro=Erro(); //Chama a função que calcula o erro de posição do veículo.

      PID=calculaPID(erro,erroant,kP,kI,kD); //Chama a função que calcula o valor de PID.
      erroant=erro; //Salva o erro atual para proxima iteração
      pwm1=150-PID; //Calcula o valor PWM para motor esquerdo
      pwm2=170+PID; //Calcula o valor PWM para motor direito (Estes valores são diferentes para compensar a diferença percebida entre os motores)
      if(pwm1>255){ //Evita que o valor PWM extrapole 255
        pwm1=255;
        }
        else if(pwm1<0){ //Evita que o valor PWM seja menor que 0
          pwm1=0;
          }
      if(pwm2>255){ //Evita que o valor PWM extrapole 255
        pwm2=255;
        }
        else if(pwm2<0){ //Evita que o valor PWM seja menor que 0
          pwm2=0;
          }
    analogWrite(3,pwm1); //Escreve o valor pwm1 na base do transistor, controlando o motor esquerdo
    analogWrite(11,pwm2);//Escreve o valor pwm1 na base do transistor, controlando o motor direito
    Serial.println(PID); //printa o valor de PID calculado (para "tunar" o PID)
    }
    
    analogWrite(3,0); //Para o motor esquerdo
    analogWrite(11,0); //Para o motor direito
    delay(1000); //Gera um delay de 1 segundo
    sensor1=Leitura1(); //chama a função para leitura do sensor1 ultrassonico
    if (sensor1<=15){ //Mesmo que reconhecido o objeto uma vez, é testado novamente se o objeto contiuna na frente do veículo, para evitar que o veículo desvie de nada.
      DesviaObj(); //Entra na função responsável por desviar do objeto
    }
    objeto=1; //Indica que o objeto ja foi ultrapassado
    erro=0; //Zera o erro
    erroant=erro; //Zera o erro anterior
    d1=digitalRead(4); //Leitura do sensor infravermelho digital 1
    d2=digitalRead(7); //Leitura do sensor infravermelho digital 2
    d3=digitalRead(8); //Leitura do sensor infravermelho digital 3
    d4=digitalRead(12); //Leitura do sensor infravermelho digital 4
    d5=digitalRead(13); //Leitura do sensor infravermelho digital 5
    if ((d1+d2+d3+d4+d5>=3)||(d1)){ //Caso 3 ou mais sensores digitais reconhçam a linha, é necessário estas instruções para que o veículo retorne no sentido correto
      analogWrite(3,0); //para motor Esquerdo
      analogWrite(11,140); //roda motor direito (Fazendo com que o carro vire para a esquerda por 0.8 segundos)
      delay(800); //Delay necessário para que os sensores parem de ler a linha
      d3=digitalRead(8); //Faz leitura do sensor digital 3
      while (!d3){ //Assim que a linha ja não esta no alcance dos sensores, o veículo continuara virando ate que à reconheça denovo
      d3=digitalRead(8); //Faz leitura do sensor digital 3
        }
      analogWrite(3,0); //Para os motores
      analogWrite(11,0);//Para os motores
      }
}

int Leitura1(){ // Função que faz a leitura do sensor ultrassonico 1
  digitalWrite(trigPin1, LOW); //não envia som
  delayMicroseconds(2); //delay de 2 ms
  digitalWrite(trigPin1,HIGH); //envia som 
  delayMicroseconds(10); //delay de 10 ms
  digitalWrite(trigPin1,LOW); //não envia o som e espera o retorno do som enviado
  duracao1 = pulseIn(echoPin1,HIGH); //Captura a duração em tempo do retorno do som.
  sensor1 = duracao1/56; //Calcula a distância
  return sensor1; //retorna o valor adquirido
  }
int Leitura2(){ // Função que faz a leitura do sensor ultrassonico 2
  digitalWrite(trigPin2, LOW); //não envia som
  delayMicroseconds(2); //delay de 2 ms
  digitalWrite(trigPin2,HIGH); //envia som 
  delayMicroseconds(10); //delay de 10 ms
  digitalWrite(trigPin2,LOW); //não envia o som e espera o retorno do som enviado
  duracao2 = pulseIn(echoPin2,HIGH); //Captura a duração em tempo do retorno do som.
  sensor2 = duracao2/56; //Calcula a distância
  return sensor2; //retorna o valor adquirido
}
int Leitura3(){ // Função que faz a leitura do sensor ultrassonico 3
  digitalWrite(trigPin3, LOW); //não envia som
  delayMicroseconds(2);
  digitalWrite(trigPin3,HIGH); //envia som 
  delayMicroseconds(10);
  digitalWrite(trigPin3,LOW); //não envia o som e espera o retorno do som enviado
  duracao3 = pulseIn(echoPin3,HIGH); //Captura a duração em tempo do retorno do som.
  sensor3 = duracao3/56; //Calcula a distância
  return sensor3; //retorna o valor adquirido
  }

double calculaPID(double erro,double erroant,double kP,double kI,double kD){ //Função que calcula o valor de PID
      t = millis();//determinação do tempo atual do código
      P=erro; //Variável proporcional ao erro
      I=I+erro; //Somatório de todos os erros
      D=erro-erroant; //Diferença entre o erro atual e o anterior
      erroant=erro; //Salva erro atual para proxima iteração
      PID=kP*P+kI*I*(t-tant)+(kD*D)/(t-tant); //Calcula o valor PID, a taxa de amostragem é determinada por (t-tant) cerca de 43ms
      tant = t; //salva o tempo atual
      return PID; //retorna o valor de PID
  }

double Erro(){//Função que calcula o erro
  d1=digitalRead(4);//Faz as leituras dos sensores digitals infravermelhos 1 2 3 4 5
  d2=digitalRead(7);
  d3=digitalRead(8);
  d4=digitalRead(12);
  d5=digitalRead(13);
  if(d1==1 && d2==0 && d3==0 && d4==0 && d5==0){ // Situação 1, apenas o sensor 1 reconhece a linha, erro associado 50.
    erro=50;
    }
  else if(d1==1 && d2==1 && d3==0 && d4==0 && d5==0){// Situação 2, apenas os sensores 1 e 2 reconhece a linha, erro associado 40.
    erro=40;
    }
  else if(d1==0 && d2==1 && d3==0 && d4==0 && d5==0){// Situação 3, apenas o sensor 2 reconhece a linha, erro associado 30.
    erro=30;
    }
  else if(d1==0 && d2==1 && d3==1 && d4==0 && d5==0){// Situação 4, apenas os sensores 2 e 3 reconhece a linha, erro associado 20.
    erro=20;
    }
  else if(d1==0 && d2==0 && d3==1 && d4==0 && d5==0){// Situação 5, apenas o sensor 3 reconhece a linha, erro associado 0.
    erro=0;
    }
  else if(d1==0 && d2==0 && d3==1 && d4==1 && d5==0){// Situação 6, apenas os sensores 3 e 4 reconhece a linha, erro associado -20.
    erro=-20;
    }
  else if(d1==0 && d2==0 && d3==0 && d4==1 && d5==0){// Situação 7, apenas o sensor 4 reconhece a linha, erro associado -30.
    erro=-30;
    }
  else if(d1==0 && d2==0 && d3==0 && d4==1 && d5==1){// Situação 8, apenas os sensores 4 e 5 reconhece a linha, erro associado -40.
    erro=-40;
    }
  else if(d1==0 && d2==0 && d3==0 && d4==0 && d5==1){// Situação 9, apenas o sensor 5 reconhece a linha, erro associado -50.
    erro=-50;
    }
    return erro;
  }

void DesviaObj(){ //Função que desvia do objeto realizando uma série de instruções determinadas.
/*GIRAR PARA ESQUERDA ATÉ RECONHECER O OBJETO*/
  int lateral2=1; //lateral2 é a variável que indica se o sensor ultrassonico 2 ja reconheceu o objeto 
  int cont2=0; //Contador para assegurar a leitura
  
  while (lateral2){ //Enquanto o objeto nao for reconhecido pelo sensor 2
    sensor2=Leitura2(); //Chama a função que faz a leitura do sensor ultrassonico 2
    analogWrite(3,0); //para motor esquerdo
    analogWrite(11,130);//roda motor direito fazendo o veículo girar para esquerda
    if (sensor2 < 20){ //caso o sensor2 indique a presença de um objeto a menos de 20 cm ele incrementa o contador cont2
      cont2=cont2+1;
      }
    else{
      cont2=0;
      }
    if (cont2>1){ //caso o sensor2 indique a presença em 2 leituras seguidas, é seguro dizer que o objeto foi reconhecido
      lateral2=0;
      }
    }
  analogWrite(3,0); //para os motores
  analogWrite(11,0);//para os motores
  delay(1000); //delay de 1 segundo que facilita a continuidade do algoritmo e a visualização do processo

/*SEGUE UMA CURVA LEVE PARA DIREITA ATÉ RECONHECER O OBJETO*/
  int lateral3=1; //lateral3 é a variável que indica se o sensor ultrassonico 3 ja reconheceu o objeto 
  int cont3=0; //Contador para assegurar a leitura
  
  while (lateral3){ //Enquanto o objeto nao for reconhecido pelo sensor 3
    sensor3=Leitura3(); //Chama a função que faz a leitura do sensor ultrassonico 3
    analogWrite(3,140);//roda motor esquerdo
    analogWrite(11,140);//roda motor direito fazendo o veículo seguir uma "curva" bem leve para direita, contornando o objeto
    if (sensor3 < 20){ // Faz a mesma filtragem para os sensores ultrassônicos, assegurando o reconhecimento do objeto a 20cm
      cont3=cont3+1;
      }
    else{
      cont3=0;
      }
    if (cont3>1){
      lateral3=0;
      }
    }
  analogWrite(3,0);//para os motores
  analogWrite(11,0);
  delay(1000); //delay de 1 segundo que facilita a continuidade do algoritmo e a visualização do processo

/*SEGUE UMA CURVA PARA DIREITA ATÉ RECONHECER O OBJETO OU A LINHA*/
  lateral2=1;
  cont2=0;
  sensor2=Leitura2();
  while (lateral2){ //Realiza uma curva para direita até reconhecer o objeto ou a linha
    d1=digitalRead(4); // le os sensores digitais infravermelhos 1, 2 e 3
    d2=digitalRead(7);
    d3=digitalRead(8);
    linha=(d1||d2||d3); // indica a presença da linha caso qualquer um dos 3 sensores à reconhça
    if (linha){ // caso a linha seja reconhecida pare os motores e saia do loop
      lateral2=0;
      analogWrite(3,0);
      analogWrite(11,0);
      }
      else{ // caso nao reconheça continue virando até reconhecer o objeto
    sensor2=Leitura2();
    analogWrite(3,130);
    analogWrite(11,0);
    if (sensor2 < 10){ // Faz a mesma filtragem para os sensores ultrassônicos, assegurando o reconhecimento do objeto
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
  analogWrite(3,0); //pare os motores
  analogWrite(11,0);
  delay(1000); //delay de 1 segundo que facilita a continuidade do algoritmo e a visualização do processo

/*SEGUE UMA CURVA LEVE PARA ESQUERDA ATÉ RECONHECER O OBJETO OU A LINHA*/
  lateral3=1;
  cont3=0;
  sensor3=Leitura3();
  while (lateral3){
    d1=digitalRead(4);// le os sensores digitais infravermelhos 1, 2 e 3
    d2=digitalRead(7);
    d3=digitalRead(8);
    linha=(d1||d2||d3); // indica a presença da linha caso qualquer um dos 3 sensores à reconhça
    if (linha){ // caso a linha seja reconhecida pare os motores e saia do loop
      lateral3=0;
      analogWrite(3,0);
      analogWrite(11,0);
      } 
    else{ // caso nao reconheça continue virando até reconhecer o objeto
      sensor3=Leitura3();
      analogWrite(3,100);//Aciona os motores de modo a seguir uma curva leve para esquerda, particamente em linha reta.
      analogWrite(11,120);
      if (sensor3 < 20){ // realiza a filtragem do sinal do sensor ultrassonico
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
  analogWrite(3,0); // para motores
  analogWrite(11,0);
  delay(1000); //delay de 1 segundo que facilita a continuidade do algoritmo e a visualização do processo

  /*SEGUE UMA CURVA PARA DIREITA ATÉ RECONHECER O OBJETO OU A LINHA*/
  if (linha){//Caso a linha ja tenha sido reconhecida, nem sequer começa esta tarefa.
    lateral2=0;
    }
  else{//caso contrário, entrará no loop
    lateral2=1;
  }
  cont2=0;
  sensor2=Leitura2();
  while (lateral2){ // enquanto o objeto nao for reconhecido o veiculo realizara uma curva para direta
  d1=digitalRead(4); // faz as leituras dos sensores digitais 1 2 3
  d2=digitalRead(7);
  d3=digitalRead(8);
  linha=(d1||d2||d3); // determina que a linha foi reconhecida, caso qualquer um dos 3 sensores indicar 1
  if(linha){ // caso a linha seja reconhecida, termine a ação
    analogWrite(3,0);
    analogWrite(11,0);
    lateral2=0;
    }
  else{  // caso contrário realiza uma curva para direita
    sensor2=Leitura2();
    analogWrite(3,140); // gira motor esquerdo
    analogWrite(11,0); // para motor direito
    if (sensor2 < 20){ // realiza filtragem necessária para assegurar o reconhecimento
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
  /*SEGUE PRATICAMENTE RETO, UMA LEVE CURVA PARA ESQUERDA, ATÉ QUE A LINHA SEJA RECONHECIDA*/
    analogWrite(3,0);// para motores
    analogWrite(11,0);
    delay(1000); //delay de 1 segundo que facilita a continuidade do algoritmo e a visualização do processo
    while(!linha){ //equanto a linha nao for reconhecida o veículo seguirá praticamente reto, uma curva muito leve para esquerda
      d1=digitalRead(4);
      d2=digitalRead(7);
      d3=digitalRead(8);
      linha=(d1||d2||d3);
      analogWrite(3,100);
      analogWrite(11,120);
      }
    analogWrite(3,0);// para motores e retorna para a função "main" do seguidor de linha.
    analogWrite(11,0);
  }

