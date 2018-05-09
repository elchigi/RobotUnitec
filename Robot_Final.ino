 /* ----------------------------------------------------------
 *  Desarrollado por: Eliseo Chica y Lizeth Melendez
 *  Proyecto Integrador
 *  Corporacion Universitaria UNITEC - 2018     
    
 * NOTA DISTANCIA: 
 * Para el cálculo de distancia se utiliza la expresión 
 * cinemática: S = V*t, donde:
 * V = velocidad del sonido (343m/s = 0.0343)
 * t = tiempo calculado del pulso enviado (ida y vuelta)
 * S = distancia calculada
 * 
 * El cálculo de distancia para cada sensor de encuentra en 
 * una función por separado
 * -----------------------------------------------------------
 * NOTA SERVOMOTOR:
 * Se utiliza el pin 12 para controlar el microservo
 * -----------------------------------------------------------
 */

/*--------------------------------------------------------------
INICIO DEL PROGRAMA
--------------------------------------------------------------*/
//Se importan las librerias para com. serial y control de servo
#include <Servo.h>
#include <SoftwareSerial.h>

//Objeto llamado "myservo" para control del servo
Servo myservo; 

//Inicialización de variables globales
int pos=0,i=0; 
float promedio=0;
int cont,jugar=0,anterior=0,robot_Unitec=0;
int delayy = 570, delayy2=560, deel = 170,acomode = 60, limite=15;

//Variables para asignación de pines de los sensores 
int trig_i=2, echo_i=3; 
int trig_f=6, echo_f=7; 
int trig_d=4, echo_d=5; 

//Variables para cálculo de distancia
long  time_i,time_f,time_d;
float dist_i,dist_f,dist_d;
float prom_i,prom_f,prom_d;
float izq, frontal, der;
int f=0;

//Variables para control de motores
int der_1=A2,der_2=A3;
int izq_1=A0,izq_2=A1;
int d1,d2,i1,i2;

//Vectores sensor frontal
float front[45];
float posi[45];
float barrido[17];
int x=0,y=0,z=0,promediototal=0;
float izquierda = 0,derecha=0,frente=0;

/*
--------------------------------------------------------------
CONFIGURACIÓN DE ENTRADAS Y SALIDAS
--------------------------------------------------------------
*/
void setup() {
   
  //Pin D12 para control
  myservo.attach(12);  
   
  //Config. de ins-outs
  pinMode(trig_i,OUTPUT);
  pinMode(trig_d,OUTPUT);
  pinMode(trig_f,OUTPUT);
  pinMode(echo_i,INPUT); 
  pinMode(echo_d,INPUT); 
  pinMode(echo_f,INPUT); 
}


/*
--------------------------------------------------------------
CÓDIGO PRINCIPAL
--------------------------------------------------------------
*/  
void loop() {
decision();
action();
conversorlogico();
TomaDecision();
andar(); 
anterior=der;
//Tiempo para cuadrar la pos. del robot
delay(1500);
}


/*
--------------------------------------------------------------
LECTURA DE SENSOR DERECHO
--------------------------------------------------------------
*/
void sen_der(){
  digitalWrite(trig_i,LOW);
  digitalWrite(trig_f,LOW); 
  
  //Para asegurar un pulso limpio se pone 4us en LOW el trigger 
  digitalWrite(trig_d,LOW);
  delayMicroseconds(4);
  //Se lanza pulso para activar el sensor
  digitalWrite(trig_d,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_d,LOW);

  time_d = (pulseIn(echo_d,HIGH)/2); 
  dist_d = float(time_d*0.0343);
  
  //Toma 20 valores y hace un filtro promedio
  for (i=0;i<20;i++){prom_d = prom_d+dist_d;}
  prom_d = (prom_d/20);
  der = prom_d;
}
/*
--------------------------------------------------------------
 SENSOR IZQUIERDO
--------------------------------------------------------------
*/
void sen_izq(){
  digitalWrite(trig_d,LOW);
  digitalWrite(trig_f,LOW); 
  
  //Para asegurar un pulso limpio se pone 4us en LOW el trigger 
  digitalWrite(trig_i,LOW);
  delayMicroseconds(4);
  //Se lanza pulso para activar el sensor
  digitalWrite(trig_i,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_i,LOW);

  time_i = (pulseIn(echo_i,HIGH)/2); 
  dist_i = float(time_i*0.0343);
  
  //Toma 20 valores y hace un filtro promedio
  for (i=0;i<20;i++){prom_i = prom_i+dist_i;}
  prom_i = (prom_i/20);
  izq = prom_i;
}
/*
--------------------------------------------------------------
SENSOR FRONTAL
--------------------------------------------------------------
*/
void sen_front(){
  digitalWrite(trig_i,LOW);
  digitalWrite(trig_d,LOW);

  //Para asegurar un pulso limpio se pone 4us en LOW el trigger 
  digitalWrite(trig_f,LOW);
  delayMicroseconds(4);
  //Se lanza pulso para activar el sensor
  digitalWrite(trig_f,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_f,LOW);

  time_f = (pulseIn(echo_f,HIGH)/2); 
  dist_f = float(time_f*0.0343);
  
  //Toma 20 valores y hace un filtro promedio
  for (i=0;i<20;i++){prom_f = prom_f+dist_f;}
  prom_f = (prom_f/20);
  frontal = prom_f;
  return frontal;
}
/*
--------------------------------------------------------------
RUTINA DE SENSADO CON SERVO Y ENVÍO DE DATOS
--------------------------------------------------------------
*/
void decision(){ 
  myservo.write(0);
  sen_der(); 
  sen_izq();    
  pos=0;
  myservo.write(pos);
  delay(50); 
  cont = 1;
  
  for (int i=0;i<36;i++){  
    sen_front();  
    front[i] = frontal;   
    

        
    if(cont==1){
      posi[i] = pos;
      pos+=10;
      myservo.write(pos);            
    }
    
    if(pos==180){cont=2;}
    
    if(cont==2){
      posi[i]=pos;
      pos-=10;
      myservo.write(pos);  
    }    
    delay(50);  
  }     
} 
/*
--------------------------------------------------------------
ASIGNACIÓN DE VALORES DE IZQ-DER-FRONT SEGÚN RANGO DE 180°
--------------------------------------------------------------
*/
void action(){
  
  for (jugar =0; jugar <36; jugar++){
   /*----------------------------------------------------------
    *Valores de ángulo entre 0° y 50°
    --------------------------------------------------------/   
   */
  if(posi[jugar]>=0 && posi[jugar]<=50){               
    if(x==6){
      z=x-1;
      cont=0;
      promediototal=0;
      if(posi[jugar]==0){
          x=0;
        }      
      for(y=31;y<36;y++){                          
        if(front[y]>=(barrido[z]-2) && front[y]<=(barrido[z]+2)){
          cont++;
          promediototal=promediototal+front[y];
        }        
        z--;
        derecha=(promediototal/cont);        
      }      
    }else{
      for(x=1; x<6; x++){
        barrido[x] = front[x];
      }
    } 
  }     
   /*----------------------------------------------------------
    *Valores de ángulo entre 60° y 110°
    --------------------------------------------------------/  
    }*/
  else if(posi[jugar]>=60 && posi[jugar]<=110){      
    if(x==12){
      z=x-1;
      cont=0;
      promediototal=0;
      if(posi[jugar]==60){
          x=6;
        }
      for(y=25;y<31;y++){
        if(front[y]>=(barrido[z]-2) && front[y]<=(barrido[z]+2)){
        cont++;
        promediototal=promediototal+front[y];
        }
        z--;
        frente=(promediototal/cont);
      }
    }else{
      for(x=6; x<12; x++){
      barrido[x] = front[x];
      }
    }
  }
   /*----------------------------------------------------------
    *Valores de ángulo entre 120° y 170°
    --------------------------------------------------------/
    }*/
    else if(posi[jugar]>=120 && posi[jugar]<=170){   
      if(x==18){
        z=x-1;
        cont=0;
        promediototal=0;
        if(posi[jugar]==120){
          x=12;
        }
        for(y=19;y<25;y++){
          if(front[y]>=(barrido[z]-2) && front[y]<=(barrido[z]+2)){
            cont++;
            promediototal=promediototal+front[y];
          }
          z--;
          izquierda=(promediototal/cont);
          
        }              
      }else{
        for(x=12; x<18; x++){
          barrido[x] = front[x];
        }
      }
    }else{
        x=18;      
    }    
  }
}
/*
--------------------------------------------------------------
CONVERSIÓN LÓGICA SEGÚN VALOR SENSADO
--------------------------------------------------------------
*/
void conversorlogico(){  
  if(izquierda>=0 && izquierda<=limite){
    izquierda=0;
  }else{
    izquierda=1;
  }
  
  if(izq>=0 && izq<=20){
    izq=0;
  }else{
    izq=1;
  }
  if(der>=0 && der<=20){
    der=0;
  }else{
    der=1;
  }
  
  if(frente>=0 && frente<=limite){
    frente=0;
  }else{
    frente=1;
  }
    if(derecha>=0 && derecha<17){
    derecha=0;
  }else{
    derecha=1;
  } 
}
/*
--------------------------------------------------------------
RUTINAS DE MOTORES
--------------------------------------------------------------
*/
void Derecha(){
  robot_Unitec=0;     
  analogWrite(izq_1,1023);
  analogWrite(izq_2,0);
  delay(deel);
  analogWrite(der_1,0);
  analogWrite(der_2,1023);  
  delay(deel);
}
void Izquierda(){
  robot_Unitec=0;
  analogWrite(der_1,1023);
  analogWrite(der_2,0);
  delay(deel);
  analogWrite(izq_1,0);
  analogWrite(izq_2,1023);  
  delay(deel);
}
void Adelante(){
  robot_Unitec=1;
  analogWrite(izq_1,1023);
  analogWrite(izq_2,0);
  delay(acomode);
  analogWrite(der_1,1023);
  analogWrite(der_2,0);
  analogWrite(izq_1,1023);
  analogWrite(izq_2,0);  
  delay(delayy);       
}
/*
--------------------------------------------------------------
RUTINA PARA TOMAR DECISIONES
Cabe resaltar que el robot tiene como prioridad la derecha
-0 es detección de pared y
-1 es espacio libre
--------------------------------------------------------------
*/
void TomaDecision(){
  if(der==0 && frente==0 && izq==0 && anterior==0){
    robot_Unitec=0;    
    analogWrite(izq_1,1023);
    analogWrite(izq_2,0);
    delay(140);
    analogWrite(der_1,0);
    analogWrite(der_2,1023);  
    delay(140);        
  }else if(der==0 && frente==0 && izq==0 && anterior==1){
    Derecha();
  }else if(der==0 && frente==0 && izq==1 && anterior==0){
    Izquierda();
  }else if(der==0 && frente==0 && izq==1 && anterior==1){
    Izquierda();
  }else if(der==0 && frente==1 && izq==0 && anterior==0){
    Adelante();
  }else if(der==0 && frente==1 && izq==0 && anterior==1){
    Adelante();
  }else if(der==0 && frente==1 && izq==1 && anterior==0){
    Adelante();
  }else if(der==0 && frente==1 && izq==1 && anterior==1){
    Adelante();
  }else if(der==1 && frente==0 && izq==0 && anterior==0){
    Derecha();
  }else if(der==1 && frente==0 && izq==0 && anterior==1){
    Derecha();
  }else if(der==1 && frente==0 && izq==1 && anterior==0){
    Derecha();
  }else if(der==1 && frente==0 && izq==1 && anterior==1){
    Derecha();
  }
  else if(der==1 && frente==1 && izq==0 && anterior==0){
    Derecha();
  }
  else if(der==1 && frente==1 && izq==0 && anterior==1 && robot_Unitec==1){      
    Derecha();
  }
  else if(der==1 && frente==1 && izq==0 && anterior==1 && robot_Unitec==0){      
    Adelante();
  }
  else if(der==1 && frente==1 && izq==1 && anterior==0){
    Derecha();
  }
  else if(der==1 && frente==1 && izq==1 && anterior==1){
    Adelante();
  }
}
/*
--------------------------------------------------------------
RUTINA PARA DETENER MOTORES
--------------------------------------------------------------
*/
void andar(){
  analogWrite(der_1,0);
  analogWrite(der_2,0);
  analogWrite(izq_1,0);
  analogWrite(izq_2,0);  
  delay(300); 
}

