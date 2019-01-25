#include <TimerOne.h>

const int CHA = 2; // Canal A del encoder del motor conectado a pin 2 (entrada)
const int CHB = 3; // Canal B del encoder del motor en pin 3

const int PWM_Motor = 9; // Pin Salida PWM para el motor DC
const int AIN1 = 11; // Enable AIN1 del puente H del motor 
const int AIN2 = 10; // Enable AIN2 del puente H del motor

//const int Volt = 0; // Valor del voltaje (-8.5 / 8.5)
double Kp = 6;
double Ki= 0.3;
double Kd = 0.5;

double Pn = 0; //Salida del PID para muestra n
double En = 0; // Error para muestra n
double Pn1 = 0; // salida PID para muestra n‐1
double En1 = 0; //Error para muestra n‐1
double En2 = 0; // Error para muestra n-2

double q0=0;
double q1=0;
double q2=0;

double Tm = 0.01; // periodo de muestreo en seg
double consigna = 6.28; // angulo de consigna
double angulo = 0; // angulo medido a partir de pulsos del encoder
double pulsos = 0; //pulsos del encoder

double SalidaPD = 0;
double EntradaPWM = 0;

int valorPWM8;

int inicio = 0;

void setup() {

  Serial.begin(9600);
  
  // Valores iniciales
  q0 = Kp+(Ki*Tm)/2+Kd/Tm;
  q1 = -Kp+(Ki*Tm)/2-(2*Kd)/Tm;
  q2 = Kd/Tm;

  // INPUT
  pinMode(CHA, INPUT_PULLUP);
  pinMode(CHB, INPUT_PULLUP);
  
  // put your setup code here, to run once:
  //La lectura de los encoder se realizara por interrupciones, una por cada canal
  attachInterrupt(0, Encoder_CHA, CHANGE); // INTERRUPCION MEGA para PIN 2
  attachInterrupt(1, Encoder_CHB, CHANGE); // INTERRUPCION MEGA para PIN 3

  //Configuracion de los pines de E/S 
  pinMode(AIN1,OUTPUT); 
  pinMode(AIN2,OUTPUT); 
  pinMode(PWM_Motor,OUTPUT);
  
 //Comenzamos con el motor parado 
  digitalWrite(AIN1,HIGH);
  digitalWrite(AIN2,HIGH);
  
  //Configuramos el temporizador para saltar la rutina de servicio cada 10ms
  Timer1.initialize(10000); // Ts=0.01 (10000 microsegundos) 
  // cada 0.01 seg se salta a la rutina de servicio: “control”
  Timer1.attachInterrupt(control);
  Timer1.start(); // inicio de las interrupciones
  
}

//Lectura de pulsos del canal A del encoder del motor en cada transición 
void Encoder_CHA(){
  if(digitalRead(CHA)==digitalRead(CHB)){ 
    pulsos = pulsos-1;
  }
  else{ 
    pulsos = pulsos+1;
  } 
}

//Lectura de pulsos del canal B del encoder del motor en cada transición
void Encoder_CHB(){
  if (digitalRead(CHA) != digitalRead(CHB)){
     pulsos = pulsos-1;
  }
  else{
    pulsos = pulsos+1;
  }
}


void control(){

  // Controlador PID
  angulo = float(pulsos*(2*3.1416/1040 )); // conversión de pulsos a radianes 
  
  En = consigna - angulo; // calculo error para la muestra actual
  Pn = Pn1 + (q0*En + q1*En1 + q2*En2); // Salida PIDx
  
  En2 = En1;
  En1 = En;
  Pn1 = Pn;
  
  // Acotar la salida Pn entre ‐8.5 y +8.5 voltios
  if(Pn > 8.5) Pn = 8.5;
  else if (Pn < -8.5) Pn = -8.5;
  
  Serial.println(angulo);
  
  if (Pn>0){
    valorPWM8 = byte(Pn*30); // PWM está comprendido entre 0 y 255
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWM_Motor, valorPWM8);
    
  } else {
    valorPWM8 = byte(-1*Pn*30); // Volt negativo: 
    digitalWrite(AIN1, HIGH); // se cambia el sentido de giro 
    digitalWrite(AIN2, LOW);
    analogWrite(PWM_Motor, valorPWM8);
  }
  
}

void loop() {
  // put your main code here, to run repeatedly:
}
