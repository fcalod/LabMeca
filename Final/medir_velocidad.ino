//***************************************************//
//***************************************************//
//*****   Posición y Velocidad de Motor DC      *****//
//*****                                         *****//
//***** by: Sergio Andres Castaño Giraldo       *****//
//***** https://controlautomaticoeducacion.com/ *****//
//*****                                         *****//
//***************************************************//
//***************************************************//

// this library includes the ATOMIC_BLOCK macro.
#include <util/atomic.h>

#define ENCODER_A       2 // Amarillo
#define ENCODER_B       3 // Verde


// Pines de Control Shield
const int E1Pin = 5;

//Variable global de posición compartida con la interrupción
volatile int theta = 0;

//Variable global de pulsos compartida con la interrupción
volatile int pulsos = 0;
unsigned long timeold;
float resolution = 374.22;

//Variable Global Velocidad
int vel = 0;

//Variable Global MODO
bool modo = false;

//Estructura del Motor
typedef struct{
  byte enPin;
}Motor;

//Creo el motor
const Motor motor = {E1Pin};

void setup(){
  // set timer 3 divisor to  1024 for PWM frequency of 30.64 Hz
  TCCR3B = TCCR3B & B11111000 | B00000101;
  Serial.begin(9600);
  //Encoders como entradas
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  //Configura Motor
  pinMode(motor.enPin, OUTPUT);
  //Configurar Interrupción
  timeold = 0;
  attachInterrupt(digitalPinToInterrupt(ENCODER_A),leerEncoder,RISING);
}

void loop(){
  float posicion;
  float rpm;
  
  //Transforma el valor del Pot a velocidad
  vel = 150;

  //Activa el motor dirección Forward con la velocidad
  setMotor(motor, vel, false);

  //Espera un segundo para el calculo de las RPM
  if (millis() - timeold >= 1000)
  {
    //Modifica las variables de la interrupción forma atómica
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      //rpm = float(pulsos * 60.0 / 374.22); //RPM
      rpm = float((60.0 * 1000.0 / resolution ) / (millis() - timeold) * pulsos);
      timeold = millis();
      pulsos = 0;
    }
    Serial.print("RPM: ");
    Serial.println(rpm);
    Serial.print("PWM: ");
    Serial.println(vel);
  }
}

//Función para dirección y velocidad del Motor
void setMotor(const Motor motor, int vel){
  analogWrite(motor.enPin, vel);
}

//Función para la lectura del encoder
void leerEncoder(){
  //Lectura de Velocidad
  pulsos++; //Incrementa una revolución
}
