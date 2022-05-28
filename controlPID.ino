/*
*Controlador PID para un motor de corriente directa
*En el caso de un robot movil diferencial sera necesario
*utilizar uno de estos controladores para cada rueda
*/
#include <util/atomic.h>
#define ENCODER_A       2

const float SET_POINT = 1600; //velocidad a seguir
const int PWM_PIN = 5; //pin de salida pwm
const int T_MSEC = 10;//tiempo de muestreo en milisegundos
const float T_SEC = 0.01;//timepo de muestreo en segundos

//Variable global de posición compartida con la interrupción
volatile int theta = 0;
volatile int pulsos = 0;
unsigned long timeold;
float resolution = 12;
//Variable Global Velocidad
int vel = 0;

struct speed_sensor {
    //Estructura que representa el sensor de velocidad
  unsigned long last_t;
  float rpm;
};

struct pid_controller {
    //Estructura que representa las variables del controlador PID
  float kp, ki, kd;//Constantes de proporcionalidad, integral y derivativa
  float delta; //Tiempo de muestreo (T)
  float sum; //suma del error para la integral numerica
  float prevError;//error previo
};

struct speed_sensor motor_speed_sensor;
struct pid_controller motor_pid_controller;
unsigned long last_t;
unsigned char n;

void speed_sensor_init(struct speed_sensor &ss) {
    //Esta función inicia el sensor de velocidad en 0
  ss.last_t = 0;
  ss.rpm = 0;
}

void speed_sensor_turn(struct speed_sensor &ss) {
    //Esta funcion se activara con la interrupcion
    //Nos permite medir la velocidad en rpm's
  unsigned long t = micros();
  unsigned long inc = t - ss.last_t;
  float tempRpm = (1.0 / inc) * 60000000;
  if (tempRpm < 8000) {
    ss.rpm = tempRpm;
    ss.last_t = t;
  }
}

void pid_controller_init(struct pid_controller &pid, float delta, float kp, float ki, float kd) {
    //inicializa las variables y constantes del control PID con valores dados
  pid.delta = delta;
  pid.kp = kp;
  pid.ki = ki;
  pid.kd = kd;
  pid.sum = 0;
  pid.prevError = 0;
}

float pid_controller_run(struct pid_controller &pid, float error) {
    //Calcula la ecuacion del contrl PID
  float p = pid.kp * error; //accion proporsional
  pid.sum += error; //sumamos el error
  float i = pid.ki * pid.delta * pid.sum;//accion integral
  float d = pid.kd * (error - pid.prevError) / pid.delta;//accion derivativa
  pid.prevError = error;//el error anterior es ahora el error actual
  return p + i + d;//PID
}

void rotorTurn() {
    //Esta funcion es llamanda por la interrupcion
    //invoca la medicion de velocidad
  speed_sensor_turn(motor_speed_sensor);
}

//Función para la lectura del encoder
void leerEncoder(){
  //Lectura de Velocidad
  pulsos++; //Incrementa una revolución
}

void setup() {
  speed_sensor_init(motor_speed_sensor);//inicializamos el sensor del motor
  pid_controller_init(motor_pid_controller, T_SEC, 0.5, 0.5, 0.5);//inicializamos el control PID
  //con valores de ganancia de Kp=Ki=Kd=0.5, este valor lo elegimos de acurdo a un articulo en internet
  //que realizo pruebas experimentales
  Serial.begin(9600);
  pinMode(ENCODER_A, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), leerEncoder, RISING); //interrupcion, llama a la funcion rotorTurn 
  //cuando en el pin 0 se detecta un flanco de subida
  //En el pin 0 hay que concectar el encoder
  last_t = 0;//iniciamos en el tiempo 0
  n = 0;
}

void loop() {
  float rpm;
  unsigned long t = millis();//obtiene el tiempo de ejecución en milisegundos
  if ((t - last_t) >= T_MSEC) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        rpm = float((60.0 * 1000.0 / resolution ) / (millis() - timeold) * pulsos);
        timeold = millis();
        pulsos = 0;
        motor_speed_sensor.rpm = rpm;
        motor_speed_sensor.last_t = timeold;
      }
      //Entra a este if si ha pasado el tiempo de muestreo
    if (n == 0) {
        //imprime en el puerto serial la velocidad cada 10 mediciones
        Serial.print(motor_speed_sensor.rpm);
        Serial.println(" rpm");
        n = 10;
    }
    else{
        n--;
    }
      
    float error = SET_POINT - motor_speed_sensor.rpm;
    analogWrite(PWM_PIN, (int) pid_controller_run(motor_pid_controller, error));
    last_t = t;
  }
}