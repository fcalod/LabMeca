#include <Servo.h>

Servo servo;
int pinServo = 5;

int pinTrig = A13;
int pinEcho = A14;

const int minAng = 0;
const int maxAng = 150;
const int pasoAng = 2; // Cambio en el ángulo
const int n = (maxAng - minAng) / pasoAng + 1; // Número de ángulos
const int centro = n/2;

// Mapea los ángulos del servo a las distancias medidas por el ultrasónico
double distancias[n]; // Empieza en minAng, termina en maxAng
double vectores[n*3]; // Lista de Vector[indice, distancia, antVacio]
int numVectores;
double direccion[2]; // [angulo, direccion]

// Medidas del Robot
double tr = 13; //cm; Tamaño del robot (diametro)
double tr2 = 169; // Tamaño del robot al cuadrado

void setup() {
	Serial.begin(9600);

	// Ultrasónico
	pinMode(pinTrig, OUTPUT);
	pinMode(pinEcho, INPUT);
	
	// Servo
  servo.write(maxAng); // Evita que el servo se mueva durante la inicialización
  servo.attach(pinServo);
}

void loop() {
	scan();
  delay(10000);
  obtenerDireccion();
  delay(1000000);
}

void scan() {
	double dist;
	int servoAng = maxAng;
	
	for(int i = 0; i < n; i++) {
		servo.write(servoAng);
		dist = ultrasonico(pinTrig, pinEcho);
		distancias[i] = dist;
		servoAng -= pasoAng;
    Serial.print(i);
    Serial.print(" : ");
    Serial.println(dist);
    delay(100);
	}
	
	// Resetea el servo
  servo.write(maxAng);
}

float fastacos(float x) {
  float negate = float(x < 0);
  x = abs(x);
  float ret = -0.0187293;
  ret = ret * x;
  ret = ret + 0.0742610;
  ret = ret * x;
  ret = ret - 0.2121144;
  ret = ret * x;
  ret = ret + 1.5707288;
  ret = ret * sqrt(1.0-x);
  ret = ret - 2 * negate * ret;
  return negate * 3.14159265358979 + ret;
}

void obtenerDireccion(){
  Serial.println("Obtener Direccion");
  optimizarRadar();
  Serial.println("Optimizar Radar");
  obtenerObstaculos();
  Serial.println("Obtener Obstaculos");

  double indice;
  double distancia;
  double antvacio;
  bool band = false;

  double ang1;
  double ang2;
  double dist1;
  double dist2;
  
  double dirAng = -90;
  double dirDist;

  int i = 0;
  while (i < numVectores){
    Serial.print("Dirección: ");
    Serial.println(dirAng);
    Serial.print("Distancia: ");
    Serial.println(dirDist);
    band = false;
    double angActual = -90;
    indice = vectores[i];
    distancia = vectores[i+1];
    antvacio = vectores[i+2];
    Serial.print("Indice: ");
    Serial.println(indice);
    Serial.print("Distancia: ");
    Serial.println(distancia);
    Serial.print("Anterior Vacio: ");
    Serial.println(antvacio);

    if (antvacio == 1){
      if (i == 0){
        if (cabeRobot(0,distancia,indice,distancia)){
          ang1 = obtenerAngulo(0);
          ang2 = obtenerAngulo(indice);
          dist1 = distancia;
          dist2 = distancia;    
          angActual = anguloDir(dist1,ang2);
        }
      }else if(band){
        double iAnt = vectores[i-3];
        double dAnt = vectores[i-2];
        if (cabeRobot(iAnt, dAnt,indice,distancia)){
            ang1 = obtenerAngulo(iAnt);
            ang2 = obtenerAngulo(indice);
            dist1 = dAnt;
            dist2 = distancia;
          if(abs(centro-iAnt) < abs(centro-indice)){
            angActual = anguloDir(dist1,ang1);
          } else {
            angActual = anguloDir(dist2,ang2);
          }
        }
        band = false;
      } else {
        Serial.println("ERROR");
      }
    }else{
      band = true;
    }

    if(abs(angActual) < abs(dirAng)){
        dirAng = angActual;
        dirDist = distDir(dist1, dist2);
    }

    i = i + 3;
  }

  direccion[0] = dirAng;
  direccion[1] = dirDist;
  Serial.println("Dirección: ");
  Serial.println(dirAng);
  Serial.println("Distancia: ");
  Serial.println(dirDist);
}

double distDir(double dist1, double dist2){
  return (dist1 + dist2) / 2;
}

double anguloDir(double x, double theta){
  float cos = (2*(x*x))/(2*x*13)/2;
  float ang;
  if (theta >= 0){
    ang = fastacos(cos) + theta;
  }else{
    ang = (fastacos(cos) + (theta)*-1)*-1;
  }
  return ang;
}

bool cabeRobot(int i1, double d1, int i2, double d2){
    int difAng = abs(i1 - i2);
    double ang = difAng * pasoAng;
    bool cabe = false;
    double dist = (d1*d1) + (d2*d2) - (2*d1*d2*cos(ang));
    if(tr2 < dist){
      cabe = true;
    }
}

double obtenerAngulo(int indice){
  double dif = indice - centro;
  double theta = dif * pasoAng;
  return theta;
}

void optimizarRadar(){
  double min = 100;
  double max = 0;
  for(int i = 0; i < n; i++) {
    double distancia = distancias[i];
		if (distancia > max){
      max = distancia;
    }
    if (distancia < min){
      min = distancia;
    }
	}
  double media = (min + max)/2;

  for(int i = 0; i < n; i++) {
    double distancia = distancias[i];
		if (distancia > media){
      distancias[i] = 10000;
    }
	}
}

void obtenerObstaculos(){
  int indice = 0;
  int ant = 1;
  bool vacio = false;
  for(int i = 1; i < n; i++) {
    Serial.println("Revisando pos");
    Serial.println(i);
    ant = i-1;
    double anterior = distancias[ant]; 
    double pos = distancias[i];
    if (!vacio && (pos == 10000 || pos>(anterior+10))){
      vectores[indice + 0] = ant;
      vectores[indice + 1] = anterior;
      vectores[indice + 2] = 0;
      indice = indice + 3;
      vacio = true;
    }else if(vacio && (anterior == 10000 || pos<(anterior+10))){
      vectores[indice + 0] = i;
      vectores[indice + 1] = pos;
      vectores[indice + 2] = 1;
      indice = indice + 3;
      vacio = false;
    }
	}
  numVectores = indice;
  Serial.print("numvectores: ");
  Serial.print(numVectores);
}

double ultrasonico(int trigPin, int echoPin) {
	digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
	digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
	digitalWrite(trigPin, LOW);
	
	return pulseIn(echoPin, HIGH) * 0.034 / 2.0; // Distancia en cm
}
