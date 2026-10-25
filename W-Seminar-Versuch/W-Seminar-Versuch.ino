#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>

TaskHandle_t Task0;
TaskHandle_t Task1;

//Variablen
#define R 0.0275 //Abstand des Sensors zur Achse in m
#define PI 3.14159
#define num_data 4000 //Anzahl der Messungen
#define dt 2 //Zeit zwischen 2 Messungen in ms

int step_count = 0;//Anzahl der tatsächlich abgeschlossenen Motorschritte
int i = 0;
int t_0; //Startzeit des Programms
double data[num_data][5]; //Array der Sensordaten: Zeit, Winkel, Geschwindigkeit, Beschleunigung, aus Zentrifugalkraft berechnete Geschwindigkeit
double a_x_offset = 0; //Offsets: systematische Abweichungen
double a_y_offset = 0; 
double v_offset = 0;

double v_uncertainty;
double v_azp_uncertainty;
double a_uncertainty;

sensors_event_t a, v, temp; //zum Auslesen der Sensordaten
double a_res = 0;

double v_res = 0;

double val[2][1000]; //Beschleunigungswerte

Adafruit_MPU6050 mpu;
Adafruit_NeoPixel rgb(1, 21, NEO_RGB + NEO_KHZ800);

//Funktionen
int sgn(double num) {
  if(num > 0) return 1;
  else if(num < 0) return -1;
  else return 0;
}

void RGB(int red, int green, int blue) {
  rgb.setPixelColor(0, rgb.Color(red, green, blue));
  rgb.show();
}

double req_step(int t) { //gibt die benötigten Schritte für eine bestimmte Zeit (Mikrosekunden) zurück
  return cos(1.0*t/2000000*2*PI) * 1500;
  //if(t>4000000) return 0;//nach 4s
  //return sin(1.0*t/8000000*2*PI) * 6000;
  //return t/300; //unbeschleunigt
  //return 1.0*t*t/1000000000/dt*1;//lineare Beschleunigung
  //return 0;
  //double a = 0.5;
  //return a*t*t/pow(10, 10)*4/PI;
}

void step() {//gibt 1 Signal an den Motor -> 1 Schritt
  digitalWrite(3, HIGH);
  delayMicroseconds(1);
  digitalWrite(3, LOW);
  step_count++;
}

void calibrate() { //kalibriert den Sensor durch Schreiben der Offsets
  for(int j = 0; j < 200; j++) {
    double v_ges = sqrt(pow(v.acceleration.x, 2)+pow(v.acceleration.y,2)+pow(v.acceleration.z,2));
    double a_x = a.acceleration.x;
    double a_y = a.acceleration.y;
    a_x_offset += a_x/200;
    a_y_offset += a_y/200;
    v_offset += v_ges/200;
    delay(5);
  }
}

int step_uncertainty_a(int t, double a) {//lineare Beschleunigung mit einer Winkelbeschleunigung von a
  return a*t*t/pow(10, 10)*4/PI;
}

void measureUncertainty_a(double a) {//misst die Unsicherheit des Beschleunigungssensors
  delay(100);
  unsigned long t_a_0 = micros();
  step_count = 0;
  int time = 2; //Messzeit in Sekunden
  int i = 0;
  while(1) { //Messung der Beschleunigungswerte
    if(i >= 1000) break;
    if(step_uncertainty_a(micros()-t_a_0, a) > step_count) {
      digitalWrite(2, LOW);//im Uhrzeigersinn
      step();
      delayMicroseconds(178);//minimale Verzögerung zwischen 2 Schritten
    }
    if(i < 1000 && 1.0*micros()-t_a_0 >= (i+1+100)*time*1000) {//messen 
      val[0][i] = -a_res/R;
      i++;
    }
  }
  double a_quadsum = 0;
  double a_mid = 0;
  for(int j = 0; j < 1000; j++) {
    a_mid += val[0][j]/1000;
  }
  for(int j = 0; j < 1000; j++) {
    a_quadsum += pow((val[0][j]-a_mid), 2);
  }
  a_uncertainty = sqrt(a_quadsum/1000/(1000-1));
  Serial.print("a: ");
  Serial.print(a);
  if(a<10) Serial.print(" ");//für die Formatierung
  Serial.print("   midValue: ");
  Serial.print(a_mid);
  if(a_mid>0&&a_mid<10) Serial.print(" ");
  Serial.print("   uncertainty: ");
  Serial.print(a_uncertainty);
  if(a_uncertainty>0) Serial.print(" ");
  Serial.print("      dif: ");
  Serial.print(a_mid-a);
  if(a_mid-a>0&&a_mid-a<10) Serial.print(" ");
  Serial.print("   fact_dif: ");
  Serial.print((a_mid/a-1)*100);
  Serial.println("%");
}

int step_uncertainty_v(int t, double v) {//unbeschleunigt mit einer Winkelgeschwindigkeit von v
  return v*t/pow(10, 6)*100*8/PI;
}

void measureUncertainty_v(double v) {//misst die Unsicherheit des Winkelgeschwindigkeitssensors
  delay(100);
  unsigned long t_v_0 = micros();
  step_count = 0;
  int time = 2; //Messzeit in Sekunden
  int i = 0;
  while(1) { //Messung der Beschleunigungswerte
    if(i >= 1000) break;
    if(step_uncertainty_v(micros()-t_v_0, v) > step_count) {
      digitalWrite(2, LOW);//im Uhrzeigersinn
      step();
      delayMicroseconds(178);//minimale Zeit zwischen 2 Schritten
    }
    if(i < 1000 && 1.0*micros()-t_v_0 >= (i+1+100)*time*1000) {//messen
      val[0][i] = v_res*1.0055;
      val[1][i] = sqrt(abs((a.acceleration.y-a_y_offset))/R);
      i++;
    }
  }
  double v_quadsum = 0;
  double v_azp_quadsum = 0;
  double v_mid = 0;
  double v_azp_mid = 0;
  for(int j = 0; j < 1000; j++) {
    v_mid += val[0][j]/1000;
    v_azp_mid += val[1][j]/1000;
  }
  for(int j = 0; j < 1000; j++) {
    v_quadsum += pow((val[0][j]-v_mid), 2);
    v_azp_quadsum += pow((val[1][j]-v_azp_mid), 2);
  }
  v_uncertainty = sqrt(v_quadsum/1000/(1000-1));
  v_azp_uncertainty = sqrt(v_azp_quadsum/1000/(1000-1));
  Serial.print("v: ");
  Serial.print(v);
  if(v<10) Serial.print(" ");//für die Formatierung
  Serial.print("   midValue: ");
  Serial.print(v_mid);
  if(v_mid>0) Serial.print(" ");
  if(v_mid<10.00) Serial.print(" ");
  Serial.print("  uncertainty: ");
  Serial.print(v_uncertainty);
  if(v_uncertainty>0) Serial.print(" ");
  Serial.print("      dif: ");
  Serial.print(v_mid-v);
  if(v_mid-v>0) Serial.print(" ");
  Serial.print("   fact_dif: ");
  Serial.print((v_mid/v-1)*100);
  Serial.print("%");
  if(abs((v_mid/v-1)*100)<10) Serial.print(" ");
  if(v==0) Serial.print("   ");
  Serial.print("                ZF: midValue: ");
  Serial.print(v_azp_mid);
  if(v_azp_mid>0) Serial.print(" ");
  if(v_azp_mid<10.00) Serial.print(" ");
  Serial.print("  uncertainty: ");
  Serial.print(v_azp_uncertainty);
  if(v_azp_uncertainty>0) Serial.print(" ");
  Serial.print("      dif: ");
  Serial.print(v_azp_mid-v);
  if(v_azp_mid-v>0) Serial.print(" ");
  Serial.print("   fact_dif: ");
  Serial.print((v_azp_mid/v-1)*100);
  Serial.println("%");
}

void measue_uncertainty() {//misst die Unsicherheit für verschiedene Beschleunigungen und Geschwindigkeiten
  calibrate();
  Serial.print("a_offset: ");
  Serial.print(a_x_offset);
  Serial.print("   v_offset: ");
  Serial.print(v_offset);
  Serial.print("   azp_offset: ");
  Serial.println(a_y_offset);
  delay(100);
  for(int i = 0; i < 14; i++) {
    measureUncertainty_a(i);
  }
  Serial.println();
  for(int i = 0; i < 21; i++) {
    measureUncertainty_v(i);
  }
  delay(1000000);
}

void error() { //blinkt bei Fehler
  while(1) {
    RGB(10, 0, 0);//rot
    delay(100);
    RGB(0, 0, 0);//aus
    delay(100);
  }
}

void core0(void* pvParameters) { //Kern0: Auslesen der Sensordaten
  while(1) {
    mpu.getEvent(&a, &v, &temp);
    a_res = a.acceleration.x-a_x_offset;
    v_res = -sgn(v.acceleration.y)*abs(sqrt(pow(v.acceleration.x, 2)+pow(v.acceleration.y,2)+pow(v.acceleration.z,2))-v_offset);
  }
}

void core1(void* pvParameters) { //Kern1: Motorsteuerung + loop()
  t_0 = micros();
  while(1) {
    delay(100);
  }
}

void setup(void) {
  Serial.begin(2000000);
  while (!Serial) delay(10); //auf seriellen Monitor warten

  rgb.begin(); // NeoPixel-Objekt initialisieren
  RGB(10, 0, 0);  // rot

  // Initialisierung des I2C-Busses
  if (!Wire.begin(8, 9)) {
    Serial.println("I2C konnte nicht gestartet werden!");
    error();
  }
  if (!mpu.begin()) { // versuchen, den Sensor zu finden
    Serial.println("MPU6050 nicht gefunden");
    error();
  }

  // Einstellungen des MPU6050-Sensors
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  pinMode(1, OUTPUT); //Pin für Ein/Aus
  pinMode(2, OUTPUT); //Pin für Richtungen
  pinMode(3, OUTPUT); //Pin für Signale
  digitalWrite(1, LOW); //ein
  digitalWrite(3, LOW);

  xTaskCreatePinnedToCore(core0, "Task0", 10000, NULL,  1, &Task0, 0);  //Task für Kern0
  measue_uncertainty();
  delay(100);
  //calibrate();
  delay(1000);
  t_0 = micros();//setzt t_0 vor dem Start des Hauptprogramms in loop()
  xTaskCreatePinnedToCore(core1, "Task1", 10000, NULL, 0, &Task1, 1);  //Task für Kern1
  RGB(0, 0, 10);//blau
}

void loop() {
  if(req_step(micros()-t_0) > step_count) { //Wenn Motor theoretisch mehr Schritte haben soll als er bisher hat
    digitalWrite(2, LOW);//im Uhrzeigersinn
    step();
    delayMicroseconds(178);//minimale Verzögerung zwischen 2 Schritten  
  }
  else if (req_step(micros()-t_0) < step_count) { //Wenn Motor theoretisch weniger Schritte haben soll als er bisher hat
    digitalWrite(2, HIGH);//gegen den Uhrzeigersinn
    step();
    step_count -= 2;
    delayMicroseconds(178);//minimale Verzögerung zwischen 2 Schritten  
  }
  if(i < num_data && 1.0*micros()-t_0 >= (i+1)*dt*1000) { //wenn dt seit letzter Messung vergangen ist
    data[i][0] = micros()-t_0;
    double a_y_res = a.acceleration.y-sgn(a.acceleration.y-a_y_offset)*a_y_offset;
    data[i][1] = step_count;
    data[i][2] = v_res;
    data[i][3] = a_res;
    data[i][4] = a.acceleration.y-a_y_offset;
    i++;
  }
  else if(i == num_data) { //Ausgabe der gemessenen Daten in den Serial Monitor mit Umrechnung in die gewünschten Einheiten
    rgb.setPixelColor(0, rgb.Color(0, 10, 0));  // Gruen
    rgb.show();
    Serial.println("Zeit in s;Winkel in rad;Winkelgeschwindigkeit in rad/s;Winkelbeschleunigung in rad/s^2;v aus Zentripetalbeschleunigung"); // ; als Abtrennungszeichen für Tabellenkalkulationsprogramm
    for(int n = 0; n < num_data; n++)
    {
      Serial.print(data[n][0]/1000000, 6);//Umrechnung ns in s
      Serial.print(";");
      Serial.print(data[n][1]/200/8*2*PI, 6);//Umrechnung der Anzahl der Schritte in rad
      Serial.print(";");
      Serial.print(data[n][2]*1.0055, 9);//Umrechnung Winkelgeschwindigkeit
      Serial.print(";");
      Serial.print(-data[n][3]/R, 9);//Umrechnung von m/s² zu rad/s²
      Serial.print(";");
      Serial.println(sqrt(abs(data[n][4])/R), 9);
    }
    i++;
  }
  else if(i > num_data) {
    delay(100);
    digitalWrite(1, HIGH);//Motor ausschalten
  }
  else {
    delayMicroseconds(1);
  }
}
