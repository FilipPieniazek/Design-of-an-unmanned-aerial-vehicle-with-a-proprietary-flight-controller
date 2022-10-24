#include "PWM.hpp"
#include "Wire.h"
#include <Arduino.h>
#include <LPS.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"

//debugger
bool debugBAR = false;
bool debugIMU = false;
bool debugIN = false;
bool debugOUT = false;
bool dont_calc = true;
bool debugTELEM = false;

//definicja kanalow PWM odbiornika
PWM ch1(2);
PWM ch2(3);
PWM ch3(4);
PWM ch4(5);
PWM ch5(6);
PWM ch6(7);
int _min[6] = {1000,1000,1000,1000,1000,1000};
int _max[6] = {2000,2000,2000,2000,2000,2000};
int out[6] = {8,9,10,11,14,15};
int inp[6]= {0,0,0,0,0,0}; 
int calc[6]= {0,0,0,0,0,0}; 

//inicjalizacja serw
Servo out1, out2, out3, out4, out5, out6;

//inicjalizacja IMU
MPU6050 imu;
int16_t roll, pitch, yaw;

//inicjalizacja barometru
LPS barometr;
float pressure, altitude, temperature;

//telemetria podpieta do Serial3
bool telem = true;

void setup() {
    Wire.begin();
    imu.initialize();
    barometr.init();
    barometr.enableDefault();
    Serial3.begin(38400);
    Serial.begin(115200);
    pinMode(13, OUTPUT);
    digitalWrite(13, telem);
    ch1.begin(true);
    ch2.begin(true);
    ch3.begin(true);
    ch4.begin(true);
    ch5.begin(true);
    ch6.begin(true);
    out1.attach(out[0]);
    out2.attach(out[1]);
    out3.attach(out[2]);
    out4.attach(out[3]);
    out5.attach(out[4]);
    out6.attach(out[5]);
}

void loop() {
 read_sensors();
 rx_values_read(); 
 PID_calculate();
 control();
 tx_telemetry();
}

//wysylanie parametrow telemetrii do stacji naziemnej
void tx_telemetry(){
  Serial3.println("Hello World");
  delay(50);
}

//odczytywanie wartosci z sensorow
void read_sensors(){
  imu.getAcceleration(&roll, &pitch, &yaw);
  if(debugIMU){
    Serial.print(-25000); Serial.print("\t");
    Serial.print(25000); Serial.print("\t");
    Serial.print(pitch); Serial.print("\t");
    Serial.print(roll); Serial.print("\t");
    Serial.print(yaw); Serial.println("\t");
    }
  pressure = barometr.readPressureMillibars();
  altitude = barometr.pressureToAltitudeMeters(pressure);
  temperature = barometr.readTemperatureC();
  if(debugBAR){
    Serial.print("p: ");
    Serial.print(pressure);
    Serial.print(" mbar\ta: ");
    Serial.print(altitude);
    Serial.print(" m\tt: ");
    Serial.print(temperature);
    Serial.println(" deg C");
  }
}


//odczytywanie wartosci PWM z odbiornika RC
void rx_values_read(){
  inp[0] = map(ch1.getValue(), _min[0], _max[0], 0, 180);
  inp[1] = map(ch2.getValue(), _min[1], _max[1], 0, 180);
  inp[2] = map(ch3.getValue(), _min[2], _max[2], 0, 180);
  inp[3] = map(ch4.getValue(), _min[3], _max[3], 0, 180);
  inp[4] = map(ch5.getValue(), _min[4], _max[4], 0, 180);
  inp[5] = map(ch6.getValue(), _min[5], _max[5], 0, 180);
  if(debugIN){
    for(int i=0; i<6; i++){
      Serial.print(inp[i]);
      Serial.print(";");
    }
    Serial.println();
  }
  delay(5);
}

//obliczenie korekty PID
void PID_calculate(){
  if(dont_calc){
    for(int i=0; i<6; i++)
      calc[i]=inp[i];
  }
}


//wysylanie sterowania na silnik i serwa
void control(){
  out1.write(calc[0]);
  out2.write(calc[1]);
  out3.write(calc[2]);
  out4.write(calc[3]);
  out5.write(calc[4]);
  out6.write(calc[5]);
  if(debugOUT){
    for(int i=0; i<6; i++){
      Serial.print(inp[i]);
      Serial.print(";");
    }
    Serial.println();
  }
  delay(5);
}
