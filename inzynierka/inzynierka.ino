#include "PWM.hpp"
#include "Wire.h"
#include <Arduino.h>
#include <LPS.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "SparkFun_Ublox_Arduino_Library.h"
#include <MicroNMEA.h>
#include <PID_v1.h>


//debugger
bool debugBAR = false;
bool debugIMU = false;
bool debugIN = false;
bool debugOUT = false;
bool debugGPS = false;
bool debugTELEM = false;

//definicja kanalow PWM odbiornika
PWM ch1(12);
PWM ch2(11);
PWM ch3(10);
PWM ch4(9);
PWM ch5(8);
PWM ch6(7);

//zakres wartosci PWM kanalow(_min,_max)
int _min[5] = {1000, 1000, 1000, 1000, 1000};
int _max[5] = {2000, 2000, 2000, 2000, 2000};
int min_imu = -16384;
int max_imu = 16384;
int out[5] = {2, 3, 4, 5, 6};
double inp[6] = {0, 0, 0, 0, 0, 0};
double calc[5] = {0, 0, 0, 0, 0};

//definicja niezbednych zmiennych
int fly_mode = 0;//0-manual, 1-stabilize, 2-loiter
bool arm = false;
float battery_voltage = 0;
float r1 = 10000;
float r2 = 100000;

//inicjalizacja IMU
MPU6050 imu;
int16_t roll, pitch, yaw;

//inicjalizacja algorytmu PID
double pitch_db, kp_pitch = 1, ki_pitch = 0.05, kd_pitch = 0.25;
double roll_db, kp_roll = 1, ki_roll = 0.05, kd_roll = 0.25;
PID pid_pitch(&pitch_db, &calc[1], &inp[1], kp_pitch, ki_pitch, kd_pitch, DIRECT);
PID pid_roll(&roll_db, &calc[3], &inp[3], kp_roll, ki_roll, kd_roll, DIRECT);

//inicjalizacja serw
Servo out1, out2, out3, out4, out5, out6;

//inicjalizacja barometru
LPS barometr;
float pressure, altitude, temperature, altitude_offset;

//inicjalizacja GPSa
SFE_UBLOX_GPS gps;
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));
long latitude_mdeg = 0;
long longitude_mdeg = 0;

//telemetria podpieta do Serial3
bool telem = true;

void setup() {
  Wire.begin();
  delay(3000);
  if (gps.begin() == false)
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }
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
  pid_pitch.SetMode(AUTOMATIC);
  pid_roll.SetMode(AUTOMATIC);
  pid_pitch.SetTunings(kp_pitch, ki_pitch, kd_pitch);
  pid_roll.SetTunings(kp_roll, ki_roll, kd_roll);
  calibrate_sensors();
}

void loop() {
  read_sensors();
  rx_values_read();
  PID_calculate();
  control();
  tx_telemetry();
}

//wysylanie parametrow telemetrii do stacji naziemnej
void tx_telemetry() {
  Serial3.println(code_telemetry());
  digitalWrite(13, telem);
  telem != telem;
  delay(50);
}

//kodowanie parametrow telemetrii
String code_telemetry() {
  String data = "";
  return data;
}

//startowa kalibracja czujnikow
void calibrate_sensors() {
  pressure = barometr.readPressureMillibars();
  altitude_offset = barometr.pressureToAltitudeMeters(pressure);
}

//odczytywanie wartosci z sensorow
void read_sensors() {
  imu.getAcceleration(&roll, &pitch, &yaw);
  //ograniczenie zakrsu wartosci imu
  roll = constrain(roll, min_imu, max_imu);
  pitch = constrain(pitch, min_imu, max_imu);
  yaw = constrain(yaw, min_imu, max_imu);
  roll = map(roll, min_imu, max_imu, 0, 180);
  pitch = map(pitch, min_imu, max_imu, 0, 180);
  yaw = map(yaw, min_imu, max_imu, 0, 180);
  if (debugIMU) {
    Serial.print(0); Serial.print("\t");
    Serial.print(180); Serial.print("\t");
    Serial.print(pitch); Serial.print("\t");
    Serial.print(roll); Serial.print("\t");
    Serial.print(yaw); Serial.println("\t");
  }
  pressure = barometr.readPressureMillibars();
  altitude = barometr.pressureToAltitudeMeters(pressure) - altitude_offset;
  temperature = barometr.readTemperatureC();
  if (debugBAR) {
    Serial.print("p: ");
    Serial.print(pressure);
    Serial.print(" mbar\ta: ");
    Serial.print(altitude);
    Serial.print(" m\tt: ");
    Serial.print(temperature);
    Serial.println(" deg C");
  }
  measure_voltage();
  GPS();
}

//measure battery voltage
void measure_voltage() {
  battery_voltage = ((analogRead(A0) * 5) / 1024) / (r2 / (r1 + r2));
}

//odczytywanie wartosci PWM z odbiornika RC
void rx_values_read() {
  inp[0] = map(ch1.getValue(), _min[0], _max[0], 0, 180);
  inp[1] = map(ch2.getValue(), _min[1], _max[1], 0, 180);
  inp[2] = map(ch3.getValue(), _min[2], _max[2], 0, 180);
  inp[3] = map(ch4.getValue(), _min[3], _max[3], 0, 180);
  inp[4] = map(ch5.getValue(), _min[4], _max[4], 0, 180);
  inp[5] = ch6.getValue();
  if (inp[5] < 1200)
    fly_mode = 0;
  else if (inp[5] < 1800)
    fly_mode = 1;
  else
    fly_mode = 2;
  if (inp[4] > 1500)
    arm = true;
  else
    arm = false;
  if (debugIN) {
    for (int i = 0; i < 6; i++) {
      Serial.print(inp[i]);
      Serial.print(";");
    }
    Serial.println();
    Serial.println(fly_mode);
  }
  delay(5);
}

//obliczenie korekty PID
void PID_calculate() {
  //manual fly-mode
  if (fly_mode == 0) {
    for (int i = 0; i < 5; i++)
      calc[i] = inp[i];
  }
  //stabilize fly-mode
  if (fly_mode == 1) {
    pid_pitch.Compute();
    pid_roll.Compute();
  }
  //loiter fly-mode
  if (fly_mode == 2) {

  }
  if (!arm)
    calc[2] = 0;
}

//wysylanie sterowania na silnik i serwa
void control() {
  out1.write(calc[0]);
  out2.write(calc[1]);
  out3.write(calc[2]);
  out4.write(calc[3]);
  out5.write(calc[4]);
  if (debugOUT) {
    for (int i = 0; i < 5; i++) {
      Serial.print(inp[i]);
      Serial.print(";");
    }
    Serial.println();
  }
  delay(5);
}

//odczyt danych z GPSu
void GPS() {
  gps.checkUblox();

  if (nmea.isValid() == true)
  {
    latitude_mdeg = nmea.getLatitude();
    longitude_mdeg = nmea.getLongitude();
    if (debugGPS) {
      Serial.print("Latitude (deg): ");
      Serial.println(latitude_mdeg / 1000000., 6);
      Serial.print("Longitude (deg): ");
      Serial.println(longitude_mdeg / 1000000., 6);
    }
  }
  else
  {
    if (debugGPS) {
      Serial.print("No Fix - ");
      Serial.print("Num. satellites: ");
      Serial.println(nmea.getNumSatellites());
    }
  }
  delay(200);
}

void SFE_UBLOX_GPS::processNMEA(char incoming)
{
  nmea.process(incoming);
}
