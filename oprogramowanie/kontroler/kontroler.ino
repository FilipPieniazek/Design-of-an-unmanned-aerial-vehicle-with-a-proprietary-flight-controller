//biblioteki
#include "PWM.hpp"
#include "Wire.h"
#include <Arduino.h>
#include <LPS.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "SparkFun_Ublox_Arduino_Library.h"
#include <PID_v1.h>

//debugger
bool debugBAR = false;
bool debugIMU = false;
bool debugIN = false;
bool debugOUT = false;
bool debugGPS = false;
bool debugPID = false;

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
int last_fly_mode = 0;
int buzzer = 22;
int rx_indicator = 23;
bool transmit = true;
bool arm = true;
long lastTime = 0;
String income_data = "";
String option = "";
String parameter = "";

//inicjalizacja IMU
MPU6050 imu;
int16_t roll, pitch, yaw;

//inicjalizacja algorytmu PID
double pitch_db, kp_pitch = 1, ki_pitch = 0, kd_pitch = 0;
double roll_db, kp_roll = 1, ki_roll = 0, kd_roll = 0;
PID pid_pitch(&pitch_db, &calc[1], &inp[1], kp_pitch, ki_pitch, kd_pitch, DIRECT);
PID pid_roll(&roll_db, &calc[0], &inp[0], kp_roll, ki_roll, kd_roll, DIRECT);

//inicjalizacja serw
Servo out1, out2, out3, out4, out5, out6;

//inicjalizacja barometru
LPS barometr;
float pressure, altitude, temperature, altitude_offset;

//inicjalizacja GPSa
SFE_UBLOX_GPS gps;
long latitude = 0;
long longitude = 0;
long altitude_gps = 0;

//telemetria podpieta do Serial3
bool telem = true;

void setup() {
  delay(3000);
  Wire.begin();
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
  pinMode(buzzer, OUTPUT);
  pinMode(rx_indicator, INPUT);
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
  pid_pitch.SetMode(AUTOMATIC);
  pid_roll.SetMode(AUTOMATIC);
  pid_pitch.SetTunings(kp_pitch, ki_pitch, kd_pitch);
  pid_roll.SetTunings(kp_roll, ki_roll, kd_roll);
  calibrate_sensors();
}

//glowna petla programu
void loop() {
  for (int k = 0; k < 3; k++) {
    for (int j = 0; j < 5; j++) {
      for (int i = 0; i < 20; i++) {
        read_sensors();
        rx_values_read();
        PID_calculate();
        control();
        //check_range();
      }
      if (transmit)
        tx_telemetry();
      else
        rx_telemetry();
    }
    GPS();
  }
  digitalWrite(buzzer, LOW);
}

//odczytywanie wartosci PWM z odbiornika RC
void rx_values_read() {
  inp[0] = map(ch1.getValue(), _min[0], _max[0], 0, 180);
  inp[1] = map(ch2.getValue(), _min[1], _max[1], 0, 180);
  inp[2] = ch3.getValue();
  inp[3] = map(ch4.getValue(), _min[3], _max[3], 0, 180);
  inp[4] = map(ch5.getValue(), _min[4], _max[4], 0, 180);
  inp[5] = ch6.getValue();
  if (inp[5] < 1200) {
    if (last_fly_mode > 0)
      digitalWrite(buzzer, HIGH);
    fly_mode = 0;
    last_fly_mode = 0;
    transmit = true;
  }
  else if ((inp[5] >= 1200) && (inp[5] < 1400)) {
    fly_mode = 0;
    transmit = false;
  }
  else if ((inp[5] >= 1400) && (inp[5] < 1800)) {
    if (last_fly_mode != 1)
      digitalWrite(buzzer, HIGH);
    fly_mode = 1;
    last_fly_mode = 1;
    transmit = true;
  }
  else {
    if (last_fly_mode != 2)
      digitalWrite(buzzer, HIGH);
    fly_mode = 2;
    last_fly_mode = 2;
    transmit = true;
  }
}

//obliczenie korekty PID
void PID_calculate() {
  //offset imu
  roll_db -= 90;
  pitch_db -= 90;
  //manual fly-mode
  if (fly_mode == 0) {
    for (int i = 0; i < 5; i++)
      calc[i] = inp[i];
  }
  //stabilize fly-mode
  if (fly_mode == 1) {
    pid_roll.Compute();
    pid_pitch.Compute();
    calc[2] = inp[2];
    calc[3] = inp[3];
    calc[4] = calc[0];
  }
  //loiter fly-mode
  if (fly_mode == 2) {
    inp[1] = 90;
    pid_roll.Compute();
    pid_pitch.Compute();
    calc[2] = inp[2];
    calc[3] = inp[3];
    calc[4] = calc[0];
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
}


////////////////////////////////////////////////////////////////////////////czujniki////////////////////////////////////////////////////////////////////////////

//sprawdzenie czy aparatura ma polaczenie z samolotem
void check_range() {
  float rx_voltage = analogRead(rx_indicator);
  if (rx_voltage > 800)
    digitalWrite(buzzer, HIGH);
  else
    digitalWrite(buzzer, LOW);
}

//odczyt danych z GPSu
void GPS() {
  if (millis() - lastTime > 1000)
  {
    lastTime = millis();
    latitude = gps.getLatitude();
    longitude = gps.getLongitude();
    altitude_gps = gps.getAltitude();
  }
}

//startowa kalibracja czujnikow
void calibrate_sensors() {
  pressure = barometr.readPressureMillibars();
  altitude_offset = barometr.pressureToAltitudeMeters(pressure);
}

//odczytywanie wartosci z sensorow
void read_sensors() {
  imu.getAcceleration(&roll, &pitch, &yaw);
  //ograniczenie zakresu wartosci imu
  roll = constrain(roll, min_imu, max_imu);
  pitch = constrain(pitch, min_imu, max_imu);
  yaw = constrain(yaw, min_imu, max_imu);
  roll = map(roll, min_imu, max_imu, 0, 180);
  pitch = map(pitch, min_imu, max_imu, 180, 0);
  yaw = map(yaw, min_imu, max_imu, 0, 180);
  pitch_db = (double)pitch;
  roll_db = (double)roll;
  pressure = barometr.readPressureMillibars();
  altitude = barometr.pressureToAltitudeMeters(pressure) - altitude_offset;
  temperature = barometr.readTemperatureC();
}


////////////////////////////////////////////////////////////////////////////telemetria////////////////////////////////////////////////////////////////////////////


//odczytywanie parametrow telemetrii ze stacji naziemnej
void rx_telemetry() {
  income_data = "";
  if (Serial3.available() > 0) {
    while (Serial3.available() > 0) {
      char temp = Serial3.read();
      income_data += temp;
    }
  }
  decode_telemetry();
}

//dekodowanie telemetrii i zmiana parametrow
void decode_telemetry() {
  option = "";
  parameter = "";
  if (income_data.length() > 0) {
    for (int i = 0; i < 3; i++)
      option += income_data[i];
    for (int i = 3; i < income_data.length(); i++)
      parameter += income_data[i];
    double data = parameter.toFloat();
    if (option == "kpp")
      kp_pitch = data;
    else if (option == "kip")
      ki_pitch = data;
    else if (option == "kdp")
      kd_pitch = data;
    else if (option == "kpr")
      kp_roll = data;
    else if (option == "kir")
      ki_roll = data;
    else if (option == "kdr")
      kd_roll = data;
    else if (option == "tel") {
      if (data == 1)
        debugBAR = true;
      else if (data == 2)
        debugBAR = false;
      else if (data == 3)
        debugIMU = true;
      else if (data == 4)
        debugIMU = false;
      else if (data == 5)
        debugIN = true;
      else if (data == 6)
        debugIN = false;
      else if (data == 7)
        debugOUT = true;
      else if (data == 8)
        debugOUT = false;
      else if (data == 9)
        debugGPS = true;
      else if (data == 10)
        debugGPS = false;
      else if (data == 11)
        debugPID = true;
      else if (data == 12)
        debugPID = false;
    }
    send_ack();
  }
}

//wysylanie parametrow telemetrii do stacji naziemnej
void tx_telemetry() {
  if (debugOUT)
    Serial3.println(send_output());
  if (debugIN)
    Serial3.println(send_input());
  if (debugGPS)
    Serial3.println(send_gps());
  if (debugIMU)
    Serial3.println(send_imu());
  if (debugPID)
    Serial3.println(send_pid());
  if (debugBAR)
    Serial3.println(send_bar());
  digitalWrite(13, telem);
  telem = !telem;
}

//wysylanie sygnalow wejsciowych
String send_input() {
  String telem_in = "/";
  telem_in += String(inp[0], 0) + "/";
  telem_in += String(inp[1], 0) + "/";
  telem_in += String(inp[2], 0) + "/";
  telem_in += String(inp[3], 0);
  return telem_in;
}

//wysylanie sygnalow wyjsciowych
String send_output() {
  String telem_out = "/";
  telem_out += String(calc[0], 0) + "/";
  telem_out += String(calc[1], 0) + "/";
  telem_out += String(calc[2], 0) + "/";
  telem_out += String(calc[3], 0);
  return telem_out;
}

//wysylanie pozycji samolotu
String send_imu() {
  String telem_imu = ":";
  telem_imu += String(roll) + ":";
  telem_imu += String(pitch) + ":";
  telem_imu += String(yaw) + ":";
  return telem_imu;
}

//wysylanie lokalizacji GPS
String send_gps() {
  String telem_gps = "_";
  telem_gps += String(latitude) + "_";
  telem_gps += String(longitude) + "_";
  telem_gps += String(altitude_gps);
  return telem_gps;
}

//wysylanie wspolczynnikow PID
String send_pid() {
  String telem_pid = ";";
  telem_pid += String(kp_pitch) + ";";
  telem_pid += String(ki_pitch) + ";";
  telem_pid += String(kd_pitch) + ";";
  telem_pid += String(kp_roll) + ";";
  telem_pid += String(ki_roll) + ";";
  telem_pid += String(kd_roll);
  return telem_pid;
}

//wysylanie danych z barometru
String send_bar() {
  String telem_bar = "-";
  telem_bar += String(altitude) + " ";
  telem_bar += String(temperature);
  return telem_bar;
}

//wysylanie potwierdzenia zmiany parametrow
void send_ack() {
  Serial3.println("ACK");
}
