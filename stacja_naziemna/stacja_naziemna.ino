#include <Arduino.h>

bool telem = true;
String income_data = "";
String param[] = {"ch1: ", "ch2: ", "ch3: ", "ch4: ", "roll: ", "pitch: ", "yaw: ", "altitude: ", "bat_volt: "};

void setup()
{
  // Otwarcie portu do komunikacji
  Serial.begin(38400);
  Serial2.begin(38400);
  pinMode(13, OUTPUT);
  digitalWrite(13, telem);
}

void loop()
{
  decode_telemetry();
}


void decode_telemetry() {

  int i = 0;
  if (Serial2.available() > 0) {
    Serial.print(param[i]);
    i++;
    while (Serial2.available() > 0) {
      char temp = Serial2.read();
      if (temp == ';') {
        Serial.print(param[i]);
        i++;
      }
      else
        Serial.write(temp);
    }
    telem = !telem;
  }
  digitalWrite(13, telem);
}
