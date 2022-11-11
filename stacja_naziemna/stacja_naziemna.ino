#include <Arduino.h>

bool telem = false;
String income_data = "";
char input;

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
  check_serial();
}


void decode_telemetry() {
  income_data = "";
  if (Serial2.available() > 0) {
    while (Serial2.available() > 0) {
      char temp = Serial2.read();
      income_data += temp;
      telem = !telem;
    }
    Serial.println(income_data);
    digitalWrite(13, telem);
  }
}

void check_serial(){
  if(Serial.available() > 0) {
    input = Serial.read();
    Serial2.write(input);
    digitalWrite(13,HIGH);
  }
  digitalWrite(13,LOW);
}
