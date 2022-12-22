#include <Arduino.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

bool telem = true;
String income_data = "";
char input;

BluetoothSerial SerialBT;

void setup()
{
  // Otwarcie portu do komunikacji
  Serial.begin(38400);
  Serial2.begin(38400);
  SerialBT.begin("UAV_BT");
  Serial.println("BT uruchomiony");
  pinMode(2, OUTPUT);
  while (1) {
    if (SerialBT.connected()) {
      digitalWrite(2, HIGH);
      break;
    }
  }
}

void loop()
{
  decode_telemetry();
  check_data_from_app();
}


void decode_telemetry() {
  income_data = "";
  if (Serial2.available() > 0) {
    while (Serial2.available() > 0) {
      char temp = Serial2.read();
      income_data += temp;
      telem = !telem;
    }
    SerialBT.println(income_data);
    digitalWrite(2, telem);
  }
}

void check_data_from_app() {
  if (SerialBT.available() > 0) {
    input = SerialBT.read();
    Serial2.write(input);
    digitalWrite(2, HIGH);
  }
  digitalWrite(2, LOW);
}
