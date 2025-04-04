#include <Wire.h>

const int slaveAddress = 8;
float vector[2] = {0.0, 0.0};
uint8_t flag1 = 0, flag2 = 0;
bool newData = false;

void setup() {
  Serial.begin(115200);
  Wire.begin(slaveAddress);
  Wire.onReceive(receiveEvent);
  Serial.println("Arduino is ready to receive data...");
}

void loop() {
  if (newData) {
    Serial.print("Flags: ");
    Serial.print(flag1);
    Serial.print(", ");
    Serial.print(flag2);
    Serial.print(" | Vector: ");
    Serial.print(vector[0], 2);  // Distance
    Serial.print(", ");
    Serial.println(vector[1], 2);  // Angle

    newData = false;  // Reset flag after printing
  }

  delay(100);  // Slight delay to avoid spamming the serial monitor
}

void receiveEvent(int bytes) {
  if (bytes >= 10) {
    flag1 = Wire.read();
    flag2 = Wire.read();

    byte* data = (byte*) &vector;
    for (int i = 0; i < sizeof(vector); i++) {
      data[i] = Wire.read();
    }

    newData = true;  // Mark new data received
  } else {
    Serial.println("Error: Not enough bytes received.");
  }
}
