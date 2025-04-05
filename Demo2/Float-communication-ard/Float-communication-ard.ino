#include <Wire.h>

const int slaveAddress = 8;
float vector[2];      // distance, angle
uint8_t flag1, flag2; // 2 flags

void setup() {
  Serial.begin(115200);
  Wire.begin(slaveAddress);
  Wire.onReceive(receiveEvent);
  Serial.println("Arduino ready to receive data...");
}

void loop() {
  // Nothing here – all done in receiveEvent
}

void receiveEvent(int bytes) {
  if (bytes >= 10) {
    // Read flags
    flag1 = Wire.read();
    flag2 = Wire.read();

    // Read 8 bytes into float vector
    byte* data = (byte*)&vector;
    for (int i = 0; i < sizeof(vector); i++) {
      data[i] = Wire.read();
    }

    // Print all values
    Serial.print("Flags: ");
    Serial.print(flag1);
    Serial.print(", ");
    Serial.print(flag2);
    Serial.print(" | Distance: ");
    Serial.print(vector[0], 2);
    Serial.print(" ft, Angle: ");
    Serial.println(vector[1], 2);
  } else {
    Serial.println("ERROR: Received < 10 bytes!");
  }
}
