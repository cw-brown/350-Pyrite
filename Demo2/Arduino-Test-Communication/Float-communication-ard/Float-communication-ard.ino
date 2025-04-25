#include <Wire.h>

const int slaveAddress = 8;

float distance = 0.0;
float angle = 0.0;
uint8_t flag1 = 0;
uint8_t flag2 = 0;
uint8_t offset = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(slaveAddress);
  Wire.onReceive(receiveEvent);
  Serial.println("Arduino ready...");
}

void loop() {
  // nothing needed here
}

void receiveEvent(int bytes) {
  if (bytes == 11) {
    offset = Wire.read();  // Read the offset byte first
    flag1 = Wire.read();
    flag2 = Wire.read();

    union { byte b[4]; float f; } uf;

    // Read distance
    for (int i = 0; i < 4; i++) uf.b[i] = Wire.read();
    distance = uf.f;

    // Read angle
    for (int i = 0; i < 4; i++) uf.b[i] = Wire.read();
    angle = uf.f;

    Serial.print("Offset: 0x");
    Serial.print(offset, HEX);
    Serial.print(" | Flags: ");
    Serial.print(flag1);
    Serial.print(", ");
    Serial.print(flag2);
    Serial.print(" | Distance: ");
    Serial.print(distance, 2);
    Serial.print(" ft | Angle: ");
    Serial.println(angle, 2);
  } else {
    Serial.print("Wrong byte count: ");
    Serial.println(bytes);
  }
}
