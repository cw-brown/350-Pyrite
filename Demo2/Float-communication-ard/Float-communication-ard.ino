#include <Wire.h>

const int slaveAddress = 8;  // I2C address of the slave device
float vector[2];             // Distance and angle
uint8_t flag1, flag2;        // Two flag bytes

void setup() {
  Serial.begin(115200);
  Wire.begin(slaveAddress);          // Start I2C as slave
  Wire.onReceive(receiveEvent);      // Register callback
  Serial.println("Arduino is ready to receive data...");
}

void loop() {
  // Nothing here; event-driven receive
}

void receiveEvent(int bytes) {
  if (bytes >= 10) {
    // Read flags
    flag1 = Wire.read();  // First byte
    flag2 = Wire.read();  // Second byte

    // Read 8 bytes into float array (2 floats)
    byte* data = (byte*) &vector;
    for (int i = 0; i < sizeof(vector); i++) {
      data[i] = Wire.read();
    }

    // Print everything
    Serial.print("Flags: ");
    Serial.print(flag1);
    Serial.print(", ");
    Serial.print(flag2);
    Serial.print(" | Vector: ");
    Serial.print(vector[0], 2);  // Distance
    Serial.print(", ");
    Serial.println(vector[1], 2);  // Angle
  } else {
    Serial.println("Error: Not enough bytes received.");
  }
}
