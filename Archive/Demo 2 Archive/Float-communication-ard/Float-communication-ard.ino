#include <Wire.h>

// I2C slave address
const int slaveAddress = 8;

// Buffers
float distance = 0.0;
float angle = 0.0;
byte controlFlags[4];  // markerFound, arrowDir, flag2, pad

void setup() {
  Serial.begin(115200);
  Wire.begin(slaveAddress);  // Join I2C bus as slave
  Wire.onReceive(receiveEvent);
  Serial.println("Ready to receive data...");
}

void loop() {
  
}

void receiveEvent(int numBytes) {
  if (numBytes == 13) {
    // Read and ignore offset byte
    byte offset = Wire.read(); 

    // Read control flags
    for (int i = 0; i < 2; i++) {
      controlFlags[i] = Wire.read();
    }
    Wire.read();
    Wire.read();

    // Read distance (float, 4 bytes)
    byte distBytes[4];
    for (int i = 0; i < 4; i++) {
      distBytes[i] = Wire.read();
    }
    memcpy(&distance, distBytes, sizeof(float));

    // Read angle (float, 4 bytes)
    byte angleBytes[4];
    for (int i = 0; i < 4; i++) {
      angleBytes[i] = Wire.read();
    }
    memcpy(&angle, angleBytes, sizeof(float));

    // Print everything
    Serial.print("markerFound: "); Serial.print(controlFlags[0]);
    Serial.print(", arrowDir: ");  Serial.print(controlFlags[1]);
//    Serial.print(", flag2: ");     Serial.print(controlFlags[2]);
    Serial.print(", distance: ");  Serial.print(distance, 2);
    Serial.print(", angle: ");     Serial.println(angle, 2);
    
  } else {
    Serial.println("❌ Error: Not enough bytes received.");
  }
}
