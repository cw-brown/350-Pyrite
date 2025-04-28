#include <Wire.h>

const int slaveAddress = 8;  // I2C address of the slave device
float vector[2];  // 2-element vector to store received floats
byte extraByte;   // Extra byte (9th byte) that we will ignore

void setup() {
  Serial.begin(115200);
  Wire.begin(slaveAddress);  // Start I2C as slave with the given address
  Wire.onReceive(receiveEvent);  // Function to call when master sends data
  Serial.println("Arduino is ready to receive data...");
}

void loop() {
  // Nothing needed here, as we handle the data in the receive event
}

// Function to receive data from the master
void receiveEvent(int bytes) {
//  Serial.print("Bytes received: ");
//  Serial.println(bytes);

  // Ensure we have at least 9 bytes, where we ignore the first byte
  if (bytes >= 9) {
    // Read and ignore the first byte (offset byte)
    Wire.read();  // Discard the first byte

    // Read 8 bytes into the vector (first 2 floats)
    byte* data = (byte*) &vector;
    for (int i = 0; i < sizeof(vector); i++) {
      data[i] = Wire.read();  // Store bytes into vector
    }

    // Print the received vector values
//    Serial.print("Received vector: ");
    Serial.print(vector[0], 2);  // Print first float with 2 decimal places
    Serial.print(", ");
    Serial.println(vector[1], 2);  // Print second float with 2 decimal places
  } else {
    // If less than 9 bytes, report an error
    Serial.println("Error: Unexpected number of bytes received.");
  }
}
