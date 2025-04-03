#include <Wire.h>

const int slaveAddress = 8;  // I2C address of the slave device
float vector[2];  // 2-element vector to receive

void setup() {
  Wire.begin(slaveAddress);  // Start I2C as slave with the given address
  Wire.onReceive(receiveEvent);  // Function to call when master sends data
  Serial.begin(9600);
}

void loop() {
  // Nothing needed here, since communication is handled by interrupt function
}

// Function to receive data from the master
void receiveEvent(int bytes) {
  if (bytes == sizeof(vector)) {
    // Read 8 bytes and convert them to float values
    byte* data = (byte*) &vector;
    for (int i = 0; i < sizeof(vector); i++) {
      data[i] = Wire.read();  // Store bytes into vector
    }

    // Print the received vector values
    Serial.print("Received vector: ");
    Serial.print(vector[0], 2);  // Print first float with 2 decimal places
    Serial.print(", ");
    Serial.println(vector[1], 2);  // Print second float with 2 decimal places
  }
}
