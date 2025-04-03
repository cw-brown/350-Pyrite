#include <Wire.h>

#define I2C_ADDR 0x08  // I2C address of the Arduino

union FloatPair {
    float values[2];  // Two floats stored together
    byte bytes[8];    // Same memory as bytes
};

FloatPair receivedData;

void receiveData(int byteCount) {
    if (byteCount == 8) {  // Ensure we receive exactly 8 bytes (2 floats)
        for (int i = 0; i < 8; i++) {
            receivedData.bytes[i] = Wire.read();  // Read bytes into union
        }

        // Print the received floats
        Serial.print("Received floats: ");
        Serial.print(receivedData.values[0]);
        Serial.print(", ");
        Serial.println(receivedData.values[1]);
    } else {
        Serial.println("Error: Incomplete data received");
    }
}

void setup() {
    Serial.begin(115200);
    Wire.begin(I2C_ADDR);  // Initialize I2C with the address
    Wire.onReceive(receiveData);  // Set the function to be called on data receipt
}

void loop() {
    // Nothing needed in the loop for now
}
