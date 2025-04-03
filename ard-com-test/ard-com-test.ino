#include <Wire.h>

#define I2C_ADDR 0x08  // Arduino's I2C address (must match with Pi)

union FloatPair {
    float values[2];  // Two floats in a union
    byte bytes[8];    // Store as bytes
};

FloatPair receivedData;

void receiveData(int byteCount) {
    Serial.print("Received byte count: ");
    Serial.println(byteCount);

    if (byteCount == 8) {  // Ensure we get 8 bytes (2 floats)
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
    Serial.begin(115200);  // Set baud rate for serial monitor
    Wire.begin(I2C_ADDR);  // Initialize I2C with the address
    Wire.onReceive(receiveData);  // Set function to be called on data receipt
    Serial.println("Ready to receive data...");
}

void loop() {
    // Nothing needed in the loop for now
}
