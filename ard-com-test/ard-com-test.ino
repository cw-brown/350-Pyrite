#include <Wire.h>

#define I2C_ADDR 0x08

union FloatPair {
    float values[2];  // Two floats stored together
    byte bytes[8];    // Same memory as bytes
};

FloatPair receivedData;

void receiveData(int byteCount) {
    Serial.println("Data received");
    if (byteCount == 8) {  // Ensure we get exactly 8 bytes
        for (int i = 0; i < 8; i++) {
            receivedData.bytes[i] = Wire.read();
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
    Wire.begin(I2C_ADDR);
    Wire.onReceive(receiveData);
    Serial.println("Ready to receive data");
}

void loop() {
    // Nothing needed in the loop for now
}
