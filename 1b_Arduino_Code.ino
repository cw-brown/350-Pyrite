#include <Wire.h>

#define MY_ADDR 8  // Arduino I2C address

volatile uint8_t receivedValue = 0;  // Stores the received integer
volatile uint8_t reply = 0;  // Stores the value to send back

void setup() {
    Serial.begin(115200);
    Wire.begin(MY_ADDR);  // Initialize as I2C slave
    Wire.onReceive(receiveData);  // Set function to run when data is received
    Wire.onRequest(sendData);  // Set function to run when Pi requests data
}

void loop() {
    // Nothing required here since I2C is interrupt-driven
}

// Function triggered when the Arduino receives data
void receiveData(int byteCount) {
    if (Wire.available()) {
        receivedValue = Wire.read();  // Read the integer sent by Pi
        reply = receivedValue + 100;  // Prepare response
        Serial.print("Received from Pi: ");
        Serial.println(receivedValue);
        Serial.print("Replying with: ");
        Serial.println(reply);
    }
}

// Function triggered when the Pi requests data
void sendData() {
    Wire.write(reply);  // Send modified value to Pi
}
