#include <Wire.h>

#define I2C_ADDR 0x08

union FloatUnion {
    float value;
    byte bytes[4];
};

FloatUnion f1, f2;
byte receivedData[8];
int index = 0;

void receiveData(int byteCount) {
    while (Wire.available()) {
        receivedData[index++] = Wire.read();
        if (index >= 8) {
            index = 0;
            memcpy(f1.bytes, receivedData, 4);
            memcpy(f2.bytes, receivedData + 4, 4);
            Serial.print("Received floats: ");
            Serial.print(f1.value);
            Serial.print(", ");
            Serial.println(f2.value);
        }
    }
}

void setup() {
    Serial.begin(9600);
    Wire.begin(I2C_ADDR);
    Wire.onReceive(receiveData);
}

void loop() {
    delay(100);
}
