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
      for (int i = 0; i < 8; i++) {
          receivedData[i] = Wire.read();
      }


            Serial.print("Received floats: ");
            Serial.print(receivedData.values[0]);
            Serial.print(", ");
            Serial.println(receivedData.values[1]);
        }
    }
}

void setup() {
    Serial.begin(115200);
    Wire.begin(I2C_ADDR);
    Wire.onReceive(receiveData);
}

void loop() {
    delay(100);
}
