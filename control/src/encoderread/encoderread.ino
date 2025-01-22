// This sketch uses the BasicEncoder library to read counts. Make sure the encoder is connected to 5V
#include <Arduino.h>
#include <BasicEncoder.h>

const int8_t clk = 4;
const int8_t dt = 9;

BasicEncoder encoder(clk, dt);

void setup() {
  Serial.begin(9600);
}

void loop() {
  encoder.service();
  int8_t count = encoder.get_change();
  if(count){
    Serial.println(encoder.get_count());
  }
}
