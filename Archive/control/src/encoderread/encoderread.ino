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

void poll(){
dt = digitalRead(dtpin);
clk = digitalRead(clkpin);
byte s = (clk << 3) | (dt << 2) | (prevclk << 1) | prevdt; // turn the

switch(s){case 0: case 5: case 10: case 15:
73. break;
74. case 1: case 7: case 8: case 14:
75. counts++; break;
76. case 4: case 11: case 13:
77. counts--; break;
78. case 3: case 12:
79. counts += 2; break;
80. default:
81. counts -= 2; break;
82. }
83. prevclk = clk;