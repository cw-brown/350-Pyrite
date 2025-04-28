#include <Arduino.h>

uint8_t D2 = 4;
uint8_t M1DIR = 7;
uint8_t M2DIR = 8;
uint8_t M1PWM = 9;
uint8_t M2PWM = 10;

void setup() {
  pinMode(D2, OUTPUT);
  pinMode(M1DIR, OUTPUT);
  pinMode(M2DIR, OUTPUT);
  pinMode(M1PWM, OUTPUT);
  pinMode(M2PWM, OUTPUT);
  digitalWrite(D2, HIGH);
  digitalWrite(M1DIR, HIGH);
  digitalWrite(M2DIR, HIGH);
}

void loop() {
  t = millis();
  while(t >= 1024 && t <= 1034){
    analogWrite(M1PWM, 100);
    analogWrite(M2PWM, 100);
  }
}
