#include <Arduino.h>
#include <BasicEncoder.h>

const uint8_t D2 = 4;
const uint8_t M1DIR = 7;
const uint8_t M2DIR = 8;
const uint8_t M1PWM = 9;
const uint8_t M2PWM = 10;
const uint8_t M1ENCA = 2;
const uint8_t M1ENCB = 5;
const uint8_t M2ENCA = 3;
const uint8_t M2ENCB = 6;

const float r = 0.0746125;

BasicEncoder encoderA(M1ENCA, M1ENCB);
BasicEncoder encoderB(M2ENCA, M2ENCB);

long int calcVelocity(){
  int changeA = encoderA.get_change();
  int changeB = encoderB.get_change();
}

void setup() {
  Serial.begin(115200);
  pinMode(D2, OUTPUT);
  pinMode(M1DIR, OUTPUT);
  pinMode(M2DIR, OUTPUT);
  pinMode(M1PWM, OUTPUT);
  pinMode(M2PWM, OUTPUT);
  digitalWrite(D2, HIGH);
  digitalWrite(M1DIR, LOW);
  digitalWrite(M2DIR, LOW);
}

void loop() {
  encoderA.service();
  encoderB.service();
  int changeA = encoderA.get_change();
  int changeB = encoderB.get_change();
  int countA, countB;
  float dThetaOldA, dThetaOldB;
  if(millis() % 2 == 0){ // Every 2 milliseconds
    countA = encoderA.get_count();
    countB = encoderB.get_count();
    float dThetaA = r * ((countA - changeA)*2*PI) / 376;
    float dThetaB = r * ((countB - changeB)*2*PI) / 376;
    float velocityA = (dThetaA - dThetaOldA) * 2000.0;
    float velocityB = (dThetaB - dThetaOldB) * 2000.0;
    dThetaOldA = dThetaA;
    dThetaOldB = dThetaB;
    Serial.print("Velocity A: ");
    Serial.print(velocityA);
    Serial.print(" Velocity B: ");
    Serial.print(velocityB);
    Serial.print("\n\r");
  }
}