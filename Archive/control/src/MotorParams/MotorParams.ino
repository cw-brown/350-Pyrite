#include <Arduino.h>

uint8_t D2 = 4;
uint8_t M1DIR = 7;
uint8_t M2DIR = 8;
uint8_t M1PWM = 9;
uint8_t M2PWM = 10;

// Pin definitions for encoders
#define A1 2                   // Encoder 1 channel A (interrupt pin)
#define B1 5                   // Encoder 1 channel B
#define A2 3                   // Encoder 2 channel A (interrupt pin)
#define B2 6                   // Encoder 2 channel B

// Global variables for Encoder 1
volatile long countL = 0, oldL = 0;

// Global variables for Encoder 2
volatile long countR = 0, oldR = 0;

// Define parameters
const float r = 0.0746125;  // Wheel radius in meters (about)
const float b = 0.375;   // Wheelbase (distance between wheels) in meters (about)
const int counts_per_rev = 3200;  // Encoder counts per wheel revolution

// Define position variables
float t=0.0, x = 0.0, y = 0.0, phi = 0.0;  // Initial robot position and orientation

float d_L, d_R, oldd_L, oldd_R;
float voltage = 0;

void setup() {
  Serial.begin(115200);

  // Encoder 1 pins
  pinMode(A1, INPUT);
  pinMode(B1, INPUT);
  attachInterrupt(digitalPinToInterrupt(A1), ISR_encoder1Change, CHANGE);

  // Encoder 2 pins
  pinMode(A2, INPUT);
  pinMode(B2, INPUT);
  attachInterrupt(digitalPinToInterrupt(A2), ISR_encoder2Change, CHANGE);

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
  float oldL = d_L;
  float oldR = d_R;
  t = millis();
  if(t <= 3000){
    if(t >= 1024){
      analogWrite(M1PWM, 100);
      analogWrite(M2PWM, 100);
      voltage = 100*7.5/255;
    }
    else{
      analogWrite(M1PWM, 0);
      analogWrite(M2PWM, 0);
      voltage = 0*7.5/255;
    }

      // Convert counts to wheel rotations (radians)
      float d_thetaL = ((countL-oldL) * 2.0 * PI) / counts_per_rev;
      float d_thetaR = ((countR-oldR) * 2.0 * PI) / counts_per_rev;

      // Compute wheel displacements
      d_L = r * d_thetaL;  // Left wheel displacement
      d_R = r * d_thetaR;  // Right wheel displacement

    float velocity = (float) (d_L-oldL)/1000;
      Serial.print(t);
      Serial.print(",");
      Serial.print(voltage);
      Serial.print(",");
      Serial.print(d_L);
      Serial.print("\r\n");
  }
  else{
      analogWrite(M1PWM, 0);
      analogWrite(M2PWM, 0);
  }
}

// ISR for Encoder 1
void ISR_encoder1Change() {
  if (digitalRead(A1) == digitalRead(B1)) {
    countL +=2;
  } else {
    countL -=2;
  }
}

// ISR for Encoder 2
void ISR_encoder2Change() {
  if (digitalRead(A2) == digitalRead(B2)) {
    countR +=2;
  } else {
    countR -=2;
  }
}