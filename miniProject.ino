#include <Arduino.h>

uint8_t D2 = 4;
uint8_t M1DIR = 7;
uint8_t M2DIR = 8;
uint8_t M1PWM = 9;
uint8_t M2PWM = 10;

// Pin definitions for encoders
#define A1 2 // Encoder 1 channel A (interrupt pin)
#define B1 5 // Encoder 1 channel B
#define A2 3 // Encoder 2 channel A (interrupt pin)
#define B2 6 // Encoder 2 channel B

// Global variables for encoders
volatile long countL = 0, oldL = 0;
volatile long countR = 0, oldR = 0;

// Define parameters
const float r = 0.0746125; // Wheel radius in meters
const float b = 0.375; // Wheelbase in meters
const int counts_per_rev = 3200; // Encoder counts per wheel revolution

float t=0.0, x = 0.0, y = 0.0, phi = 0.0; // Initial robot position and orientation
float d_L, d_R, oldd_L, oldd_R;

// Control parameters as arrays
float kp[2] = {2.5, 2.5};
float ki[2] = {2, 2};
float integral_error[2] = {0, 0};
unsigned long desired_Ts_ms = 100;

// Position and speed control variables
float desired_pos[2] = {3.14, 3.14};
float actual_pos[2] = {0, 0};
float desired_speed[2] = {0, 0};
float actual_speed[2] = {0, 0};
float pos_error[2] = {0, 0};
float error[2] = {0, 0};
float Voltage[2] = {0, 0};
float Kp_pos = 71.2;
float Ki_pos = 978.5;

void setup() {
    Serial.begin(115200);

    pinMode(A1, INPUT);
    pinMode(B1, INPUT);
    attachInterrupt(digitalPinToInterrupt(A1), ISR_encoder1Change, CHANGE);

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
  if (millis()%desired_Ts_ms==0){
    oldL = d_L;
    oldR = d_R;
    // Convert counts to wheel rotations (radians)
    float d_thetaL = ((countL-oldL) * 2.0 * PI) / counts_per_rev;
    float d_thetaR = ((countR-oldR) * 2.0 * PI) / counts_per_rev;

    // Compute wheel displacements
    d_L = r * d_thetaL; // Left wheel displacement
    d_R = r * d_thetaR; // Right wheel displacement

    actual_pos[0]+=d_L;
    actual_pos[1]+=d_R;
    actual_speed[0] = (float) (d_L-oldL) / (float)desired_Ts_ms;
    actual_speed[1] = (float) (d_R-oldR) / (float)desired_Ts_ms;
  }
  // Compute position control
  for (int i = 0; i < 2; i++) {
      pos_error[i] = desired_pos[i] - actual_pos[i];
      Serial.print(actual_pos[i]);
      integral_error[i] += pos_error[i] * (desired_Ts_ms / 1000.0);
      desired_speed[i] = Kp_pos * pos_error[i] + Ki_pos * integral_error[i];
      error[i] = desired_speed[i] - actual_speed[i];
      Voltage[i] = kp[i] * error[i];
  }

  // Apply control outputs
  for (int i = 0; i < 2; i++) {
      bool direction = Voltage[i] > 0;
      digitalWrite(i == 0 ? M1DIR : M2DIR, direction);
      int pwm = min(255, max(0, 255 * abs(Voltage[i]) / 7.5));
      analogWrite(i == 0 ? M1PWM : M2PWM, pwm);
  }
}

// ISR for Encoder 1
void ISR_encoder1Change() {
    if (digitalRead(A1) == digitalRead(B1)) {
        countL += 2;
    } else {
        countL -= 2;
    }
}

// ISR for Encoder 2
void ISR_encoder2Change() {
    if (digitalRead(A2) == digitalRead(B2)) {
        countR += 2;
    } else {
        countR -= 2;
    }
}
