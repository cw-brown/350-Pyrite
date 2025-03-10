/* 
This File implements a PI controller for the motors.
*/
#include <Arduino.h>
#include <Wire.h>

// Wire stuff
#define MY_ADDR 8  // Arduino I2C address
volatile uint8_t receivedValue = 0;  // Stores the received integer

// Pin Definitions
const uint8_t D2 = 4;
const uint8_t M1DIR = 7;
const uint8_t M2DIR = 8;
const uint8_t M1PWM = 9;
const uint8_t M2PWM = 10;
const uint8_t M1ENCA = 2; // Encoder 1 channel A (interrupt pin)
const uint8_t M1ENCB = 5; // Encoder 1 channel B
const uint8_t M2ENCA = 3; // Encoder 2 channel A (interrupt pin)
const uint8_t M2ENCB = 6; // Encoder 2 channel B

// Global variables for encoders
volatile long countL = 0, oldL = 0;
volatile long countR = 0, oldR = 0;

// Define parameters
const float r = 0.0746125; // Wheel radius in meters
const float b = 0.375; // Wheelbase in meters
const int counts_per_rev = 3200; // Encoder counts per wheel revolution
const unsigned long ts = 100; // Sample time in ms

// Initial robot position and orientation
float t=0.0, x = 0.0, y = 0.0, phi = 0.0; 
float new_pos[] = {0,0};
float old_pos[] = {0,0};

//Drive variables
float battery_V = 7.8;
float Voltage[] = {0, 0};

// Control variables
float pos_error[] = {0, 0};
float desired_pos[] = {0, 0};
float actual_pos[] = {0, 0};
float integral_error[] = {0, 0};
float desired_speed[] = {0, 0};
float actual_speed[] = {0, 0};
float error[] = {0, 0};

//Control constraints
float K[] = {1.75,1.75};
float sigma[] = {15,15};
float Kp[] = {2.5,2.5};
float P[] = {30.26,30.26};
float I[] = {150.72,150.72};

void setup() {
  Serial.begin(115200);

  Wire.begin(MY_ADDR);  // Initialize as I2C slave
  Wire.onReceive(receiveData);  // Set function to run when data is received

  pinMode(M1ENCA, INPUT);
  pinMode(M1ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(M1ENCA), ISR_encoder1Change, CHANGE);

  pinMode(M2ENCA, INPUT);
  pinMode(M2ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(M2ENCA), ISR_encoder2Change, CHANGE);

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
  if (millis() % ts == 0){
    old_pos[0] = new_pos[0];
    old_pos[1] = new_pos[1];

    // Convert counts to wheel rotations (radians)
    float d_thetaL = ((countL-oldL) * 2.0 * PI) / counts_per_rev;
    float d_thetaR = ((countR-oldR) * 2.0 * PI) / counts_per_rev;

    // Compute wheel displacements
    new_pos[0] = r * d_thetaL; // Left wheel displacement
    new_pos[1] = r * d_thetaR; // Right wheel displacement

    actual_pos[0]+=new_pos[0];
    actual_pos[1]+=new_pos[1];
    actual_speed[0] = (float) (actual_pos[0]-old_pos[0]) / (float) ts;
    actual_speed[1] = (float) (actual_pos[1]-old_pos[1]) / (float) ts;

    oldL = countL;
    oldR = countR;
  }
  // Compute position control
  for (int i = 0; i < 2; i++) {
      pos_error[i] = desired_pos[i] - actual_pos[i];
      integral_error[i] += pos_error[i] * ((float) ts / 1000.0);
      desired_speed[i] = P[i] * pos_error[i] + I[i] * integral_error[i];
      error[i] = desired_speed[i] - actual_speed[i];
      Voltage[i] = Kp[i] * error[i];
  }

  // Apply control outputs
  for (int i = 0; i < 2; i++) {
    const uint8_t dir_array[2] = {M1DIR, M2DIR};
    const uint8_t pwm_array[2] = {M1PWM, M2PWM};
    if(Voltage[i] > 0){
      digitalWrite(dir_array[i], HIGH);
    }
    else{
      digitalWrite(dir_array[i], LOW);
    }
    int pwm = min(255, max(0, 255 * abs(Voltage[i]) / 7.5));
    analogWrite(pwm_array[i], pwm);
  }
}

// ISR for Encoder 1
void ISR_encoder1Change() {
  if (digitalRead(M1ENCA) == digitalRead(M1ENCB)) {
    countL += 2;
  } else {
    countL -= 2;
  }
}

// ISR for Encoder 2
void ISR_encoder2Change() {
  if (digitalRead(M2ENCA) == digitalRead(M2ENCB)) {
    countR += 2;
  } else {
    countR -= 2;
  }
}

void receiveData(int numBytes) {
  if (Wire.available()) {
    String receivedMessage = "";
    
    // Read the entire message
    while (Wire.available()) {
      char c = Wire.read();
      receivedMessage += c;
    }

    Serial.print("Received from Pi: ");
    Serial.println(receivedMessage);

    // Parse the received string (expected format: "0 0", "0 1", "1 0", "1 1")
    desired_pos[0] = receivedMessage[0] - '0'; // Convert char to int
    desired_pos[1] = receivedMessage[2] - '0'; // Convert char to int
  }
}
