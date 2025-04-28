#include <Arduino.h> // for PI
#include <Encoder.h> // for reading wheel encoders
#include <Wire.h> // for communicating with Rasberry Pi

// Wire and Pi stuff
#define MY_ADDR 8  // Arduino I2C address
volatile uint8_t reg;
int quad = 1; // Pi updates quadrant for wheels

// Pin Definitions
const uint8_t M_ENABLE = 4; // So motors are on
int M_DIR[] = {7,8}; // Motor to reverse and forward spin (index 0 is left motor, index 1 is right motor)
const uint8_t M_PWM[] = {9, 10}; //Motor driver pins (index 0 is left motor, index 1 is right motor)
const uint8_t M1_ENC_PINS[] = {2,5}; // Motor 1 encoder
const uint8_t M2_ENC_PINS[] = {3,6}; // Motor 2 encoder

// Set up encoder library and wheel rotation count stuff
Encoder M1Enc(M1_ENC_PINS[0], M1_ENC_PINS[1]); // index 0 is ISR pin
Encoder M2Enc(M2_ENC_PINS[0], M2_ENC_PINS[1]); // index 0 is ISR pin
float old_pos_count[] = {0,0}, old_pos_rad[] = {0,0}; // old positions to compare against
const int counts_per_rev = 3200; // Encoder counts per wheel revolution

// Timing Var
float ts = 10;  // Sample time in milliseconds

// Drive Vars
float pwm[] = {0, 0}; // PWM for motors
float battery_V = 7.8; // Battery voltage as set on robot's motor shield

// Motor control params
// float K[] = {1.8,1.75};
// float sigma[] = {10,15};
float Kp[] = {.05,.05}; // .5 goes quicker but harder to test anti-windup
float P[] = {.065,.0635};
float I[] = {0.001,0.001};

// Localization stuff
const float r = 0.0746125; // Wheel radius in meters
const float b = 0.375; // Wheelbase in meters
float t=0.0, x = 0.0, y = 0.0, phi = 0.0;

void setup() {
  Serial.begin(115200);

  // Recieve from Pi stuff
  Wire.begin(MY_ADDR);  // Initialize as I2C slave
  Wire.onReceive(receiveData);  // Set function to run when data is received

  // Enable Pin declaration
  pinMode(M_ENABLE, OUTPUT);
  digitalWrite(M_ENABLE, HIGH);

  // Initialize Left Motor
  pinMode(M_PWM[0], OUTPUT);
  digitalWrite(M_PWM[0], LOW);
  pinMode(M_DIR[0], OUTPUT);
  digitalWrite(M_DIR[0], LOW);

  // Initialize Right Motor
  pinMode(M_PWM[1], OUTPUT);
  digitalWrite(M_PWM[1], LOW);
  pinMode(M_DIR[1], OUTPUT);
  digitalWrite(M_DIR[1], LOW);

  // Sets the counts from the encoder on left and right motors
  old_pos_count[0] = M1Enc.read();
  old_pos_count[1] = M2Enc.read();

  // Gets initial position in radians for left and right motors
  old_pos_rad[0] = 2 * PI * (float)old_pos_count[0] / counts_per_rev; 
  old_pos_rad[1] = 2 * PI * (float)old_pos_count[1] / counts_per_rev;
}

void loop() {
  // Deal with Pi's sent info
  float desired_pos[] = {1, 1};
  if (quad == 1){ //Quad 1 (NE): Wheels @ 0,0
    desired_pos[0] = 0;
    desired_pos[1] = 0;
  } else if (quad == 2){ //Quad 2 (NW): Wheels @ 0,1
    desired_pos[0] = 0;
    desired_pos[1] = 6400; // should be 1600

  } else if (quad == 3){ //Quad 3 (SW): Wheels @ 1,1
    desired_pos[0] = 6400; // should be 1600
    desired_pos[1] = 6400; // should be 1600

  } else if (quad == 4){ //Quad 4 (SE): Wheels @ 1,0
    desired_pos[0] = 6400; // should be 1600
    desired_pos[1] = 0;
  }

  float rad_pos[] = {0, 0};
  float actual_pos[] = {0, 0};
  actual_pos[0] = M1Enc.read(); // Gets actual position in counts for left motor
  actual_pos[1] = M2Enc.read(); // Gets actual position in counts for right motor

  // Handle left wheel roll over to reset rotational position
  if (actual_pos[0] >= counts_per_rev){
    actual_pos[0] = actual_pos[0] - counts_per_rev;
  } else if (actual_pos[0] <= -counts_per_rev){
    actual_pos[0] = actual_pos[0] + counts_per_rev;
  }
  
   // Handle right wheel roll over to reset rotational position
  if (actual_pos[1] >= counts_per_rev){
    actual_pos[1] = actual_pos[1] - counts_per_rev;
  } else if (actual_pos[0] <= -counts_per_rev){
    actual_pos[1] = actual_pos[1] + counts_per_rev;
  }
  // Serial.print("Right Current Position: "); // debugging
  // Serial.println(actual_pos[1]); // debugging

  // Calculate position in radians for left and right motors (used for finding pos_error)
  rad_pos[0] = 2 * PI * (float)actual_pos[0] / counts_per_rev;
  rad_pos[1] = 2 * PI * (float)actual_pos[1] / counts_per_rev;

  // Update old position in rads and counts for next time
  for (int i = 0; i < 2; i++){
    old_pos_rad[i] = rad_pos[i];
    old_pos_count[i] = actual_pos[i];
  }

  // Speed calculations
  float old_pos_rad[] = {0, 0};
  float actual_speed[] = {0, 0};
  actual_speed[0] = (rad_pos[0] - old_pos_rad[0]) / (ts / 1000); // Calculates the current speed of left motor
  actual_speed[1] = (rad_pos[1] - old_pos_rad[1]) / (ts / 1000); //Calculates the current speed of right motor

  // Control variables
  float integral_error[] = {0, 0};
  float pos_error[] = {0, 0};
  float error[] = {0, 0};
  float desired_speed[] = {0, 0};
  float Voltage[] = {0, 0};

  // Compute position calculations and control for each wheel in array
  for (int i = 0; i < 2; i++) {
    pos_error[i] = desired_pos[i] - actual_pos[i];
    // Serial.print("Position Error: "); // debugging
    // Serial.println(pos_error[i]); // debugging
    integral_error[i] += pos_error[i] * ((float) ts / 1000.0);
    desired_speed[i] = P[i] * pos_error[i] + I[i] * integral_error[i];
    error[i] = desired_speed[i] - actual_speed[i];
    Voltage[i] = Kp[i] * error[i];
    // if (abs(Voltage[i]) > battery_V) {
    //   if (Voltage[i] > 0) {
    //       Voltage[i] = battery_V;
    //   } else {
    //       Voltage[i] = -battery_V;
    //   }
    // }
    if(Voltage[i] > 0){
      digitalWrite(M_DIR[i], HIGH);
    }
    else{
      digitalWrite(M_DIR[i], LOW);
    }
    pwm[i] = 255 * abs(Voltage[i]) / battery_V;
    analogWrite(M_PWM[i], min(pwm[i], 255));
  }

  // oldL = countL;
  // oldR = countR;

  // old_pos[0] = new_pos[0];
  // old_pos[1] = new_pos[1];

  // // Convert counts to wheel rotations (radians)
  // float d_thetaL = ((countL-oldL) * 2.0 * PI) / counts_per_rev;
  // float d_thetaR = ((countR-oldR) * 2.0 * PI) / counts_per_rev;

  // // Compute left wheel displacements
  // new_pos[0] = r * d_thetaL;
  // actual_pos[0]+=new_pos[0];
  // actual_speed[0] = (float) (actual_pos[0]-old_pos[0]) / (float) ts;

  // // Compute right wheel displacements
  // new_pos[1] = r * d_thetaR;
  // actual_pos[1]+=new_pos[1];
  // actual_speed[1] = (float) (actual_pos[1]-old_pos[1]) / (float) ts;
}

// // ISR for Encoder 1
// void ISR_encoder1Change() {
//   if (digitalRead(M1ENCA) == digitalRead(M1ENCB)) {
//     countL += 2;
//   } else {
//     countL -= 2;
//   }
// }

// // ISR for Encoder 2
// void ISR_encoder2Change() {
//   if (digitalRead(M2ENCA) == digitalRead(M2ENCB)) {
//     countR += 2;
//   } else {
//     countR -= 2;
//   }
// }

// I2C interrupt for recieving data from Rasberry Pi
void receiveData (){
  // Set user num
  reg = Wire.read();
  while (Wire.available()){
    quad = Wire.read(); // gets the number of the quadrant sent by the Pi
    // Serial.print("Received from Pi: "); // debugging
    // Serial.println(quad); // debugging
  }
}

// void receiveData(int numBytes) {
//   if (Wire.available()) {
//     String receivedMessage = "";
    
//     // Read the entire message
//     while (Wire.available()) {
//       char c = Wire.read();
//       receivedMessage += c;
//     }

//     Serial.print("Received from Pi: ");
//     Serial.println(receivedMessage);

//     // Parse the received string (expected format: "0 0", "0 1", "1 0", "1 1")
//     desired_pos[0] = 3.14 * receivedMessage[0] - '0'; // Convert char to int
//     desired_pos[1] = 3.14 * receivedMessage[2] - '0'; // Convert char to int
//   }
// }
