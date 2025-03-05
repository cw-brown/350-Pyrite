#include <Arduino.h> // for PI
#include <Encoder.h> // for reading wheel encoders
#include <Wire.h> // for communicating with Rasberry Pi

// // Input stuff
// float dist_feet = 10; // in feet
// float distance = dist_feet/3.281; // in m

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
int curEncCount[] = {0,0}; //Intilization for Encoder Counts to be used in loop (easier to reference)  // new
float curEncRad[] = {0,0}; //Converted counts to radian for velocity in rad  // new
int initEncCount[] = {0,0}; // new
float initEncRad[] = {0,0}; // new

// Timing Var
float ts = 10;  // Sample time in milliseconds
float initialTime; // new
float last_time_ms; // new
float timeElapsed; // new

// Drive Vars
float pwm[] = {0, 0}; // PWM for motors
float battery_V = 7.8; // Battery voltage as set on robot's motor shield
float vel[] = {0,0}; // new
float voltage[] = {0, 0}; // new
float Vbar = 0; // new
float deltaV = 0; // new

// Motor control params
// float K[] = {1.8,1.75};
// float sigma[] = {10,15};
float Kp[] = {.05,.05}; // .5 goes quicker but harder to test anti-windup
float P[] = {.065*4,.0635*4};
float I[] = {0.001,0.001};

// Localization stuff
const float r = 0.24; // Wheel radius in feet
const float b = 1.23; // Wheelbase in feet
float t=0.0, x = 0.0, y = 0.0;

// angle stuff
float desiredPhi = PI; // in rad // new
float phi = 0; // new
float desiredPhiVel = 0; // new
float errorPhi = 0; // new
float derivativePhi = 0; // new
float errorPhiInitial = 0; // new
float integralPhi = 0; // new
float KpPhi = 20; // new
float KdPhi = 0.2; // new
float KiPhi = 0.45; // new
float phiVel = 0; // new
float errorPhiVel = 0; // new
float KpPhiVel = .2; // new

// distance stuff
float desiredRho = 0; // in feet // new
float desiredRhoInit = desiredRho; // in feet // new
float rho = 0; // new
float desiredRhoVel = 0; // new
float errorRho = 0; // new
float derivativeRho = 0; // new
float errorRhoInitial = 0; // new
float integralRho = 0; // new
float KpRho = 23.3514; // new
float KdRho = .2038; // new
float KiRho = .383; // new
float rhoVel = 0; // new
float errorRhoVel = 0; // new
float KpRhoVel = .02; // new

enum Mode { ROTATE, MOVE_FWD, STOP };  // Define the states
Mode mode = ROTATE;  // Initialize to a mode

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

  // Gets inital counts from the encoder on left and right motors
  initEncCount[0] = M1Enc.read();
  initEncCount[1] = M2Enc.read();

  // Gets initial position in radians for left and right motors
  initEncRad[0] = 2 * PI * (float)old_pos_count[0] / counts_per_rev; 
  initEncRad[1] = 2 * PI * (float)old_pos_count[1] / counts_per_rev;

  initialTime = millis();
}

void loop() {
  // position calcs
  timeElapsed = (float)(millis()-initialTime)/1000;
  curEncCount[0] = M1Enc.read();
  curEncCount[1] = M2Enc.read();
  for(int i = 0;i<2;i++){ // set encoders to radians and finds velocity
    curEncRad[i] = 2*PI*(float)(curEncCount[i])/counts_per_rev;
    if(timeElapsed > 0 ){
        vel[i] = (curEncRad[i]-initEncRad[i])/timeElapsed;
    }
  }
  phi = (r/b) * (curEncRad[0]-curEncRad[1]);
  rho = (r/2) * (curEncRad[0]+curEncRad[1]);

  // fsm for what the robot should do
  switch (mode) {
    case ROTATE:  // Align to desiredPhi
      if (desiredPhi >= 0) {
        desiredRhoVel = 0;
        desiredPhiVel = 10; // arbitrary
      } else {
        desiredRhoVel = 0;
        desiredPhiVel = -10; // arbitrary
      }
      desiredRhoInit = desiredRho;
      desiredRho = 0;
      if (phi <= desiredPhi + PI/180 && phi >= desiredPhi - PI/180) {
        mode = MOVE_FWD;
      }
      break;
    case MOVE_FWD:  // Move forward to desiredRho
      desiredRho = desiredRhoInit; // arbitrary
      desiredPhi = 0;
      if (abs(rho) == desiredRho) {
        mode = STOP;
      }
      break;

    case STOP:  // Stop all motion
      desiredPhi = 0;
      desiredRho = 0;
      analogWrite(M_PWM[0], 0);
      analogWrite(M_PWM[1], 0);
      break;
  }

  // control
  if(mode == ROTATE || mode == MOVE_FWD) {
    errorPhi = desiredPhi - phi;
    derivativePhi = (errorPhi - errorPhiInitial)/((float)(ts/1000));
    integralPhi += errorPhi*((float)(ts/1000));
    desiredPhiVel = KpPhi*errorPhi + KdPhi*derivativePhi + KiPhi*integralPhi;
    // if(abs(desiredPhiVel) > 10){
    //   desiredPhiVel = 10*desiredPhiVel/abs(desiredPhiVel);
    // }
    phiVel = (r/b)*(vel[0]-vel[1]);
    errorPhiVel = desiredPhiVel - phiVel;

    errorRho = desiredRho - rho;
    derivativeRho = (errorRho - errorRhoInitial)/((float)(ts/1000));
    integralRho += errorRho*((float)(ts/1000));
    desiredRhoVel = errorRho*KpRho + KdRho*derivativeRho + KiRho*integralRho;
    // if(abs(desiredRhoVel) > 10){
    //     desiredRhoVel = 10*desiredRhoVel/abs(desiredRhoVel);
    // }
    rhoVel = (r/2)*(vel[0]+vel[1]);
    errorRhoVel = desiredRhoVel - rhoVel;

    Serial.print(rho);
    Serial.print("\t");
    Serial.print(desiredRho);
    Serial.print("\t");
    Serial.print(phi);
    Serial.print("\t");
    Serial.print(desiredPhi);
    Serial.print("\t");
    Serial.print(voltage[0]);
    Serial.print("\t");
    Serial.println(voltage[1]);

    Vbar = errorRhoVel*KpRhoVel;
    deltaV = errorPhiVel*KpPhiVel;
    voltage[0] = (Vbar+deltaV)/2;
    voltage[1] = (Vbar-deltaV)/2;
    // if (mode == ROTATE) {
    //   voltage[1] = abs(voltage[1])*-1;
    // }
    for (int i = 0; i < 2; i++) {
      if(voltage[i] > 0){
        digitalWrite(M_DIR[i], HIGH);
      }
      else{
        digitalWrite(M_DIR[i], LOW);
      }
      pwm[i] = 255 * abs(voltage[i]) / battery_V;
      analogWrite(M_PWM[i], min(pwm[i], 255));
    }
  }

  // update prev to current
  errorRhoInitial = errorRho;
  errorPhiInitial = errorPhi;
  for(int i = 0;i<2;i++){ 
    initEncCount[i] = curEncCount[i];
    initEncRad[i] = curEncRad[i];
  }
  
  initialTime = millis();
  while(millis()<last_time_ms+ts){
    //wait till desired time passes
  }
  last_time_ms = millis();



  // float rad_pos[] = {0, 0};
  // float actual_pos[] = {0, 0};
  // actual_pos[0] = M1Enc.read(); // Gets actual position in counts for left motor
  // actual_pos[1] = M2Enc.read(); // Gets actual position in counts for right motor

  // // Calculate position in radians for left and right motors (used for finding pos_error)
  // rad_pos[0] = 2 * PI * (float)actual_pos[0] / counts_per_rev;
  // rad_pos[1] = 2 * PI * (float)actual_pos[1] / counts_per_rev;

  // // Update old position in rads and counts for next time
  // for (int i = 0; i < 2; i++){
  //   old_pos_rad[i] = rad_pos[i];
  //   old_pos_count[i] = actual_pos[i];
  // }

  // // Speed calculations
  // float actual_speed[] = {0, 0};
  // actual_speed[0] = (rad_pos[0] - old_pos_rad[0]) / (ts / 1000); // Calculates the current speed of left motor
  // actual_speed[1] = (rad_pos[1] - old_pos_rad[1]) / (ts / 1000); //Calculates the current speed of right motor
  // Serial.print("Left Speed: "); // debugging
  // Serial.println(actual_speed[0]); // debugging


  // // Control variables
  // float integral_error[] = {0, 0};
  // float pos_error[] = {0, 0};
  // float error[] = {0, 0};
  // float desired_speed[] = {0, 0};
  // float Voltage[] = {0, 0};

  // // Compute position calculations and control for each wheel in array
  // for (int i = 0; i < 2; i++) {
  //   pos_error[i] = desired_pos[i] - actual_pos[i];
  //   // Serial.print("Position Error: "); // debugging
  //   // Serial.println(pos_error[i]); // debugging
  //   integral_error[i] += pos_error[i] * ((float) ts / 1000.0);
  //   desired_speed[i] = P[i] * pos_error[i] + I[i] * integral_error[i];
  //   error[i] = desired_speed[i] - actual_speed[i];
  //   Voltage[i] = Kp[i] * error[i];
  //   // if (abs(Voltage[i]) > battery_V) {
  //   //   if (Voltage[i] > 0) {
  //   //       Voltage[i] = battery_V;
  //   //   } else {
  //   //       Voltage[i] = -battery_V;
  //   //   }
  //   // }
  //   if(Voltage[i] > 0){
  //     digitalWrite(M_DIR[i], HIGH);
  //   }
  //   else{
  //     digitalWrite(M_DIR[i], LOW);
  //   }
  //   pwm[i] = 255 * abs(Voltage[i]) / battery_V;
  //   analogWrite(M_PWM[i], min(pwm[i], 255));

  //   // Convert counts to wheel rotations (radians) using provided equations
  //   float d_thetaL = ((actual_pos[0]-old_pos_count[0]) * 2.0 * PI) / counts_per_rev;
  //   float d_thetaR = ((actual_pos[1]-old_pos_count[1]) * 2.0 * PI) / counts_per_rev;

  //   // Compute wheel displacements using provided equations
  //   float d_L = r * d_thetaL;  // Left wheel displacement
  //   float d_R = r * d_thetaR;  // Right wheel displacement

  //   // Update robot position using provided equations
  //   x += cos(phi) * (d_L + d_R) / 2.0;
  //   y += sin(phi) * (d_L + d_R) / 2.0;
  //   phi += (d_R - d_L) / b;
  // }

  // oldL = countL;
  // oldR = countR;

  // old_pos[0] = new_pos[0];
  // old_pos[1] = new_pos[1];

  // // Convert counts to wheel rotations (radians)
  // float d_thetaL = ((actual_pos[0]-old_pos_count[0]) * 2.0 * PI) / counts_per_rev;
  // float d_thetaR = ((actual_pos[1]-old_pos_count[1]) * 2.0 * PI) / counts_per_rev;

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