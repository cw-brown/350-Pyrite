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
const float r = 0.251791667; // Wheel radius in feet
const float b = 1.425; // Wheelbase in feet
float t=0.0, x = 0.0, y = 0.0;

// angle stuff
float desiredPhi = 0 * (-PI/180); // convert deg rad (only change # before the * sign) // new
float phi = 0; // new
float desiredPhiVel = 0; // new
float errorPhi = 0; // new
float derivativePhi = 0; // new
float errorPhiInitial = 0; // new
float integralPhi = 0; // new
float KpPhi = 50; // new
float KdPhi = 5; // new
float KiPhi = 3.75; // new
float phiVel = 0; // new
float errorPhiVel = 0; // new
float KpPhiVel = 2; // new

// distance stuff
float desiredRho = 0; // in feet // new
float desiredRhoInit = 0; // in feet // new
float rho = 0; // new
float desiredRhoVel = 0; // new
float errorRho = 0; // new
float derivativeRho = 0; // new
float errorRhoInitial = 0; // new
float integralRho = 0; // new
float KpRho = 30.3514; // new
float KdRho = 2.2038; // new
float KiRho = 0.383; // new
float rhoVel = 0; // new
float errorRhoVel = 0; // new
float KpRhoVel = .8; // new

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

  desiredRhoInit = desiredRho;
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
      desiredRho = 0;
      if (phi <= desiredPhi + PI/360 && phi >= desiredPhi - PI/360) {
        mode = MOVE_FWD;
      }
      break;
    case MOVE_FWD:  // Move forward to desiredRho
      desiredRho = desiredRhoInit; // arbitrary
      //desiredPhi = 0;
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
    // PID control for angle
    errorPhi = desiredPhi - phi;
    derivativePhi = (errorPhi - errorPhiInitial)/((float)(ts/1000));
    integralPhi += errorPhi*((float)(ts/1000));
    desiredPhiVel = KpPhi*errorPhi + KdPhi*derivativePhi + KiPhi*integralPhi;
    // limit set angular speed
    if(abs(desiredPhiVel) > 10){
      desiredPhiVel = 10*desiredPhiVel/abs(desiredPhiVel);
    }
    // phiVel stuff
    phiVel = (r/b)*(vel[0]-vel[1]);
    errorPhiVel = desiredPhiVel - phiVel;

    // PID control for distance
    errorRho = desiredRho - rho;
    derivativeRho = (errorRho - errorRhoInitial)/((float)(ts/1000));
    integralRho += errorRho*((float)(ts/1000));
    desiredRhoVel = errorRho*KpRho + KdRho*derivativeRho + KiRho*integralRho;
    // limit set fwd speed
    if(abs(desiredRhoVel) > 10){
        desiredRhoVel = 10*desiredRhoVel/abs(desiredRhoVel);
    }
    // rhoVel stuff
    rhoVel = (r/2)*(vel[0]+vel[1]);
    errorRhoVel = desiredRhoVel - rhoVel;

    // print/debugging statements
    Serial.print(mode);
    Serial.print("\t");
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

    // voltage calculations and setting
    Vbar = errorRhoVel*KpRhoVel;
    deltaV = errorPhiVel*KpPhiVel;
    voltage[0] = (Vbar+deltaV)/2;
    voltage[1] = (Vbar-deltaV)/2;
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
}

// I2C interrupt for recieving data from Rasberry Pi (not used in demo 1)
void receiveData (){
  // Set user num
  reg = Wire.read();
  while (Wire.available()){
    quad = Wire.read(); // gets the number of the quadrant sent by the Pi
    // Serial.print("Received from Pi: "); // debugging
    // Serial.println(quad); // debugging
  }
}
