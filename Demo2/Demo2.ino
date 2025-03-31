#include <Arduino.h> // for PI
#include <Encoder.h> // for reading wheel encoders
#include <Wire.h> // for communicating with Rasberry Pi

// Wire and Pi stuff
#define MY_ADDR 8  // Arduino I2C address
volatile uint8_t reg;
// recieved data from camera
float markerPhi;  // new
float markerRho;  // new

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
int curEncCount[] = {0,0}; //Intilization for Encoder Counts to be used in loop (easier to reference)
float curEncRad[] = {0,0}; //Converted counts to radian for velocity in rad
int initEncCount[] = {0,0};
float initEncRad[] = {0,0};

// Timing Var
float ts = 10;  // Sample time in milliseconds
float initialTime;
float last_time_ms;
float timeElapsed;
float pause = 0;  // allow camera to send // new
float startTime;  // new
float lastTime; // new
float currentTime;  // new

// Drive Vars
float pwm[] = {0, 0}; // PWM for motors
float battery_V = 7.8; // Battery voltage as set on robot's motor shield
float vel[] = {0,0};
float voltage[] = {0, 0};
float Vbar = 0;
float deltaV = 0;

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
float desiredPhi = -45 * (-PI/180); // convert deg rad (only change # before the * sign)
float phi = 0;
float desiredPhiVel = 0;
float errorPhi = 0;
float derivativePhi = 0;
float errorPhiInitial = 0;
float integralPhi = 0;
float KpPhi = 50;
float KdPhi = 5;
float KiPhi = 3.75;
float phiVel = 0;
float errorPhiVel = 0;
float KpPhiVel = 2;

// distance stuff
float desiredRho = 4.2426; // in feet
float prevDesiredRho = 0; // in feet
float rho = 0;
float desiredRhoVel = 0;
float errorRho = 0;
float derivativeRho = 0;
float errorRhoInitial = 0;
float integralRho = 0;
float KpRho = 30.3514;
float KdRho = 2.2038;
float KiRho = 0.383;
float rhoVel = 0;
float errorRhoVel = 0;
float KpRhoVel = .8;

enum Mode {SEEK, MOVE_FWD, ROTATE, WAIT};  // Define the states // new
Mode mode = SEEK;  // Initialize to a mode
Mode prevMode = SEEK; // new

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

  startTime = millis();
  initialTime = millis();

  prevDesiredRho = desiredRho;
}

void loop() {
  // If there is data, read it
  if (msgLength > 0) {  //data from PI
    char buffer[32]; // Adjust the size as needed
    int index = 0;

    // Read data into the buffer
    while (Wire.available() && index < sizeof(buffer) - 1) {
        buffer[index++] = Wire.read();
    }
    buffer[index] = '\0'; // Null-terminate the string

    // Convert the buffer to a string and parse values
    String receivedData = String(buffer);
    int firstSpace = receivedData.indexOf(' ');
    int secondSpace = receivedData.indexOf(' ', firstSpace + 1);

    if (firstSpace != -1 && secondSpace != -1) {
        mode = receivedData.substring(0, firstSpace).toInt();
        markerPhi = receivedData.substring(firstSpace + 1, secondSpace).toFloat();
        markerRho = receivedData.substring(secondSpace + 1).toFloat();
    }
  }

  // time-keeping
  lastTime = millis(); //Compute current time
  currentTime = (float)(millis()-startTime)/1000;
  timeElapsed = (float)(millis()-initialTime)/1000;

  // position calcs
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
    case SEEK:  // turn until finding marker
      desiredRhoVel = 0;
      //desiredPhiVel = 7.5;

      if (currentTime - pause < 0.5 && f_detected != true) {         // allow camera time to and send by going to wait case
        desiredPhiVel = -10;
        desiredRhoVel = 0;
      } else {
        pause = currentTime;       //keep track of pause time
        prevMode = mode;
        phi = 0;
        rho = 0;
        errorRho = 0;
        errorPhi = 0;
        errorPhiVel = 0;
        errorRhoVel = 0;
        desiredPhi = 0;
        desiredRho = 0;
        curEncCount[0] = 0;
        curEncCount[1] = 0;
        mode = WAIT;
      }
      if (f_detected == true) {     // camera sends data
        Serial.print("Marker Found");
        phi = 0;
        rho = 0;
        errorRho = 0;
        errorPhi = 0;
        errorPhiVel = 0;
        errorRhoVel = 0;
        desiredPhi = 0;
        desiredRho = 0;
        curEncCount[0] = 0;
        curEncCount[1] = 0;
        prevMode = SEEK;
        mode = ROTATE;
        desiredPhi = markerPhi;       //turn towards marker
        delay(1500);
      }
      break;

    case ROTATE:  // Align to desiredPhi
      desiredRho = 0;
      desiredRhoVel = 0;
      if (phi <= desiredPhi + PI/360 && phi >= desiredPhi - PI/360) { // if robot is oriented toward marker within .5 degrees
      // Serial.print("desiredPhi Reached"); // debugging
        if (prevMode == SEEK || f_movefwd == true) {  // go into straight case 
          phi = 0;
          rho = 0;
          errorRho = 0;
          errorPhi = 0;
          errorPhiVel = 0;
          errorRhoVel = 0;
          desiredPhi = 0;
          desiredRho = 0;   //void junk values
          desiredRhoVel = 0; 
          prevMode = ROTATE;
          mode = MOVE_FWD; // transistion to moving straight forward
          desiredRho = markerRho - 1;  //stop 1 ft away from marker
        }
      }
      break;
    
    case MOVE_FWD:  // Move forward to desiredRho
      desiredRho = prevDesiredRho; // arbitrary
      //desiredPhi = 0;
      if (abs(rho) == abs(desiredRho)) {
        prevMode = MOVE_FWD;
        mode = WAIT;
      }
      break;

    case WAIT:  // Wait for camera or wait indefinitely (stop)
      Serial.print("wait . . .");
      desiredPhiVel = 0;
      desiredRhoVel = 0; 
      desiredPhi = 0;
      desiredRho = 0;
      if (prevMode == SEEK && (currentTime - pause >= 2.5)) {  // [go to scan case when done waiting for camera
        prevMode = WAIT;
        mode = SEEK;
        pause = currentTime;
      }
      if (f_turn){

      }

      //essentially STOP case
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

    // PID control for distance
    errorRho = desiredRho - rho;
    derivativeRho = (errorRho - errorRhoInitial)/((float)(ts/1000));
    integralRho += errorRho*((float)(ts/1000));
    desiredRhoVel = errorRho*KpRho + KdRho*derivativeRho + KiRho*integralRho;
    // limit set fwd speed
    if(abs(desiredRhoVel) > 10){
        desiredRhoVel = 10*desiredRhoVel/abs(desiredRhoVel);
    }
  }

  // phiVel stuff
  phiVel = (r/b)*(vel[0]-vel[1]);
  errorPhiVel = desiredPhiVel - phiVel;

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
  if(mode!=WAIT){
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
  }else{
    analogWrite(M_PWM[0], 0);
    analogWrite(M_PWM[1], 0);
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
  // Set the offset, always first byte.
  offset = Wire.read();
  // If there is information after the offset, it is telling us more about the command.
  while (Wire.available()) {
    msg[msgLength] = Wire.read();
    msgLength++;
  }
}