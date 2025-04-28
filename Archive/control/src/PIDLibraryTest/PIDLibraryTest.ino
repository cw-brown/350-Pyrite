#include <Arduino.h> // for PI
#include <Encoder.h> // for reading wheel encoders
#include <Wire.h> // for communicating with Rasberry Pi
#include <PID_v1.h> // PID control library

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
long unsigned int ts = 10;  // Sample time in milliseconds

// Drive Vars
float pwm[] = {0, 0}; // PWM for motors
float battery_V = 7.3; // Battery voltage as set on robot's motor shield

float r = 0.0746125; // Wheel radius in meters
float b = 0.375; // Wheelbase in meters
float rad_pos[] = {0, 0};
float actual_pos[] = {0, 0};
float actual_speed[] = {0, 0};
float delta_voltage = 0;
float voltage_a = 0;
float voltage[2] = {0, 0};

// Angle controls 
double desired_rot_angle, rot_velocity, inst_velocity, angle_error, phi;
double desired_angle = 2*PI;
PID angle_inner(&rot_velocity, &angle_error, &desired_rot_angle, 1, 0, 0, DIRECT);
PID angle_outer(&phi, &desired_rot_angle, &desired_angle, 7, 20.26, 8, DIRECT);

// Position Controls
double velocity_error, pos;
double desired_inst_velocity = 0;
double desired_pos = 2;
PID pos_inner(&inst_velocity, &velocity_error, &desired_inst_velocity, 1, 0, 0, DIRECT);
// PID pos_outer(&pos, &desired_inst_velocity, &desired_pos, 2.5, 0.35, 0.9, DIRECT);

void setup() {
  Serial.begin(115200);
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

  angle_inner.SetMode(AUTOMATIC);
  angle_inner.SetSampleTime(ts);
  angle_inner.SetOutputLimits(-30, 30);
  angle_outer.SetMode(AUTOMATIC);
  angle_outer.SetSampleTime(ts);
  angle_outer.SetOutputLimits(-30, 30);

  pos_inner.SetMode(AUTOMATIC);
  pos_inner.SetSampleTime(ts);
  pos_inner.SetOutputLimits(-30, 30);
  // pos_outer.SetMode(AUTOMATIC);
  // pos_outer.SetSampleTime(ts);
  // pos_outer.SetOutputLimits(-30, 30);
}

void loop() {
  actual_pos[0] = M1Enc.read(); // Gets actual position in counts for left motor
  actual_pos[1] = M2Enc.read(); // Gets actual position in counts for right motor

  // Calculate position in radians for left and right motors (used for finding pos_error)
  rad_pos[0] = 2 * PI * (float)actual_pos[0] / counts_per_rev;
  rad_pos[1] = 2 * PI * (float)actual_pos[1] / counts_per_rev;

  
  // Speed calculations
  actual_speed[0] = (rad_pos[0] - old_pos_rad[0]) / ((float)ts / 1000.0); // Calculates the current speed of left motor
  actual_speed[1] = (rad_pos[1] - old_pos_rad[1]) / ((float)ts / 1000.0); // Calculates the current speed of right motor

  // Calculate instantaneous forward velocity
  inst_velocity = r*(actual_speed[0]+actual_speed[1])/2; 
  pos = inst_velocity*cos(phi);

  // Calculate angle
  rot_velocity = r*(actual_speed[0]-actual_speed[1])/b;
  phi = r*(rad_pos[0]-rad_pos[1])/b;

  angle_outer.Compute();
  angle_inner.Compute();

  // pos_outer.Compute();
  pos_inner.Compute();

  delta_voltage = angle_error;
  voltage_a = velocity_error;

  voltage[0] = (voltage_a+delta_voltage)/2;
  voltage[1] = (voltage_a-delta_voltage)/2;

  for (int i = 0; i < 2; i++){
    if(voltage[i] > 0){
      digitalWrite(M_DIR[i], HIGH);
    }
    else{
      digitalWrite(M_DIR[i], LOW);
    }
    pwm[i] = 255 * max(abs(voltage[i]),0.01) / battery_V;
    analogWrite(M_PWM[i], min(pwm[i], 255));
    old_pos_rad[i] = rad_pos[i];
    old_pos_count[i] = actual_pos[i];
  }

  if(millis() % 100 == 0){
    Serial.print("Time: ");
    Serial.print(millis());
    Serial.print(",");
    Serial.print("Voltage: ");
    Serial.print((voltage[0]+voltage[1])/2);
    Serial.print(",");
    Serial.print("Angle: ");
    Serial.print(phi);
    Serial.print(",");
    Serial.print("Velocity: ");
    Serial.print(inst_velocity);
    Serial.print(",");
    Serial.print("Position: ");
    Serial.print(pos);
    Serial.print(",");
    Serial.print("Angle Error: ");
    Serial.print(angle_error);
    Serial.print(",");
    Serial.print("Velocity Error: ");
    Serial.print(velocity_error);
    Serial.print("\r\n");
  }
}
