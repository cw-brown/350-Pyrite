#include <Arduino.h> // for PI
#include <Encoder.h> // for reading wheel encoders
#include <Wire.h> // for communicating with Rasberry Pi

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

// inner control parameters - Angle
float Kp_inner_angle = 1;
float desired_rot_angle;
float angle_error;

// Outer control loop - Angle
float Kp_outer_angle = 2.5;
float Ki_outer_angle = 0.35;
float Kd_outer_angle = 0.9;
float rot_error, rot_error_prev;
float desired_angle = 2*PI;
float rot_integral, rot_deriv;

// Inner control parameters - Position
float desired_inst_velocity=2.5;
float Kp_inner_velocity = 1;
float velocity_error;

// Outer control parameters - Position
float Kp_outer_pos = 1.3514;
float Ki_outer_pos = .883;
float Kd_outer_pos = 0.001;
float pos_error, pos_error_prev;
float desired_pos = 0;
float pos_integral, pos_deriv;

// Localization stuff
float r = 0.0746125; // Wheel radius in meters
float b = 0.375; // Wheelbase in meters
float t=0.0, x = 0.0, y = 0.0, phi = 0.0;

float rad_pos[] = {0, 0};
float actual_pos[] = {0, 0};
float actual_speed[] = {0, 0};

float delta_voltage = 0;
float voltage_a = 0;
float voltage[2] = {0, 0};

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
}

void loop() {
  actual_pos[0] = M1Enc.read(); // Gets actual position in counts for left motor
  actual_pos[1] = M2Enc.read(); // Gets actual position in counts for right motor

  if(millis() % ts == 0){
    // Calculate position in radians for left and right motors (used for finding pos_error)
    rad_pos[0] = 2 * PI * (float)actual_pos[0] / counts_per_rev;
    rad_pos[1] = 2 * PI * (float)actual_pos[1] / counts_per_rev;

    
    // Speed calculations
    actual_speed[0] = (rad_pos[0] - old_pos_rad[0]) / ((float)ts / 1000.0); // Calculates the current speed of left motor
    actual_speed[1] = (rad_pos[1] - old_pos_rad[1]) / ((float)ts / 1000.0); // Calculates the current speed of right motor

    // Calculate instantaneous forward velocity
    float inst_velocity = r*(actual_speed[0]+actual_speed[1])/2; 

    // Calculate angle
    float rot_velocity = r*(actual_speed[0]-actual_speed[1])/b;
    float phi = r*(rad_pos[0]-rad_pos[1])/b;

    // Outer control loop - Angle
    rot_error = desired_angle - phi; // radians
    rot_integral += rot_error * ((float)ts/1000.0);
    rot_deriv = (rot_error - rot_error_prev) / ((float)ts/1000.0);
    desired_rot_angle = Kp_outer_angle*rot_error + Ki_outer_angle*rot_integral + Kd_outer_angle*rot_deriv;

    // Inner control loop - Angular Velocity
    angle_error = desired_rot_angle - rot_velocity;
    delta_voltage = Kp_inner_angle * angle_error;

    // Inner control loop - Velocity
    velocity_error = desired_inst_velocity - inst_velocity;
    voltage_a = Kp_inner_velocity * velocity_error;

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
      Serial.print("Angle Error: ");
      Serial.print(angle_error);
      Serial.print(",");
      Serial.print("Velocity Error: ");
      Serial.print(velocity_error);
      Serial.print(",");
      Serial.print("\r\n");
    }
    // Update old position in rads and counts for next time
    for (int i = 0; i < 2; i++){
      old_pos_rad[i] = rad_pos[i];
      old_pos_count[i] = actual_pos[i];
    }

    rot_error_prev = rot_error;
  }
}
