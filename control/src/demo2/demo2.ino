#include <Arduino.h> // for PI
#include <Encoder.h> // for reading wheel encoders
#include <Wire.h> // for communicating with Rasberry Pi
#include <PID_v1.h>

// Pin Definitions
const uint8_t M_ENABLE = 4; // So motors are on
const uint8_t M_DIR[] = {7, 8}; // Motor to reverse and forward spin (index 0 is left motor, index 1 is right motor)
const uint8_t M_PWM[] = {9, 10}; //Motor driver pins (index 0 is left motor, index 1 is right motor)
const uint8_t M1_ENC_PINS[] = {2,5}; // Motor 1 encoder
const uint8_t M2_ENC_PINS[] = {3,6}; // Motor 2 encoder

// Set up encoder library and wheel rotation counts
Encoder M1Enc(M1_ENC_PINS[0], M1_ENC_PINS[1]); // index 0 is ISR pin
Encoder M2Enc(M2_ENC_PINS[0], M2_ENC_PINS[1]); // index 0 is ISR pin
const int counts_per_rev = 3200; // Encoder counts per wheel revolution
long int encoder_counts[2] = {0, 0};
long unsigned int ts = 15;
double r = 0.0746125; // Wheel radius in meters
double b = 0.375; // Wheelbase in meters
double rad_pos[2] = {0, 0};
double rad_pos_prev[2] = {0, 0};
double wheel_velocity[2] = {0, 0};

// PWM variables
double pwm[2] = {0, 0};
double battery_voltage = 7.5;

// Velocity Inner Control Loop
double velocity_error, velocity;
double desired_velocity;
PID velocity_control(&velocity, &velocity_error, &desired_velocity, 1, 0, 0, DIRECT);

// Position Outer Control Loop
double pos_error, pos;
double desired_position = 0;
double Kp_pos = 9.5;
double Ki_pos = 0.044;
double Kd_pos = 4.45;
PID pos_control(&pos, &desired_velocity, &desired_position, Kp_pos, Ki_pos, Kd_pos, DIRECT);

// Angular Velocity Inner Control Loop
double desired_ang_velocity, ang_velocity, ang_velocity_error;
PID ang_velocity_control(&ang_velocity, &ang_velocity_error, &desired_ang_velocity, 1, 0, 0, DIRECT);

// Angular Position Outer Control Loop
double desired_angle = 2*PI;
double phi;
double Kp_ang = 12;
double Ki_ang = 19.26;
double Kd_ang = 9.23;
PID ang_pos_control(&phi, &desired_ang_velocity, &desired_angle, Kp_ang, Ki_ang, Kd_ang, DIRECT);

const double addr = 8;
volatile uint8_t received_value = 0;

void setup() {
  Serial.begin(115200);

  Wire.begin(MY_ADDR);  // Initialize as I2C slave
  Wire.onReceive(receiveData); 

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

  ang_velocity_control.SetMode(AUTOMATIC);
  ang_velocity_control.SetSampleTime(ts);
  ang_velocity_control.SetOutputLimits(-10000, 10000);

  ang_pos_control.SetMode(AUTOMATIC);
  ang_pos_control.SetSampleTime(ts);
  ang_pos_control.SetOutputLimits(-10000, 10000);

  velocity_control.SetMode(AUTOMATIC);
  velocity_control.SetSampleTime(ts);
  velocity_control.SetOutputLimits(-10000, 10000);

  pos_control.SetMode(AUTOMATIC);
  pos_control.SetSampleTime(ts);
  pos_control.SetOutputLimits(-10000, 10000);
}

void loop() {
  encoder_counts[0] = M1Enc.read();
  encoder_counts[1] = M2Enc.read();

  rad_pos[0] = 2*PI*encoder_counts[0]/counts_per_rev;
  rad_pos[1] = 2*PI*encoder_counts[1]/counts_per_rev;

  if(millis() % ts == 0){
    wheel_velocity[0] = (rad_pos[0]-rad_pos_prev[0])/((double)ts/1000.0);
    wheel_velocity[1] = (rad_pos[1]-rad_pos_prev[1])/((double)ts/1000.0);

    rad_pos_prev[0] = rad_pos[0]; 
    rad_pos_prev[1] = rad_pos[1]; 

    velocity = r*(wheel_velocity[0]+wheel_velocity[1])/2;
    pos += velocity*((double)ts/1000.0)/10;
    
    ang_velocity = r*(wheel_velocity[0]-wheel_velocity[1])/b;
    phi = r*(rad_pos[0]-rad_pos[1])/b;
  }

  ang_pos_control.Compute();
  ang_velocity_control.Compute();

  pos_control.Compute();
  velocity_control.Compute();

  double delta_voltage = ang_velocity_error;
  double voltage_a = velocity_error;

  double voltage[2] = {(voltage_a+delta_voltage)/2, (voltage_a-delta_voltage)/2};
  
  for (int i = 0; i < 2; i++){
    if(voltage[i] > 0){
      digitalWrite(M_DIR[i], HIGH);
    }
    else{
      digitalWrite(M_DIR[i], LOW);
    }
    pwm[i] = 255 * max(abs(voltage[i]),0.01) / battery_voltage;
    analogWrite(M_PWM[i], min(pwm[i], 255));
  }

  if(millis() % 250 == 0){
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
    Serial.print(velocity);
    Serial.print(",");
    Serial.print("Position: ");
    Serial.print(pos);
    Serial.print(",");
    Serial.print("Angle Error: ");
    Serial.print(ang_velocity_error);
    Serial.print(",");
    Serial.print("Velocity Error: ");
    Serial.print(velocity_error);
    Serial.print("\r\n");
  }
}

void receiveData(int numBytes) {

}
