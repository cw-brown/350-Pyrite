#include <Wire.h>
#define MY_ADDR 8
// Global variables to be used for I2C communication

volatile uint8_t instruction[32] = {0};
//volatile uint8_t instruction= 0;

volatile uint8_t msgLength = 0;

void setup() {
  Serial.begin(115200); // set the baud rate of the serial monitor
  // We want to control the built-in LED (pin 13)
  pinMode(LED_BUILTIN, OUTPUT);
  // Initialize I2C
  Wire.begin(MY_ADDR);
  // Set callbacks for I2C interrupts
  Wire.onReceive(receive);
}
void loop() {
  // If there is data on the buffer, read it
  if (msgLength > 0) {
    printReceived();
    msgLength = 0;
  }
}

// printReceived helps us see what data we are getting from the leader
void printReceived() {
  // Print on serial console
//  Serial.print(instruction);

  for (int i=0;i<msgLength;i++) {
    Serial.print("     ");
    Serial.print(char(instruction[i]));
//      Serial.print(instruction[i]);
    Serial.print("\t\r\n");
    
  }
  Serial.println("");
}

// function called when an I2C interrupt event happens
void receive() {
  // Set the offset, this will always be the first byte.

  // If there is information after the offset, it is telling us more about the command.
  while (Wire.available()) {
    instruction[msgLength] = Wire.read();
//    instruction = Wire.read();
    msgLength++;
  }
}
