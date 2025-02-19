#Set up connections for PI an Ardiuno
#Plug LCD screen into first 13 of avalible 20 pins
#Connect row closest to op amp on Arduino
#Reading left to right and top to bottom first pin
#is arduino with second being the lcd screen pins connect the following
#pin 1 to 2nd row pin 3
#pin 2 to 2nd row pin 2
#pin 8 or GND to 1st row 3rd pin


import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from smbus2 import SMBus

# I2C Address of Arduino
ARD_ADDR = 8  

# Initialize I2C bus for Arduino communication
i2c = SMBus(1)  

# LCD Configuration
lcd_columns = 16
lcd_rows = 2

# Initialize I2C for LCD
i2c_lcd = board.I2C()  
lcd = character_lcd.Character_LCD_RGB_I2C(i2c_lcd, lcd_columns, lcd_rows)

def display_on_lcd(message, color=[100, 100, 100]):
    """Clears LCD and displays the given message with a specified color."""
    lcd.clear()
    lcd.color = color
    lcd.message = message

while True:
    try:
        # Get user input
        user_input = int(input("Enter an integer between 0 and 100 (or -1 to quit): "))

        if user_input == -1:
            display_on_lcd("Exiting...\nGoodbye!", [100, 0, 0])  # Red
            time.sleep(2)
            lcd.color = (0,0,0)
            lcd.clear()
            break

        if 0 <= user_input <= 100:
            # Send integer to Arduino
            i2c.write_byte(ARD_ADDR, user_input)
            time.sleep(0.1)  # Short delay

            # Request data from Arduino
            received_value = i2c.read_byte(ARD_ADDR)

            # Print and display the response
            print(f"Arduino responded with: {received_value}")
            display_on_lcd(f"Sent: {user_input}\nRecv: {received_value}", [0, 100, 0])  # Green

        else:
            print("Please enter a number between 0 and 100.")
            display_on_lcd("Invalid Input!\nTry Again", [100, 100, 0])  # Yellow

    except Exception as e:
        print(f"Error: {e}")
        display_on_lcd("Error!\nCheck I2C", [100, 0, 0])  # Red
        time.sleep(2)
