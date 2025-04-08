
from mfrc522 import MFRC522
from machine import Pin
from machine import SPI
from machine import Pin, PWM 
spi = SPI(2, baudrate=2500000, polarity=0, phase=0)
# Using Hardware SPI pins:
sck=18   
mosi=23  
miso=19  
rst=4    
cs=5     
spi.init()
rdr = MFRC522(spi=spi, gpioRst=4, gpioCs=5)
from machine import Pin
from time import sleep

from machine import ADC, Pin
from time import sleep

# Configure the ADC pin (use appropriate pin for your board)
adc = ADC(Pin(35))  # GPIO 36 is ADC1 channel 0 on ESP32
adc.atten(ADC.ATTN_11DB)  # Configure for full 3.3V range

def read_co_sensor():
    """
    Reads analog values from the MQ-9 sensor and returns the CO concentration level.
    """
    analog_value = adc.read()  # Read raw ADC value (0-4095)
    voltage = analog_value * (3.3 / 4095)  # Convert to voltage (if 3.3V ADC reference)
    
    # Map or scale the voltage to approximate CO concentration in ppm
    # You will need to calibrate this formula based on the datasheet and your setup.
    co_concentration = voltage * 100  # Example scaling (calibrate for accuracy)
    
    return voltage, co_concentration
# Define the servo control pin
servo_pin = PWM(Pin(15))  # Replace 15 with your GPIO pin
servo_pin.freq(50)        # Standard PWM frequency for servos (50 Hz)

def rotate_motor(duration=2.0):
    """
    Rotates the motor using PWM for a specific duration.
    Args:
        duration (float): Time in seconds to run the motor.
    """
    try:
        print("Motor rotating...")
        servo_pin.duty(100)  # Set PWM duty cycle (adjust for your motor)
        sleep(duration)      # Run motor for the specified duration
        servo_pin.duty(0)    # Stop motor
        print("Motor stopped.")
    except KeyboardInterrupt:
        print("Operation interrupted.")


print("Place card")

while True:
    
    voltage, co_level = read_co_sensor()
    print(f"Analog Voltage: {voltage:.2f} V, Estimated CO Level: {co_level:.2f} ppm")
    sleep(1)  # Read every second
    (stat, tag_type) = rdr.request(rdr.REQIDL)
    if stat == rdr.OK:
        (stat, raw_uid) = rdr.anticoll()
        if stat == rdr.OK:
            card_id = "0x%02x%02x%02x%02x" % (raw_uid[0], raw_uid[1], raw_uid[2], raw_uid[3])
            print(card_id)
            if card_id=='0x43f535da'or card_id=='0x1d743302' :
                print('access accepted')
                rotate_motor(5)  # Rotate for 2 seconds
            else:
                print("access denied")