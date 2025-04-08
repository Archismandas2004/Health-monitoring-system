from machine import SoftI2C, Pin, ADC, UART
from utime import sleep
from max30102 import MAX30102, MAX30105_PULSE_AMP_HIGH
from micropyGPS import MicropyGPS
import ssd1306
# Configure the ADC (Analog-to-Digital Converter) pin for vibration sensor
vibration_sensor_pin = ADC(Pin(34))  # Use an ADC-capable pin (e.g., GPIO 34)
vibration_sensor_pin.atten(ADC.ATTN_11DB)  # Set the attenuation for a 0-3.3V range
vibration_sensor_pin.width(ADC.WIDTH_12BIT)  # Set the resolution to 12 bits (0-4095)
# I2C for OLED
i2c_oled = SoftI2C(scl=Pin(25), sda=Pin(26))
oled_width, oled_height = 128, 64
oled = ssd1306.SSD1306_I2C(oled_width, oled_height, i2c_oled)

# I2C setup for MPU6050
mpu_i2c = SoftI2C(sda=Pin(19), scl=Pin(18), freq=400000)

# I2C setup for MAX30102
max_i2c = SoftI2C(sda=Pin(22), scl=Pin(21), freq=400000)

# UART setup for GPS
gps_serial = UART(2, baudrate=9600, tx=17, rx=16)
my_gps = MicropyGPS()

# Initialize MAX30102
sensor = MAX30102(i2c=max_i2c)

# MPU6050 Registers
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

# Wake up MPU6050
try:
    mpu_i2c.writeto_mem(MPU6050_ADDR, PWR_MGMT_1, b'\x00')
except OSError:
    print("Failed to initialize MPU6050. Check I2C connection.")

# Function to read raw data from MPU6050
def read_raw_data(addr):
    try:
        high = mpu_i2c.readfrom_mem(MPU6050_ADDR, addr, 1)[0]
        low = mpu_i2c.readfrom_mem(MPU6050_ADDR, addr + 1, 1)[0]
        value = (high << 8) | low
        if value > 32768:
            value -= 65536
        return value
    except OSError:
        print("Failed to read data from MPU6050.")
        return 0

# Function to smooth the signal
def smooth_signal(data, window_size=5):
    """Smooth the signal using a moving average filter."""
    if len(data) < window_size:
        return data  # Return original data if insufficient length
    smoothed = []
    for i in range(len(data) - window_size + 1):
        smoothed.append(sum(data[i:i + window_size]) / window_size)
    return smoothed

# Function to calculate pulse rate and SpO2
def calculate_pulse_and_spo2(ir_data, red_data, sampling_rate):
    ir_smoothed = smooth_signal(ir_data)
    red_smoothed = smooth_signal(red_data)

    # Dynamic threshold for peak detection
    threshold = max(ir_smoothed) * 0.2 if ir_smoothed else 0
    peaks = []
    for i in range(1, len(ir_smoothed) - 1):
        if ir_smoothed[i] > threshold and ir_smoothed[i] > ir_smoothed[i - 1] and ir_smoothed[i] > ir_smoothed[i + 1]:
            peaks.append(i)

    if len(peaks) > 1:
        intervals = [peaks[i + 1] - peaks[i] for i in range(len(peaks) - 1)]
        avg_interval = sum(intervals) / len(intervals)
        pulse_rate = (sampling_rate * 60) / avg_interval
    else:
        pulse_rate = 0

    # SpO2 Calculation
    ir_ac = max(ir_data) - min(ir_data) if ir_data else 0
    red_ac = max(red_data) - min(red_data) if red_data else 0
    ir_dc = sum(ir_data) / len(ir_data) if ir_data else 0
    red_dc = sum(red_data) / len(red_data) if red_data else 0

    if ir_dc > 0 and red_dc > 0:
        ratio = (red_ac / red_dc) / (ir_ac / ir_dc)
        spo2 = 110 - 25 * ratio
    else:
        spo2 = 0

    return pulse_rate, spo2

def is_valid_fix(gps):
    """Check if GPS has a valid fix based on satellites and HDOP."""
    return gps.satellites_in_use > 0 and gps.hdop < 20

def detect_fall():
    """Check gyroscope and vibration sensor data to detect a fall."""
    gyro_x = read_raw_data(GYRO_XOUT_H) / 131.0
    gyro_y = read_raw_data(GYRO_YOUT_H) / 131.0
    gyro_z = read_raw_data(GYRO_ZOUT_H) / 131.0
    vibration_value = vibration_sensor_pin.read()

    # Check if gyroscope angles exceed thresholds and vibration is above threshold
    if abs(gyro_x) > 180 or abs(gyro_y) > 180 or abs(gyro_z) > 200:
        if vibration_value > 3000:
            return True
    return False

def main():
    # Scan I2C bus to ensure that the sensors are connected
    if sensor.i2c_address not in max_i2c.scan():
        print("MAX30102 Sensor not found.")
        return

    print("Sensors connected and recognized.")

    # Sensor setup for MAX30102
    try:
        sensor.setup_sensor()
        sensor.set_sample_rate(400)
        sensor.set_fifo_average(8)
        sensor.set_active_leds_amplitude(MAX30105_PULSE_AMP_HIGH)
    except Exception as e:
        print(f"Error initializing MAX30102: {e}")
        return

    sleep(1)

    print("Starting data acquisition...", '\n')

    sampling_rate = 40
    sample_count = 200  # Number of samples to collect per cycle

    ir_data = []
    red_data = []
    pulse_values = []  # To store 10 pulse rate values
    averages = []  # To store averages of the 10 values

    while True:
        # Detect fall
        if detect_fall():
            print("Fall detected!")
            while gps_serial.any():
                data = gps_serial.read()
                for byte in data:
                    my_gps.update(chr(byte))
                    if is_valid_fix(my_gps):
                        latitude = my_gps.latitude_string()
                        longitude = my_gps.longitude_string()
                        print(f"Fall detected at location: https://www.google.com/maps?q={latitude},{longitude}")
                        break

        # Read vibration sensor value
        vibration_value = vibration_sensor_pin.read()
        if vibration_value > 3000:
            print("Significant vibration detected!")
                
        # MAX30102 Pulse Rate and SpO2 calculations
        try:
            sensor.check()
            if sensor.available():
                ir_data.append(sensor.pop_ir_from_storage())
                red_data.append(sensor.pop_red_from_storage())
        except Exception as e:
            print(f"Error reading MAX30102 data: {e}")

        if len(ir_data) >= sample_count:
            pulse_rate, spo2 = calculate_pulse_and_spo2(ir_data, red_data, sampling_rate)
            if pulse_rate > 0:
                print(f"Pulse Rate: {pulse_rate:.2f} BPM, SpO2: {spo2:.2f}%")
                #oled.text('{spo2:.2f}%',0,10)
                #oled.show()
                #sleep(5)
                #oled.fill(0)
                pulse_values.append(pulse_rate)

                if len(pulse_values) == 10:
                    avg = sum(pulse_values) / len(pulse_values)
                    averages.append(avg)
                    print(f"Average of 10 values: {avg:.2f} BPM")
                    pulse_values.clear()

                    if len(averages) == 2:
                        final_average = sum(averages) / len(averages)
                        print(f"Final Average Heart Rate: {final_average:.2f} BPM")
                        #oled.text('{final_average:.2f} BPM',0,0)
                        #oled.show()
                        #sleep(5)
                        #oled.fill(0)
                        averages.clear()
            else:
                print("Unable to detect Pulse Rate. Ensure proper finger placement.")

            ir_data.clear()
            red_data.clear()

if __name__ == '__main__':
    main()
