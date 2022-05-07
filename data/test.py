
from board import SCL, SDA
import busio
import time
from adafruit_pca9685 import PCA9685

try:
    # Create the I2C bus interface.
    i2c_bus = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
    pca = PCA9685(i2c_bus)

# Set the PWM frequency to 60hz.
    pca.frequency = 60
    # print(dir(pca))
    print(pca.i2c_device)
# Set the PWM duty cycle for channel zero to 50%. duty_cycle is 16 bits to match other PWM objects
# but the PCA9685 will only actually give 12 bits of resolution.
    pca.channels[8].duty_cycle = 65535
    pca.channels[9].duty_cycle = 65535
    pca.channels[13].duty_cycle = 65535
    pca.channels[12].duty_cycle = 65535

    time.sleep(1)
    pca.channels[8].duty_cycle = 0
    pca.channels[9].duty_cycle = 0
    pca.channels[13].duty_cycle = 65535
    pca.channels[12].duty_cycle = 65535

except Exception as e:
    print(e)
