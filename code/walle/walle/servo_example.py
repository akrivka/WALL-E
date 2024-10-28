# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# Outputs a 50% duty cycle PWM single on the 0th channel.
# Connect an LED and resistor in series to the pin
# to visualize duty cycle changes and its impact on brightness.

import board
from adafruit_pca9685 import PCA9685
import time
import numpy as np

# Create the I2C bus interface.
i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = busio.I2C(board.GP1, board.GP0)    # Pi Pico RP2040

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)

# Set the PWM frequency to 60hz.
pca.frequency = 50

# Set the PWM duty cycle for channel zero to 50%. duty_cycle is 16 bits to match other PWM objects
# but the PCA9685 will only actually give 12 bits of resolution.
# for pwm in range(0, 0xFFFF, 100):
#     print(pwm, end='\r')
#     pca.channels[0].duty_cycle = pwm #0x7FFF
#     time.sleep(0.01)
start = time.time()
while True:
    # pwm = int(input('Enter PWM: '))
    pca.channels[3].duty_cycle = int(3400 * np.sin(3*(time.time() - start)) + 4600)


# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# """Simple test for a standard servo on channel 0 and a continuous rotation servo on channel 1."""
# import time
# from adafruit_servokit import ServoKit
# import numpy as np
# # Set channels to the number of servo channels on your kit.
# # 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
# kit = ServoKit(channels=16)
# start = time.time()
# while True:
#     angle = int(180 * np.sin(time.time() - start) + 180)
#     print(angle)
#     kit.servo[0].angle = angle
#     time.sleep(0.01)
# kit.servo[0].angle = 180
# time.sleep(1)
# kit.servo[0].angle = 0
# time.sleep(1)
