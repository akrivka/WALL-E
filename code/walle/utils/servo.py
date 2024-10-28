import board
from adafruit_pca9685 import PCA9685
import time
import numpy as np

class MultiServo:
    
    PWM_MAX = 8000
    PWM_MIN = 1200
    
    def __init__(self, init_angles=[0]):
        i2c = board.I2C()
        pca = PCA9685(i2c)
        pca.frequency = 50
        self.pca = pca
        self.t = time.time()
        self.angles = init_angles
       
    @property 
    def pwms(self):
        return [(self.PWM_MAX - self.PWM_MIN) / 180 * angle + self.PWM_MIN for angle in self.angles]
    
    def cmd_angles(self, channels=None):
        pwms_to_cmd = self.pwms
        if channels is None: channels = list(range(len(pwms_to_cmd)))
        for ch in channels:
            self.pca.channels[ch].duty_cycle = int(pwms_to_cmd[ch])
            
    def kill(self):
        for ch in range(16):
            self.pca.channels[ch].duty_cycle = 0
            
if __name__ == '__main__':
    ms = MultiServo()
    start = time.time()
    while True:
        cmd = int(input('cmd: '))
        time.sleep(0.05)
        # ms.angles = [90 * np.sin(time.time() - start) + 90]
        # ms.angles = [90 + cmd, 90 - cmd]
        ms.angles = [90]*7
        ms.angles[3] = cmd
        ms.cmd_angles()
        
# make it horizontal, then go to 110 (range ends up being 90-110)


#left, right
#[60,90], [90,120]