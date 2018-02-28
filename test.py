import serial
from sys import version_info
import time
import maestro

def main():
    pololu = maestro.Controller()
    # cmd = chr(0xFF) + chr(chan) + chr(lsb) + chr(msb)
    # pololu.sendCmd(cmd)
    # pololu.setTarget(0, 0)
    # for i in range(-256,256):
    #     print i
    #     pololu.setTarget(0, i)
    #     time.sleep(0.01)
    # usb = serial.Serial('/dev/ttyACM0')
    # PololuCmd = chr(0xaa) + chr(0x0c)
    # Targets = [0] * 24
    # Mins = [0] * 24
    # Maxs = [0] * 24
    # for i in range(0,256):
    #     cmdStr = PololuCmd + chr(0xFF) + chr(0) + chr(i)
    #     usb.write(cmdStr)
    #     time.sleep(0.01)

    # 0x84, channel number, target low bits, target high bits
    # usb.close()

if __name__ == '__main__':
        main()
