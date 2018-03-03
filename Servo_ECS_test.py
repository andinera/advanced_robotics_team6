import serial
import time

def mini_Protocol(usb):
    time.sleep(1)
    usb.write(chr(0xFF)+chr(0x01)+chr(125))

    usb.write(chr(0xFF)+chr(0x00)+chr(125))

    time.sleep(2)
    usb.write(chr(0xFF)+chr(0x01)+chr(140))
    turn(220)
    time.sleep(1)
    turn(15)
    time.sleep(1)
    turn(125)
    reverse(90)

def turn(angle):
    usb.write(chr(0xFF)+chr(0x00)+chr(angle))
def stop():
    usb.write(chr(0xFF)+chr(0x01)+chr(50))
    time.sleep(2)
    usb.write(chr(0xFF)+chr(0x01)+chr(125))
    time.sleep(1)
def reverse(speed):
    stop()
    usb.write(chr(0xFF)+chr(0x01)+chr(speed))
    time.sleep(1)

if __name__ == '__main__':
    usb = serial.Serial('/dev/ttyACM0')

    mini_Protocol(usb)      # Simple, less precise protocol


    usb.write(chr(0xA1))
    usb.write(chr(0xA2))
    usb.close()
