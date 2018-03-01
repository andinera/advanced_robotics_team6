import serial
import time
import maestro

def main():
    usb = serial.Serial('/dev/ttyACM0')
    for i in range(255):
        usb.write(chr(0xFF)+chr(0x00)+chr(i))
        print i
        usb.write(chr(0x90)+chr(0x00))
        print ord(usb.read())
        print ord(usb.read())
        print
        time.sleep(0.01)
    for i in range(254, -1, -1):
        usb.write(chr(0xFF)+chr(0x00)+chr(i))
        print i
        usb.write(chr(0x90)+chr(0x00))
        print ord(usb.read())
        print ord(usb.read())
        print
        time.sleep(0.01)
    usb.close()

if __name__ == '__main__':
    main()
