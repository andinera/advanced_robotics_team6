import serial
import time

# Uses Pololu's Mini SSC protocol
def mini_protocol(usb):
    # Sweep wheels left to right
    for i in range(0, 255):
        # Send set_target command to Pololu
        usb.write(chr(0xFF)+chr(0x00)+chr(i))
        print i
        # Send get_position command to Pololu
        usb.write(chr(0x90)+chr(0x00))
        # Read servo's position from Pololu
        print ord(usb.read())
        print ord(usb.read())
        print
        time.sleep(0.01)
    # Same as above but from right to left
    for i in range(254, -1, -1):
        usb.write(chr(0xFF)+chr(0x00)+chr(i))
        print i
        usb.write(chr(0x90)+chr(0x00))
        print ord(usb.read())
        print ord(usb.read())
        print
        time.sleep(0.01)

# Uses Pololu's Compact protocol
def compact_protocol(usb):
    # Sweep wheels left to right
    for i in range(4095,7906):
        # Convert integer to high and low binary values
        low = int("{0:b}".format(i)[-7:], base=2)
        high = int("{0:b}".format(i)[0:-7], base=2)
        # Send set_target command to Pololu
        usb.write(chr(0x84)+chr(0x00)+chr(low)+chr(high))
        print i
        # Send get_position command to Pololu
        usb.write(chr(0x90)+chr(0x00))
        # Read servo's position from Pololu
        print ord(usb.read())
        print ord(usb.read())
        print
        time.sleep(0.0001)
    # Same as above but from right to left
    for i in range(7905, 4094, -1):
        low = int("{0:b}".format(i)[-7:], base=2)
        high = int("{0:b}".format(i)[0:-7], base=2)
        usb.write(chr(0x84)+chr(0x00)+chr(low)+chr(high))
        print i
        usb.write(chr(0x90)+chr(0x00))
        print ord(usb.read())
        print ord(usb.read())
        print
        time.sleep(0.0001)

# Main function
if __name__ == '__main__':
    # Setup serial connection with Pololu
    usb = serial.Serial('/dev/ttyACM0')

    mini_protocol(usb)      # Simple, less precise protocol
    compact_protocol(usb)   # Complex, more precise protocol

    usb.write(chr(0xA1))    # Get errors
    usb.write(chr(0xA2))    # Go home/clear erros
    usb.close()             # Close connection with Pololu
