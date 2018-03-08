#!/usr/bin/env python

import serial

########### Example call file ###########
# from __future__ import with_statement
# from pololu import Controller
#
# if __name__ == '__main__':
#     with Controller(0) as cont:
#         cont.set_target(6000)
#         Controller.go_home()
########################################

# Pololu driver
class Controller:

    usb = serial.Serial('/dev/ttyACM0')

    # Pass component channel as an integer in the range [0,17] and serial port
    # as a string
    # Example call
    # cont = Controller(0)
    # This example will initialize the Controller with channel 0 and the serial
    # connection to '/dev/ttyACM0'.
    def __init__(self, channel=0, usb='/dev/ttyACM0'):
        Controller.usb = serial.Serial(usb)
        self.channel = channel
        self.target = 6000

    # Required for using in conjunction with a "with" statement
    def __enter__(self):
        return self

    # Required for using in conjunction with a "with" statement
    def __exit__(self, type, value, traceback):
        pass

    # This will close the usb port upon deleting this object
    def __del__(self):
        if self.target != 6000:
            self.set_target(6000)

    # Pass servo target as an integer in the range [4095, 7905]
    # Center value is 6000
    # Example call
    # cont.set_target(6000)
    # This example will set the servo target to 6000.
    def set_target(self, target):
        self.target = target
        low = int("{0:08b}".format(target)[-7:], base=2)
        high = int("{0:08b}".format(target)[0:-7], base=2)
        command = chr(0x84)+chr(self.channel)+chr(low)+chr(high)
        Controller.usb.write(command)

    # Pass first channel and servo targets
    # First channel must be an integer in the range [0, 17]
    # Targets must be a list or tuple with each value in the range [4095, 7905]
    # Example call
    # Controller.set_multiple_targets(3, 5000, 6000, 7000)
    # or
    # cont.set_multiple_targets(3, 5000, 6000, 7000)
    # This example will set servo on channel 3 to 5000, servo on channel 4 to
    # 6000, and servo on channel 5 to 7000.
    @staticmethod
    def set_multiple_targets(first_channel, targets):
        num_of_targets = len(targets)
        command = chr(0x9F)+chr(num_of_targets)+chr(first_channel)
        for target in targets:
            low = int("{0:08b}".format(target)[-7:], base=2)
            high = int("{0:08b}".format(target)[0:-7], base=2)
            command = command+chr(low)+chr(high)
        Controller.usb.write(command)

    # Pass servo speed as an integer in the range [0, TBD]
    # Example call
    # cont.set_speed(0)
    # This example will set the servo speed to 0.
    def set_speed(self, speed=0):
        low = int("{0:08b}".format(speed)[-7:], base=2)
        high = int("{0:08b}".format(speed)[0:-7], base=2)
        command = chr(0x87)+chr(self.channel)+chr(low)+chr(high)
        Controller.usb.write(command)

    # Pass servo acceleration as an integer in the range [0, TBD]
    # Example call
    # cont.set_acceleration(0)
    # This example will set the servo acceleration to 0.
    def set_acceleration(self, acceleration=0):
        low = int("{0:08b}".format(acceleration)[-7:], base=2)
        high = int("{0:08b}".format(acceleration)[0:-7], base=2)
        command = chr(0x89)+chr(self.channel)+chr(low)+chr(high)
        Controller.usb.write(command)

    # Pass PWM on time and period as integers in range [0, TBD]
    # Example call
    # Controller.set_PWM(1, 2)
    #   or
    # cont.set_PWM(1, 2)
    # This example will set the PWM on time to 1 and period to 2.
    @staticmethod
    def set_PWM(on_time, period):
        on_time_low = int("{0:08b}".format(on_time)[-7:], base=2)
        on_time_high = int("{0:08b}".format(on_time)[0:-7], base=2)
        period_low = int("{0:08b}".format(period)[-7:], base=2)
        period_high = int("{0:08b}".format(period)[0:-7], base=2)
        command = chr(0x8A)+chr(on_time_low)+chr(on_time_high)+chr(period_low) \
                  +chr(period_high)
        Controller.usb.write(command)

    # Example call
    # cont.get_position()
    # This example will return the position of the servo an an integer in the
    # range [4095, 7905].
    def get_position(self):
        Controller.usb.write(chr(0x90)+chr(self.channel))
        least = ord(Controller.usb.read())
        most = ord(Controller.usb.read())
        least = "{0:08b}".format(least)
        most = "{0:08b}".format(most)
        pos = "{}{}".format(most, least)
        pos = int(pos, base=2)
        pos = 1/float(pos)
        pos = 3212000000*pos**3 - 17229000*pos**2 + 129560*pos - 117.11
        # pos = 1000/float(pos) - 1
        # pos = 0.1674*pos**3 - 0.6824*pos**2 + 1.8431*pos - 0.3559
        return pos

    # Example call
    # Controller.get_moving_state()
    #   or
    # cont.get_moving_state()
    # This example will return whether or not any servo has not achieved its
    # target.
    # 1 = At least one servo has not achieved its target state
    # 0 = All servos have achieved their target states
    @staticmethod
    def get_moving_state():
        Controller.usb.write(chr(0x93))
        state = ord(Controller.usb.read())
        return state

    # Example call
    # Controller.get_errors()
    #   or
    # cont.get_errors()
    # This example will return any errors present on the pololu as an integer
    # in the range [0, TBD] and clear all the errors.
    @staticmethod
    def get_errors():
        Controller.usb.write(chr(0xA1))
        least = ord(Controller.usb.read())
        most = ord(Controller.usb.read())
        least = "{0:08b}".format(least)
        most = "{0:08b}".format(most)
        position = "{}{}".format(most, least)
        position = int(position, base=2)
        return position

    # Example call
    # Controller.go_home()
    #   or
    # cont.go_home()
    # This example will send all servos and outputs to their home positions.
    # Home positions can be set via Maestro Control Center.
    @staticmethod
    def go_home():
        command = chr(0xA2)
        Controller.usb.write(command)

    # Example call
    # Controller.stop_script()
    #   or
    # cont.stop_script()
    # This example will stop a script if one is running.
    @staticmethod
    def stop_script():
        command = chr(0xA4)
        Controller.usb.write(command)

    # Pass subroutine as an integer
    # Example call
    # Controller.restart_script_at_subroutine(0)
    #   or
    # cont.restart_script_at_subroutine(0)
    # This example will start the preprogrammed script at subroutine 0.
    @staticmethod
    def restart_script_at_subroutine(subroutine=0):
        command = chr(0xA7)+chr(subroutine)
        Controller.usb.write(command)

    # Pass subroutine as an integer and parameter as an integer
    # Example call
    # Controller.restart_script_at_subroutine_with_parameter(0, 5)
    #   or
    # cont.restart_script_at_subroutine_with_parameter(0, 5)
    # This example will place a 5 on top of the Pololu's stack and start the
    # preprogrammed script at subroutine 0.
    @staticmethod
    def restart_script_at_subroutine_with_parameter(subroutine, parameter):
        low = int("{0:08b}".format(parameter)[-7:], base=2)
        high = int("{0:08b}".format(parameter)[0:-7], base=2)
        command = chr(0xA8)+chr(subroutine)+chr(low)+chr(high)
        Controller.usb.write(command)

    # Example call
    # Controller.get_script_status()
    #   or
    # cont.get_script_status()
    # This example will return whether or not a script is running.
    # 1 = No script is running
    # 0 = The preprogrammed script is running
    @staticmethod
    def get_script_status():
        command = chr(0xAE)
        Controller.usb.write(command)
        status = ord(Controller.usb.read())
        return status
