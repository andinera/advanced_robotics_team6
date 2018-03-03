#!/usr/bin/env python

import rospy
import serial

# Pololu driver
class Controller:

    # Pass component channel as an integer in the range [0,17]
    # Example call
    # cont = pololu.Controller(0)
    # This example will initialize the Controller with channel 0.
    def __init__(channel):
        usb = serial.Serial('/dev/ttyACM0')
        self.channel = channel

    # Pass servo target as an integer in the range [4095, 7905]
    # Center value is 6000
    # Example call
    # cont.set_target(6000)
    # This example will set the servo target to 6000.
    def set_target(self, target):
        low = int("{0:08b}".format(target)[-7:], base=2)
        high = int("{0:08b}".format(target)[0:-7], base=2)
        command = chr(0x84)+chr(self.channel)+chr(low)+chr(high)
        usb.write(command)

    # Pass first channel and servo targets
    # First channel must be an integer in the range [0, 17]
    # Targets must be a list or tuple with each value in the range [4095, 7905]
    # Example call
    # pololu.Controller.set_multiple_targets(3, 5000, 6000, 7000)
    # This example will set servo on channel 3 to 5000, servo on channel 4 to
    # 6000, and servo on channel 5 to 7000.
    @staticmethod
    def set_multiple_targets(first_channel, targets):
        num_of_targets = len(targets)
        command = chr(0x9F)+chr(num_of_targets)+chr(first_channel)
        print command
        for target in targets:
            low = int("{0:08b}".format(target)[-7:], base=2)
            high = int("{0:08b}".format(target)[0:-7], base=2)
            command = command+chr(channel)+chr(low)+chr(high)
        usb.write(command)

    # Pass servo speed as an integer in the range [0, TBD]
    # Example call
    # cont.set_speed(0)
    # This example will set the servo speed to 0.
    def set_speed(self, speed):
        low = int("{0:08b}".format(speed)[-7:], base=2)
        high = int("{0:08b}".format(speed)[0:-7], base=2)
        command = chr(0x87)+chr(self.channel)+chr(low)+chr(high)
        usb.write(command)

    # Pass servo acceleration as an integer in the range [0, TBD]
    # Example call
    # cont.set_acceleration(0)
    # This example will set the servo acceleration to 0.
    def set_acceleration(self, acceleration):
        low = int("{0:08b}".format(acceleration)[-7:], base=2)
        high = int("{0:08b}".format(acceleration)[0:-7], base=2)
        command = chr(0x89)+chr(self.channel)+chr(low)+chr(high)
        usb.write(command)

    # Pass PWM on time and period as integers in range [0, TBD]
    # Example call
    # pololu.Controller.set_PWM(1, 2)
    # This example will set the PWM on time to 1 and period to 2.
    @staticmethod
    def set_PWM(on_time, period):
        on_time_low = int("{0:08b}".format(on_time)[-7:], base=2)
        on_time_high = int("{0:08b}".format(on_time)[0:-7], base=2)
        period_low = int("{0:08b}".format(period)[-7:], base=2)
        period_high = int("{0:08b}".format(period)[0:-7], base=2)
        command = chr(0x8A)+chr(on_time_low)+chr(on_time_high)+chr(period_low) \
                  +chr(period_high)
        usb.write(command)

    # Example call
    # cont.get_position()
    # This example will return the position of the servo an an integer in the
    # range [4095, 7905].
    def get_position(self):
        usb.write(chr(0x90)+chr(self.channel))
        least = ord(usb.read())
        most = ord(usb.read())
        least = "{0:08b}".format(least)
        most = "{0:08b}".format(most)
        position = "{}{}".format(most, least)
        position = int(position, base=2)
        return position

    # Example call
    # pololu.Controller.get_moving_state()
    # This example will return the whether or not any servo has not achieved its
    # target.
    # 1 = At least one servo has not achieved its target state
    # 0 = All servos have achieved their target states
    @staticmethod
    def get_moving_state():
        usb.write(chr(0x93))
        state = ord(usb.read())
        return state

    # Example call
    # pololu.Controller.get_errors()
    # This example will return any errors present on the pololu as an integer
    # in the range [0, TBD] and clear all the errors.
    @staticmethod
    def get_errors():
        usb.write(chr(0xA1))
        least = ord(usb.read())
        most = ord(usb.read())
        least = "{0:08b}".format(least)
        most = "{0:08b}".format(most)
        position = "{}{}".format(most, least)
        position = int(position, base=2)
        return position

    # Example call
    # pololu.Controller.go_home()
    # This example will send all servos and outputs to their home positions.
    # Home positions can be set via Maestro Control Center.
    @staticmethod
    def go_home():
        usb.write(chr(0xA2))
