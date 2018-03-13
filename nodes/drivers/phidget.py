#!/usr/bin/env python

class Controller:

    def __init__(self):
        self.angles = [0]

    # Required for using in conjunction with a "with" statement
    def __enter__(self):
        return self

    # Required for using in conjunction with a "with" statement
    def __exit__(self, type, value, traceback):
        pass

    # This will close the usb port upon deleting this object
    def __del__(self):
        pass
