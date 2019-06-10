from __future__ import print_function

import serial
import sys
import termios
import time

from serial_range_report import RangeReport


class TagHandler:
    """
    This handles communication with a connected UWB tag.
    Reading is blocking
    """

    def __init__(self, port, usb_unplugged_recovery=False):
        """
        :param port: USB port name. Windows example: COM4. Linux example: /dev/ttyACM0
        :param usb_unplugged_recovery: Whether the program should wait for the device to be plugged back.
        """
        self.serial_port = serial.Serial()
        self.serial_port.port = port
        self.serial_port.baudrate = termios.B115200
        self.serial_port.bytesize = serial.EIGHTBITS
        self.serial_port.parity = serial.PARITY_NONE
        self.serial_port.stopbits = serial.STOPBITS_ONE
        self.serial_port.timeout = 0
        self.usb_unplugged_recovery = usb_unplugged_recovery
        self.info_logger = print
        self.error_logger = lambda message: print(message, file=sys.stderr)

    def is_open(self):
        return self.serial_port.is_open

    def open(self):
        try:
            self.serial_port.open()
        except serial.SerialException as e:
            self.error_logger("Could not find Decawave tag on port " + str(self.serial_port.port))
            raise e

    def close(self):
        self.serial_port.close()

    def usb_recover(self):
        """
        Waits until the tag is plugged back into its USB port
        """
        self.close()
        time.sleep(1)
        try:
            self.open()
            self.serial_port.read(65)
        except serial.SerialException:
            self.usb_recover()

    def read_report(self):
        line = ""
        while len(line) != 65:
            try:
                line = self.serial_port.read(65)
            except serial.SerialException:
                self.error_logger("Device seems to be unplugged from " + str(self.serial_port.port))
                if self.usb_unplugged_recovery:
                    self.error_logger("USB unplugged recovery mode started..")
                    self.usb_recover()
                else:
                    raise serial.SerialException("")
        return RangeReport(line)
