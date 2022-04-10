#!/usr/bin/env python3

# Script for Keyence SZ-16D with RS-232/422 communication and 270 degrees of range
import serial # warning!! this is a pyserial library, not a Serial library
import binascii
import time


class KeyenceSZ16Dping:

    def __init__(self):
        try:
            # self.obj_port = serial.Serial('/dev/ttyUSB0', 19200, 8, 'N', 1, None)  # open serial port to communicate with the device (linux)
            # self.obj_port = serial.Serial('COM3', 19200, 8, 'N', 1, None)  # open serial port (windows)
            self.obj_port = serial.Serial('/dev/tty.usbserial-1420', 19200, 8, 'N', 1, None)  # open serial port (osx)
        except serial.SerialException:
            print("ERROR: check if already have another device on this port!")
            serial.close()

    def tick(self):
        points = []
        value = []
        upper_level = 0
        try:
            self.obj_port.write(b'\x90\x00\x18\xEB')
        except serial.SerialException:
            # print("ERRO: Verifique se ha algum dispositivo conectado na porta!")
            print("ERROR: check if already have another device on this port!")
        time.sleep(1)
        while self.obj_port.in_waiting > 0:
            try:
                serial_data = self.obj_port.read()  # read a string
                value.append(binascii.hexlify(serial_data).decode('utf-8'))
            except serial.SerialException:
                # print("ERRO: Verifique se ha algum dispositivo conectado na porta!")
                print("ERROR: check if already have another device on this port!")
        list = value[9:-2]
        list_int = [int(item, 16) for item in list]
        for i in range(len(list_int)):
            if i % 2 == 0:
                upper_level = (list_int[i] & 63) << 8
            else:
                lower_level = upper_level + list_int[i]
                points.append(lower_level)
        # convert to float
        points_f = []
        for i in range(len(points)):
            points_f.append(float(points[i]))
        # return a list of points
        # return points # if you want to return a list of points (int)
        return points_f # if you want to return a list of points (float)


if __name__ == '__main__':
    app = KeyenceSZ16Dping()
    while True:
        data = app.tick()
        print(data)
        # print(type(data))
        # print(type(data[0]))
        print(len(data))
        time.sleep(1)
