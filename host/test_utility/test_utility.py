#!/usr/bin/env python3

import os
import select
import sys
import termios
import tty
import socket
import serial
import struct

def main():
    print("Hello boss!")
    port = serialPort = serial.Serial(port="/dev/ttyUSB0", baudrate=9600)
    while True:
        cnt = 0
        ping = struct.pack('b', cnt)
        cnt += 1
        port.write(ping)

        #pong = port.read()
        #print(pong)
        

if __name__ == '__main__':
    main()