#!/usr/bin/env python3

import os
import select
import sys
import termios
import tty
import socket
import serial
import struct
import time

CMD_PING = 0x0001
LEN_PING = 2

CRC = 0xCACB

def main():
    print("Hello boss!")
    port = serialPort = serial.Serial(port="/dev/ttyUSB0", baudrate=9600, timeout = 0.1)
    seq = 0
    while True:
        cmd = CMD_PING
        len = LEN_PING
        
        ping = struct.pack('<2sHHHH', b'mb', cmd, len, seq, CRC) 
        port.write(ping)

        print("ping #" + str(seq))

        pong = port.read(25)
        print(pong)

        time.sleep(5)

        seq += 1

        

if __name__ == '__main__':
    main()