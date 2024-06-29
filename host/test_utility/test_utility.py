#!/usr/bin/env python3

import serial
import struct
import time
import threading

CMD_PING = 0x0001
LEN_PING = 2

CMD_THROTTLE = 0x0200
LEN_THROTTLE = 4 * 4 + 1

MSG_ACK = 0x0100
MSG_NAK = 0x0101

CRC = 0xCACB

HIP_SYMBOL_B = b'b'
HIP_SYMBOL_M = b'm'

ackAwait = -1
ackRx = -1

pingSeq = 0
pingRx = -1

def throttle_set(val, port):
    global ackRx, ackAwait

    throttles = [val, val, val, val]
    flags = 0x0f << 1 | 0x01

    cmd = CMD_THROTTLE
    len = LEN_THROTTLE

    throttle = struct.pack('<2sHHB4fH', b'mb', cmd, len, flags, throttles[0], \
            throttles[1], throttles[2],  throttles[3],  CRC)
    port.write(throttle)

    ackAwait = CMD_THROTTLE

    time.sleep(0.1)

    if ackRx != ackAwait:
        print("WRN: throttle set not acked")

    ackRx = ackAwait = -1

def throttle_enable(en, port):
    global ackRx, ackAwait

    throttles = [0.0, 0.0, 0.0, 0.0]
    flags = en

    cmd = CMD_THROTTLE
    len = LEN_THROTTLE

    throttle = struct.pack('<2sHHB4fH', b'mb', cmd, len, flags, throttles[0], \
            throttles[1], throttles[2],  throttles[3],  CRC)
    port.write(throttle)

    ackAwait = CMD_THROTTLE

    time.sleep(0.1)

    if ackRx != ackAwait:
        print("WRN: throttle enable not acked")

    ackRx = ackAwait = -1


def handle_ping(payload):
    global pingRx
    cmd = -1
    [pingRx, ] = struct.unpack('<H', b''.join(payload))


def handle_acknak(ack, payload):
    global ackRx
    cmd = -1
    [cmd, ] = struct.unpack('<H', b''.join(payload))

    if ack == 0:
        print(cmd, " NAKed")
    else:
        ackRx = cmd


def pinger_function(name, port):
    global pingSeq
    while True:
        cmd = CMD_PING
        len = LEN_PING

        ping = struct.pack('<2sHHHH', b'mb', cmd, len, pingSeq, CRC)
        port.write(ping)

        time.sleep(0.1)

        if pingRx != pingSeq:
            print("WRN: no ping")

        pingSeq = pingSeq + 1

        time.sleep(1)


def console_function(name, port):
    while True:
        command = input("> ").split()
        if command[0] == 't':
            if command[1] == 'e':
                throttle_enable(True, port)
            elif command[1] == 'd':
                throttle_enable(False, port)
            else:
                throttle_set(int(command[1]) / 100.0, port)


def listener_function(name, port):
    ProtoState_m = 0
    ProtoState_b = 1
    ProtoState_len0 = 2
    ProtoState_len1 = 3
    ProtoState_cmd0 = 4
    ProtoState_cmd1 = 5
    ProtoState_payload = 6
    ProtoState_crc0 = 7
    ProtoState_crc1 = 8

    state = ProtoState_m
    cmd = -1
    pllen = 0
    payload = []
    rxCrc = -1

    while True:
        c = port.read(1)
        if c == b'':
            continue

        if state == ProtoState_b:
            if c == HIP_SYMBOL_B:
                state = ProtoState_cmd0
            else:
                state = ProtoState_m

        elif state == ProtoState_cmd0:
            cmd = int.from_bytes(c, "little")
            state = ProtoState_cmd1

        elif state == ProtoState_cmd1:
            cmd = cmd + int.from_bytes(c, "little") * 256
            state = ProtoState_len0

        elif state == ProtoState_len0:
            pllen = int.from_bytes(c, "little")
            state = ProtoState_len1

        elif state == ProtoState_len1:
            pllen = pllen + int.from_bytes(c, "little") * 256
            state = ProtoState_payload

        elif state == ProtoState_payload:
            payload.append(c)
            if len(payload) == pllen:
                state = ProtoState_crc0

        elif state == ProtoState_crc0:
            rxCrc = int.from_bytes(c, "little")
            state = ProtoState_crc1;

        elif state == ProtoState_crc1:
            rxCrc = rxCrc + int.from_bytes(c, "little") * 256

            if rxCrc == CRC:
                if cmd == CMD_PING:
                    handle_ping(payload)
                elif cmd == MSG_ACK:
                    handle_acknak(1, payload)
                elif cmd == MSG_NAK:
                    handle_acknak(0, payload)

            state = ProtoState_m

        elif state == ProtoState_m:
            if c == HIP_SYMBOL_M:
                cmd = -1
                pllen = 0
                payload = []
                rxCrc = -1
                state = ProtoState_b

def main():
    print("Hello boss!")

    port = serial.Serial(port="/dev/ttyUSB0", baudrate=115200, timeout = 0.12)

    pinger = threading.Thread(target=pinger_function, args=("pinger", port,))
    pinger.start()

    listener = threading.Thread(target=listener_function, args=("listener", port,))
    listener.start()

    console = threading.Thread(target=console_function, args=("console", port,))
    console.start()

if __name__ == '__main__':
    main()
