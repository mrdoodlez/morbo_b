#!/usr/bin/env python3

import serial
import struct
import time
import threading
import sys

from typing import Optional, TYPE_CHECKING
from circuitpython_typing import WriteableBuffer, ReadableBuffer
import _bleio

from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services import Service
from adafruit_ble.uuid import VendorUUID
from adafruit_ble.advertising import Advertisement
from adafruit_ble.characteristics import Characteristic
from adafruit_ble.characteristics.stream import StreamOut, StreamIn

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np
from stl import mesh

import matplotlib.pyplot as plt
import matplotlib.animation as animation

################################################################################

class MService(Service):
    uuid = VendorUUID("0000ffe0-0000-1000-8000-00805f9b34fb")

    _server_tx = StreamOut(
        uuid=VendorUUID("0000ffe1-0000-1000-8000-00805f9b34fb"),
        timeout=0.1,
        buffer_size=64,
    )
    _server_rx = StreamIn(
        uuid=VendorUUID("0000ffe1-0000-1000-8000-00805f9b34fb"),
        timeout=5,
        buffer_size=64,
    )

    def __init__(self, service: Optional[_bleio.Service] = None) -> None:
        super().__init__(service=service)
        self.connectable = True
        if not service:
            self._rx = self._server_rx
            self._tx = self._server_tx
        else:
            # If we're a client then swap the characteristics we use.
            self._tx = self._server_rx
            self._rx = self._server_tx

    def read(self, nbytes: Optional[int] = None) -> Optional[bytes]:
        """
        Read characters. If ``nbytes`` is specified then read at most that many bytes.
        Otherwise, read everything that arrives until the connection times out.
        Providing the number of bytes expected is highly recommended because it will be faster.

        :return: Data read
        :rtype: bytes or None
        """
        return self._rx.read(nbytes)

    def readinto(
        self, buf: WriteableBuffer, nbytes: Optional[int] = None
    ) -> Optional[int]:
        """
        Read bytes into the ``buf``. If ``nbytes`` is specified then read at most
        that many bytes. Otherwise, read at most ``len(buf)`` bytes.

        :return: number of bytes read and stored into ``buf``
        :rtype: int or None (on a non-blocking error)
        """
        return self._rx.readinto(buf, nbytes)

    def write(self, buf: ReadableBuffer) -> None:
        """Write a buffer of bytes."""
        self._tx.write(buf)


################################################################################

class BLEPort():
    def __init__(self):
        self.radio = BLERadio()

        print("scanning....")

        found = set()
        scan_responses = set()
        # By providing Advertisement as well we include everything, not just specific advertisements.
        for advertisement in self.radio.start_scan(ProvideServicesAdvertisement, Advertisement):
            addr = advertisement.address
            if advertisement.scan_response and addr not in scan_responses:
                scan_responses.add(addr)
            elif not advertisement.scan_response and addr not in found:
                found.add(addr)
            else:
                continue
            print(addr, advertisement)
            print("\t" + repr(advertisement))
            print()

            if advertisement.complete_name == "DSD TECH":
                if MService in advertisement.services:
                    print("DSD service found!")
                    self.connection = self.radio.connect(advertisement)
                    break

        print("scan done")

        if self.connection and self.connection.connected:
            print("connected")
            self.service = self.connection[MService]
        else:
            sys.exit(app.exec_())


################################################################################

CMD_PING = 0x0001
LEN_PING = 2

CMD_THROTTLE = 0x0200
LEN_THROTTLE = 4 * 4 + 1

CMD_EM = 0x0300
LEN_EM = 2 + 2

MSG_ACK = 0x0100
MSG_NAK = 0x0101

MSG_PAT = 0x0A00
MSG_ACC = 0x0B00
MSG_CAL_ACC = 0x0B01

CRC = 0xCACB

HIP_SYMBOL_B = b'b'
HIP_SYMBOL_M = b'm'

ackAwait = -1
ackRx = -1

pingSeq = 0
pingRx = -1

doExit = False

################################################################################

# copter position

pos_x = 0.0
pos_y = 0.0
pos_z = 0.0

pos_roll = 0.0
pos_pitch = 0.0
pos_yaw = 0.0

################################################################################

time_window = 100  # Number of points to keep in the plot
t_data = list(range(-time_window, 0))
raw_x, raw_y, raw_z = [0] * time_window, [0] * time_window, [0] * time_window
cal_x, cal_y, cal_z = [0] * time_window, [0] * time_window, [0] * time_window

################################################################################

acc_scale = [0] * (3 * 3)
acc_bias = [0] * 3

################################################################################

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

    time.sleep(0.5)

    if ackRx != ackAwait:
        print("WRN: command not acked")

    ackRx = ackAwait = -1


def handle_ping(payload):
    global pingRx
    [pingRx, ] = struct.unpack('<H', b''.join(payload))
    #print("ping", pingRx)


def handle_acknak(ack, payload):
    global ackRx
    cmd = -1
    [cmd, ] = struct.unpack('<H', b''.join(payload))

    if ack == 0:
        print(cmd, " NAKed")
    else:
        ackRx = cmd

def handle_acc(payload):
    global raw_x, raw_y, raw_z, cal_x, cal_y, cal_z
    acc_data = struct.unpack('<6f', b''.join(payload))

    raw_x.append(acc_data[0])
    raw_y.append(acc_data[1])
    raw_z.append(acc_data[2])

    cal_x.append(acc_data[3])
    cal_y.append(acc_data[4])
    cal_z.append(acc_data[5])

    print(acc_data)

def handle_cal_acc(payload):
    global acc_scale, acc_bias
    cal_data = struct.unpack('<12fB', b''.join(payload))

    print('[', cal_data[0], cal_data[4], cal_data[8], '] ', \
        ' [', cal_data[9], cal_data[10], cal_data[11], ']')

def handle_pat(payload):
    global pos_yaw, pos_pitch, pos_roll
    global pos_x, pos_y, pos_z
    pat_data = struct.unpack('<7f', b''.join(payload))
    pos_x = pat_data[0]
    pos_y = pat_data[1]
    pos_z = 0 # pat_data[2]
    pos_yaw = pat_data[3]
    pos_pitch = pat_data[4]
    pos_roll = pat_data[5]
    time = pat_data[6]

    print(time, "(", pos_x, pos_y, pos_z, ")", "(", pos_yaw, pos_pitch, pos_roll, ")")

def em_command(msgId, msgPeriod, port):
    global ackRx, ackAwait

    msgPeriod = int(msgPeriod)
    msgPeriod = int(msgPeriod / 100)

    if msgId == "acc":
        msgId = MSG_ACC
    elif msgId == "ac":
        msgId = MSG_CAL_ACC
    elif msgId == "pat":
        msgId = MSG_PAT
    else:
        print("command not supported")
        return

    cmd = CMD_EM
    len = LEN_EM

    em = struct.pack('<2sHHHHH', b'mb', cmd, len, msgId, msgPeriod, CRC)
    port.write(em)

    ackAwait = CMD_EM

    time.sleep(0.5)

    if ackRx != ackAwait:
        print("WRN: command not acked")

    ackRx = ackAwait = -1

    pass

def pinger_function(name, port):
    global pingSeq
    global doExit
    while not doExit:
        cmd = CMD_PING
        len = LEN_PING

        ping = struct.pack('<2sHHHH', b'mb', cmd, len, pingSeq, CRC)
        port.write(ping)

        time.sleep(0.5)

        if pingRx != pingSeq:
            print("WRN: no ping")

        pingSeq = pingSeq + 1

        time.sleep(1)


def console_function(name, port):
    global doExit
    while not doExit:
        command = input("> ").split()
        if command[0] == 't':
            if command[1] == 'e':
                throttle_enable(True, port)
            elif command[1] == 'd':
                throttle_enable(False, port)
            else:
                throttle_set(int(command[1]) / 100.0, port)
        elif command[0] == "em":
            em_command(command[1], command[2], port)
        if command[0] == 'q':
            doExit = True

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

    global doExit

    while not doExit:
        c = port.read(1)
        if c == b'':
            continue

        if c is None:
            state = ProtoState_m
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
            state = ProtoState_crc1

        elif state == ProtoState_crc1:
            rxCrc = rxCrc + int.from_bytes(c, "little") * 256

            if rxCrc == CRC:
                if cmd == CMD_PING:
                    handle_ping(payload)
                elif cmd == MSG_ACK:
                    handle_acknak(1, payload)
                elif cmd == MSG_NAK:
                    handle_acknak(0, payload)
                elif cmd == MSG_ACC:
                    handle_acc(payload)
                elif cmd == MSG_CAL_ACC:
                    handle_cal_acc(payload)
                elif cmd == MSG_PAT:
                    handle_pat(payload)

            state = ProtoState_m

        elif state == ProtoState_m:
            if c == HIP_SYMBOL_M:
                cmd = -1
                pllen = 0
                payload = []
                rxCrc = -1
                state = ProtoState_b

################################################################################

def load_stl(filename):
    # Load the STL file
    model = mesh.Mesh.from_file(filename)
    model.update_normals()
    return model.vectors, model.normals

def draw_model(vertices, normals):
    glBegin(GL_TRIANGLES)
    i = 0
    for f in vertices:
        glNormal3fv(normals[i])
        glColor3f(0, 0, 1)
        for vertex in f:
            glVertex3fv(vertex)
        i += 1
    glEnd()

def draw_xy_plane():
    """Draw the fixed XY plane grid."""
    glColor3fv((0.5, 0.5, 0.5))  # Set color for the grid lines (light gray)
    glBegin(GL_LINES)

    # Draw grid lines along the X axis (from -10 to 10)
    for x in range(-10, 11):
        glVertex3fv((x, -10, 0))
        glVertex3fv((x, 10, 0))

    # Draw grid lines along the Y axis (from -10 to 10)
    for y in range(-10, 11):
        glVertex3fv((-10, y, 0))
        glVertex3fv((10, y, 0))
    glEnd()

def draw_trajectory(traj):
    glLineWidth(2)  # Set line width for trajectory
    glColor3fv((1, 1, 1))  # White color for the trajectory
    glBegin(GL_LINE_STRIP)  # Draw a continuous line for the trajectory
    for point in traj:
        glVertex3fv(point)  # Draw each point in the trajectory
    glEnd()

def setup_lighting():
    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    glLightfv(GL_LIGHT0, GL_POSITION, [-0.1, 0, 2.0, 1.0])  # Light position
    glLightfv(GL_LIGHT0, GL_AMBIENT, [0.3, 0.3, 0.3, 1.0])   # Softer ambient light
    glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.7, 0.7, 0.7, 1.0])   # Softer diffuse light
    glLightfv(GL_LIGHT0, GL_SPECULAR, [0.5, 0.5, 0.5, 1.0])  # Softer specular light

    # Material properties for soft reflections
    glMaterialfv(GL_FRONT, GL_AMBIENT, [0.3, 0.3, 0.3, 1.0])  # Soft ambient reflection
    glMaterialfv(GL_FRONT, GL_DIFFUSE, [0.6, 0.6, 0.6, 1.0])  # Softer diffuse reflection
    glMaterialfv(GL_FRONT, GL_SPECULAR, [0.5, 0.5, 0.5, 1.0]) # Softer specular reflections
    glMaterialf(GL_FRONT, GL_SHININESS, 10.0)                 # Low shininess for a softer shine

def visio_flight_function(name):
    pygame.init()
    display = (1024, 768)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)

    # Perspective settings
    gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)

    gluLookAt(
        -1, -1,  2,  # Camera position
        0.0, 0.0, 0.0,  # Look at the origin
        0.0, 0.0, 1.0   # Up direction
    )

    glEnable(GL_DEPTH_TEST)  # Enable depth testing
    setup_lighting()         # Set up lighting

    # Load the STL model
    faces, normals = load_stl('copter_model-Body.stl')  # Replace with your STL file path

    for i in range(0, len(faces)):
        faces[i] /= 1000

    clock = pygame.time.Clock()

    traj = []

    global pos_yaw
    global pos_pitch
    global pos_roll

    global pos_x
    global pos_y
    global pos_z

    global doExit

    while not doExit:
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        draw_xy_plane()

        glPushMatrix()

        glTranslatef(pos_x, pos_y, pos_z)

        glRotatef(pos_yaw, 0.0, 0.0, 1.0)  # Rotate around Z-axis
        glRotatef(pos_roll, 1.0, 0.0, 0.0)  # Rotate around X-axis
        glRotatef(pos_pitch, 0.0, 1.0, 0.0)  # Rotate around Y-axis

        # Draw the STL model
        draw_model(faces, normals)

        glPopMatrix()

        traj.append((pos_x, pos_y, pos_z))
        draw_trajectory(traj)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

raw_lines = []
cal_lines = []

fig = None

def update_plot(frame):
    """Fetches data and updates the plot"""
    global raw_x, raw_y, raw_z, cal_x, cal_y, cal_z
    global doExit
    global fig

    if doExit:
        print("Stopping animation...")
        plt.close(fig)
        return

    # Keep only the last 'time_window' points
    raw_x, raw_y, raw_z = raw_x[-time_window:], raw_y[-time_window:], raw_z[-time_window:]
    cal_x, cal_y, cal_z = cal_x[-time_window:], cal_y[-time_window:], cal_z[-time_window:]

    # Update plots
    raw_lines[0].set_ydata(raw_x)
    raw_lines[1].set_ydata(raw_y)
    raw_lines[2].set_ydata(raw_z)
    cal_lines[0].set_ydata(cal_x)
    cal_lines[1].set_ydata(cal_y)
    cal_lines[2].set_ydata(cal_z)

    return raw_lines + cal_lines


def visio_calib_function(name):

    global fig

    # Create figure and axis
    fig, ax = plt.subplots(3, 1, figsize=(8, 6), sharex=True)

    # Set labels
    ax[0].set_ylabel("X Acceleration (g)")
    ax[1].set_ylabel("Y Acceleration (g)")
    ax[2].set_ylabel("Z Acceleration (g)")
    ax[2].set_xlabel("Time (arbitrary units)")

    global raw_lines, cal_lines
    raw_lines = [ax[i].plot(t_data, [0] * time_window, label="Raw", color='red')[0] for i in range(3)]
    cal_lines = [ax[i].plot(t_data, [0] * time_window, label="Calibrated", color='blue')[0] for i in range(3)]

    for i in range(3):
        ax[i].set_ylim(-1.5, 1.5)  # Fixed scale for better comparison
        ax[i].legend(loc="upper right")
        ax[i].grid(True)

    # Set legends
    for i in range(3):
        ax[i].legend(loc="upper right")
        ax[i].grid(True)

    # Set up animation
    ani = animation.FuncAnimation(fig, update_plot, interval=100, blit=True)

    plt.show()

    pass

################################################################################

def main():
    print("Hello boss!")

    port = BLEPort()

    mode = input("mode? ").split()

    #port = serial.Serial(port="/dev/ttyUSB0", baudrate=115200, timeout = 0.12)

    pinger = threading.Thread(target=pinger_function, args=("pinger", port.service,))
    pinger.start()

    listener = threading.Thread(target=listener_function, args=("listener", port.service,))
    listener.start()

    console = threading.Thread(target=console_function, args=("console", port.service,))
    console.start()

    if mode[0] == "flight":
        visio = threading.Thread(target=visio_flight_function, args=("visio",))
    elif mode[0] == "cal":
        visio = threading.Thread(target=visio_calib_function, args=("visio",))

    visio.start()

if __name__ == '__main__':
    main()
