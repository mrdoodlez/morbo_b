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

import matplotlib
matplotlib.use('Qt5Agg')
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

CMD_WM = 0x0400
LEN_WM = 1 + 1

CMD_RP = 0x0500
LEN_RP = 1

CMD_SET_PID = 0x0600
LEN_SET_PID = (3 + 3 * 3) * 4

MSG_ACK = 0x0100
MSG_NAK = 0x0101

MSG_PAT = 0x0A00
MSG_ACC = 0x0A01
MSG_MFX = 0x0A02
MSG_DST = 0x0A03
MSG_STB = 0x0A04

MSG_CAL_ACC = 0x0B01
MSG_CAL_GYRO = 0x0B02

MSG_MON = 0x0C00

CRC = 0xCACB

HIP_SYMBOL_B = b'b'
HIP_SYMBOL_M = b'm'

ackAwait = -1
ackRx = -1

pingSeq = 0
pingRx = -1

doExit = False

################################################################################

time_window = 100  # Number of points to keep in the plot
t_data = list(range(-time_window, 0))

acr_x, acr_y, acr_z = [0] * time_window, [0] * time_window, [0] * time_window
acc_x, acc_y, acc_z = [0] * time_window, [0] * time_window, [0] * time_window
acw_x, acw_y, acw_z = [0] * time_window, [0] * time_window, [0] * time_window
acl_x, acl_y, acl_z = [0] * time_window, [0] * time_window, [0] * time_window

pos_x, pos_y, pos_z = [0] * time_window, [0] * time_window, [0] * time_window
roll, pitch, yaw = [0] * time_window, [0] * time_window, [0] * time_window
roll_rate, pitch_rate, yaw_rate = [0] * time_window, [0] * time_window, [0] * time_window
pid_roll, pid_pitch, pid_yaw = [0] * time_window, [0] * time_window, [0] * time_window

thrust1, thrust2, thrust3, thrust4 = [0] * time_window, [0] * time_window, [0] * time_window, [0] * time_window

################################################################################

acc_scale = [0] * (3 * 3)
acc_bias = [0] * 3

################################################################################

fLog = None

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
    global acr_x, acr_y, acr_z
    global acc_x, acc_y, acc_z
    global acw_x, acw_y, acw_z

    acc_data = struct.unpack('<12f', b''.join(payload))

    acr_x.append(acc_data[0])
    acr_y.append(acc_data[1])
    acr_z.append(acc_data[2])

    acc_x.append(acc_data[3])
    acc_y.append(acc_data[4])
    acc_z.append(acc_data[5])

    acl_x.append(acc_data[6])
    acl_y.append(acc_data[7])
    acl_z.append(acc_data[8])

    acw_x.append(acc_data[9])
    acw_y.append(acc_data[10])
    acw_z.append(acc_data[11])

    print(acc_data)

def handle_cal_acc(payload):
    global acc_scale, acc_bias
    cal_data = struct.unpack('<12fB', b''.join(payload))

    print('[', cal_data[0], cal_data[4], cal_data[8], '] ', \
        ' [', cal_data[9], cal_data[10], cal_data[11], ']')

def handle_pat(payload):
    global yaw, pitch, roll
    global pos_x, pos_y, pos_z
    pat_data = struct.unpack('<7f', b''.join(payload))

    pos_x.append(pat_data[0])
    pos_y.append(pat_data[1])
    pos_z.append(pat_data[2])

    roll.append(pat_data[3])
    pitch.append(pat_data[4])
    yaw.append(pat_data[5])

    time = pat_data[6]

    print(time, "(", pos_x[-1], pos_y[-1], pos_z[-1], ")", "(", roll[-1], pitch[-1], yaw[-1], ")")

def handle_mfx(payload):
    mfx_data = struct.unpack('<9f', b''.join(payload))

    print(mfx_data[3], mfx_data[4], mfx_data[5], mfx_data[6], mfx_data[7], mfx_data[8])

def handle_mon(payload):
    global fLog
    mon_data = struct.unpack('<6f', b''.join(payload))
    print(mon_data)

    log_str = ' '.join(f'{v:.6f}' for v in mon_data)
    fLog.write("MON: " + log_str + "\n")

def handle_stb(payload):
    global yaw, pitch, roll
    global yaw_rate, pitch_rate, roll_rate
    global pid_roll, pid_pitch, pid_yaw
    global thrust1, thrust2, thrust3, thrust4

    stb_data = struct.unpack('<13f', b''.join(payload))

    roll.append(stb_data[0])
    pitch.append(stb_data[1])
    yaw.append(stb_data[2])

    roll_rate.append(stb_data[3])
    pitch_rate.append(stb_data[4])
    yaw_rate.append(stb_data[5])

    pid_roll.append(stb_data[6])
    pid_pitch.append(stb_data[7])
    pid_yaw.append(stb_data[8])

    thrust1.append(stb_data[9])
    thrust2.append(stb_data[10])
    thrust3.append(stb_data[11])
    thrust4.append(stb_data[12])

    global fLog
    log_str = ' '.join(f'{v:.13f}' for v in stb_data)
    fLog.write("STB: " + log_str + "\n")

    #print(stb_data)

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
    elif msgId == "mfx":
        msgId = MSG_MFX
    elif msgId == "mon":
        msgId = MSG_MON
    elif msgId == "stb":
        msgId = MSG_STB
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

def wm_command(wm, fc, port):
    global ackRx, ackAwait

    imuMode = 0
    fsMode = 0

    if wm == "fuse":
        imuMode = 4
    elif wm == "cal":
        imuMode = 1
    else:
        print("imu mode not supported")
        return

    if fc == "dbg":
        fsMode = 1
    if fc == "wu":
        fsMode = 2

    cmd = CMD_WM
    len = LEN_WM

    wm = struct.pack('<2sHHBBH', b'mb', cmd, len, imuMode, fsMode, CRC)
    port.write(wm)

    ackAwait = CMD_WM

    time.sleep(0.5)

    if ackRx != ackAwait:
        print("WRN: command not acked")

    ackRx = ackAwait = -1

def reset_pos_command(port):
    global ackRx, ackAwait

    cmd = CMD_RP
    len = LEN_RP

    dummy = 0

    rp = struct.pack('<2sHHBH', b'mb', cmd, len, dummy, CRC)
    port.write(rp)

    ackAwait = CMD_RP

    time.sleep(0.5)

    if ackRx != ackAwait:
        print("WRN: command not acked")

    ackRx = ackAwait = -1

def set_pid_command(port):

    global ackRx, ackAwait

    cmd = CMD_SET_PID
    len = LEN_SET_PID

    koeffs = [0.0] * 12

    kppr = 0.02
    kpyaw = 0.02

    ki = 0.0

    kdpr = 0.2 * kppr
    kdyaw = 0.2 * kpyaw

    koeffs[3] = koeffs[4] = kppr
    koeffs[5] = kpyaw

    koeffs[6] = koeffs[7] = kdpr
    koeffs[8] = kdyaw

    koeffs[9] = koeffs[10] = ki

    sp = struct.pack('<2sHH12fH', b'mb', cmd, len, *koeffs, CRC)
    port.write(sp)

    ackAwait = CMD_WM

    time.sleep(0.5)

    if ackRx != ackAwait:
        print("WRN: command not acked")

    ackRx = ackAwait = -1

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
        command = input("> ")
        if command != "":
            command = command.split()
            if command[0] == 't':
                if command[1] == 'e':
                    throttle_enable(True, port)
                elif command[1] == 'd':
                    throttle_enable(False, port)
                else:
                    throttle_set(int(command[1]) / 100.0, port)
            elif command[0] == "em":
                em_command(command[1], command[2], port)
            elif command[0] == "wm":
                wm_command(command[1], command[2], port)
            elif command[0] == "rp":
                reset_pos_command(port)
            elif command[0] == "sp":
                set_pid_command(port)
            elif command[0] == 'q':
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
                elif cmd == MSG_MFX:
                    handle_mfx(payload)
                elif cmd == MSG_MON:
                    handle_mon(payload)
                elif cmd == MSG_STB:
                    handle_stb(payload)
                else:
                    print("unknown message: ", cmd)

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

        glTranslatef(-pos_x, -pos_y, pos_z)

        glRotatef(-pos_yaw, 0.0, 0.0, 1.0)  # Rotate around Z-axis
        glRotatef(-pos_roll, 1.0, 0.0, 0.0)  # Rotate around X-axis
        glRotatef(-pos_pitch, 0.0, 1.0, 0.0)  # Rotate around Y-axis

        # Draw the STL model
        draw_model(faces, normals)

        glPopMatrix()

        traj.append((-pos_x, -pos_y, pos_z))
        # draw_trajectory(traj)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

ani_pose = None
ani_acc = None
ani_stb = None

fig_acc = None
axs_acc = None

lines_acr_acc = None
lines_acc_acc = None
lines_acw_acc = None
lines_acl_acc = None

fig_pose = None
axs_pose = None
lines_pos = None
lines_rpy = None

fig_stb = None
axs_stb = None
lines_attitude_stb = None
lines_thrusts_stb = None
lines_rates_stb = None
lines_pid_stb = None

def init_acc_plot():
    global fig_acc, axs_acc, lines_acr_acc, lines_acc_acc, lines_acw_acc, lines_acl_acc

    fig_acc, axs_acc = plt.subplots(2, 3, figsize=(12, 6))
    titles = ['X', 'Y', 'Z']

    lines_acr_acc, lines_acc_acc = [], []
    lines_acw_acc, lines_acl_acc = [], []

    for i in range(3):
        ax = axs_acc[0][i]
        line1, = ax.plot([], [], 'r-', label='Raw')
        line2, = ax.plot([], [], 'b-', label='Calibrated')
        lines_acr_acc.append(line1)
        lines_acc_acc.append(line2)
        ax.set_ylim(-10, 10)
        ax.set_title(f'Raw/Calibrated {titles[i]}')
        ax.legend()

    for i in range(3):
        ax = axs_acc[1][i]
        line1, = ax.plot([], [], 'g-', label='World')
        line2, = ax.plot([], [], 'm-', label='Body')
        lines_acw_acc.append(line1)
        lines_acl_acc.append(line2)
        ax.set_ylim(-10, 10)
        ax.set_title(f'World/Body {titles[i]}')
        ax.legend()

def init_pose_plot():
    global fig_pose, axs_pose, lines_pos, lines_rpy

    fig_pose, axs_pose = plt.subplots(2, 3, figsize=(12, 6))
    lines_pos, lines_rpy = [], []

    for i, name in enumerate(['X', 'Y', 'Z']):
        ax = axs_pose[0][i]
        line, = ax.plot([], [], 'c-', label='Pos')
        lines_pos.append(line)
        ax.set_ylim(-5, 5)
        ax.set_title(f'Position {name}')
        ax.legend()

    for i, name in enumerate(['Roll', 'Pitch', 'Yaw']):
        ax = axs_pose[1][i]
        line, = ax.plot([], [], 'y-', label='Rad')
        lines_rpy.append(line)
        ax.set_ylim(-4, 4)
        ax.set_title(name)
        ax.legend()

def init_stb_plot():
    global fig_stb, axs_stb, lines_attitude_stb, lines_rates_stb, lines_thrusts_stb, lines_pid_stb

    fig_stb, axs_stb = plt.subplots(5, 3, figsize=(12, 14))
    lines_attitude_stb = []
    lines_rates_stb = []
    lines_pid_stb = []
    lines_thrusts_stb = [None] * 4  # One line per motor

    # Row 0: roll, pitch, yaw vs time
    for i, (title, color, data) in enumerate([
        ("Roll (rad)", 'r', roll),
        ("Pitch (rad)", 'g', pitch),
        ("Yaw (rad)", 'b', yaw)
    ]):
        ax = axs_stb[0][i]
        line, = ax.plot([], [], color + '-', label=title)
        lines_attitude_stb.append(line)
        ax.set_ylabel("rad")
        ax.set_title(title)
        ax.set_xlim(0, time_window)

        # Set individual y-limits
        if title.startswith("Yaw"):
            ax.set_ylim(-4, 4)
        else:
            ax.set_ylim(-0.2, 0.2)

        ax.legend()

    for i, (title, color, data) in enumerate([
        ("ω Roll (rad/s)", 'r', roll_rate),
        ("ω Pitch (rad/s)", 'g', pitch_rate),
        ("ω Yaw (rad/s)", 'b', yaw_rate)
    ]):
        ax = axs_stb[1][i]
        line, = ax.plot([], [], color + '--', label=title)
        lines_rates_stb.append(line)
        ax.set_ylabel("rad/s")
        ax.set_title(title)
        ax.set_xlim(0, time_window)
        ax.set_ylim(-2.0, 2.0)  # or adjust as needed
        ax.legend()

    for i, (title, color, data) in enumerate([
        ("PID Roll", 'r', pid_roll),
        ("PID Pitch", 'g', pid_pitch),
        ("PID Yaw", 'b', pid_yaw)
    ]):
        ax = axs_stb[2][i]
        line, = ax.plot([], [], color + ':', label=title)
        lines_pid_stb.append(line)
        ax.set_ylabel("τ (Nm)")  # or other units
        ax.set_title(title)
        ax.set_xlim(0, time_window)
        ax.set_ylim(-0.5, 0.5)  # adjust as needed
        ax.legend()

    # Row 2–3: thrusts
    thrust_titles = ['Thrust #2', 'Thrust #4', 'Thrust #1', 'Thrust #3']
    thrust_axes = [(3, 0), (3, 1), (4, 0), (4, 1)]
    thrust_colors = ['m', 'c', 'y', 'k']
    for i, ((row, col), title, color) in enumerate(zip(thrust_axes, thrust_titles, thrust_colors)):
        ax = axs_stb[row][col]
        line, = ax.plot([], [], color + '-', label=title)
        lines_thrusts_stb[i] = line
        ax.set_ylabel("N")
        ax.set_title(title)
        ax.set_xlim(0, time_window)
        ax.set_ylim(0, 5)
        ax.legend()

    # Hide unused subplots
    axs_stb[3][2].axis('off')
    axs_stb[4][2].axis('off')

def update_acc(frame):
    global ani_acc, doExit

    if doExit and ani_acc is not None:
        ani_acc.event_source.stop()
        plt.close(fig_acc)
        return []

    length = len(acr_x)
    if length < 2:
        return lines_acr_acc + lines_acc_acc + lines_acw_acc + lines_acl_acc

    start = max(0, length - time_window)
    x_vals = list(range(length - start))

    # Raw + calibrated
    for i, (raw, cal) in enumerate(zip(
        [acr_x, acr_y, acr_z],
        [acc_x, acc_y, acc_z]
    )):
        lines_acr_acc[i].set_data(x_vals, raw[start:])
        lines_acc_acc[i].set_data(x_vals, cal[start:])
        axs_acc[0][i].set_xlim(0, len(x_vals))

    # World-frame + linear
    for i, (world, linear) in enumerate(zip(
        [acw_x, acw_y, acw_z],
        [acl_x, acl_y, acl_z]
    )):
        lines_acw_acc[i].set_data(x_vals, world[start:])
        lines_acl_acc[i].set_data(x_vals, linear[start:])
        axs_acc[1][i].set_xlim(0, len(x_vals))

    return lines_acr_acc + lines_acc_acc + lines_acw_acc + lines_acl_acc

def update_pose(frame):
    global ani_pose, doExit

    if doExit and ani_pose is not None:
        ani_pose.event_source.stop()
        plt.close(fig_pose)
        return []

    length = len(pos_x)
    if length < 2:
        return lines_pos + lines_rpy

    start = max(0, length - time_window)
    x_vals = list(range(length - start))

    for i, data in enumerate([pos_x, pos_y, pos_z]):
        lines_pos[i].set_data(x_vals, data[start:])
        axs_pose[0][i].set_xlim(0, len(x_vals))

    for i, data in enumerate([roll, pitch, yaw]):
        lines_rpy[i].set_data(x_vals, data[start:])
        axs_pose[1][i].set_xlim(0, len(x_vals))

    return lines_pos + lines_rpy

def update_stb(frame):
    global ani_stb, doExit

    if doExit and ani_stb is not None:
        ani_stb.event_source.stop()
        plt.close(fig_stb)
        return []

    length = len(roll)
    if length < 2:
        return lines_attitude_stb + lines_thrusts_stb

    start = max(0, length - time_window)
    x_vals = list(range(length - start))

    # Update roll, pitch, yaw
    for line, data in zip(lines_attitude_stb, [roll, pitch, yaw]):
        line.set_data(x_vals, data[start:])

    # Update angular rates
    for line, data in zip(lines_rates_stb, [roll_rate, pitch_rate, yaw_rate]):
        line.set_data(x_vals, data[start:])

    # Update PID outputs
    for line, data in zip(lines_pid_stb, [pid_roll, pid_pitch, pid_yaw]):
        line.set_data(x_vals, data[start:])

    # Update thrusts
    for line, data in zip(lines_thrusts_stb, [thrust2, thrust4, thrust1, thrust3]):
        line.set_data(x_vals, data[start:])

    for ax_row in axs_stb:
        for ax in ax_row:
            if ax.has_data():
                ax.set_xlim(0, len(x_vals))

    return lines_attitude_stb + lines_rates_stb + lines_pid_stb + lines_thrusts_stb

################################################################################

def main():
    print("Hello boss!")

    global fLog, ani_acc, ani_pose, ani_stb
    global doExit
    fLog = open('log.txt', 'w')

    port = BLEPort()

    mode = input("mode? ")

    #port = serial.Serial(port="/dev/ttyUSB0", baudrate=115200, timeout = 0.12)

    pinger = threading.Thread(target=pinger_function, args=("pinger", port.service,))
    pinger.start()

    listener = threading.Thread(target=listener_function, args=("listener", port.service,))
    listener.start()

    console = threading.Thread(target=console_function, args=("console", port.service,))
    console.start()

    if mode == "flight":
        visio = threading.Thread(target=visio_flight_function, args=("visio",))
        visio.start()
    elif mode == "plot acc":
        init_acc_plot()
        ani_acc = animation.FuncAnimation(fig_acc, update_acc, interval=50, blit=False)
        plt.tight_layout()
        plt.show()
    elif mode == "plot pat":
        init_pose_plot()
        ani_pose = animation.FuncAnimation(fig_pose, update_pose, interval=50, blit=False)
        plt.tight_layout()
        plt.show()
    elif mode == "plot stb":
        init_stb_plot()
        ani_stb = animation.FuncAnimation(fig_stb, update_stb, interval=50, blit=False)
        plt.tight_layout()
        plt.show()
    else:
        doExit = True

if __name__ == '__main__':
    main()