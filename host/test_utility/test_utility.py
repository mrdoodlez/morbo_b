#!/usr/bin/env python3

import serial
import struct
import time
import threading
import sys
import math
import socket

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
# TCP constants
################################################################################

TCP_HOST = "morbob.local"
TCP_PORT = 5555

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
# BLE port wrapper with read()/write() like serial / TCP
################################################################################

class BLEPort:
    def __init__(self):
        self.radio = BLERadio()

        print("scanning....")

        self.connection = None
        self.service = None

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
            # original code had something like sys.exit(app.exec_()), keep simple
            print("BLE connection failed")
            sys.exit(1)

    # unified API
    def read(self, nbytes: Optional[int] = None) -> Optional[bytes]:
        return self.service.read(nbytes)

    def write(self, buf: ReadableBuffer) -> None:
        self.service.write(buf)


################################################################################
# TCP client port
################################################################################

class TcpPort:
    def __init__(self, host: str = TCP_HOST, port: int = TCP_PORT):
        self.host = host
        self.port = port
        print(f"Connecting TCP to {host}:{port} ...")
        self.sock = socket.create_connection((host, port))
        # Blocking mode, but we can add timeouts later if needed
        self.sock.settimeout(None)
        print("TCP connected")

    def read(self, nbytes: Optional[int] = None) -> Optional[bytes]:
        if nbytes is None:
            nbytes = 1
        try:
            data = self.sock.recv(nbytes)
            if not data:
                # connection closed
                return b''
            return data
        except socket.timeout:
            return b''

    def write(self, buf: bytes) -> None:
        # We don't care about return value in the rest of the code
        self.sock.sendall(buf)


################################################################################

CMD_PING = 0x0001

CMD_THROTTLE = 0x0200

CMD_EM = 0x0300

CMD_WM = 0x0400

CMD_RP = 0x0500

CMD_AZ5 = 0xFFF0

CMD_SET_PID = 0x0600

CMD_SET_VELS = 0x0700

CMD_SET_POS = 0x0701

CMD_TRG_POS = 0x0702

MSG_ACK = 0x0100
MSG_NAK = 0x0101

MSG_PAT = 0x0A00
MSG_ACC = 0x0A01
MSG_MFX = 0x0A02
MSG_DST = 0x0A03
MSG_STB = 0x0A04
MSG_PVT = 0x0A05
MSG_WHT = 0x0A06

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
# Struct helpers
################################################################################

def unpack_payload(fmt: str, payload):
    """Join payload bytes list and unpack with little-endian format."""
    return struct.unpack('<' + fmt, b''.join(payload))


def pack_message(cmd: int, payload_fmt: str = '', *fields: object) -> bytes:
    """
    Build full frame: 'm' 'b' + cmd + len + payload + CRC.

    payload_fmt: struct format for payload without '<'
    fields: values for that payload
    """
    if payload_fmt:
        payload = struct.pack('<' + payload_fmt, *fields)
    else:
        payload = b''

    length = len(payload)
    header = struct.pack('<2sHH', b'mb', cmd, length)
    tail = struct.pack('<H', CRC)
    return header + payload + tail

################################################################################

time_window = 100  # Number of points to keep in the plot
t_data = list(range(-time_window, 0))

acr_x, acr_y, acr_z = [0] * time_window, [0] * time_window, [0] * time_window
acc_x, acc_y, acc_z = [0] * time_window, [0] * time_window, [0] * time_window
acw_x, acw_y, acw_z = [0] * time_window, [0] * time_window, [0] * time_window
acl_x, acl_y, acl_z = [0] * time_window, [0] * time_window, [0] * time_window

pos_x, pos_y, pos_z = [0] * time_window, [0] * time_window, [0] * time_window
vel_x, vel_y, vel_z = [0] * time_window, [0] * time_window, [0] * time_window

roll, pitch, yaw = [0] * time_window, [0] * time_window, [0] * time_window
roll_rate, pitch_rate, yaw_rate = [0] * time_window, [0] * time_window, [0] * time_window
pid_roll, pid_pitch, pid_yaw = [0] * time_window, [0] * time_window, [0] * time_window

thrust1, thrust2, thrust3, thrust4 = [0] * time_window, [0] * time_window, [0] * time_window, [0] * time_window

trg_dx = 0.0
trg_dy = 0.0
trg_locked = False

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

    throttle = pack_message(
        CMD_THROTTLE,
        'B4f',
        flags,
        throttles[0],
        throttles[1],
        throttles[2],
        throttles[3],
    )
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

    throttle = pack_message(
        CMD_THROTTLE,
        'B4f',
        flags,
        throttles[0],
        throttles[1],
        throttles[2],
        throttles[3],
    )
    port.write(throttle)

    ackAwait = CMD_THROTTLE

    time.sleep(0.5)

    if ackRx != ackAwait:
        print("WRN: command not acked")

    ackRx = ackAwait = -1


def handle_ping(payload):
    global pingRx
    (pingRx,) = unpack_payload('H', payload)
    # print("ping", pingRx)


def handle_acknak(ack, payload):
    global ackRx
    (cmd,) = unpack_payload('H', payload)

    if ack == 0:
        print(cmd, " NAKed")
    else:
        ackRx = cmd

def handle_acc(payload):
    global acr_x, acr_y, acr_z
    global acc_x, acc_y, acc_z
    global acw_x, acw_y, acw_z
    global acl_x, acl_y, acl_z

    acc_data = unpack_payload('12f', payload)

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
    cal_data = unpack_payload('12fB', payload)

    print('[', cal_data[0], cal_data[4], cal_data[8], '] ',
          ' [', cal_data[9], cal_data[10], cal_data[11], ']')

def handle_pat(payload):
    global yaw, pitch, roll
    global pos_x, pos_y, pos_z

    pat_data = unpack_payload('7f', payload)

    pos_x.append(pat_data[0])
    pos_y.append(pat_data[1])
    pos_z.append(pat_data[2])

    roll.append(pat_data[3])
    pitch.append(pat_data[4])
    yaw.append(pat_data[5])

    time_ = pat_data[6]

    print(time_, "(", pos_x[-1], pos_y[-1], pos_z[-1], ")",
          "(", roll[-1], pitch[-1], yaw[-1], ")")

def handle_mfx(payload):
    mfx_data = unpack_payload('9f', payload)

    print(mfx_data[3], mfx_data[4], mfx_data[5],
          mfx_data[6], mfx_data[7], mfx_data[8])

def handle_mon(payload):
    global fLog
    mon_data = unpack_payload('6f', payload)
    print(mon_data)

    log_str = ' '.join(f'{v:.6f}' for v in mon_data)
    fLog.write("MON: " + log_str + "\n")

def handle_stb(payload):
    global yaw, pitch, roll
    global yaw_rate, pitch_rate, roll_rate
    global pid_roll, pid_pitch, pid_yaw
    global thrust1, thrust2, thrust3, thrust4
    global fLog

    stb_data = unpack_payload('13f', payload)

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

    log_str = ' '.join(f'{v:.13f}' for v in stb_data)
    fLog.write("STB: " + log_str + "\n")

def handle_pvt(payload):
    global pos_x, pos_y, pos_z
    global vel_x, vel_y, vel_z

    pvt_data = unpack_payload('7f', payload)

    pos_x.append(pvt_data[0])
    pos_y.append(pvt_data[1])
    pos_z.append(pvt_data[2])

    vel_x.append(pvt_data[3])
    vel_y.append(pvt_data[4])
    vel_z.append(pvt_data[5])

    time_ = pvt_data[6]

def handle_wht(payload):
    global yaw
    wht_data = unpack_payload('3f', payload)
    yaw.append(wht_data[0])

def handle_trg_pos(payload):
    global trg_dx, trg_dy, trg_locked

    trg_dx, trg_dy, tdx, tdy, flags = unpack_payload('4fB', payload)
    trg_locked = bool(flags & 0x01)

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
    elif msgId == "pvt":
        msgId = MSG_PVT
    elif msgId == "wht":
        msgId = MSG_WHT
    else:
        print("command not supported")
        return

    em = pack_message(CMD_EM, 'HH', msgId, msgPeriod)
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
    elif fc == "vc":
        fsMode = 2
    elif fc == "gt":
        fsMode = 3

    wm_frame = pack_message(CMD_WM, 'BB', imuMode, fsMode)
    port.write(wm_frame)

    ackAwait = CMD_WM

    time.sleep(0.5)

    if ackRx != ackAwait:
        print("WRN: command not acked")

    ackRx = ackAwait = -1

def reset_pos_command(port):
    global ackRx, ackAwait

    rp = pack_message(CMD_RP, 'B', 0)
    port.write(rp)

    ackAwait = CMD_RP

    time.sleep(0.5)

    if ackRx != ackAwait:
        print("WRN: command not acked")

    ackRx = ackAwait = -1

def az5_command(port):
    global doExit
    rp = pack_message(CMD_AZ5, 'B', 0)
    port.write(rp)

    time.sleep(0.5)

    doExit = True

def set_pid_command(port):

    global ackRx, ackAwait

    cmd = CMD_SET_PID

    koeffs = [0.0] * 10

    kppr = 0.01
    kpyaw = 0

    ki = 0.0

    kdpr = 0.0 * kppr
    kdyaw = 0.2 * kpyaw

    mass = 300.0 * 1.0e-3

    koeffs[0] = koeffs[1] = kppr
    koeffs[2] = kpyaw

    koeffs[3] = koeffs[4] = kdpr
    koeffs[5] = kdyaw

    koeffs[6] = koeffs[7] = ki

    koeffs[9] = mass

    sp = pack_message(cmd, '10f', *koeffs)
    port.write(sp)

    ackAwait = CMD_SET_PID

    time.sleep(0.5)

    if ackRx != ackAwait:
        print("WRN: command not acked")

    ackRx = ackAwait = -1

def set_vels_command(port, v, w, stop):
    global ackRx, ackAwait

    cmd = CMD_SET_VELS

    flags = 0
    if stop:
        flags = 1

    sp = pack_message(cmd, 'ffB', v, w, flags)
    port.write(sp)

    ackAwait = CMD_SET_VELS

    time.sleep(0.5)

    if ackRx != ackAwait:
        print("WRN: command not acked")

    ackRx = ackAwait = -1

def set_pos_command(port, x, y, phi, rel):
    global ackRx, ackAwait

    cmd = CMD_SET_POS

    flags = 0
    if rel:
        flags = 1

    sp = pack_message(cmd, 'fffB', x, y, phi, flags)
    port.write(sp)

    ackAwait = CMD_SET_POS

    time.sleep(0.5)

    if ackRx != ackAwait:
        print("WRN: command not acked")

    ackRx = ackAwait = -1

noPingCount = 0

def pinger_function(name, port):
    global pingSeq
    global doExit
    global noPingCount
    while not doExit:
        ping = pack_message(CMD_PING, 'H', pingSeq)
        port.write(ping)

        time.sleep(0.5)

        if pingRx != pingSeq:
            noPingCount = noPingCount + 1
        else:
            noPingCount = 0

        if noPingCount > 5:
            print("Connection lost!")
            doExit = True

        pingSeq = pingSeq + 1

        time.sleep(1)

def console_function(name, port):
    v = 0.0
    w = 0.0
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
            elif command[0] == "5":
                az5_command(port)
            elif command[0] == "sp":
                set_pid_command(port)
            elif command[0] == "v+":
                v = v + 0.1
                set_vels_command(port, v, w, False)
            elif command[0] == "v-":
                v = v - 0.1
                set_vels_command(port, v, w, False)
            elif command[0] == "w+":
                w = w + 1.0
                set_vels_command(port, v, w, False)
            elif command[0] == "w-":
                w = w - 1.0
                set_vels_command(port, v, w, False)
            elif command[0] == "vs":
                v = 0.0
                w = 0.0
                set_vels_command(port, v, w, True)
            elif command[0] == "pd":
                dx = float(command[1])
                dy = float(command[2])
                dphi = math.radians(float(command[3]))
                set_pos_command(port, dx, dy, dphi, True)
            elif command[0] == "pa":
                x = float(command[1])
                y = float(command[2])
                phi = math.radians(float(command[3]))
                set_pos_command(port, x, y, phi, False)

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
                elif cmd == MSG_PVT:
                    handle_pvt(payload)
                elif cmd == MSG_WHT:
                    handle_wht(payload)
                elif cmd == CMD_TRG_POS:
                    handle_trg_pos(payload)
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

def visio_track_function(name):
    pygame.init()
    screen = pygame.display.set_mode((800, 600))
    clock  = pygame.time.Clock()

    PIXELS_PER_METER = 160  # 1 m = 160 px
    font = pygame.font.SysFont("consolas", 20)

    rover_img_raw = pygame.image.load("rover.png").convert_alpha()
    SPRITE_TO_X_DEG = 90
    rover_img_raw = pygame.transform.rotate(rover_img_raw, -SPRITE_TO_X_DEG)

    max_scale_factor = screen.get_width() // 5
    w, h = rover_img_raw.get_size()
    scale_ratio = min(max_scale_factor / w, max_scale_factor / h)
    new_size = (int(w * scale_ratio), int(h * scale_ratio))
    rover_img = pygame.transform.smoothscale(rover_img_raw, new_size)

    def world_to_screen(wx, wy, cam_x, cam_y):
        cx, cy = screen.get_width() // 2, screen.get_height() // 2
        sx = cx + (wx - cam_x) * PIXELS_PER_METER
        sy = cy - (wy - cam_y) * PIXELS_PER_METER  # minus keeps Y up
        return int(sx), int(sy)

    def draw_grid(cam_x, cam_y, step_m=0.5, color=(180,180,180)):
        w, h = screen.get_width(), screen.get_height()
        half_w_m = w / (2 * PIXELS_PER_METER)
        half_h_m = h / (2 * PIXELS_PER_METER)

        x0 = math.floor((cam_x - half_w_m) / step_m) * step_m
        x1 = math.ceil ((cam_x + half_w_m) / step_m) * step_m
        y0 = math.floor((cam_y - half_h_m) / step_m) * step_m
        y1 = math.ceil ((cam_y + half_h_m) / step_m) * step_m

        gx = x0
        while gx <= x1 + 1e-9:
            x_s, y_top = world_to_screen(gx, y0, cam_x, cam_y)
            _,   y_bot = world_to_screen(gx, y1, cam_x, cam_y)
            pygame.draw.line(screen, color, (x_s, y_top), (x_s, y_bot))
            gx += step_m

        gy = y0
        while gy <= y1 + 1e-9:
            x_left, y_s = world_to_screen(x0, gy, cam_x, cam_y)
            x_right, _  = world_to_screen(x1, gy, cam_x, cam_y)
            pygame.draw.line(screen, color, (x_left, y_s), (x_right, y_s))
            gy += step_m

    def draw_trace(trace, cam_x, cam_y):
        if len(trace) > 1:
            pts = [world_to_screen(wx, wy, cam_x, cam_y) for (wx, wy) in trace]
            pygame.draw.lines(screen, (255,0,0), False, pts, 2)

    def draw_telemetry(x, y, heading_deg, vx, vy, tdx, tdy, locked):
        info = [
            f"X: {x:.2f}",
            f"Y: {y:.2f}",
            f"Heading: {heading_deg:.1f}°",
            f"Vx: {vx:.2f}",
            f"Vy: {vy:.2f}",
            f"Target dx: {tdx:.2f}",
            f"Target dy: {tdy:.2f}",
        ]

        # Draw normal white text first
        for i, line in enumerate(info):
            screen.blit(font.render(line, True, (0,0,0)), (10, 10 + i*22))

        # --- Target status line ---
        if locked:
            txt = "TARGET LOCKED"
            color = (0, 160, 0)     # dark green
        else:
            txt = "TARGET LOST"
            color = (200, 0, 0)     # red

        screen.blit(
            font.render(txt, True, color),
            (10, 10 + len(info) * 22)
        )

    def draw_target(x, y, heading_rad, cam_x, cam_y):
        if not trg_locked:
            return

        # body (dx,dy) -> world
        dx_b = trg_dx
        dy_b = trg_dy
        c = math.cos(heading_rad)
        s = math.sin(heading_rad)

        tx_w = x + dx_b * c - dy_b * s
        ty_w = y + dx_b * s + dy_b * c

        sx, sy = world_to_screen(tx_w, ty_w, cam_x, cam_y)

        # color depends on lock
        color = (0, 200, 0) if trg_locked else (200, 0, 0)

        # Draw a circle and a small cross
        pygame.draw.circle(screen, color, (sx, sy), 6, 2)
        pygame.draw.line(screen, color, (sx - 8, sy), (sx + 8, sy), 1)
        pygame.draw.line(screen, color, (sx, sy - 8), (sx, sy + 8), 1)

    trace_points = []

    global doExit, pos_x, pos_y, vel_x, vel_y, yaw
    global trg_dx, trg_dy, trg_locked
    while not doExit:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                doExit = True

        x = pos_x[-1]
        y = pos_y[-1]
        vx = vel_x[-1]
        vy = vel_y[-1]

        trace_points.append((x, y))

        # --- heading: CCW-positive, 0° along +X ---
        heading_deg = math.degrees(yaw[-1])
        heading_rad = yaw[-1]

        # --- draw ---
        screen.fill((230,230,230))
        draw_grid(x, y)
        draw_trace(trace_points, x, y)
        draw_target(x, y, heading_rad, x, y)

        # rotate CCW by +heading (pre-rotation already aligned sprite to +X)
        rotated = pygame.transform.rotate(rover_img, heading_deg)
        rect = rotated.get_rect(center=(screen.get_width()//2, screen.get_height()//2))
        screen.blit(rotated, rect.topleft)

        draw_telemetry(x, y, heading_deg, vx, vy, trg_dx, trg_dy, trg_locked)

        pygame.display.flip()
        clock.tick(30)

    pygame.quit()

################################################################################

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
        ax.set_ylim(-0.05, 0.05)  # adjust as needed
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
        ax.set_ylim(0, 2)
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

    transport = "tcp" #input("transport? [ble/serial/tcp] ").strip().lower()

    if transport == "serial":
        port = serial.Serial(port="/dev/ttyUSB0", baudrate=115200, timeout=0.12)
        print("Using SERIAL /dev/ttyUSB0")
    elif transport == "tcp":
        port = TcpPort()
    else:
        port = BLEPort()

    mode = input("mode? ").strip()

    # pinger = threading.Thread(target=pinger_function, args=("pinger", port,))
    # pinger.start()

    listener = threading.Thread(target=listener_function, args=("listener", port,))
    listener.start()

    console = threading.Thread(target=console_function, args=("console", port,))
    console.start()

    if mode == "track":
        visio = threading.Thread(target=visio_track_function, args=("track",))
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