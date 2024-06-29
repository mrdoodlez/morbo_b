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

	time.sleep(0.5)

	if ackRx != ackAwait:
		print("WRN: throttle enable not acked")

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


def pinger_function(name, port):
	global pingSeq
	while True:
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
	port = BLEPort()

	#port = serial.Serial(port="/dev/ttyUSB0", baudrate=115200, timeout = 0.12)

	pinger = threading.Thread(target=pinger_function, args=("pinger", port.service,))
	pinger.start()

	listener = threading.Thread(target=listener_function, args=("listener", port.service,))
	listener.start()

	console = threading.Thread(target=console_function, args=("console", port.service,))
	console.start()


if __name__ == '__main__':
	main()
