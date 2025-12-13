package com.example.morbo_bremote

import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * Incremental stream parser for HIP protocol.
 *
 * Ported from listener_function() in test_utility.py:
 *  'm' 'b' cmd(lo) cmd(hi) len(lo) len(hi) payload... crc(lo) crc(hi)
 *
 * CRC: CRC-16/CCITT over header('m','b',cmd,len) + payload.
 */
class HipParser(
    private val onPvt: (TelemetryState) -> Unit,
    private val onWht: (Float) -> Unit,
    private val onPingRx: (Int) -> Unit,
    private val onTrgPos: (TargetState) -> Unit
) {

    private enum class State {
        M, B, CMD0, CMD1, LEN0, LEN1, PAYLOAD, CRC0, CRC1
    }

    private var state = State.M
    private var cmd = 0
    private var length = 0
    private val payload = mutableListOf<Byte>()
    private var rxCrc = 0

    fun feed(bytes: ByteArray) {
        for (b in bytes) {
            processByte(b)
        }
    }

    private fun reset() {
        state = State.M
        cmd = 0
        length = 0
        payload.clear()
        rxCrc = 0
    }

    private fun processByte(b: Byte) {
        when (state) {
            State.M -> {
                if (b == 'm'.code.toByte()) {
                    state = State.B
                }
            }
            State.B -> {
                state = if (b == 'b'.code.toByte()) State.CMD0 else State.M
            }
            State.CMD0 -> {
                cmd = b.toInt() and 0xFF
                state = State.CMD1
            }
            State.CMD1 -> {
                cmd = cmd + ((b.toInt() and 0xFF) shl 8)
                state = State.LEN0
            }
            State.LEN0 -> {
                length = b.toInt() and 0xFF
                state = State.LEN1
            }
            State.LEN1 -> {
                length = length + ((b.toInt() and 0xFF) shl 8)
                payload.clear()
                state = if (length == 0) State.CRC0 else State.PAYLOAD
            }
            State.PAYLOAD -> {
                payload.add(b)
                if (payload.size == length) {
                    state = State.CRC0
                }
            }
            State.CRC0 -> {
                rxCrc = b.toInt() and 0xFF
                state = State.CRC1
            }
            State.CRC1 -> {
                rxCrc = rxCrc + ((b.toInt() and 0xFF) shl 8)

                val header = byteArrayOf(
                    'm'.code.toByte(),
                    'b'.code.toByte(),
                    (cmd and 0xFF).toByte(),
                    ((cmd shr 8) and 0xFF).toByte(),
                    (length and 0xFF).toByte(),
                    ((length shr 8) and 0xFF).toByte()
                )
                val payloadBytes = payload.toByteArray()
                val calcCrc = crc16Ccitt(header + payloadBytes)

                if (calcCrc == rxCrc) {
                    dispatch(cmd, payloadBytes)
                } else {
                    // CRC mismatch: ignore frame
                }

                reset()
            }
        }
    }

    private fun dispatch(cmd: Int, payload: ByteArray) {
        when (cmd) {
            HipProtocol.MSG_PVT -> handlePvt(payload)
            HipProtocol.MSG_WHT -> handleWht(payload)
            HipProtocol.CMD_PING -> handlePing(payload)
            HipProtocol.CMD_TRG_POS -> handleTrgPos(payload)
        }
    }

    private fun handlePvt(payload: ByteArray) {
        // format '7f' -> pos_x,pos_y,pos_z, vel_x,vel_y,vel_z, time
        if (payload.size < 7 * 4) return
        val bb = ByteBuffer.wrap(payload).order(ByteOrder.LITTLE_ENDIAN)
        val posX = bb.float
        val posY = bb.float
        val posZ = bb.float
        val velX = bb.float
        val velY = bb.float
        val velZ = bb.float
        val time = bb.float

        val t = TelemetryState(
            posX = posX,
            posY = posY,
            posZ = posZ,
            velX = velX,
            velY = velY,
            velZ = velZ,
            time = time
        )
        onPvt(t)
    }

    private fun handleWht(payload: ByteArray) {
        // format '3f' -> yaw, (we ignore the rest)
        if (payload.size < 1 * 4) return
        val bb = ByteBuffer.wrap(payload).order(ByteOrder.LITTLE_ENDIAN)
        val yaw = bb.float
        onWht(yaw)
    }

    private fun handlePing(payload: ByteArray) {
        // Python uses struct.unpack("H", payload) -> uint16 LE
        if (payload.size < 2) return
        val bb = ByteBuffer.wrap(payload).order(ByteOrder.LITTLE_ENDIAN)
        val seq = bb.short.toInt() and 0xFFFF
        onPingRx(seq)
    }

    private fun handleTrgPos(payload: ByteArray) {
        // payload format: '4fB'
        if (payload.size < 4 * 4 + 1) return

        val bb = ByteBuffer.wrap(payload).order(ByteOrder.LITTLE_ENDIAN)

        val trgDx = bb.float
        val trgDy = bb.float

        bb.float   // tdx (ignored)
        bb.float   // tdy (ignored)

        val flags = bb.get().toInt() and 0xFF
        val locked = (flags and 0x01) != 0

        onTrgPos(
            TargetState(
                dx = trgDx,
                dy = trgDy,
                locked = locked
            )
        )
    }

    // Local copy of CRC-16/CCITT (same as in HipProtocol / Python)
    private fun crc16Ccitt(data: ByteArray, init: Int = 0xFFFF): Int {
        var crc = init
        for (b in data) {
            crc = crc xor ((b.toInt() and 0xFF) shl 8)
            repeat(8) {
                crc = if ((crc and 0x8000) != 0) {
                    ((crc shl 1) xor 0x1021) and 0xFFFF
                } else {
                    (crc shl 1) and 0xFFFF
                }
            }
        }
        return crc and 0xFFFF
    }
}
