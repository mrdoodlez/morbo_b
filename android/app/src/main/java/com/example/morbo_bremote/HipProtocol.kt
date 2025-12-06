package com.example.morbo_bremote

import java.nio.ByteBuffer
import java.nio.ByteOrder

object HipProtocol {

    // === Commands ===

    const val CMD_PING = 0x0001
    const val CMD_AZ5  = 0xFFF0

    // === Messages (must match C) ===
    const val MSG_PVT  = 0x0A05
    const val MSG_WHT  = 0x0A06

    const val CMD_TRG_POS = 0x0702

    private const val HIP_M: Byte = 'm'.code.toByte()
    private const val HIP_B: Byte = 'b'.code.toByte()

    // CRC-16/CCITT (init = 0xFFFF, poly = 0x1021), same as in Python listener
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

    /**
     * Build full frame:
     *   'm' 'b' + cmd(uint16 LE) + len(uint16 LE) + payload + CRC(uint16 LE)
     *
     * CRC is CRC-16/CCITT over:
     *   ['m','b', cmd_lo, cmd_hi, len_lo, len_hi, payload...]
     */
    fun packMessage(cmd: Int, payload: ByteArray? = null): ByteArray {
        val pl = payload ?: ByteArray(0)
        val length = pl.size

        // Body used for CRC: header + payload
        val bodyBuf = ByteBuffer
            .allocate(2 + 2 + 2 + length) // m,b + cmd + len + payload
            .order(ByteOrder.LITTLE_ENDIAN)

        bodyBuf.put(HIP_M)
        bodyBuf.put(HIP_B)
        bodyBuf.putShort(cmd.toShort())
        bodyBuf.putShort(length.toShort())
        if (length > 0) {
            bodyBuf.put(pl)
        }

        val body = bodyBuf.array()
        val crc = crc16Ccitt(body)

        // Final frame = body + crc
        val frameBuf = ByteBuffer
            .allocate(body.size + 2)
            .order(ByteOrder.LITTLE_ENDIAN)

        frameBuf.put(body)
        frameBuf.putShort(crc.toShort())

        return frameBuf.array()
    }

    /**
     * AZ5 (stop) command.
     * Same payload as Python: one byte 0x00.
     */
    fun buildAz5Frame(): ByteArray {
        val payload = byteArrayOf(0x00)
        return packMessage(CMD_AZ5, payload)
    }

    fun buildPingFrame(seq: Int): ByteArray {
        // payload 'H' -> uint16 little-endian
        val payload = ByteBuffer
            .allocate(2)
            .order(ByteOrder.LITTLE_ENDIAN)
            .putShort((seq and 0xFFFF).toShort())
            .array()

        return packMessage(CMD_PING, payload)
    }

}