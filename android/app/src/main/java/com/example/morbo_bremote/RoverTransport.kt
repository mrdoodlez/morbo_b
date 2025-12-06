package com.example.morbo_bremote

/**
 * Minimal transport interface used by UI / higher-level logic.
 * Real implementation is BleRoverTransport.
 */
interface RoverTransport {
    fun send(data: ByteArray)
}
