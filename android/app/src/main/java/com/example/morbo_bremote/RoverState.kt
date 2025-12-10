package com.example.morbo_bremote

import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue

object RoverState {

    var isConnected: Boolean by mutableStateOf(false)
        private set

    var telemetry: TelemetryState? by mutableStateOf(null)
        private set

    private var lastYaw: Float? = null

    private val parser = HipParser(
        onPvt = { pvt ->
            val yaw = lastYaw
            telemetry = if (yaw != null) pvt.copy(yaw = yaw) else pvt
        },
        onWht = { yaw ->
            lastYaw = yaw
            telemetry = telemetry?.copy(yaw = yaw)
        }
    )

    fun updateConnectionState(connected: Boolean) {
        isConnected = connected
    }

    /**
     * Called when BLE notifications give us bytes.
     */
    fun feedFromBle(bytes: ByteArray) {
        parser.feed(bytes)
    }
}
