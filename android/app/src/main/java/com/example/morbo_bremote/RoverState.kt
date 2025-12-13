package com.example.morbo_bremote

import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue

object RoverState {

    var isConnected: Boolean by mutableStateOf(false)
        private set

    var telemetry: TelemetryState? by mutableStateOf(null)
        private set

    var ping: PingState by mutableStateOf(PingState())
        private set

    var target: TargetState? by mutableStateOf(null)
        private set

    private var lastYaw: Float? = null

    private var transportSend: ((ByteArray) -> Unit)? = null

    fun attachTransportSender(sender: (ByteArray) -> Unit) {
        transportSend = sender
    }

    fun startPinging() {
        pingManager.start()
    }

    fun stopPinging() {
        pingManager.stop()
    }

    private val pingManager = PingManager(
        sendFrame = { frame -> transportSend?.invoke(frame) },
        isBleConnected = { isConnected },
        updateUi = { st -> ping = st }
    )

    private val parser = HipParser(
        onPvt = { pvt ->
            val yaw = lastYaw
            telemetry = if (yaw != null) pvt.copy(yaw = yaw) else pvt
        },
        onWht = { yaw ->
            lastYaw = yaw
            telemetry = telemetry?.copy(yaw = yaw)
        },
        onPingRx = { seqRx ->
            pingManager.onPong(seqRx)
        },
       onTrgPos = { trg ->
            target = trg
            RoverLog.d(
                if (trg.locked)
                    "Target locked: dx=%.2f dy=%.2f".format(trg.dx, trg.dy)
                else
                    "Target unlocked"
            )
        }
    )

    fun updateConnectionState(connected: Boolean) {
        isConnected = connected
        if (!connected) {
            pingManager.stop()
        }
    }

    /**
     * Called when BLE notifications give us bytes.
     */
    fun feedFromBle(bytes: ByteArray) {
        parser.feed(bytes)
    }
}
