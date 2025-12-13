package com.example.morbo_bremote

import kotlinx.coroutines.*
import java.util.concurrent.ConcurrentHashMap

class PingManager(
    private val sendFrame: (ByteArray) -> Unit,
    private val isBleConnected: () -> Boolean,
    private val updateUi: (PingState) -> Unit
) {
    private val scope = CoroutineScope(SupervisorJob() + Dispatchers.Default)

    private var job: Job? = null
    private var seq: Int = 0

    private val sentAtMs = ConcurrentHashMap<Int, Long>()

    // tune these later if needed
    private val periodMs = 1000L
    private val pongTimeoutMs = 3000L

    @Volatile
    private var lastPongSeq: Int = -1

    @Volatile
    private var lastPongTimeMs: Long = 0L

    @Volatile
    private var lastRttMs: Long? = null

    fun start() {
        if (job != null) return
        if (!isBleConnected()) return

        RoverLog.d("Pinger: START")

        // set initial UI state
        updateUi(
            PingState(
                isPinging = true,
                lastPingSeq = seq,
                lastPongSeq = lastPongSeq,
                pongOk = false,
                lastPongAgeMs = Long.MAX_VALUE,
                lastRttMs = lastRttMs
            )
        )

        job = scope.launch {
            while (isActive) {
                if (!isBleConnected()) {
                    RoverLog.d("Pinger: BLE disconnected -> stopping pinger")
                    stop()
                    return@launch
                }

                val currentSeq = seq and 0xFFFF
                val frame = HipProtocol.buildPingFrame(currentSeq)

                sentAtMs[currentSeq] = System.currentTimeMillis()
                sendFrame(frame)

                // update UI immediately after send
                val now = System.currentTimeMillis()
                val age = if (lastPongTimeMs == 0L) Long.MAX_VALUE else (now - lastPongTimeMs)
                val ok = (lastPongTimeMs != 0L) && (age <= pongTimeoutMs)

                updateUi(
                    PingState(
                        isPinging = true,
                        lastPingSeq = currentSeq,
                        lastPongSeq = lastPongSeq,
                        pongOk = ok,
                        lastPongAgeMs = age,
                        lastRttMs = lastRttMs
                    )
                )

                seq = (seq + 1) and 0xFFFF
                delay(periodMs)
            }
        }
    }

    fun stop() {
        job?.cancel()
        job = null

        RoverLog.d("Pinger: STOP")

        // update UI state
        val now = System.currentTimeMillis()
        val age = if (lastPongTimeMs == 0L) Long.MAX_VALUE else (now - lastPongTimeMs)
        val ok = (lastPongTimeMs != 0L) && (age <= pongTimeoutMs)

        updateUi(
            PingState(
                isPinging = false,
                lastPingSeq = (seq - 1) and 0xFFFF,
                lastPongSeq = lastPongSeq,
                pongOk = ok,
                lastPongAgeMs = age,
                lastRttMs = lastRttMs
            )
        )
    }

    /**
     * Called by parser when CMD_PING arrives from rover (pong).
     */
    fun onPong(seqRx: Int) {
        lastPongSeq = seqRx
        lastPongTimeMs = System.currentTimeMillis()

        val sent = sentAtMs.remove(seqRx)
        if (sent != null) {
            lastRttMs = lastPongTimeMs - sent
        }

        RoverLog.d("Ponger: RX seq=$seqRx rtt=${lastRttMs ?: -1}ms")

        // update UI immediately on pong
        updateUi(
            PingState(
                isPinging = (job != null),
                lastPingSeq = (seq - 1) and 0xFFFF,
                lastPongSeq = lastPongSeq,
                pongOk = true,
                lastPongAgeMs = 0,
                lastRttMs = lastRttMs
            )
        )
    }
}
