package com.example.morbo_bremote

data class PingState(
    val isPinging: Boolean = false,
    val lastPingSeq: Int = 0,
    val lastPongSeq: Int = -1,
    val pongOk: Boolean = false,
    val lastPongAgeMs: Long = Long.MAX_VALUE,
    val lastRttMs: Long? = null
)
