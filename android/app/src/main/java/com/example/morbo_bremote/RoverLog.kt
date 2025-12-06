package com.example.morbo_bremote

import android.util.Log
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue

/**
 * Logger that writes to Logcat and also keeps a rolling log
 * for display in the UI.
 */
object RoverLog {

    private const val TAG = "MorboB"

    // Rolling log lines, observable by Compose
    var lines: List<String> by mutableStateOf(emptyList())
        private set

    private fun append(raw: String) {
        val ts = System.currentTimeMillis() % 100000  // short timestamp
        val msg = "[$ts] $raw"
        lines = (lines + msg).takeLast(200)   // keep last 200
    }

    fun d(msg: String) {
        Log.d(TAG, msg)
        append(msg)
    }

    fun e(msg: String, t: Throwable? = null) {
        Log.e(TAG, msg, t)
        append("ERR: $msg")
    }
}
