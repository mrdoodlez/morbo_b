package com.example.morbo_bremote

import kotlin.math.sqrt

/**
 * Telemetry decoded from PVT + WHT messages.
 *
 * PVT (payload '7f'):
 *  pos_x, pos_y, pos_z,
 *  vel_x, vel_y, vel_z,
 *  time
 *
 * WHT (payload '3f'):
 *  yaw, (we ignore the other 2 floats)
 */
data class TelemetryState(
    val posX: Float,
    val posY: Float,
    val posZ: Float,
    val velX: Float,
    val velY: Float,
    val velZ: Float,
    val time: Float,
    val yaw: Float? = null,   // updated from WHT, if present
) {
    val speed: Float
        get() = sqrt(velX * velX + velY * velY + velZ * velZ)
}
