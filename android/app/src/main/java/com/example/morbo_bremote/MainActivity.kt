package com.example.morbo_bremote

import android.Manifest
import android.os.Build
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.material3.Button
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.remember
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import com.example.morbo_bremote.ui.theme.MorboBRemoteTheme

class MainActivity : ComponentActivity() {

    private val permissionRequest = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
        val granted = permissions.values.all { it }
        if (granted) {
            RoverLog.d("All BLE permissions granted")
        } else {
            RoverLog.e("BLE permissions NOT granted – BLE disabled")
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()

        requestBlePermissions()

        setContent {
            MorboBRemoteTheme {
                val context = LocalContext.current
                val transport = remember { BleRoverTransport(context) }

                LaunchedEffect(Unit) {
                    RoverLog.d("Starting BLE transport")
                    transport.start()
                }

                MainScreen(transport = transport)
            }
        }
    }

    private fun requestBlePermissions() {
        val permissions = mutableListOf<String>()
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            permissions += Manifest.permission.BLUETOOTH_SCAN
            permissions += Manifest.permission.BLUETOOTH_CONNECT
        } else {
            permissions += Manifest.permission.ACCESS_FINE_LOCATION
        }
        permissionRequest.launch(permissions.toTypedArray())
    }
}

/* ============================ UI ============================ */

@Composable
fun MainScreen(transport: RoverTransport) {

    // Attach transport sender for PingManager
    RoverState.attachTransportSender { bytes -> transport.send(bytes) }

    val isConnected = RoverState.isConnected
    val telemetry = RoverState.telemetry
    val ping = RoverState.ping
    val target = RoverState.target
    val logs = RoverLog.lines

    val pongText = when {
        !ping.isPinging -> "Pong: N/A"
        ping.pongOk -> {
            val rtt = ping.lastRttMs?.toString() ?: "-"
            "Pong: OK (rtt=${rtt} ms)"
        }
        else -> "Pong: NO RESPONSE"
    }

    Scaffold(modifier = Modifier.fillMaxSize()) { padding ->
        Column(
            modifier = Modifier
                .padding(padding)
                .padding(16.dp)
                .fillMaxSize()
        ) {

            Text(text = "Morbo-B Remote")

            Spacer(Modifier.height(8.dp))
            Text(text = if (isConnected) "BLE: CONNECTED" else "BLE: DISCONNECTED")

            Spacer(Modifier.height(8.dp))
            Text(text = "Ping: ${if (ping.isPinging) "ACTIVE" else "OFF"}")
            Text(text = pongText)

            Spacer(Modifier.height(12.dp))

            Row {
                Button(
                    onClick = { RoverState.startPinging() },
                    enabled = isConnected && !ping.isPinging
                ) {
                    Text("START")
                }

                Spacer(Modifier.width(12.dp))

                Button(
                    onClick = {
                        RoverState.stopPinging()
                        transport.send(HipProtocol.buildAz5Frame())
                    },
                    enabled = isConnected
                ) {
                    Text("STOP")
                }
            }

            Spacer(Modifier.height(20.dp))

            /* -------- Telemetry -------- */

            if (telemetry != null) {
                Text(
                    text = "Pos: x=%.2f y=%.2f z=%.2f"
                        .format(telemetry.posX, telemetry.posY, telemetry.posZ)
                )
                Text(text = "Speed: %.2f m/s".format(telemetry.speed))
                telemetry.yaw?.let {
                    Text(text = "Yaw: %.2f".format(it))
                }
            } else {
                Text("Telemetry: no data")
            }

            Spacer(Modifier.height(12.dp))

            /* -------- Target -------- */

            when {
                target == null ->
                    Text("Target: N/A")

                target.locked ->
                    Text(
                        text = "Target: LOCKED  dx=%.2f  dy=%.2f"
                            .format(target.dx, target.dy)
                    )

                else ->
                    Text("Target: NOT LOCKED")
            }

            Spacer(Modifier.height(12.dp))

            /* -------- Log window -------- */

            Text("Log:")
            Spacer(Modifier.height(4.dp))

            LazyColumn(
                modifier = Modifier
                    .weight(1f)
                    .fillMaxSize()
            ) {
                items(logs) { line ->
                    Text(text = line)
                }
            }
        }
    }
}

@Preview(showBackground = true)
@Composable
fun MainScreenPreview() {
    MorboBRemoteTheme {
        val dummyTransport = object : RoverTransport {
            override fun send(data: ByteArray) {}
        }
        MainScreen(transport = dummyTransport)
    }
}
