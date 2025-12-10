package com.example.morbo_bremote

import android.Manifest
import android.os.Build
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
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

    // Launcher for runtime permission requests
    private val permissionRequest = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
        val granted = permissions.values.all { it }
        if (granted) {
            RoverLog.d("All BLE permissions granted")
        } else {
            RoverLog.e("BLE permissions NOT granted -> BLE won't work")
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()

        // Ask for BLE-related permissions when activity starts
        requestBlePermissions()

        setContent {
            MorboBRemoteTheme {
                val context = LocalContext.current
                val transport = remember { BleRoverTransport(context) }

                // Start BLE scan once, after composition
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
            // Android 12+
            permissions += Manifest.permission.BLUETOOTH_SCAN
            permissions += Manifest.permission.BLUETOOTH_CONNECT
        } else {
            // Android < 12 – BLE scan historically required location
            permissions += Manifest.permission.ACCESS_FINE_LOCATION
        }

        permissionRequest.launch(permissions.toTypedArray())
    }
}

// -------------------- UI --------------------

@Composable
fun MainScreen(transport: RoverTransport) {

    val isConnected = RoverState.isConnected
    val telemetry = RoverState.telemetry
    val logLines = RoverLog.lines

    Scaffold(
        modifier = Modifier.fillMaxSize()
    ) { innerPadding ->
        Column(
            modifier = Modifier
                .padding(innerPadding)
                .padding(16.dp)
                .fillMaxSize()
        ) {
            Text(text = "Morbo-B Remote")

            Spacer(modifier = Modifier.height(8.dp))

            // Connection status
            Text(
                text = if (isConnected) "Status: CONNECTED" else "Status: DISCONNECTED"
            )

            Spacer(modifier = Modifier.height(16.dp))

            // STOP button
            Button(
                onClick = {
                    val frame = HipProtocol.buildAz5Frame()
                    transport.send(frame)
                },
                enabled = isConnected   // only active when BLE connected
            ) {
                Text(text = "STOP")
            }

            Spacer(modifier = Modifier.height(24.dp))

            // Telemetry panel
            if (telemetry != null) {
                Text(
                    text = "Coords: x=%.2f, y=%.2f, z=%.2f".format(
                        telemetry.posX,
                        telemetry.posY,
                        telemetry.posZ
                    )
                )
                Text(
                    text = "Speed: %.2f m/s".format(telemetry.speed)
                )
                telemetry.yaw?.let { yaw ->
                    Text(text = "Yaw: %.2f".format(yaw))
                }
            } else {
                Text(text = "Coords: (no data)")
                Text(text = "Speed: (no data)")
            }

            Spacer(modifier = Modifier.height(16.dp))

            // Log window
            Text(text = "Log:")
            Spacer(modifier = Modifier.height(4.dp))

            LazyColumn(
                modifier = Modifier
                    .weight(1f)
                    .fillMaxSize()
            ) {
                items(logLines) { line ->
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
        // For preview we fake a transport that does nothing
        val dummyTransport = object : RoverTransport {
            override fun send(data: ByteArray) { /* no-op in preview */ }
        }
        MainScreen(transport = dummyTransport)
    }
}
