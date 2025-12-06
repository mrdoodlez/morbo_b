package com.example.morbo_bremote

import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.content.Context
import java.util.UUID

/**
 * Real BLE transport:
 * - Scans for device with name "DSD TECH"
 * - Connects to service 0000ffe0-0000-1000-8000-00805f9b34fb
 * - Uses characteristic 0000ffe1-0000-1000-8000-00805f9b34fb
 * - Enables notifications and forwards data to RoverState
 */
class BleRoverTransport(private val context: Context) : RoverTransport {

    companion object {
        private const val TARGET_NAME = "DSD TECH"

        private val SERVICE_UUID: UUID =
            UUID.fromString("0000ffe0-0000-1000-8000-00805f9b34fb")
        private val CHAR_UUID: UUID =
            UUID.fromString("0000ffe1-0000-1000-8000-00805f9b34fb")
        private val CCCD_UUID: UUID =
            UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")
    }

    private val bluetoothManager: BluetoothManager =
        context.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
    private val adapter: BluetoothAdapter? = bluetoothManager.adapter
    private val scanner = adapter?.bluetoothLeScanner

    @Volatile
    private var gatt: BluetoothGatt? = null

    @Volatile
    private var ioCharacteristic: BluetoothGattCharacteristic? = null

    private var scanning = false

    fun start() {
        if (adapter == null || !adapter.isEnabled) {
            RoverLog.e("Bluetooth adapter not available or disabled")
            return
        }
        if (scanner == null) {
            RoverLog.e("BLE scanner not available")
            return
        }

        RoverLog.d("Starting BLE scan for $TARGET_NAME...")
        scanning = true
        scanner.startScan(scanCallback)
    }

    fun stop() {
        if (scanning) {
            scanner?.stopScan(scanCallback)
            scanning = false
            RoverLog.d("BLE scan stopped")
        }
        gatt?.close()
        gatt = null
        ioCharacteristic = null
        RoverState.updateConnectionState(false)
    }

    override fun send(data: ByteArray) {
        val localGatt = gatt
        val ch = ioCharacteristic
        if (localGatt == null || ch == null) {
            RoverLog.e("Cannot send: not connected")
            return
        }

        ch.value = data
        val ok = localGatt.writeCharacteristic(ch)
        if (!ok) {
            RoverLog.e("writeCharacteristic() failed")
        } else {
            val hex = data.joinToString(" ") { "%02X".format(it) }
            RoverLog.d("TX: $hex")
        }
    }

    // --- Scan callback ---

    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            val device: BluetoothDevice = result.device
            val name = device.name ?: result.scanRecord?.deviceName

            if (name == TARGET_NAME) {
                RoverLog.d("Found target device: $name (${device.address})")
                scanner?.stopScan(this)
                scanning = false
                connect(device)
            }
        }

        override fun onScanFailed(errorCode: Int) {
            RoverLog.e("BLE scan failed: $errorCode")
            scanning = false
        }
    }

    // --- GATT / connection ---

    private fun connect(device: BluetoothDevice) {
        RoverLog.d("Connecting to ${device.address} ...")
        gatt = device.connectGatt(context, false, gattCallback)
    }

    private val gattCallback = object : BluetoothGattCallback() {

        override fun onConnectionStateChange(
            gatt: BluetoothGatt,
            status: Int,
            newState: Int
        ) {
            when (newState) {
                BluetoothProfile.STATE_CONNECTED -> {
                    RoverLog.d("BLE connected, discovering services...")
                    RoverState.updateConnectionState(true)
                    gatt.discoverServices()
                }
                BluetoothProfile.STATE_DISCONNECTED -> {
                    RoverLog.d("BLE disconnected (status=$status)")
                    RoverState.updateConnectionState(false)
                    ioCharacteristic = null
                    gatt.close()
                }
            }
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            if (status != BluetoothGatt.GATT_SUCCESS) {
                RoverLog.e("Service discovery failed: $status")
                return
            }

            val service = gatt.getService(SERVICE_UUID)
            if (service == null) {
                RoverLog.e("Service $SERVICE_UUID not found")
                return
            }

            val ch = service.getCharacteristic(CHAR_UUID)
            if (ch == null) {
                RoverLog.e("Characteristic $CHAR_UUID not found")
                return
            }

            ioCharacteristic = ch
            RoverLog.d("Service + characteristic OK, enabling notifications...")

            gatt.setCharacteristicNotification(ch, true)
            val cccd: BluetoothGattDescriptor? = ch.getDescriptor(CCCD_UUID)
            if (cccd != null) {
                cccd.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                gatt.writeDescriptor(cccd)
            } else {
                RoverLog.e("CCCD descriptor not found; notifications may not work")
            }
        }

        override fun onCharacteristicChanged(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic
        ) {
            val data = characteristic.value ?: return
            val hex = data.joinToString(" ") { "%02X".format(it) }
            RoverLog.d("RX: $hex")
            RoverState.feedFromBle(data)
        }
    }
}
