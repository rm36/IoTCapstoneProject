package com.google.ar.core.examples.kotlin.helloar

import android.os.Bundle
import android.util.Log
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import com.google.ar.core.Config
import com.google.ar.core.Config.InstantPlacementMode
import com.google.ar.core.Session
import com.google.ar.core.examples.java.common.helpers.CameraPermissionHelper
import com.google.ar.core.examples.java.common.helpers.DepthSettings
import com.google.ar.core.examples.java.common.helpers.FullScreenHelper
import com.google.ar.core.examples.java.common.helpers.InstantPlacementSettings
import com.google.ar.core.examples.java.common.samplerender.SampleRender
import com.google.ar.core.examples.kotlin.common.helpers.ARCoreSessionLifecycleHelper
import com.google.ar.core.exceptions.CameraNotAvailableException
import com.google.ar.core.exceptions.UnavailableApkTooOldException
import com.google.ar.core.exceptions.UnavailableDeviceNotCompatibleException
import com.google.ar.core.exceptions.UnavailableSdkTooOldException
import com.google.ar.core.exceptions.UnavailableUserDeclinedInstallationException
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.bluetooth.le.BluetoothLeScanner
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter

/**
 * AR frontend to locate a beacon emitting an RSSI signal. Based on example HelloAR.
 */
class SignalFollowerArActivity : AppCompatActivity() {
  companion object {
    private const val TAG = "SignalFollowerArActivity"
  }

  lateinit var arCoreSessionHelper: ARCoreSessionLifecycleHelper
  lateinit var view: HelloArView
  lateinit var renderer: SignalFollowerArRenderer

  val instantPlacementSettings = InstantPlacementSettings()
  val depthSettings = DepthSettings()


//  // BLE scanner code
//  private val bleScanner = object : ScanCallback() {
//    override fun onScanResult(callbackType: Int, result: ScanResult?) {
//      Log.d("ScanDeviceActivity", "onScanResult(): ${result?.device?.address} - ${result?.device?.name}")
//    }
//  }
//
//  private val bluetoothLeScanner: BluetoothLeScanner
//    get() {
//      val bluetoothManager = applicationContext.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
//      return bluetoothManager.adapter.bluetoothLeScanner
//    }

  var signalRssi : Float? = null

  // Create a BroadcastReceiver for ACTION_FOUND.
  private val receiver = object : BroadcastReceiver() {

    override fun onReceive(context: Context, intent: Intent) {
      Log.d("Scan", "Scanning")
      when(intent.action) {
        BluetoothDevice.ACTION_FOUND -> {
          // Discovery has found a device. Get the BluetoothDevice
          // object and its info from the Intent.
          val device: BluetoothDevice? =
            intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE)
          val rssi = intent.getShortExtra(BluetoothDevice.EXTRA_RSSI, Short.MIN_VALUE)
            .toFloat()
          Log.d("Scan", "onScanResult(): ${device?.address} - ${device?.name} : $rssi")
          if (device != null && device?.name.contains("pi")) {
            signalRssi = rssi
          }
        }
      }
    }
  }

  fun startTrilateration() {
    Toast.makeText(applicationContext, "Performing Trilateration...", Toast.LENGTH_SHORT).show()
  }

  override fun onCreate(savedInstanceState: Bundle?) {
    super.onCreate(savedInstanceState)

    val bluetoothAdapter = BluetoothAdapter.getDefaultAdapter()
    val intentFilter = IntentFilter()
    intentFilter.addAction(BluetoothAdapter.ACTION_DISCOVERY_STARTED)
    intentFilter.addAction(BluetoothAdapter.ACTION_DISCOVERY_FINISHED)
    intentFilter.addAction(BluetoothAdapter.ACTION_STATE_CHANGED)
    intentFilter.addAction(BluetoothDevice.ACTION_FOUND)
    registerReceiver(receiver, intentFilter)

    if (bluetoothAdapter.isDiscovering) {
      println(bluetoothAdapter.cancelDiscovery())
    }
    if (bluetoothAdapter.scanMode != BluetoothAdapter.SCAN_MODE_CONNECTABLE_DISCOVERABLE) {
      println(BluetoothAdapter.SCAN_MODE_CONNECTABLE_DISCOVERABLE)
    }

    if (!bluetoothAdapter.isEnabled) {
      val enableBtIntent = Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE)
      startActivityForResult(enableBtIntent, 1)
      Toast.makeText(applicationContext, "Bluetooth turned on", Toast.LENGTH_SHORT).show()
    } else {
      Toast.makeText(applicationContext, "Bluetooth is already on", Toast.LENGTH_SHORT).show()
    }


    // Register for broadcasts when a device is discovered.
    val filter = IntentFilter(BluetoothDevice.ACTION_FOUND)
    registerReceiver(receiver, filter)

    // Setup ARCore session lifecycle helper and configuration.
    arCoreSessionHelper = ARCoreSessionLifecycleHelper(this)
    // If Session creation or Session.resume() fails, display a message and log detailed
    // information.
    arCoreSessionHelper.exceptionCallback =
      { exception ->
        val message =
          when (exception) {
            is UnavailableUserDeclinedInstallationException ->
              "Please install Google Play Services for AR"
            is UnavailableApkTooOldException -> "Please update ARCore"
            is UnavailableSdkTooOldException -> "Please update this app"
            is UnavailableDeviceNotCompatibleException -> "This device does not support AR"
            is CameraNotAvailableException -> "Camera not available. Try restarting the app."
            else -> "Failed to create AR session: $exception"
          }
        Log.e(TAG, "ARCore threw an exception", exception)
        view.snackbarHelper.showError(this, message)
      }

    // Configure session features, including: Lighting Estimation, Depth mode, Instant Placement.
    arCoreSessionHelper.beforeSessionResume = ::configureSession
    lifecycle.addObserver(arCoreSessionHelper)

    // Set up the Signal Follower renderer.
    renderer = SignalFollowerArRenderer(this)
    lifecycle.addObserver(renderer)

    // Set up Signal Follower UI.
    view = HelloArView(this)
    lifecycle.addObserver(view)
    setContentView(view.root)

    // Sets up an example renderer using our SignalFollowerARRenderer.
    SampleRender(view.surfaceView, renderer, assets)

    depthSettings.onCreate(this)
    instantPlacementSettings.onCreate(this)
  }

  override fun onDestroy() {
    super.onDestroy()

    // Don't forget to unregister the ACTION_FOUND receiver.
    unregisterReceiver(receiver)
  }

  override fun onStart() {
    Log.d("ScanDeviceActivity", "onStart()")
    super.onStart()

    val pairedDevices: Set<BluetoothDevice>? = BluetoothAdapter.getDefaultAdapter()?.bondedDevices
    pairedDevices?.forEach { device ->
      val deviceName = device.name
      val deviceHardwareAddress = device.address // MAC address
      val rssi = intent.getShortExtra(BluetoothDevice.EXTRA_RSSI, Short.MIN_VALUE)
        .toInt()
      Log.d("Scan", "${device.address} - ${device.name}")
    }


    if (!BluetoothAdapter.getDefaultAdapter().startDiscovery()) {
      val myToast = Toast.makeText(this, "Bluetooth Discovery couldn't start", Toast.LENGTH_SHORT)
      myToast.show()
    }

    val myToast = Toast.makeText(applicationContext, "Initializing...", Toast.LENGTH_SHORT)
    myToast.show()
  }

  override fun onStop() {
    Log.d("ScanDeviceActivity", "onStop()")
    super.onStop()
  }

  // Configure the session, using Lighting Estimation, and Depth mode.
  fun configureSession(session: Session) {
    session.configure(
      session.config.apply {
        lightEstimationMode = Config.LightEstimationMode.ENVIRONMENTAL_HDR

        // Depth API is used if it is configured in Hello AR's settings.
        depthMode =
          if (session.isDepthModeSupported(Config.DepthMode.AUTOMATIC)) {
            Config.DepthMode.AUTOMATIC
          } else {
            Config.DepthMode.DISABLED
          }

        // Instant Placement is used if it is configured in Hello AR's settings.
        instantPlacementMode =
          if (instantPlacementSettings.isInstantPlacementEnabled) {
            InstantPlacementMode.LOCAL_Y_UP
          } else {
            InstantPlacementMode.DISABLED
          }
      }
    )
  }

  override fun onRequestPermissionsResult(
    requestCode: Int,
    permissions: Array<String>,
    results: IntArray
  ) {
    super.onRequestPermissionsResult(requestCode, permissions, results)

    if (!CameraPermissionHelper.hasCameraPermission(this)) {
      // Use toast instead of snackbar here since the activity will exit.
      Toast.makeText(this, "Camera permission is needed to run this application", Toast.LENGTH_LONG)
        .show()
      if (!CameraPermissionHelper.shouldShowRequestPermissionRationale(this)) {
        // Permission denied with checking "Do not ask again".
        CameraPermissionHelper.launchPermissionSettings(this)
      }
      finish()
    }
  }

  override fun onWindowFocusChanged(hasFocus: Boolean) {
    super.onWindowFocusChanged(hasFocus)
    FullScreenHelper.setFullScreenOnWindowFocusChanged(this, hasFocus)
  }
}
