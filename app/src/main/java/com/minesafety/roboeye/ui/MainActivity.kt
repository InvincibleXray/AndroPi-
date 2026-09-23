package com.minesafety.roboeye.ui

import android.Manifest
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.result.ActivityResultLauncher
import androidx.activity.result.contract.ActivityResultContracts
import androidx.activity.viewModels
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Surface
import androidx.compose.ui.Modifier
import androidx.core.content.ContextCompat
import com.minesafety.roboeye.ui.theme.RoboEyeTheme

/**
 * Main Activity for Robo Eye.
 * Hosts the full-screen landscape Compose interface.
 */
class MainActivity : ComponentActivity() {

  private val vm: RoboEyeViewModel by viewModels()
  private lateinit var permissionLauncher: ActivityResultLauncher<Array<String>>

  override fun onCreate(savedInstanceState: Bundle?) {
    super.onCreate(savedInstanceState)

    permissionLauncher =
      registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) {
        vm.onPermissionsChanged()
      }

    enableEdgeToEdge()
    setContent {
      RoboEyeTheme {
        Surface(
          modifier = Modifier.fillMaxSize(),
          color = MaterialTheme.colorScheme.background,
        ) {
          RoboEyeScreen(viewModel = vm)
        }
      }
    }

    if (missingPermissions().isNotEmpty()) requestNodePermissions()
    vm.maybeAutoStart()
  }

  override fun onResume() {
    super.onResume()
    vm.onPermissionsChanged()
  }

  private fun requestNodePermissions() {
    val missing = missingPermissions()
    if (missing.isEmpty()) {
      vm.onPermissionsChanged()
      return
    }
    permissionLauncher.launch(missing.toTypedArray())
  }

  private fun missingPermissions(): List<String> {
    val wanted = mutableListOf(Manifest.permission.CAMERA, Manifest.permission.ACCESS_FINE_LOCATION)
    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
      wanted += Manifest.permission.POST_NOTIFICATIONS
    }
    return wanted.filter {
      ContextCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED
    }
  }
}
