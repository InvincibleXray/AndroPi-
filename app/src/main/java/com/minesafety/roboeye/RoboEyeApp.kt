package com.minesafety.roboeye

import android.app.Application
import android.content.Context
import com.minesafety.roboeye.core.RoboEyeController

/**
 * Process-wide application class holding the singleton [RoboEyeController].
 */
class RoboEyeApp : Application() {

  val controller: RoboEyeController by lazy { RoboEyeController(this) }

  companion object {
    fun controllerOf(context: Context): RoboEyeController =
      (context.applicationContext as RoboEyeApp).controller
  }
}
