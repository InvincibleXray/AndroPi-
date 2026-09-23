package com.minesafety.roboeye.ui.theme

import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.Composable

private val DarkColorScheme = darkColorScheme(
    primary = RadarCyan,
    secondary = RadarAmber,
    tertiary = RadarBlue,
    background = RadarDarkBg,
    surface = RadarCardBg,
    onPrimary = RadarDarkBg,
    onSecondary = RadarDarkBg,
    onBackground = TextPrimary,
    onSurface = TextPrimary,
)

@Composable
fun RoboRadarTheme(content: @Composable () -> Unit) {
    MaterialTheme(
        colorScheme = DarkColorScheme,
        typography = Typography,
        content = content
    )
}

@Composable
fun RoboEyeTheme(content: @Composable () -> Unit) {
    MaterialTheme(
        colorScheme = DarkColorScheme,
        typography = Typography,
        content = content
    )
}
