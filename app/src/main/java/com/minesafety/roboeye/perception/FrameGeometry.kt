package com.minesafety.roboeye.perception

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Matrix
import android.graphics.Paint
import android.graphics.RectF
import com.minesafety.roboeye.core.NormalizedRect

/**
 * Geometry of the camera-frame -> model-tensor transform, and its inverse.
 *
 * A CameraX [androidx.camera.core.ImageProxy] is delivered in *sensor* orientation.
 * `imageInfo.rotationDegrees` is the clockwise rotation required to make that buffer
 * upright for the display. The preview surface applies it; an analyzer must apply it
 * itself, otherwise the detector sees a scene rotated by 90 degrees on a phone held
 * in portrait.
 *
 * Everything downstream of the detector -- [BearingEstimator] (horizontal FOV against
 * `centerX`) and [DistanceEstimator] (vertical extent and `bottom` against the mount
 * height) -- assumes bounding boxes normalized to the **upright** frame. These helpers
 * exist so that assumption actually holds.
 *
 * The math here is deliberately free of `android.graphics` so it is unit-testable on
 * the JVM; [UprightFrameConverter] holds the parts that need a real Canvas.
 */
object FrameGeometry {

    /** Pixel dimensions of the upright frame produced by rotating a buffer. */
    fun uprightSize(bufferWidth: Int, bufferHeight: Int, rotationDegrees: Int): Pair<Int, Int> {
        val normalized = ((rotationDegrees % 360) + 360) % 360
        return if (normalized == 90 || normalized == 270) {
            bufferHeight to bufferWidth
        } else {
            bufferWidth to bufferHeight
        }
    }

    /**
     * Aspect-preserving fit of a [srcWidth] x [srcHeight] frame onto a square
     * [target] x [target] tensor, centred, with the remainder left as padding.
     *
     * Ultralytics trains with letterboxing; a plain stretch to 640x640 feeds the
     * network a distorted aspect ratio it never saw during training.
     */
    data class Letterbox(
        val scale: Float,
        val padX: Float,
        val padY: Float,
        val scaledWidth: Float,
        val scaledHeight: Float,
    )

    fun letterboxFor(srcWidth: Int, srcHeight: Int, target: Int): Letterbox {
        if (srcWidth <= 0 || srcHeight <= 0 || target <= 0) {
            return Letterbox(1f, 0f, 0f, 0f, 0f)
        }
        val scale = minOf(target.toFloat() / srcWidth, target.toFloat() / srcHeight)
        val scaledWidth = srcWidth * scale
        val scaledHeight = srcHeight * scale
        return Letterbox(
            scale = scale,
            padX = (target - scaledWidth) * 0.5f,
            padY = (target - scaledHeight) * 0.5f,
            scaledWidth = scaledWidth,
            scaledHeight = scaledHeight,
        )
    }

    /**
     * Maps a YOLO box, expressed in letterboxed tensor pixel space as
     * (centre, size), back to coordinates normalized to the upright frame.
     *
     * Returns null when the box lies wholly inside the letterbox padding, or is
     * degenerate once clamped to the frame.
     */
    fun toUprightNormalized(
        centerX: Float,
        centerY: Float,
        width: Float,
        height: Float,
        letterbox: Letterbox,
        uprightWidth: Int,
        uprightHeight: Int,
    ): NormalizedRect? {
        if (uprightWidth <= 0 || uprightHeight <= 0 || letterbox.scale <= 0f) return null

        val left = unpad(centerX - width * 0.5f, letterbox.padX, letterbox.scale, uprightWidth)
        val top = unpad(centerY - height * 0.5f, letterbox.padY, letterbox.scale, uprightHeight)
        val right = unpad(centerX + width * 0.5f, letterbox.padX, letterbox.scale, uprightWidth)
        val bottom = unpad(centerY + height * 0.5f, letterbox.padY, letterbox.scale, uprightHeight)

        if (right <= left || bottom <= top) return null
        return NormalizedRect(left = left, top = top, right = right, bottom = bottom)
    }

    private fun unpad(tensorPx: Float, pad: Float, scale: Float, uprightExtent: Int): Float =
        (((tensorPx - pad) / scale) / uprightExtent).coerceIn(0f, 1f)
}

/**
 * Renders an [android.graphics.Bitmap] from the camera into an upright, letterboxed,
 * square tensor bitmap using a single Canvas draw.
 *
 * Rotation, scale and padding are composed into one [Matrix], so the frame is
 * resampled once rather than allocating an intermediate rotated bitmap per frame.
 * The destination bitmap is allocated once and reused.
 */
class UprightFrameConverter(private val target: Int) {

    /** Neutral grey padding, matching the fill Ultralytics letterboxing uses. */
    private val padColor = Color.rgb(114, 114, 114)

    /**
     * Bilinear sampling, for transforms that actually resample.
     *
     * ANTI_ALIAS is deliberately absent: it governs geometry edge coverage, not bitmap
     * sampling, so on a full-frame draw it buys nothing and costs a slower Skia path.
     */
    private val filteredPaint = Paint(Paint.FILTER_BITMAP_FLAG)

    /**
     * Point sampling, used when the matrix is a quarter-turn at unit scale. Source and
     * destination pixel centres coincide there, so filtering would interpolate between a
     * pixel and itself -- identical output, measurably cheaper.
     */
    private val exactPaint = Paint()

    private val matrix = Matrix()
    private val bounds = RectF()

    val bitmap: Bitmap = Bitmap.createBitmap(target, target, Bitmap.Config.ARGB_8888)
    private val canvas = Canvas(bitmap)

    /** Upright frame size of the most recent [render], in pixels. */
    var uprightWidth: Int = 0
        private set
    var uprightHeight: Int = 0
        private set

    /** Letterbox transform of the most recent [render]. */
    var letterbox: FrameGeometry.Letterbox = FrameGeometry.letterboxFor(0, 0, target)
        private set

    /**
     * Draws [source] rotated clockwise by [rotationDegrees] and letterboxed into [bitmap].
     *
     * @param stretchToFill when true the frame fills the square tensor with no padding,
     *   for models trained on a plain resize rather than letterboxing.
     */
    fun render(source: Bitmap, rotationDegrees: Int, stretchToFill: Boolean = false): Bitmap {
        matrix.reset()
        matrix.postRotate(rotationDegrees.toFloat())

        // Rotation about the origin can push content negative; measure and correct.
        bounds.set(0f, 0f, source.width.toFloat(), source.height.toFloat())
        matrix.mapRect(bounds)
        matrix.postTranslate(-bounds.left, -bounds.top)

        uprightWidth = bounds.width().toInt().coerceAtLeast(1)
        uprightHeight = bounds.height().toInt().coerceAtLeast(1)

        letterbox = if (stretchToFill) {
            matrix.postScale(target / bounds.width(), target / bounds.height())
            FrameGeometry.Letterbox(
                scale = 1f, padX = 0f, padY = 0f,
                scaledWidth = target.toFloat(), scaledHeight = target.toFloat(),
            )
        } else {
            val lb = FrameGeometry.letterboxFor(uprightWidth, uprightHeight, target)
            matrix.postScale(lb.scale, lb.scale)
            matrix.postTranslate(lb.padX, lb.padY)
            lb
        }

        val isQuarterTurn = (((rotationDegrees % 360) + 360) % 360) % 90 == 0
        val isUnitScale = !stretchToFill && kotlin.math.abs(letterbox.scale - 1f) < 1e-4f
        val paint = if (isQuarterTurn && isUnitScale) exactPaint else filteredPaint

        canvas.drawColor(padColor)
        canvas.drawBitmap(source, matrix, paint)
        return bitmap
    }

    fun recycle() {
        if (!bitmap.isRecycled) bitmap.recycle()
    }
}
