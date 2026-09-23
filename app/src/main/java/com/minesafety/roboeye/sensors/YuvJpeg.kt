package com.minesafety.roboeye.sensors

import android.graphics.ImageFormat
import android.graphics.Rect
import android.graphics.YuvImage
import androidx.camera.core.ImageProxy
import java.io.ByteArrayOutputStream

/**
 * Converts a CameraX `YUV_420_888` [ImageProxy] to JPEG bytes for
 * `POST /api/vision/frame`.
 *
 * Uses the platform [YuvImage] encoder (hardware-assisted on most SoCs) rather than a
 * Bitmap round-trip — allocating a 640×480 ARGB_8888 Bitmap per frame would be roughly
 * 1.2 MB of churn per frame on a 2 GB handset.
 *
 * **Rotation is intentionally not applied.** The backend's only consumer of these frames is
 * the visibility heuristic in `ai/visibility.py`, which computes rotation-invariant
 * statistics (mean, std, Laplacian variance, dynamic range) over the whole greyscale image.
 * Rotating would cost a full extra pass for no change in the result. If a frame is ever
 * shown to a human or fed to a detector, rotation must be applied here first.
 */
object YuvJpeg {

  const val DEFAULT_QUALITY = 70

  // Thread-local scratch buffers sized up to 1280×720×1.5 = 1,382,400 bytes to eliminate GC allocations
  private val nv21Scratch = ThreadLocal.withInitial { ByteArray(1280 * 720 * 3 / 2) }
  private val streamScratch = ThreadLocal.withInitial { ByteArrayOutputStream(1280 * 720 / 4) }

  /** Returns JPEG bytes, or null if the image is not a YUV format this encoder handles. */
  fun encode(image: ImageProxy, quality: Int = DEFAULT_QUALITY): ByteArray? {
    if (image.format != ImageFormat.YUV_420_888) return null
    val requiredSize = image.width * image.height + 2 * ((image.width + 1) / 2) * ((image.height + 1) / 2)
    var nv21 = nv21Scratch.get() ?: ByteArray(requiredSize).also { nv21Scratch.set(it) }
    if (nv21.size < requiredSize) {
      nv21 = ByteArray(requiredSize)
      nv21Scratch.set(nv21)
    }

    if (!fillNv21(image, nv21)) return null
    val yuv = YuvImage(nv21, ImageFormat.NV21, image.width, image.height, null)

    val out = streamScratch.get() ?: ByteArrayOutputStream(1280 * 720 / 4).also { streamScratch.set(it) }
    out.reset()
    val ok =
      runCatching {
          yuv.compressToJpeg(Rect(0, 0, image.width, image.height), quality, out)
        }
        .getOrDefault(false)
    return if (ok && out.size() > 0) out.toByteArray() else null
  }

  /**
   * Packs the three planes into NV21 (Y plane followed by interleaved V,U at half
   * resolution), honouring the row and pixel strides CameraX may report.
   */
  private fun fillNv21(image: ImageProxy, out: ByteArray): Boolean {
    val width = image.width
    val height = image.height
    if (width <= 0 || height <= 0) return false
    val planes = image.planes
    if (planes.size < 3) return false

    // ---- Y ----
    val yPlane = planes[0]
    val yBuf = yPlane.buffer
    yBuf.rewind()
    val yRowStride = yPlane.rowStride
    val yPixStride = yPlane.pixelStride
    var pos = 0
    if (yPixStride == 1 && yRowStride == width) {
      val len = minOf(yBuf.remaining(), width * height)
      yBuf.get(out, 0, len)
      pos = len
      if (len < width * height) {
        out.fill(0, len, width * height)
        pos = width * height
      }
    } else {
      val row = ByteArray(yRowStride)
      for (r in 0 until height) {
        val start = r * yRowStride
        if (start < yBuf.limit()) {
          val avail = minOf(yRowStride, yBuf.limit() - start)
          yBuf.position(start)
          yBuf.get(row, 0, avail)
          var src = 0
          for (c in 0 until width) {
            out[pos++] = if (src < avail) row[src] else 0
            src += yPixStride
          }
        } else {
          for (c in 0 until width) {
            out[pos++] = 0
          }
        }
      }
    }

    // ---- VU interleaved ----
    val chromaWidth = (width + 1) / 2
    val chromaHeight = (height + 1) / 2
    val uPlane = planes[1]
    val vPlane = planes[2]
    val uBuf = uPlane.buffer
    val vBuf = vPlane.buffer
    val uRowStride = uPlane.rowStride
    val uPixStride = uPlane.pixelStride
    val vRowStride = vPlane.rowStride
    val vPixStride = vPlane.pixelStride
    val uLimit = uBuf.limit()
    val vLimit = vBuf.limit()
    uBuf.rewind()
    vBuf.rewind()
    for (r in 0 until chromaHeight) {
      for (c in 0 until chromaWidth) {
        val vIdx = r * vRowStride + c * vPixStride
        val uIdx = r * uRowStride + c * uPixStride
        out[pos++] = if (vIdx < vLimit) vBuf.get(vIdx) else 128.toByte()
        out[pos++] = if (uIdx < uLimit) uBuf.get(uIdx) else 128.toByte()
      }
    }
    yBuf.rewind()
    uBuf.rewind()
    vBuf.rewind()
    return true
  }
}
