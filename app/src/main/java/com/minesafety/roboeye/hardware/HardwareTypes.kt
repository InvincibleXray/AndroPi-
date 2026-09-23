package com.minesafety.roboeye.hardware

/**
 * High-level device classification for targeted hardware adaptation.
 */
enum class DeviceFamily(val label: String) {
    MI_11X("Xiaomi Mi 11X"),
    S24_ULTRA("Samsung Galaxy S24 Ultra"),
    GENERIC_ANDROID("Generic Android Device"),
}

/**
 * Verified host device identity extracted from Android Build and system properties.
 */
data class DeviceIdentity(
    val manufacturer: String,
    val model: String,
    val brand: String,
    val hardware: String,
    val board: String,
    val socManufacturer: String,
    val socModel: String,
    val androidSdk: Int,
    val androidRelease: String,
    val deviceFamily: DeviceFamily,
    val marketingName: String,
)

/**
 * Host CPU compute capabilities.
 */
data class CpuCapabilities(
    val architecture: String,
    val coreCount: Int,
    val supportedAbis: List<String>,
    val is64Bit: Boolean,
    val instructionSetDetails: String,
)

/**
 * Host GPU compute capabilities.
 */
data class GpuCapabilities(
    val glEsVersion: String,
    val isVulkanSupported: Boolean,
    val vulkanVersion: String,
    val vulkanLevel: Int,
    val renderer: String,
    val vendor: String,
)

/**
 * Honest NPU & Neural Acceleration status.
 */
enum class AccelerationVerificationStatus(val label: String) {
    NOT_AVAILABLE("NOT AVAILABLE"),
    UNKNOWN("UNKNOWN"),
    AVAILABLE_BUT_NOT_VERIFIED_FOR_MODEL("AVAILABLE (UNVERIFIED FOR MODEL)"),
    VERIFIED_FOR_MODEL("VERIFIED FOR MODEL"),
}

/**
 * Neural acceleration discovery report.
 */
data class NeuralAccelerationCapabilities(
    val status: AccelerationVerificationStatus,
    val supportedBackends: List<String>,
    val nnapiAvailable: Boolean,
    val nnapiFeatureLevel: Int?,
    val npuHonestyNote: String,
)

/**
 * Active on-device TFLite inference backend.
 */
enum class InferenceBackend(val label: String) {
    CPU("CPU (Multithreaded)"),
    NNAPI("NNAPI (Neural Networks API)"),
    GPU("GPU (OpenGL/OpenCL)"),
}

/**
 * Runtime execution and fallback diagnostics for the TFLite perception engine.
 */
data class InferenceExecutionReport(
    val backend: InferenceBackend = InferenceBackend.CPU,
    val threadCount: Int = 4,
    val isFallbackActive: Boolean = false,
    val fallbackReason: String? = null,
    val accelerationStatus: AccelerationVerificationStatus = AccelerationVerificationStatus.UNKNOWN,
    val activeModelName: String = "SSD MobileNet V1 INT8 (Quantized UINT8)",
    val preferredModelName: String = "YOLO (TFLite Neural Detector)",
    val runtimePath: String? = null,
)

/**
 * Optical camera classification derived from physical characteristics.
 */
enum class CameraLensType(val label: String) {
    MAIN_WIDE("Main / Wide"),
    ULTRA_WIDE("Ultra-Wide"),
    TELEPHOTO("Telephoto"),
    FRONT("Front Facing"),
    EXTERNAL("External Camera"),
    UNKNOWN("Unknown Lens"),
}

/**
 * Camera lens facing orientation.
 */
enum class LensFacing(val label: String) {
    BACK("Rear"),
    FRONT("Front"),
    EXTERNAL("External"),
}

/**
 * Detailed technical characteristics for an individual camera stream.
 */
data class CameraDeviceInfo(
    val id: String,
    val facing: LensFacing,
    val classification: CameraLensType,
    val focalLengthsMm: List<Float> = emptyList(),
    val sensorPhysicalSizeMm: Pair<Float, Float>? = null,
    val calculatedHfovDeg: Float? = null,
    val equivalent35mmFocalLengthMm: Float? = null,
    val isLogicalMultiCamera: Boolean = false,
    val physicalSubCameraIds: List<String> = emptyList(),
    val maxResolution: Pair<Int, Int>? = null,
    val supportedFpsRanges: List<Pair<Int, Int>> = emptyList(),
    val isAccessible: Boolean = true,
)

/**
 * Aggregated camera discovery matrix.
 */
data class CameraCapabilitySet(
    val mainCamera: CameraDeviceInfo? = null,
    val ultraWideCamera: CameraDeviceInfo? = null,
    val telephotoCameras: List<CameraDeviceInfo> = emptyList(),
    val frontCamera: CameraDeviceInfo? = null,
    val allCameras: List<CameraDeviceInfo> = emptyList(),
    val totalCount: Int = 0,
)

/**
 * Individual physical sensor descriptor.
 */
data class SensorDeviceInfo(
    val type: Int,
    val typeName: String,
    val name: String,
    val vendor: String,
    val version: Int,
    val powerMa: Float,
    val maxRange: Float,
    val resolution: Float,
    val isAvailable: Boolean = true,
)

/**
 * Aggregated on-board sensor capabilities.
 */
data class SensorCapabilitySet(
    val accelerometer: SensorDeviceInfo? = null,
    val gyroscope: SensorDeviceInfo? = null,
    val magnetometer: SensorDeviceInfo? = null,
    val linearAcceleration: SensorDeviceInfo? = null,
    val rotationVector: SensorDeviceInfo? = null,
    val additionalSensors: List<SensorDeviceInfo> = emptyList(),
    val hasMinimumImu: Boolean = false,
)

/**
 * Structured, immutable hardware capability descriptor for the running device.
 */
data class HardwareCapabilities(
    val device: DeviceIdentity,
    val cpu: CpuCapabilities,
    val gpu: GpuCapabilities,
    val neuralAcceleration: NeuralAccelerationCapabilities,
    val cameras: CameraCapabilitySet,
    val sensors: SensorCapabilitySet,
    val inferenceReport: InferenceExecutionReport = InferenceExecutionReport(),
    val discoveryTimestampMs: Long = System.currentTimeMillis(),
)
