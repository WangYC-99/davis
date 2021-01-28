#include "dvs132s.h"

#include <math.h>

static void dvs132sLog(enum caer_log_level logLevel, dvs132sHandle handle, const char *format, ...) ATTRIBUTE_FORMAT(3);
static bool dvs132sSendDefaultFPGAConfig(caerDeviceHandle cdh);
static bool dvs132sSendDefaultBiasConfig(caerDeviceHandle cdh);
static void dvs132sEventTranslator(void *vhd, const uint8_t *buffer, size_t bytesSent);
static void dvs132sTSMasterStatusUpdater(void *userDataPtr, int status, uint32_t param);

// FX3 Debug Transfer Support
static void allocateDebugTransfers(dvs132sHandle handle);
static void cancelAndDeallocateDebugTransfers(dvs132sHandle handle);
static void LIBUSB_CALL libUsbDebugCallback(struct libusb_transfer *transfer);
static void debugTranslator(dvs132sHandle handle, const uint8_t *buffer, size_t bytesSent);

static void dvs132sLog(enum caer_log_level logLevel, dvs132sHandle handle, const char *format, ...) {
	va_list argumentList;
	va_start(argumentList, format);
	caerLogVAFull(atomic_load_explicit(&handle->state.deviceLogLevel, memory_order_relaxed), logLevel,
		handle->info.deviceString, format, argumentList);
	va_end(argumentList);
}

static void populateDeviceInfo(
	caerDeviceDiscoveryResult result, struct usb_info *usbInfo, libusb_device_handle *devHandle) {
	// This is a DVS132S.
	result->deviceType         = CAER_DEVICE_DVS132S;
	result->deviceErrorOpen    = usbInfo->errorOpen;
	result->deviceErrorVersion = usbInfo->errorVersion;

	struct caer_dvs132s_info *dvs132sInfoPtr = &(result->deviceInfo.dvs132sInfo);

	// SN, BusNumber, DevAddress always defined.
	// FirmwareVersion and LogicVersion either defined or zero.
	strncpy(dvs132sInfoPtr->deviceSerialNumber, usbInfo->serialNumber, MAX_SERIAL_NUMBER_LENGTH + 1);
	dvs132sInfoPtr->deviceUSBBusNumber     = usbInfo->busNumber;
	dvs132sInfoPtr->deviceUSBDeviceAddress = usbInfo->devAddress;
	dvs132sInfoPtr->firmwareVersion        = usbInfo->firmwareVersion;
	dvs132sInfoPtr->logicVersion           = usbInfo->logicVersion;

	if (devHandle != NULL) {
		// Populate info variables based on data from device.
		uint32_t param32 = 0;

		startupSPIConfigReceive(devHandle, DVS132S_CONFIG_SYSINFO, DVS132S_CONFIG_SYSINFO_CHIP_IDENTIFIER, &param32);
		dvs132sInfoPtr->chipID = I16T(param32);
		startupSPIConfigReceive(devHandle, DVS132S_CONFIG_SYSINFO, DVS132S_CONFIG_SYSINFO_DEVICE_IS_MASTER, &param32);
		dvs132sInfoPtr->deviceIsMaster = param32;

		startupSPIConfigReceive(devHandle, DVS132S_CONFIG_DVS, DVS132S_CONFIG_DVS_SIZE_COLUMNS, &param32);
		int16_t dvsSizeX = I16T(param32);
		startupSPIConfigReceive(devHandle, DVS132S_CONFIG_DVS, DVS132S_CONFIG_DVS_SIZE_ROWS, &param32);
		int16_t dvsSizeY = I16T(param32);

		startupSPIConfigReceive(devHandle, DVS132S_CONFIG_DVS, DVS132S_CONFIG_DVS_ORIENTATION_INFO, &param32);
		bool dvsInvertXY = param32 & 0x04;

		if (dvsInvertXY) {
			dvs132sInfoPtr->dvsSizeX = dvsSizeY;
			dvs132sInfoPtr->dvsSizeY = dvsSizeX;
		}
		else {
			dvs132sInfoPtr->dvsSizeX = dvsSizeX;
			dvs132sInfoPtr->dvsSizeY = dvsSizeY;
		}

		startupSPIConfigReceive(devHandle, DVS132S_CONFIG_IMU, DVS132S_CONFIG_IMU_TYPE, &param32);
		dvs132sInfoPtr->imuType = U8T(param32);

		// Extra features:
		startupSPIConfigReceive(devHandle, DVS132S_CONFIG_MUX, DVS132S_CONFIG_MUX_HAS_STATISTICS, &param32);
		dvs132sInfoPtr->muxHasStatistics = param32;

		startupSPIConfigReceive(devHandle, DVS132S_CONFIG_DVS, DVS132S_CONFIG_DVS_HAS_STATISTICS, &param32);
		dvs132sInfoPtr->dvsHasStatistics = param32;

		startupSPIConfigReceive(devHandle, DVS132S_CONFIG_EXTINPUT, DVS132S_CONFIG_EXTINPUT_HAS_GENERATOR, &param32);
		dvs132sInfoPtr->extInputHasGenerator = param32;
	}

	// Always unset here.
	dvs132sInfoPtr->deviceID     = -1;
	dvs132sInfoPtr->deviceString = NULL;
}

ssize_t dvs132sFind(caerDeviceDiscoveryResult *discoveredDevices) {
	return (usbDeviceFind(USB_DEFAULT_DEVICE_VID, DVS132S_DEVICE_PID, DVS132S_REQUIRED_LOGIC_VERSION,
		DVS132S_REQUIRED_LOGIC_PATCH_LEVEL, DVS132S_REQUIRED_FIRMWARE_VERSION, discoveredDevices, &populateDeviceInfo));
}

static inline float calculateIMUAccelScale(uint8_t imuAccelScale) {
	// Accelerometer scale is:
	// 0 - +- 2 g  - 16384 LSB/g
	// 1 - +- 4 g  - 8192 LSB/g
	// 2 - +- 8 g  - 4096 LSB/g
	// 3 - +- 16 g - 2048 LSB/g
	float accelScale = 65536.0F / (float) U32T(4 * (1 << imuAccelScale));

	return (accelScale);
}

static inline float calculateIMUGyroScale(uint8_t imuGyroScale) {
	// Invert for ascending scale:
	uint8_t imuGyroScaleAsc = U8T(4 - imuGyroScale);

	// Gyroscope ascending scale is:
	// 0 - +- 125 °/s  - 262.4 LSB/°/s
	// 1 - +- 250 °/s  - 131.2 LSB/°/s
	// 2 - +- 500 °/s  - 65.6 LSB/°/s
	// 3 - +- 1000 °/s - 32.8 LSB/°/s
	// 4 - +- 2000 °/s - 16.4 LSB/°/s
	float gyroScale = 65536.0F / (float) U32T(250 * (1 << imuGyroScaleAsc));

	return (gyroScale);
}

static inline void freeAllDataMemory(dvs132sState state) {
	dataExchangeDestroy(&state->dataExchange);

	// Since the current event packets aren't necessarily
	// already assigned to the current packet container, we
	// free them separately from it.
	if (state->currentPackets.polarity != NULL) {
		free(&state->currentPackets.polarity->packetHeader);
		state->currentPackets.polarity = NULL;

		containerGenerationSetPacket(&state->container, POLARITY_EVENT, NULL);
	}

	if (state->currentPackets.special != NULL) {
		free(&state->currentPackets.special->packetHeader);
		state->currentPackets.special = NULL;

		containerGenerationSetPacket(&state->container, SPECIAL_EVENT, NULL);
	}

	if (state->currentPackets.imu6 != NULL) {
		free(&state->currentPackets.imu6->packetHeader);
		state->currentPackets.imu6 = NULL;

		containerGenerationSetPacket(&state->container, IMU6_EVENT_PKT_POS, NULL);
	}

	containerGenerationDestroy(&state->container);
}

caerDeviceHandle dvs132sOpen(
	uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict, const char *serialNumberRestrict) {
	errno = 0;

	caerLog(CAER_LOG_DEBUG, __func__, "Initializing %s.", DVS132S_DEVICE_NAME);

	dvs132sHandle handle = calloc(1, sizeof(*handle));
	if (handle == NULL) {
		// Failed to allocate memory for device handle!
		caerLog(CAER_LOG_CRITICAL, __func__, "Failed to allocate memory for device handle.");
		errno = CAER_ERROR_MEMORY_ALLOCATION;
		return (NULL);
	}

	// Set main deviceType correctly right away.
	handle->deviceType = CAER_DEVICE_DVS132S;

	dvs132sState state = &handle->state;

	// Initialize state variables to default values (if not zero, taken care of by calloc above).
	dataExchangeSettingsInit(&state->dataExchange);

	// Packet settings (size (in events) and time interval (in µs)).
	containerGenerationSettingsInit(&state->container);

	// Logging settings (initialize to global log-level).
	enum caer_log_level globalLogLevel = caerLogLevelGet();
	atomic_store(&state->deviceLogLevel, globalLogLevel);
	usbSetLogLevel(&state->usbState, globalLogLevel);

	// Set device thread name. Maximum length of 15 chars due to Linux limitations.
	char usbThreadName[MAX_THREAD_NAME_LENGTH + 1];
	snprintf(usbThreadName, MAX_THREAD_NAME_LENGTH + 1, "%s %" PRIu16, DVS132S_DEVICE_NAME, deviceID);
	usbThreadName[MAX_THREAD_NAME_LENGTH] = '\0';

	usbSetThreadName(&state->usbState, usbThreadName);
	handle->info.deviceString = usbThreadName; // Temporary, until replaced by full string.

	// Try to open a DVS132S device on a specific USB port.
	struct caer_device_discovery_result deviceInfo;

	if (!usbDeviceOpen(&state->usbState, USB_DEFAULT_DEVICE_VID, DVS132S_DEVICE_PID, busNumberRestrict,
			devAddressRestrict, serialNumberRestrict, DVS132S_REQUIRED_LOGIC_VERSION,
			DVS132S_REQUIRED_LOGIC_PATCH_LEVEL, DVS132S_REQUIRED_FIRMWARE_VERSION, &deviceInfo, &populateDeviceInfo)) {
		if (errno == CAER_ERROR_OPEN_ACCESS) {
			dvs132sLog(
				CAER_LOG_CRITICAL, handle, "Failed to open device, no matching device could be found or opened.");
		}
		else {
			dvs132sLog(CAER_LOG_CRITICAL, handle,
				"Failed to open device, see above log message for more information (errno=%d).", errno);
		}

		free(handle);

		// errno set by usbDeviceOpen().
		return (NULL);
	}

	// At this point we can get some more precise data on the device and update
	// the logging string to reflect that and be more informative.
	char *usbInfoString = malloc(USB_INFO_STRING_SIZE);
	if (usbInfoString == NULL) {
		dvs132sLog(CAER_LOG_CRITICAL, handle, "Failed to generate USB information string.");

		usbDeviceClose(&state->usbState);
		free(handle);

		errno = CAER_ERROR_MEMORY_ALLOCATION;
		return (NULL);
	}

	snprintf(usbInfoString, USB_INFO_STRING_SIZE, DVS132S_DEVICE_NAME " ID-%" PRIu16 " SN-%s [%" PRIu8 ":%" PRIu8 "]",
		deviceID, deviceInfo.deviceInfo.dvs132sInfo.deviceSerialNumber,
		deviceInfo.deviceInfo.dvs132sInfo.deviceUSBBusNumber, deviceInfo.deviceInfo.dvs132sInfo.deviceUSBDeviceAddress);

	// Setup USB.
	usbSetDataCallback(&state->usbState, &dvs132sEventTranslator, handle);
	usbSetDataEndpoint(&state->usbState, USB_DEFAULT_DATA_ENDPOINT);
	usbSetTransfersNumber(&state->usbState, 8);
	usbSetTransfersSize(&state->usbState, 8192);

	// Start USB handling thread.
	if (!usbThreadStart(&state->usbState)) {
		usbDeviceClose(&state->usbState);
		free(usbInfoString);
		free(handle);

		errno = CAER_ERROR_COMMUNICATION;
		return (NULL);
	}

	// Populate info variables based on data from device.
	handle->info = deviceInfo.deviceInfo.dvs132sInfo;

	handle->info.deviceID     = I16T(deviceID);
	handle->info.deviceString = usbInfoString;

	uint32_t param32 = 0;

	spiConfigReceive(&state->usbState, DVS132S_CONFIG_SYSINFO, DVS132S_CONFIG_SYSINFO_LOGIC_CLOCK, &param32);
	state->deviceClocks.logicClock = U16T(param32);
	spiConfigReceive(&state->usbState, DVS132S_CONFIG_SYSINFO, DVS132S_CONFIG_SYSINFO_USB_CLOCK, &param32);
	state->deviceClocks.usbClock = U16T(param32);
	spiConfigReceive(&state->usbState, DVS132S_CONFIG_SYSINFO, DVS132S_CONFIG_SYSINFO_CLOCK_DEVIATION, &param32);
	state->deviceClocks.clockDeviationFactor = U16T(param32);

	// Calculate actual clock frequencies.
	state->deviceClocks.logicClockActual = (float) ((double) state->deviceClocks.logicClock
													* ((double) state->deviceClocks.clockDeviationFactor / 1000.0));
	state->deviceClocks.usbClockActual   = (float) ((double) state->deviceClocks.usbClock
                                                  * ((double) state->deviceClocks.clockDeviationFactor / 1000.0));

	dvs132sLog(CAER_LOG_DEBUG, handle, "Clock frequencies: LOGIC %f, USB %f.",
		(double) state->deviceClocks.logicClockActual, (double) state->deviceClocks.usbClockActual);

	spiConfigReceive(&state->usbState, DVS132S_CONFIG_DVS, DVS132S_CONFIG_DVS_SIZE_COLUMNS, &param32);
	state->dvs.sizeX = I16T(param32);
	spiConfigReceive(&state->usbState, DVS132S_CONFIG_DVS, DVS132S_CONFIG_DVS_SIZE_ROWS, &param32);
	state->dvs.sizeY = I16T(param32);

	spiConfigReceive(&state->usbState, DVS132S_CONFIG_DVS, DVS132S_CONFIG_DVS_ORIENTATION_INFO, &param32);
	state->dvs.invertXY = param32 & 0x04;

	dvs132sLog(CAER_LOG_DEBUG, handle, "DVS Size X: %d, Size Y: %d, Invert: %d.", state->dvs.sizeX, state->dvs.sizeY,
		state->dvs.invertXY);

	spiConfigReceive(&state->usbState, DVS132S_CONFIG_IMU, DVS132S_CONFIG_IMU_ORIENTATION_INFO, &param32);
	state->imu.flipX = param32 & 0x04;
	state->imu.flipY = param32 & 0x02;
	state->imu.flipZ = param32 & 0x01;

	dvs132sLog(CAER_LOG_DEBUG, handle, "IMU Flip X: %d, Flip Y: %d, Flip Z: %d.", state->imu.flipX, state->imu.flipY,
		state->imu.flipZ);

	// On FX3, start the debug transfers once everything else is ready.
	allocateDebugTransfers(handle);

	dvs132sLog(CAER_LOG_DEBUG, handle, "Initialized device successfully with USB Bus=%" PRIu8 ":Addr=%" PRIu8 ".",
		handle->info.deviceUSBBusNumber, handle->info.deviceUSBDeviceAddress);

	return ((caerDeviceHandle) handle);
}

bool dvs132sClose(caerDeviceHandle cdh) {
	dvs132sHandle handle = (dvs132sHandle) cdh;
	dvs132sState state   = &handle->state;

	dvs132sLog(CAER_LOG_DEBUG, handle, "Shutting down ...");

	// Stop debug transfers on FX3 devices.
	cancelAndDeallocateDebugTransfers(handle);

	// Shut down USB handling thread.
	usbThreadStop(&state->usbState);

	// Finally, close the device fully.
	usbDeviceClose(&state->usbState);

	dvs132sLog(CAER_LOG_DEBUG, handle, "Shutdown successful.");

	// Free memory.
	free(handle->info.deviceString);
	free(handle);

	return (true);
}

struct caer_dvs132s_info caerDVS132SInfoGet(caerDeviceHandle cdh) {
	dvs132sHandle handle = (dvs132sHandle) cdh;

	// Check if the pointer is valid.
	if (handle == NULL) {
		struct caer_dvs132s_info emptyInfo = {0, .deviceString = NULL};
		return (emptyInfo);
	}

	// Check if device type is supported.
	if (handle->deviceType != CAER_DEVICE_DVS132S) {
		struct caer_dvs132s_info emptyInfo = {0, .deviceString = NULL};
		return (emptyInfo);
	}

	// Return a copy of the device information.
	return (handle->info);
}

bool dvs132sSendDefaultConfig(caerDeviceHandle cdh) {
	// First send default bias config.
	if (!dvs132sSendDefaultBiasConfig(cdh)) {
		return (false);
	}

	// Send default FPGA config.
	if (!dvs132sSendDefaultFPGAConfig(cdh)) {
		return (false);
	}

	return (true);
}

static bool dvs132sSendDefaultFPGAConfig(caerDeviceHandle cdh) {
	dvs132sHandle handle = (dvs132sHandle) cdh;

	dvs132sConfigSet(cdh, DVS132S_CONFIG_MUX, DVS132S_CONFIG_MUX_TIMESTAMP_RESET, false);
	dvs132sConfigSet(cdh, DVS132S_CONFIG_MUX, DVS132S_CONFIG_MUX_DROP_EXTINPUT_ON_TRANSFER_STALL, true);
	dvs132sConfigSet(cdh, DVS132S_CONFIG_MUX, DVS132S_CONFIG_MUX_DROP_DVS_ON_TRANSFER_STALL, false);

	dvs132sConfigSet(cdh, DVS132S_CONFIG_DVS, DVS132S_CONFIG_DVS_WAIT_ON_TRANSFER_STALL, true);
	dvs132sConfigSet(cdh, DVS132S_CONFIG_DVS, DVS132S_CONFIG_DVS_FILTER_AT_LEAST_2_UNSIGNED, false);
	dvs132sConfigSet(cdh, DVS132S_CONFIG_DVS, DVS132S_CONFIG_DVS_FILTER_NOT_ALL_4_UNSIGNED, false);
	dvs132sConfigSet(cdh, DVS132S_CONFIG_DVS, DVS132S_CONFIG_DVS_FILTER_AT_LEAST_2_SIGNED, false);
	dvs132sConfigSet(cdh, DVS132S_CONFIG_DVS, DVS132S_CONFIG_DVS_FILTER_NOT_ALL_4_SIGNED, false);
	dvs132sConfigSet(cdh, DVS132S_CONFIG_DVS, DVS132S_CONFIG_DVS_RESTART_TIME, 100);     // in µs
	dvs132sConfigSet(cdh, DVS132S_CONFIG_DVS, DVS132S_CONFIG_DVS_CAPTURE_INTERVAL, 500); // in µs
	dvs132sConfigSet(cdh, DVS132S_CONFIG_DVS, DVS132S_CONFIG_DVS_ROW_ENABLE_31_TO_0, 0xFFFFFFFF);
	dvs132sConfigSet(cdh, DVS132S_CONFIG_DVS, DVS132S_CONFIG_DVS_ROW_ENABLE_63_TO_32, 0xFFFFFFFF);
	dvs132sConfigSet(cdh, DVS132S_CONFIG_DVS, DVS132S_CONFIG_DVS_ROW_ENABLE_65_TO_64, 0x03);
	dvs132sConfigSet(cdh, DVS132S_CONFIG_DVS, DVS132S_CONFIG_DVS_COLUMN_ENABLE_31_TO_0, 0xFFFFFFFF);
	dvs132sConfigSet(cdh, DVS132S_CONFIG_DVS, DVS132S_CONFIG_DVS_COLUMN_ENABLE_51_TO_32, 0x0FFFFF);

	dvs132sConfigSet(cdh, DVS132S_CONFIG_IMU, DVS132S_CONFIG_IMU_ACCEL_DATA_RATE, BOSCH_ACCEL_800HZ); // 800 Hz.
	dvs132sConfigSet(cdh, DVS132S_CONFIG_IMU, DVS132S_CONFIG_IMU_ACCEL_FILTER, BOSCH_ACCEL_NORMAL);   // Normal mode.
	dvs132sConfigSet(cdh, DVS132S_CONFIG_IMU, DVS132S_CONFIG_IMU_ACCEL_RANGE, BOSCH_ACCEL_4G);        // +- 4 g.
	dvs132sConfigSet(cdh, DVS132S_CONFIG_IMU, DVS132S_CONFIG_IMU_GYRO_DATA_RATE, BOSCH_GYRO_800HZ);   // 800 Hz.
	dvs132sConfigSet(cdh, DVS132S_CONFIG_IMU, DVS132S_CONFIG_IMU_GYRO_FILTER, BOSCH_GYRO_NORMAL);     // Normal mode.
	dvs132sConfigSet(cdh, DVS132S_CONFIG_IMU, DVS132S_CONFIG_IMU_GYRO_RANGE, BOSCH_GYRO_500DPS);      // +- 500 °/s

	dvs132sConfigSet(cdh, DVS132S_CONFIG_EXTINPUT, DVS132S_CONFIG_EXTINPUT_DETECT_RISING_EDGES, false);
	dvs132sConfigSet(cdh, DVS132S_CONFIG_EXTINPUT, DVS132S_CONFIG_EXTINPUT_DETECT_FALLING_EDGES, false);
	dvs132sConfigSet(cdh, DVS132S_CONFIG_EXTINPUT, DVS132S_CONFIG_EXTINPUT_DETECT_PULSES, true);
	dvs132sConfigSet(cdh, DVS132S_CONFIG_EXTINPUT, DVS132S_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY, true);
	dvs132sConfigSet(cdh, DVS132S_CONFIG_EXTINPUT, DVS132S_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH,
		10); // in µs, converted to cycles @ LogicClock later

	if (handle->info.extInputHasGenerator) {
		// Disable generator by default. Has to be enabled manually after sendDefaultConfig() by user!
		dvs132sConfigSet(cdh, DVS132S_CONFIG_EXTINPUT, DVS132S_CONFIG_EXTINPUT_RUN_GENERATOR, false);
		dvs132sConfigSet(cdh, DVS132S_CONFIG_EXTINPUT, DVS132S_CONFIG_EXTINPUT_GENERATE_PULSE_POLARITY, true);
		dvs132sConfigSet(cdh, DVS132S_CONFIG_EXTINPUT, DVS132S_CONFIG_EXTINPUT_GENERATE_PULSE_INTERVAL,
			10); // in µs, converted to cycles @ LogicClock later
		dvs132sConfigSet(cdh, DVS132S_CONFIG_EXTINPUT, DVS132S_CONFIG_EXTINPUT_GENERATE_PULSE_LENGTH,
			5); // in µs, converted to cycles @ LogicClock later
		dvs132sConfigSet(cdh, DVS132S_CONFIG_EXTINPUT, DVS132S_CONFIG_EXTINPUT_GENERATE_INJECT_ON_RISING_EDGE, false);
		dvs132sConfigSet(cdh, DVS132S_CONFIG_EXTINPUT, DVS132S_CONFIG_EXTINPUT_GENERATE_INJECT_ON_FALLING_EDGE, false);
	}

	dvs132sConfigSet(
		cdh, DVS132S_CONFIG_USB, DVS132S_CONFIG_USB_EARLY_PACKET_DELAY, 8); // in 125µs time-slices (defaults to 1ms)

	return (true);
}

static bool dvs132sSendDefaultBiasConfig(caerDeviceHandle cdh) {
	// Default bias configuration.
	dvs132sConfigSet(cdh, DVS132S_CONFIG_BIAS, DVS132S_CONFIG_BIAS_PRBP,
		caerBiasCoarseFine1024Generate(caerBiasCoarseFine1024FromCurrent(100 * 1000)));
	dvs132sConfigSet(cdh, DVS132S_CONFIG_BIAS, DVS132S_CONFIG_BIAS_PRSFBP,
		caerBiasCoarseFine1024Generate(caerBiasCoarseFine1024FromCurrent(1)));
	dvs132sConfigSet(cdh, DVS132S_CONFIG_BIAS, DVS132S_CONFIG_BIAS_BLPUBP,
		caerBiasCoarseFine1024Generate(caerBiasCoarseFine1024FromCurrent(0)));
	dvs132sConfigSet(cdh, DVS132S_CONFIG_BIAS, DVS132S_CONFIG_BIAS_BIASBUFBP,
		caerBiasCoarseFine1024Generate(caerBiasCoarseFine1024FromCurrent(10 * 1000)));
	dvs132sConfigSet(cdh, DVS132S_CONFIG_BIAS, DVS132S_CONFIG_BIAS_OFFBN,
		caerBiasCoarseFine1024Generate(caerBiasCoarseFine1024FromCurrent(200)));
	dvs132sConfigSet(cdh, DVS132S_CONFIG_BIAS, DVS132S_CONFIG_BIAS_DIFFBN,
		caerBiasCoarseFine1024Generate(caerBiasCoarseFine1024FromCurrent(10 * 1000)));
	dvs132sConfigSet(cdh, DVS132S_CONFIG_BIAS, DVS132S_CONFIG_BIAS_ONBN,
		caerBiasCoarseFine1024Generate(caerBiasCoarseFine1024FromCurrent(400 * 1000)));
	dvs132sConfigSet(cdh, DVS132S_CONFIG_BIAS, DVS132S_CONFIG_BIAS_CASBN,
		caerBiasCoarseFine1024Generate(caerBiasCoarseFine1024FromCurrent(400 * 1000)));
	dvs132sConfigSet(cdh, DVS132S_CONFIG_BIAS, DVS132S_CONFIG_BIAS_DPBN,
		caerBiasCoarseFine1024Generate(caerBiasCoarseFine1024FromCurrent(100 * 1000)));
	dvs132sConfigSet(cdh, DVS132S_CONFIG_BIAS, DVS132S_CONFIG_BIAS_BIASBUFBN,
		caerBiasCoarseFine1024Generate(caerBiasCoarseFine1024FromCurrent(10 * 1000)));
	dvs132sConfigSet(cdh, DVS132S_CONFIG_BIAS, DVS132S_CONFIG_BIAS_ABUFBN,
		caerBiasCoarseFine1024Generate(caerBiasCoarseFine1024FromCurrent(0)));

	return (true);
}

bool dvs132sConfigSet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t param) {
	dvs132sHandle handle = (dvs132sHandle) cdh;
	dvs132sState state   = &handle->state;

	switch (modAddr) {
		case CAER_HOST_CONFIG_USB:
			return (usbConfigSet(&state->usbState, paramAddr, param));
			break;

		case CAER_HOST_CONFIG_DATAEXCHANGE:
			return (dataExchangeConfigSet(&state->dataExchange, paramAddr, param));
			break;

		case CAER_HOST_CONFIG_PACKETS:
			return (containerGenerationConfigSet(&state->container, paramAddr, param));
			break;

		case CAER_HOST_CONFIG_LOG:
			switch (paramAddr) {
				case CAER_HOST_CONFIG_LOG_LEVEL:
					atomic_store(&state->deviceLogLevel, U8T(param));

					// Set USB log-level to this value too.
					usbSetLogLevel(&state->usbState, param);
					break;

				default:
					return (false);
					break;
			}
			break;

		case DVS132S_CONFIG_MUX:
			switch (paramAddr) {
				case DVS132S_CONFIG_MUX_RUN:
				case DVS132S_CONFIG_MUX_TIMESTAMP_RUN:
				case DVS132S_CONFIG_MUX_RUN_CHIP:
				case DVS132S_CONFIG_MUX_DROP_EXTINPUT_ON_TRANSFER_STALL:
				case DVS132S_CONFIG_MUX_DROP_DVS_ON_TRANSFER_STALL:
					return (spiConfigSend(&state->usbState, DVS132S_CONFIG_MUX, paramAddr, param));
					break;

				case DVS132S_CONFIG_MUX_TIMESTAMP_RESET: {
					// Use multi-command VR for more efficient implementation of reset,
					// that also guarantees returning to the default state.
					if (param) {
						struct spi_config_params spiMultiConfig[2];

						spiMultiConfig[0].moduleAddr = DVS132S_CONFIG_MUX;
						spiMultiConfig[0].paramAddr  = DVS132S_CONFIG_MUX_TIMESTAMP_RESET;
						spiMultiConfig[0].param      = true;

						spiMultiConfig[1].moduleAddr = DVS132S_CONFIG_MUX;
						spiMultiConfig[1].paramAddr  = DVS132S_CONFIG_MUX_TIMESTAMP_RESET;
						spiMultiConfig[1].param      = false;

						return (spiConfigSendMultiple(&state->usbState, spiMultiConfig, 2));
					}
					break;
				}

				default:
					return (false);
					break;
			}
			break;

		case DVS132S_CONFIG_DVS:
			switch (paramAddr) {
				case DVS132S_CONFIG_DVS_RUN:
				case DVS132S_CONFIG_DVS_WAIT_ON_TRANSFER_STALL:
				case DVS132S_CONFIG_DVS_FILTER_AT_LEAST_2_UNSIGNED:
				case DVS132S_CONFIG_DVS_FILTER_NOT_ALL_4_UNSIGNED:
				case DVS132S_CONFIG_DVS_FILTER_AT_LEAST_2_SIGNED:
				case DVS132S_CONFIG_DVS_FILTER_NOT_ALL_4_SIGNED:
				case DVS132S_CONFIG_DVS_ROW_ENABLE_31_TO_0:
				case DVS132S_CONFIG_DVS_ROW_ENABLE_63_TO_32:
				case DVS132S_CONFIG_DVS_ROW_ENABLE_65_TO_64:
				case DVS132S_CONFIG_DVS_COLUMN_ENABLE_31_TO_0:
				case DVS132S_CONFIG_DVS_COLUMN_ENABLE_51_TO_32:
					return (spiConfigSend(&state->usbState, DVS132S_CONFIG_DVS, paramAddr, param));
					break;

				case DVS132S_CONFIG_DVS_RESTART_TIME:
				case DVS132S_CONFIG_DVS_CAPTURE_INTERVAL: {
					// DVS long times are in µs on host, but in cycles @ LOGIC_CLOCK_FREQ
					// on FPGA, so we must multiply here.
					float delayCC = roundf((float) param * state->deviceClocks.logicClockActual);
					return (spiConfigSend(&state->usbState, DVS132S_CONFIG_DVS, paramAddr, U32T(delayCC)));
					break;
				}

				default:
					return (false);
					break;
			}
			break;

		case DVS132S_CONFIG_IMU:
			switch (paramAddr) {
				case DVS132S_CONFIG_IMU_RUN_ACCELEROMETER:
				case DVS132S_CONFIG_IMU_RUN_GYROSCOPE:
				case DVS132S_CONFIG_IMU_RUN_TEMPERATURE:
				case DVS132S_CONFIG_IMU_ACCEL_DATA_RATE:
				case DVS132S_CONFIG_IMU_ACCEL_FILTER:
				case DVS132S_CONFIG_IMU_ACCEL_RANGE:
				case DVS132S_CONFIG_IMU_GYRO_DATA_RATE:
				case DVS132S_CONFIG_IMU_GYRO_FILTER:
				case DVS132S_CONFIG_IMU_GYRO_RANGE:
					return (spiConfigSend(&state->usbState, DVS132S_CONFIG_IMU, paramAddr, param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case DVS132S_CONFIG_EXTINPUT:
			switch (paramAddr) {
				case DVS132S_CONFIG_EXTINPUT_RUN_DETECTOR:
				case DVS132S_CONFIG_EXTINPUT_DETECT_RISING_EDGES:
				case DVS132S_CONFIG_EXTINPUT_DETECT_FALLING_EDGES:
				case DVS132S_CONFIG_EXTINPUT_DETECT_PULSES:
				case DVS132S_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY:
					return (spiConfigSend(&state->usbState, DVS132S_CONFIG_EXTINPUT, paramAddr, param));
					break;

				case DVS132S_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH: {
					// Times are in µs on host, but in cycles @ LOGIC_CLOCK_FREQ
					// on FPGA, so we must multiply here.
					float timeCC = roundf((float) param * state->deviceClocks.logicClockActual);
					return (spiConfigSend(&state->usbState, DVS132S_CONFIG_EXTINPUT, paramAddr, U32T(timeCC)));
					break;
				}

				case DVS132S_CONFIG_EXTINPUT_RUN_GENERATOR:
				case DVS132S_CONFIG_EXTINPUT_GENERATE_PULSE_POLARITY:
				case DVS132S_CONFIG_EXTINPUT_GENERATE_INJECT_ON_RISING_EDGE:
				case DVS132S_CONFIG_EXTINPUT_GENERATE_INJECT_ON_FALLING_EDGE:
					if (handle->info.extInputHasGenerator) {
						return (spiConfigSend(&state->usbState, DVS132S_CONFIG_EXTINPUT, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DVS132S_CONFIG_EXTINPUT_GENERATE_PULSE_INTERVAL:
				case DVS132S_CONFIG_EXTINPUT_GENERATE_PULSE_LENGTH: {
					if (handle->info.extInputHasGenerator) {
						// Times are in µs on host, but in cycles @ LOGIC_CLOCK_FREQ
						// on FPGA, so we must multiply here.
						float timeCC = roundf((float) param * state->deviceClocks.logicClockActual);
						return (spiConfigSend(&state->usbState, DVS132S_CONFIG_EXTINPUT, paramAddr, U32T(timeCC)));
					}
					else {
						return (false);
					}
					break;
				}

				default:
					return (false);
					break;
			}
			break;

		case DVS132S_CONFIG_BIAS:
			switch (paramAddr) {
				case DVS132S_CONFIG_BIAS_PRBP:
				case DVS132S_CONFIG_BIAS_PRSFBP:
				case DVS132S_CONFIG_BIAS_BLPUBP:
				case DVS132S_CONFIG_BIAS_BIASBUFBP:
				case DVS132S_CONFIG_BIAS_OFFBN:
				case DVS132S_CONFIG_BIAS_DIFFBN:
				case DVS132S_CONFIG_BIAS_ONBN:
				case DVS132S_CONFIG_BIAS_CASBN:
				case DVS132S_CONFIG_BIAS_DPBN:
				case DVS132S_CONFIG_BIAS_BIASBUFBN:
				case DVS132S_CONFIG_BIAS_ABUFBN:
					return (spiConfigSend(&state->usbState, DVS132S_CONFIG_BIAS, paramAddr, param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case DVS132S_CONFIG_SYSINFO:
			// No SystemInfo parameters can ever be set!
			return (false);
			break;

		case DVS132S_CONFIG_USB:
			switch (paramAddr) {
				case DVS132S_CONFIG_USB_RUN:
					return (spiConfigSend(&state->usbState, DVS132S_CONFIG_USB, paramAddr, param));
					break;

				case DVS132S_CONFIG_USB_EARLY_PACKET_DELAY: {
					// Early packet delay is 125µs slices on host, but in cycles
					// @ USB_CLOCK_FREQ on FPGA, so we must multiply here.
					float delayCC = roundf((float) param * 125.0F * state->deviceClocks.usbClockActual);
					return (spiConfigSend(&state->usbState, DVS132S_CONFIG_USB, paramAddr, U32T(delayCC)));
					break;
				}

				default:
					return (false);
					break;
			}
			break;

		default:
			return (false);
			break;
	}

	return (true);
}

bool dvs132sConfigGet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t *param) {
	dvs132sHandle handle = (dvs132sHandle) cdh;
	dvs132sState state   = &handle->state;

	switch (modAddr) {
		case CAER_HOST_CONFIG_USB:
			return (usbConfigGet(&state->usbState, paramAddr, param));
			break;

		case CAER_HOST_CONFIG_DATAEXCHANGE:
			return (dataExchangeConfigGet(&state->dataExchange, paramAddr, param));
			break;

		case CAER_HOST_CONFIG_PACKETS:
			return (containerGenerationConfigGet(&state->container, paramAddr, param));
			break;

		case CAER_HOST_CONFIG_LOG:
			switch (paramAddr) {
				case CAER_HOST_CONFIG_LOG_LEVEL:
					*param = atomic_load(&state->deviceLogLevel);
					break;

				default:
					return (false);
					break;
			}
			break;

		case DVS132S_CONFIG_MUX:
			switch (paramAddr) {
				case DVS132S_CONFIG_MUX_RUN:
				case DVS132S_CONFIG_MUX_TIMESTAMP_RUN:
				case DVS132S_CONFIG_MUX_RUN_CHIP:
				case DVS132S_CONFIG_MUX_DROP_EXTINPUT_ON_TRANSFER_STALL:
				case DVS132S_CONFIG_MUX_DROP_DVS_ON_TRANSFER_STALL:
					return (spiConfigReceive(&state->usbState, DVS132S_CONFIG_MUX, paramAddr, param));
					break;

				case DVS132S_CONFIG_MUX_TIMESTAMP_RESET:
					// Always false because it's an impulse, it resets itself automatically.
					*param = false;
					break;

				case DVS132S_CONFIG_MUX_STATISTICS_EXTINPUT_DROPPED:
				case DVS132S_CONFIG_MUX_STATISTICS_EXTINPUT_DROPPED + 1:
				case DVS132S_CONFIG_MUX_STATISTICS_DVS_DROPPED:
				case DVS132S_CONFIG_MUX_STATISTICS_DVS_DROPPED + 1:
					if (handle->info.muxHasStatistics) {
						return (spiConfigReceive(&state->usbState, DVS132S_CONFIG_MUX, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				default:
					return (false);
					break;
			}
			break;

		case DVS132S_CONFIG_DVS:
			switch (paramAddr) {
				case DVS132S_CONFIG_DVS_RUN:
				case DVS132S_CONFIG_DVS_WAIT_ON_TRANSFER_STALL:
				case DVS132S_CONFIG_DVS_FILTER_AT_LEAST_2_UNSIGNED:
				case DVS132S_CONFIG_DVS_FILTER_NOT_ALL_4_UNSIGNED:
				case DVS132S_CONFIG_DVS_FILTER_AT_LEAST_2_SIGNED:
				case DVS132S_CONFIG_DVS_FILTER_NOT_ALL_4_SIGNED:
				case DVS132S_CONFIG_DVS_ROW_ENABLE_31_TO_0:
				case DVS132S_CONFIG_DVS_ROW_ENABLE_63_TO_32:
				case DVS132S_CONFIG_DVS_ROW_ENABLE_65_TO_64:
				case DVS132S_CONFIG_DVS_COLUMN_ENABLE_31_TO_0:
				case DVS132S_CONFIG_DVS_COLUMN_ENABLE_51_TO_32:
					return (spiConfigReceive(&state->usbState, DVS132S_CONFIG_DVS, paramAddr, param));
					break;

				case DVS132S_CONFIG_DVS_RESTART_TIME:
				case DVS132S_CONFIG_DVS_CAPTURE_INTERVAL: {
					// DVS long times are in µs on host, but in cycles @ LOGIC_CLOCK_FREQ
					// on FPGA, so we must divide here.
					uint32_t cyclesValue = 0;
					if (!spiConfigReceive(&state->usbState, DVS132S_CONFIG_DVS, paramAddr, &cyclesValue)) {
						return (false);
					}

					float delayCC = roundf((float) cyclesValue / state->deviceClocks.logicClockActual);
					*param        = U32T(delayCC);

					return (true);
					break;
				}

				case DVS132S_CONFIG_DVS_STATISTICS_TRANSACTIONS_SUCCESS:
				case DVS132S_CONFIG_DVS_STATISTICS_TRANSACTIONS_SUCCESS + 1:
				case DVS132S_CONFIG_DVS_STATISTICS_TRANSACTIONS_SKIPPED:
				case DVS132S_CONFIG_DVS_STATISTICS_TRANSACTIONS_SKIPPED + 1:
				case DVS132S_CONFIG_DVS_STATISTICS_TRANSACTIONS_ERRORED:
					if (handle->info.dvsHasStatistics) {
						return (spiConfigReceive(&state->usbState, DVS132S_CONFIG_DVS, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				default:
					return (false);
					break;
			}
			break;

		case DVS132S_CONFIG_IMU:
			switch (paramAddr) {
				case DVS132S_CONFIG_IMU_RUN_ACCELEROMETER:
				case DVS132S_CONFIG_IMU_RUN_GYROSCOPE:
				case DVS132S_CONFIG_IMU_RUN_TEMPERATURE:
				case DVS132S_CONFIG_IMU_ACCEL_DATA_RATE:
				case DVS132S_CONFIG_IMU_ACCEL_FILTER:
				case DVS132S_CONFIG_IMU_ACCEL_RANGE:
				case DVS132S_CONFIG_IMU_GYRO_DATA_RATE:
				case DVS132S_CONFIG_IMU_GYRO_FILTER:
				case DVS132S_CONFIG_IMU_GYRO_RANGE:
					return (spiConfigReceive(&state->usbState, DVS132S_CONFIG_IMU, paramAddr, param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case DVS132S_CONFIG_EXTINPUT:
			switch (paramAddr) {
				case DVS132S_CONFIG_EXTINPUT_RUN_DETECTOR:
				case DVS132S_CONFIG_EXTINPUT_DETECT_RISING_EDGES:
				case DVS132S_CONFIG_EXTINPUT_DETECT_FALLING_EDGES:
				case DVS132S_CONFIG_EXTINPUT_DETECT_PULSES:
				case DVS132S_CONFIG_EXTINPUT_DETECT_PULSE_POLARITY:
					return (spiConfigReceive(&state->usbState, DVS132S_CONFIG_EXTINPUT, paramAddr, param));
					break;

				case DVS132S_CONFIG_EXTINPUT_DETECT_PULSE_LENGTH: {
					// Times are in µs on host, but in cycles @ LOGIC_CLOCK_FREQ
					// on FPGA, so we must divide here.
					uint32_t cyclesValue = 0;
					if (!spiConfigReceive(&state->usbState, DVS132S_CONFIG_EXTINPUT, paramAddr, &cyclesValue)) {
						return (false);
					}

					float delayCC = roundf((float) cyclesValue / state->deviceClocks.logicClockActual);
					*param        = U32T(delayCC);

					return (true);
					break;
				}

				case DVS132S_CONFIG_EXTINPUT_RUN_GENERATOR:
				case DVS132S_CONFIG_EXTINPUT_GENERATE_PULSE_POLARITY:
				case DVS132S_CONFIG_EXTINPUT_GENERATE_INJECT_ON_RISING_EDGE:
				case DVS132S_CONFIG_EXTINPUT_GENERATE_INJECT_ON_FALLING_EDGE:
					if (handle->info.extInputHasGenerator) {
						return (spiConfigReceive(&state->usbState, DVS132S_CONFIG_EXTINPUT, paramAddr, param));
					}
					else {
						return (false);
					}
					break;

				case DVS132S_CONFIG_EXTINPUT_GENERATE_PULSE_INTERVAL:
				case DVS132S_CONFIG_EXTINPUT_GENERATE_PULSE_LENGTH: {
					if (handle->info.extInputHasGenerator) {
						// Times are in µs on host, but in cycles @ LOGIC_CLOCK_FREQ
						// on FPGA, so we must divide here.
						uint32_t cyclesValue = 0;
						if (!spiConfigReceive(&state->usbState, DVS132S_CONFIG_EXTINPUT, paramAddr, &cyclesValue)) {
							return (false);
						}

						float delayCC = roundf((float) cyclesValue / state->deviceClocks.logicClockActual);
						*param        = U32T(delayCC);

						return (true);
					}
					else {
						return (false);
					}
					break;
				}

				default:
					return (false);
					break;
			}
			break;

		case DVS132S_CONFIG_BIAS:
			switch (paramAddr) {
				case DVS132S_CONFIG_BIAS_PRBP:
				case DVS132S_CONFIG_BIAS_PRSFBP:
				case DVS132S_CONFIG_BIAS_BLPUBP:
				case DVS132S_CONFIG_BIAS_BIASBUFBP:
				case DVS132S_CONFIG_BIAS_OFFBN:
				case DVS132S_CONFIG_BIAS_DIFFBN:
				case DVS132S_CONFIG_BIAS_ONBN:
				case DVS132S_CONFIG_BIAS_CASBN:
				case DVS132S_CONFIG_BIAS_DPBN:
				case DVS132S_CONFIG_BIAS_BIASBUFBN:
				case DVS132S_CONFIG_BIAS_ABUFBN:
					return (spiConfigReceive(&state->usbState, DVS132S_CONFIG_BIAS, paramAddr, param));
					break;

				default:
					return (false);
					break;
			}
			break;

		case DVS132S_CONFIG_SYSINFO:
			// No SystemInfo parameters can ever be get! Use the info struct!
			return (false);
			break;

		case DVS132S_CONFIG_USB:
			switch (paramAddr) {
				case DVS132S_CONFIG_USB_RUN:
					return (spiConfigReceive(&state->usbState, DVS132S_CONFIG_USB, paramAddr, param));
					break;

				case DVS132S_CONFIG_USB_EARLY_PACKET_DELAY: {
					// Early packet delay is 125µs slices on host, but in cycles
					// @ USB_CLOCK_FREQ on FPGA, so we must divide here.
					uint32_t cyclesValue = 0;
					if (!spiConfigReceive(&state->usbState, DVS132S_CONFIG_USB, paramAddr, &cyclesValue)) {
						return (false);
					}

					float delayCC = roundf((float) cyclesValue / (125.0F * state->deviceClocks.usbClockActual));
					*param        = U32T(delayCC);

					return (true);
					break;
				}

				default:
					return (false);
					break;
			}
			break;

		default:
			return (false);
			break;
	}

	return (true);
}

bool dvs132sDataStart(caerDeviceHandle cdh, void (*dataNotifyIncrease)(void *ptr),
	void (*dataNotifyDecrease)(void *ptr), void *dataNotifyUserPtr, void (*dataShutdownNotify)(void *ptr),
	void *dataShutdownUserPtr) {
	dvs132sHandle handle = (dvs132sHandle) cdh;
	dvs132sState state   = &handle->state;

	usbSetShutdownCallback(&state->usbState, dataShutdownNotify, dataShutdownUserPtr);

	// Store new data available/not available anymore call-backs.
	dataExchangeSetNotify(&state->dataExchange, dataNotifyIncrease, dataNotifyDecrease, dataNotifyUserPtr);

	containerGenerationCommitTimestampReset(&state->container);

	if (!dataExchangeBufferInit(&state->dataExchange)) {
		dvs132sLog(CAER_LOG_CRITICAL, handle, "Failed to initialize data exchange buffer.");
		return (false);
	}

	// Allocate packets.
	if (!containerGenerationAllocate(&state->container, DVS132S_EVENT_TYPES)) {
		freeAllDataMemory(state);

		dvs132sLog(CAER_LOG_CRITICAL, handle, "Failed to allocate event packet container.");
		return (false);
	}

	state->currentPackets.polarity
		= caerPolarityEventPacketAllocate(DVS132S_POLARITY_DEFAULT_SIZE, I16T(handle->info.deviceID), 0);
	if (state->currentPackets.polarity == NULL) {
		freeAllDataMemory(state);

		dvs132sLog(CAER_LOG_CRITICAL, handle, "Failed to allocate polarity event packet.");
		return (false);
	}

	state->currentPackets.special
		= caerSpecialEventPacketAllocate(DVS132S_SPECIAL_DEFAULT_SIZE, I16T(handle->info.deviceID), 0);
	if (state->currentPackets.special == NULL) {
		freeAllDataMemory(state);

		dvs132sLog(CAER_LOG_CRITICAL, handle, "Failed to allocate special event packet.");
		return (false);
	}

	state->currentPackets.imu6 = caerIMU6EventPacketAllocate(DVS132S_IMU_DEFAULT_SIZE, I16T(handle->info.deviceID), 0);
	if (state->currentPackets.imu6 == NULL) {
		freeAllDataMemory(state);

		dvs132sLog(CAER_LOG_CRITICAL, handle, "Failed to allocate IMU6 event packet.");
		return (false);
	}

	// Ignore multi-part events (IMU) at startup, so that any initial
	// incomplete event is ignored. The START events reset this as soon as
	// the first one is observed.
	state->imu.ignoreEvents = true;

	// Ensure no data is left over from previous runs, if the camera
	// wasn't shut-down properly. First ensure it is shut down completely.
	dvs132sConfigSet(cdh, DVS132S_CONFIG_DVS, DVS132S_CONFIG_DVS_RUN, false);
	dvs132sConfigSet(cdh, DVS132S_CONFIG_IMU, DVS132S_CONFIG_IMU_RUN_ACCELEROMETER, false);
	dvs132sConfigSet(cdh, DVS132S_CONFIG_IMU, DVS132S_CONFIG_IMU_RUN_GYROSCOPE, false);
	dvs132sConfigSet(cdh, DVS132S_CONFIG_IMU, DVS132S_CONFIG_IMU_RUN_TEMPERATURE, false);
	dvs132sConfigSet(cdh, DVS132S_CONFIG_EXTINPUT, DVS132S_CONFIG_EXTINPUT_RUN_DETECTOR, false);

	dvs132sConfigSet(cdh, DVS132S_CONFIG_MUX, DVS132S_CONFIG_MUX_RUN, false);
	dvs132sConfigSet(cdh, DVS132S_CONFIG_MUX, DVS132S_CONFIG_MUX_TIMESTAMP_RUN, false);
	dvs132sConfigSet(cdh, DVS132S_CONFIG_USB, DVS132S_CONFIG_USB_RUN, false);

	dvs132sConfigSet(cdh, DVS132S_CONFIG_MUX, DVS132S_CONFIG_MUX_RUN_CHIP, false);

	// Then wait 10ms for FPGA device side buffers to clear.
	struct timespec clearSleep = {.tv_sec = 0, .tv_nsec = 10000000};
	thrd_sleep(&clearSleep, NULL);

	// And reset the USB side of things.
	usbControlResetDataEndpoint(&state->usbState, USB_DEFAULT_DATA_ENDPOINT);

	if (!usbDataTransfersStart(&state->usbState)) {
		freeAllDataMemory(state);

		dvs132sLog(CAER_LOG_CRITICAL, handle, "Failed to start data transfers.");
		return (false);
	}

	if (dataExchangeStartProducers(&state->dataExchange)) {
		// Enable data transfer on USB end-point 2.
		dvs132sConfigSet(cdh, DVS132S_CONFIG_MUX, DVS132S_CONFIG_MUX_RUN_CHIP, true);

		// Wait 200 ms for biases to stabilize.
		struct timespec biasEnSleep = {.tv_sec = 0, .tv_nsec = 200000000};
		thrd_sleep(&biasEnSleep, NULL);

		dvs132sConfigSet(cdh, DVS132S_CONFIG_USB, DVS132S_CONFIG_USB_RUN, true);
		dvs132sConfigSet(cdh, DVS132S_CONFIG_MUX, DVS132S_CONFIG_MUX_TIMESTAMP_RUN, true);
		dvs132sConfigSet(cdh, DVS132S_CONFIG_MUX, DVS132S_CONFIG_MUX_RUN, true);

		// Wait 50 ms for data transfer to be ready.
		struct timespec noDataSleep = {.tv_sec = 0, .tv_nsec = 50000000};
		thrd_sleep(&noDataSleep, NULL);

		dvs132sConfigSet(cdh, DVS132S_CONFIG_DVS, DVS132S_CONFIG_DVS_RUN, true);
		dvs132sConfigSet(cdh, DVS132S_CONFIG_IMU, DVS132S_CONFIG_IMU_RUN_ACCELEROMETER, true);
		dvs132sConfigSet(cdh, DVS132S_CONFIG_IMU, DVS132S_CONFIG_IMU_RUN_GYROSCOPE, true);
		dvs132sConfigSet(cdh, DVS132S_CONFIG_IMU, DVS132S_CONFIG_IMU_RUN_TEMPERATURE, true);
		dvs132sConfigSet(cdh, DVS132S_CONFIG_EXTINPUT, DVS132S_CONFIG_EXTINPUT_RUN_DETECTOR, true);
	}

	return (true);
}

bool dvs132sDataStop(caerDeviceHandle cdh) {
	dvs132sHandle handle = (dvs132sHandle) cdh;
	dvs132sState state   = &handle->state;

	if (dataExchangeStopProducers(&state->dataExchange)) {
		// Disable data transfer on USB end-point 2. Reverse order of enabling.
		dvs132sConfigSet(cdh, DVS132S_CONFIG_DVS, DVS132S_CONFIG_DVS_RUN, false);
		dvs132sConfigSet(cdh, DVS132S_CONFIG_IMU, DVS132S_CONFIG_IMU_RUN_ACCELEROMETER, false);
		dvs132sConfigSet(cdh, DVS132S_CONFIG_IMU, DVS132S_CONFIG_IMU_RUN_GYROSCOPE, false);
		dvs132sConfigSet(cdh, DVS132S_CONFIG_IMU, DVS132S_CONFIG_IMU_RUN_TEMPERATURE, false);
		dvs132sConfigSet(cdh, DVS132S_CONFIG_EXTINPUT, DVS132S_CONFIG_EXTINPUT_RUN_DETECTOR, false);

		dvs132sConfigSet(cdh, DVS132S_CONFIG_MUX, DVS132S_CONFIG_MUX_RUN, false);
		dvs132sConfigSet(cdh, DVS132S_CONFIG_MUX, DVS132S_CONFIG_MUX_TIMESTAMP_RUN, false);
		dvs132sConfigSet(cdh, DVS132S_CONFIG_USB, DVS132S_CONFIG_USB_RUN, false);

		dvs132sConfigSet(cdh, DVS132S_CONFIG_MUX, DVS132S_CONFIG_MUX_RUN_CHIP, false);
	}

	usbDataTransfersStop(&state->usbState);

	dataExchangeBufferEmpty(&state->dataExchange);

	// Free current, uncommitted packets and ringbuffer.
	freeAllDataMemory(state);

	// Reset packet positions.
	state->currentPackets.polarityPosition = 0;
	state->currentPackets.specialPosition  = 0;
	state->currentPackets.imu6Position     = 0;

	// Reset private composite events.
	memset(&state->imu.currentEvent, 0, sizeof(struct caer_imu6_event));

	return (true);
}

caerEventPacketContainer dvs132sDataGet(caerDeviceHandle cdh) {
	dvs132sHandle handle = (dvs132sHandle) cdh;
	dvs132sState state   = &handle->state;

	return (dataExchangeGet(&state->dataExchange, &state->usbState.dataTransfersRun));
}

#define TS_WRAP_ADD 0x8000

static inline bool ensureSpaceForEvents(
	caerEventPacketHeader *packet, size_t position, size_t numEvents, dvs132sHandle handle) {
	if ((position + numEvents) <= (size_t) caerEventPacketHeaderGetEventCapacity(*packet)) {
		return (true);
	}

	caerEventPacketHeader grownPacket
		= caerEventPacketGrow(*packet, caerEventPacketHeaderGetEventCapacity(*packet) * 2);
	if (grownPacket == NULL) {
		dvs132sLog(CAER_LOG_CRITICAL, handle, "Failed to grow event packet of type %d.",
			caerEventPacketHeaderGetEventType(*packet));
		return (false);
	}

	*packet = grownPacket;
	return (true);
}

static void dvs132sEventTranslator(void *vhd, const uint8_t *buffer, size_t bufferSize) {
	dvs132sHandle handle = vhd;
	dvs132sState state   = &handle->state;

	// Return right away if not running anymore. This prevents useless work if many
	// buffers are still waiting when shut down, as well as incorrect event sequences
	// if a TS_RESET is stuck on ring-buffer commit further down, and detects shut-down;
	// then any subsequent buffers should also detect shut-down and not be handled.
	if (!usbDataTransfersAreRunning(&state->usbState)) {
		return;
	}

	// Truncate off any extra partial event.
	if ((bufferSize & 0x01) != 0) {
		dvs132sLog(CAER_LOG_ALERT, handle, "%zu bytes received via USB, which is not a multiple of two.", bufferSize);
		bufferSize &= ~((size_t) 0x01);
	}

	for (size_t bufferPos = 0; bufferPos < bufferSize; bufferPos += 2) {
		// Allocate new packets for next iteration as needed.
		if (!containerGenerationAllocate(&state->container, DVS132S_EVENT_TYPES)) {
			dvs132sLog(CAER_LOG_CRITICAL, handle, "Failed to allocate event packet container.");
			return;
		}

		if (state->currentPackets.special == NULL) {
			state->currentPackets.special = caerSpecialEventPacketAllocate(
				DVS132S_SPECIAL_DEFAULT_SIZE, I16T(handle->info.deviceID), state->timestamps.wrapOverflow);
			if (state->currentPackets.special == NULL) {
				dvs132sLog(CAER_LOG_CRITICAL, handle, "Failed to allocate special event packet.");
				return;
			}
		}

		if (state->currentPackets.polarity == NULL) {
			state->currentPackets.polarity = caerPolarityEventPacketAllocate(
				DVS132S_POLARITY_DEFAULT_SIZE, I16T(handle->info.deviceID), state->timestamps.wrapOverflow);
			if (state->currentPackets.polarity == NULL) {
				dvs132sLog(CAER_LOG_CRITICAL, handle, "Failed to allocate polarity event packet.");
				return;
			}
		}

		if (state->currentPackets.imu6 == NULL) {
			state->currentPackets.imu6 = caerIMU6EventPacketAllocate(
				DVS132S_IMU_DEFAULT_SIZE, I16T(handle->info.deviceID), state->timestamps.wrapOverflow);
			if (state->currentPackets.imu6 == NULL) {
				dvs132sLog(CAER_LOG_CRITICAL, handle, "Failed to allocate IMU6 event packet.");
				return;
			}
		}

		bool tsReset   = false;
		bool tsBigWrap = false;

		uint16_t event = le16toh(*((const uint16_t *) (&buffer[bufferPos])));

		// Check if timestamp.
		if ((event & 0x8000) != 0) {
			handleTimestampUpdateNewLogic(&state->timestamps, event, handle->info.deviceString, &state->deviceLogLevel);

			containerGenerationCommitTimestampInit(&state->container, state->timestamps.current);
		}
		else {
			// Look at the code, to determine event and data type.
			uint8_t code  = U8T((event & 0x7000) >> 12);
			uint16_t data = (event & 0x0FFF);

			switch (code) {
				case 0: // Special event
					switch (data) {
						case 0: // Ignore this, but log it.
							dvs132sLog(CAER_LOG_ERROR, handle, "Caught special reserved event!");
							break;

						case 1: { // Timetamp reset
							handleTimestampResetNewLogic(
								&state->timestamps, handle->info.deviceString, &state->deviceLogLevel);

							containerGenerationCommitTimestampReset(&state->container);
							containerGenerationCommitTimestampInit(&state->container, state->timestamps.current);

							// Defer timestamp reset event to later, so we commit it
							// alone, in its own packet.
							// Commit packets when doing a reset to clearly separate them.
							tsReset = true;

							// Update Master/Slave status on incoming TS resets.
							// Async call to not deadlock here.
							spiConfigReceiveAsync(&state->usbState, DVS132S_CONFIG_SYSINFO,
								DVS132S_CONFIG_SYSINFO_DEVICE_IS_MASTER, &dvs132sTSMasterStatusUpdater, &handle->info);

							break;
						}

						case 2: { // External input (falling edge)
							dvs132sLog(CAER_LOG_DEBUG, handle, "External input (falling edge) event received.");

							if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.special,
									(size_t) state->currentPackets.specialPosition, 1, handle)) {
								caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
									state->currentPackets.special, state->currentPackets.specialPosition);
								caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
								caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT_FALLING_EDGE);
								caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
								state->currentPackets.specialPosition++;
							}

							break;
						}

						case 3: { // External input (rising edge)
							dvs132sLog(CAER_LOG_DEBUG, handle, "External input (rising edge) event received.");

							if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.special,
									(size_t) state->currentPackets.specialPosition, 1, handle)) {
								caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
									state->currentPackets.special, state->currentPackets.specialPosition);
								caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
								caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT_RISING_EDGE);
								caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
								state->currentPackets.specialPosition++;
							}

							break;
						}

						case 4: { // External input (pulse)
							dvs132sLog(CAER_LOG_DEBUG, handle, "External input (pulse) event received.");

							if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.special,
									(size_t) state->currentPackets.specialPosition, 1, handle)) {
								caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
									state->currentPackets.special, state->currentPackets.specialPosition);
								caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
								caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_INPUT_PULSE);
								caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
								state->currentPackets.specialPosition++;
							}

							break;
						}

						case 5: { // IMU Start (6 axes)
							dvs132sLog(CAER_LOG_DEBUG, handle, "IMU6 Start event received.");

							state->imu.ignoreEvents = false;
							state->imu.count        = 0;
							state->imu.type         = 0;

							memset(&state->imu.currentEvent, 0, sizeof(struct caer_imu6_event));

							break;
						}

						case 7: { // IMU End
							if (state->imu.ignoreEvents) {
								break;
							}
							dvs132sLog(CAER_LOG_DEBUG, handle, "IMU End event received.");

							if (state->imu.count == IMU_TOTAL_COUNT) {
								// Timestamp at event-stream insertion point.
								caerIMU6EventSetTimestamp(&state->imu.currentEvent, state->timestamps.current);

								caerIMU6EventValidate(&state->imu.currentEvent, state->currentPackets.imu6);

								// IMU6 and APS operate on an internal event and copy that to the actual output
								// packet here, in the END state, for a reason: if a packetContainer, with all its
								// packets, is committed due to hitting any of the triggers that are not TS reset
								// or TS wrap-around related, like number of polarity events, the event in the packet
								// would be left incomplete, and the event in the new packet would be corrupted.
								// We could avoid this like for the TS reset/TS wrap-around case (see forceCommit) by
								// just deleting that event, but these kinds of commits happen much more often and the
								// possible data loss would be too significant. So instead we keep a private event,
								// fill it, and then only copy it into the packet here in the END state, at which point
								// the whole event is ready and cannot be broken/corrupted in any way anymore.
								if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.imu6,
										(size_t) state->currentPackets.imu6Position, 1, handle)) {
									caerIMU6Event imuCurrentEvent = caerIMU6EventPacketGetEvent(
										state->currentPackets.imu6, state->currentPackets.imu6Position);
									memcpy(imuCurrentEvent, &state->imu.currentEvent, sizeof(struct caer_imu6_event));
									state->currentPackets.imu6Position++;
								}
							}
							else {
								dvs132sLog(CAER_LOG_INFO, handle,
									"IMU End: failed to validate IMU sample count (%" PRIu8 "), discarding samples.",
									state->imu.count);
							}

							break;
						}

						case 16: { // External generator (falling edge)
							dvs132sLog(CAER_LOG_DEBUG, handle, "External generator (falling edge) event received.");

							if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.special,
									(size_t) state->currentPackets.specialPosition, 1, handle)) {
								caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
									state->currentPackets.special, state->currentPackets.specialPosition);
								caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
								caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_GENERATOR_FALLING_EDGE);
								caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
								state->currentPackets.specialPosition++;
							}

							break;
						}

						case 17: { // External generator (rising edge)
							dvs132sLog(CAER_LOG_DEBUG, handle, "External generator (rising edge) event received.");

							if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.special,
									(size_t) state->currentPackets.specialPosition, 1, handle)) {
								caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
									state->currentPackets.special, state->currentPackets.specialPosition);
								caerSpecialEventSetTimestamp(currentSpecialEvent, state->timestamps.current);
								caerSpecialEventSetType(currentSpecialEvent, EXTERNAL_GENERATOR_RISING_EDGE);
								caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
								state->currentPackets.specialPosition++;
							}

							break;
						}

						default:
							dvs132sLog(CAER_LOG_ERROR, handle, "Caught special event that can't be handled: %d.", data);
							break;
					}
					break;

				case 1: // Y group address
					// Check range conformity.
					if (data >= state->dvs.sizeY) {
						dvs132sLog(CAER_LOG_ALERT, handle,
							"DVS: Y address out of range (0-%d): %" PRIu16 ", due to USB communication issue.",
							state->dvs.sizeY - 1, data);
						break; // Skip invalid Y address (don't update lastY).
					}

					state->dvs.lastY = data;
					break;

				case 2: // X group address
					// Check range conformity.
					if (data >= state->dvs.sizeX) {
						dvs132sLog(CAER_LOG_ALERT, handle,
							"DVS: X address out of range (0-%d): %" PRIu16 ", due to USB communication issue.",
							state->dvs.sizeX - 1, data);
						break; // Skip invalid X address (don't update lastX).
					}

					state->dvs.lastX = data;
					break;

				case 3: { // 4-pixel group event presence and polarity.
					if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.polarity,
							(size_t) state->currentPackets.polarityPosition, 4, handle)) {
						bool pres0 = data & 0x0010;
						bool pres1 = data & 0x0020;
						bool pres2 = data & 0x0040;
						bool pres3 = data & 0x0080;
						bool pol0  = data & 0x0001;
						bool pol1  = data & 0x0002;
						bool pol2  = data & 0x0004;
						bool pol3  = data & 0x0008;

						// Pixel 0: Top Left for host-packet-order.
						if (pres0) {
							// Received event!
							caerPolarityEvent currentPolarityEvent = caerPolarityEventPacketGetEvent(
								state->currentPackets.polarity, state->currentPackets.polarityPosition);

							// Timestamp at event-stream insertion point.
							caerPolarityEventSetTimestamp(currentPolarityEvent, state->timestamps.current);
							caerPolarityEventSetPolarity(currentPolarityEvent, pol0);
							if (state->dvs.invertXY) {
								caerPolarityEventSetY(currentPolarityEvent, state->dvs.lastX);
								caerPolarityEventSetX(currentPolarityEvent, state->dvs.lastY);
							}
							else {
								caerPolarityEventSetY(currentPolarityEvent, state->dvs.lastY);
								caerPolarityEventSetX(currentPolarityEvent, state->dvs.lastX);
							}
							caerPolarityEventValidate(currentPolarityEvent, state->currentPackets.polarity);
							state->currentPackets.polarityPosition++;
						}

						// Pixel 1: Top Right for host-packet-order.
						if (pres1) {
							// Received event!
							caerPolarityEvent currentPolarityEvent = caerPolarityEventPacketGetEvent(
								state->currentPackets.polarity, state->currentPackets.polarityPosition);

							// Timestamp at event-stream insertion point.
							caerPolarityEventSetTimestamp(currentPolarityEvent, state->timestamps.current);
							caerPolarityEventSetPolarity(currentPolarityEvent, pol1);
							if (state->dvs.invertXY) {
								caerPolarityEventSetY(currentPolarityEvent, U16T(state->dvs.lastX + 1));
								caerPolarityEventSetX(currentPolarityEvent, state->dvs.lastY);
							}
							else {
								caerPolarityEventSetY(currentPolarityEvent, state->dvs.lastY);
								caerPolarityEventSetX(currentPolarityEvent, U16T(state->dvs.lastX + 1));
							}
							caerPolarityEventValidate(currentPolarityEvent, state->currentPackets.polarity);
							state->currentPackets.polarityPosition++;
						}

						if (pres2) {
							// Received event!
							caerPolarityEvent currentPolarityEvent = caerPolarityEventPacketGetEvent(
								state->currentPackets.polarity, state->currentPackets.polarityPosition);

							// Timestamp at event-stream insertion point.
							caerPolarityEventSetTimestamp(currentPolarityEvent, state->timestamps.current);
							caerPolarityEventSetPolarity(currentPolarityEvent, pol2);
							if (state->dvs.invertXY) {
								caerPolarityEventSetY(currentPolarityEvent, state->dvs.lastX);
								caerPolarityEventSetX(currentPolarityEvent, U16T(state->dvs.lastY + 1));
							}
							else {
								caerPolarityEventSetY(currentPolarityEvent, U16T(state->dvs.lastY + 1));
								caerPolarityEventSetX(currentPolarityEvent, state->dvs.lastX);
							}
							caerPolarityEventValidate(currentPolarityEvent, state->currentPackets.polarity);
							state->currentPackets.polarityPosition++;
						}

						if (pres3) {
							// Received event!
							caerPolarityEvent currentPolarityEvent = caerPolarityEventPacketGetEvent(
								state->currentPackets.polarity, state->currentPackets.polarityPosition);

							// Timestamp at event-stream insertion point.
							caerPolarityEventSetTimestamp(currentPolarityEvent, state->timestamps.current);
							caerPolarityEventSetPolarity(currentPolarityEvent, pol3);
							if (state->dvs.invertXY) {
								caerPolarityEventSetY(currentPolarityEvent, U16T(state->dvs.lastX + 1));
								caerPolarityEventSetX(currentPolarityEvent, U16T(state->dvs.lastY + 1));
							}
							else {
								caerPolarityEventSetY(currentPolarityEvent, U16T(state->dvs.lastY + 1));
								caerPolarityEventSetX(currentPolarityEvent, U16T(state->dvs.lastX + 1));
							}
							caerPolarityEventValidate(currentPolarityEvent, state->currentPackets.polarity);
							state->currentPackets.polarityPosition++;
						}
					}

					break;
				}

				case 5: {
					// Misc 8bit data.
					uint8_t misc8Code = U8T((data & 0x0F00) >> 8);
					uint8_t misc8Data = U8T(data & 0x00FF);

					switch (misc8Code) {
						case 0:
							if (state->imu.ignoreEvents) {
								break;
							}
							dvs132sLog(CAER_LOG_DEBUG, handle, "IMU Data event (%" PRIu8 ") received.", misc8Data);

							// IMU data event.
							switch (state->imu.count) {
								case 0:
								case 2:
								case 4:
								case 6:
								case 8:
								case 10:
								case 12:
									state->imu.tmpData = misc8Data;
									break;

								case 1: {
									int16_t accelX = I16T((state->imu.tmpData << 8) | misc8Data);
									if (state->imu.flipX) {
										accelX = I16T(-accelX);
									}
									caerIMU6EventSetAccelX(&state->imu.currentEvent, accelX / state->imu.accelScale);
									break;
								}

								case 3: {
									int16_t accelY = I16T((state->imu.tmpData << 8) | misc8Data);
									if (state->imu.flipY) {
										accelY = I16T(-accelY);
									}
									caerIMU6EventSetAccelY(&state->imu.currentEvent, accelY / state->imu.accelScale);
									break;
								}

								case 5: {
									int16_t accelZ = I16T((state->imu.tmpData << 8) | misc8Data);
									if (state->imu.flipZ) {
										accelZ = I16T(-accelZ);
									}
									caerIMU6EventSetAccelZ(&state->imu.currentEvent, accelZ / state->imu.accelScale);

									// IMU parser count depends on which data is present.
									if (!(state->imu.type & IMU_TYPE_TEMP)) {
										if (state->imu.type & IMU_TYPE_GYRO) {
											// No temperature, but gyro.
											state->imu.count = U8T(state->imu.count + 2);
										}
										else {
											// No others enabled.
											state->imu.count = U8T(state->imu.count + 8);
										}
									}
									break;
								}

								case 7: {
									// Temperature is signed. Formula for converting to °C:
									// (SIGNED_VAL / 512) + 23
									int16_t temp = I16T((state->imu.tmpData << 8) | misc8Data);
									caerIMU6EventSetTemp(&state->imu.currentEvent, (temp / 512.0F) + 23.0F);

									// IMU parser count depends on which data is present.
									if (!(state->imu.type & IMU_TYPE_GYRO)) {
										// No others enabled.
										state->imu.count = U8T(state->imu.count + 6);
									}
									break;
								}

								case 9: {
									int16_t gyroX = I16T((state->imu.tmpData << 8) | misc8Data);
									if (state->imu.flipX) {
										gyroX = I16T(-gyroX);
									}
									caerIMU6EventSetGyroX(&state->imu.currentEvent, gyroX / state->imu.gyroScale);
									break;
								}

								case 11: {
									int16_t gyroY = I16T((state->imu.tmpData << 8) | misc8Data);
									if (state->imu.flipY) {
										gyroY = I16T(-gyroY);
									}
									caerIMU6EventSetGyroY(&state->imu.currentEvent, gyroY / state->imu.gyroScale);
									break;
								}

								case 13: {
									int16_t gyroZ = I16T((state->imu.tmpData << 8) | misc8Data);
									if (state->imu.flipZ) {
										gyroZ = I16T(-gyroZ);
									}
									caerIMU6EventSetGyroZ(&state->imu.currentEvent, gyroZ / state->imu.gyroScale);
									break;
								}

								default:
									dvs132sLog(CAER_LOG_ERROR, handle, "Got invalid IMU update sequence.");
									break;
							}

							state->imu.count++;

							break;

						case 3: {
							if (state->imu.ignoreEvents) {
								break;
							}
							dvs132sLog(CAER_LOG_DEBUG, handle, "IMU Scale Config event (%" PRIu16 ") received.", data);

							// Set correct IMU accel and gyro scales, used to interpret subsequent
							// IMU samples from the device.
							state->imu.accelScale = calculateIMUAccelScale(U16T(data >> 3) & 0x03);
							state->imu.gyroScale  = calculateIMUGyroScale(data & 0x07);

							// Set expected type of data to come from IMU (accel, gyro, temp).
							state->imu.type = (data >> 5) & 0x07;

							// IMU parser start count depends on which data is present.
							if (state->imu.type & IMU_TYPE_ACCEL) {
								// Accelerometer.
								state->imu.count = 0;
							}
							else if (state->imu.type & IMU_TYPE_TEMP) {
								// Temperature
								state->imu.count = 6;
							}
							else if (state->imu.type & IMU_TYPE_GYRO) {
								// Gyroscope.
								state->imu.count = 8;
							}
							else {
								// Nothing, should never happen.
								state->imu.count = 14;

								dvs132sLog(CAER_LOG_ERROR, handle, "IMU Scale Config: no IMU sensors enabled.");
							}

							break;
						}

						default:
							dvs132sLog(CAER_LOG_ERROR, handle, "Caught Misc8 event that can't be handled.");
							break;
					}

					break;
				}

				case 7: { // Timestamp wrap
					tsBigWrap = handleTimestampWrapNewLogic(
						&state->timestamps, data, TS_WRAP_ADD, handle->info.deviceString, &state->deviceLogLevel);

					if (tsBigWrap) {
						if (ensureSpaceForEvents((caerEventPacketHeader *) &state->currentPackets.special,
								(size_t) state->currentPackets.specialPosition, 1, handle)) {
							caerSpecialEvent currentSpecialEvent = caerSpecialEventPacketGetEvent(
								state->currentPackets.special, state->currentPackets.specialPosition);
							caerSpecialEventSetTimestamp(currentSpecialEvent, INT32_MAX);
							caerSpecialEventSetType(currentSpecialEvent, TIMESTAMP_WRAP);
							caerSpecialEventValidate(currentSpecialEvent, state->currentPackets.special);
							state->currentPackets.specialPosition++;
						}
					}
					else {
						containerGenerationCommitTimestampInit(&state->container, state->timestamps.current);
					}

					break;
				}

				default:
					dvs132sLog(CAER_LOG_ERROR, handle, "Caught event that can't be handled.");
					break;
			}
		}

		// Thresholds on which to trigger packet container commit.
		// tsReset and tsBigWrap are already defined above.
		// Trigger if any of the global container-wide thresholds are met.
		int32_t currentPacketContainerCommitSize = containerGenerationGetMaxPacketSize(&state->container);
		bool containerSizeCommit                 = (currentPacketContainerCommitSize > 0)
								   && ((state->currentPackets.polarityPosition >= currentPacketContainerCommitSize)
									   || (state->currentPackets.specialPosition >= currentPacketContainerCommitSize)
									   || (state->currentPackets.imu6Position >= currentPacketContainerCommitSize));

		bool containerTimeCommit = containerGenerationIsCommitTimestampElapsed(
			&state->container, state->timestamps.wrapOverflow, state->timestamps.current);

		// Commit packet containers to the ring-buffer, so they can be processed by the
		// main-loop, when any of the required conditions are met.
		if (tsReset || tsBigWrap || containerSizeCommit || containerTimeCommit) {
			// One or more of the commit triggers are hit. Set the packet container up to contain
			// any non-empty packets. Empty packets are not forwarded to save memory.
			bool emptyContainerCommit = true;

			if (state->currentPackets.polarityPosition > 0) {
				containerGenerationSetPacket(
					&state->container, POLARITY_EVENT, (caerEventPacketHeader) state->currentPackets.polarity);

				state->currentPackets.polarity         = NULL;
				state->currentPackets.polarityPosition = 0;
				emptyContainerCommit                   = false;
			}

			if (state->currentPackets.specialPosition > 0) {
				containerGenerationSetPacket(
					&state->container, SPECIAL_EVENT, (caerEventPacketHeader) state->currentPackets.special);

				state->currentPackets.special         = NULL;
				state->currentPackets.specialPosition = 0;
				emptyContainerCommit                  = false;
			}

			if (state->currentPackets.imu6Position > 0) {
				containerGenerationSetPacket(
					&state->container, IMU6_EVENT_PKT_POS, (caerEventPacketHeader) state->currentPackets.imu6);

				state->currentPackets.imu6         = NULL;
				state->currentPackets.imu6Position = 0;
				emptyContainerCommit               = false;
			}

			if (tsReset || tsBigWrap) {
				// Ignore all IMU6 (composite) events, until a new IMU6
				// Start event comes in, for the next packet.
				// This is to correctly support the forced packet commits that a TS reset,
				// or a TS big wrap, impose. Continuing to parse events would result
				// in a corrupted state of the first event in the new packet, as it would
				// be incomplete, incorrect and miss vital initialization data.
				// See IMU6 END states for more details on a related issue.
				state->imu.ignoreEvents = true;
			}

			containerGenerationExecute(&state->container, emptyContainerCommit, tsReset, state->timestamps.wrapOverflow,
				state->timestamps.current, &state->dataExchange, &state->usbState.dataTransfersRun,
				handle->info.deviceID, handle->info.deviceString, &state->deviceLogLevel);
		}
	}
}

static void dvs132sTSMasterStatusUpdater(void *userDataPtr, int status, uint32_t param) {
	// If any USB error happened, discard.
	if (status != LIBUSB_TRANSFER_COMPLETED) {
		return;
	}

	// Get new Master/Slave information from device.
	struct caer_dvs132s_info *info = userDataPtr;

	atomic_thread_fence(memory_order_seq_cst);
	info->deviceIsMaster = param;
	atomic_thread_fence(memory_order_seq_cst);
}

uint32_t caerBiasCoarseFine1024Generate(struct caer_bias_coarsefine1024 coarseFine1024Bias) {
	uint32_t biasValue = 0;

	// Build up bias value from all its components.
	biasValue |= U32T((coarseFine1024Bias.coarseValue & 0x03FF) << 16);
	biasValue |= U32T((coarseFine1024Bias.fineValue & 0x03FF) << 0);

	return (biasValue);
}

struct caer_bias_coarsefine1024 caerBiasCoarseFine1024Parse(uint32_t coarseFine1024Bias) {
	struct caer_bias_coarsefine1024 biasValue;

	// Decompose bias integer into its parts.
	biasValue.coarseValue = (coarseFine1024Bias >> 16) & 0x03FF;
	biasValue.fineValue   = (coarseFine1024Bias >> 0) & 0x03FF;

	return (biasValue);
}

struct caer_bias_coarsefine1024 caerBiasCoarseFine1024FromCurrent(uint32_t picoAmps) {
	struct caer_bias_coarsefine1024 biasValue;

	// Disable bias.
	if (picoAmps == 0) {
		biasValue.coarseValue = 0;
		biasValue.fineValue   = 0;

		return (biasValue);
	}

	// We support between 1 pA and 1 uA (1 mio pA).
	if (picoAmps > 1000000) {
		picoAmps = 1000000; // Limit to 1 uA.
	}

	// We try to keep fine towards the mid-range as much as possible.
	// Above 0.5 uA, that will gradually go towards the maximum value so we
	// can reach 1 uA at all.
	double fineNudge;

	if (picoAmps <= 500000) {
		fineNudge = 2.0; // 2.0 for coarse with fine at midpoint.
	}
	else if (picoAmps <= 600000) {
		fineNudge = 10.0 / 6.0;
	}
	else if (picoAmps <= 700000) {
		fineNudge = 10.0 / 7.0;
	}
	else if (picoAmps <= 800000) {
		fineNudge = 10.0 / 8.0;
	}
	else if (picoAmps <= 900000) {
		fineNudge = 10.0 / 9.0;
	}
	else {               // Up to 1 uA.
		fineNudge = 1.0; // 1.0 to reach maximum coarse.
	}

	// Calculate coarse value in pico-Ampere, where 0 is disabled and 1023 is max.
	double coarseValueDouble = ceil(((1023.0 * fineNudge * (double) picoAmps) / 1000000.0));
	int32_t coarseValue      = I32T(coarseValueDouble);

	// Ensure within range.
	if (coarseValue < 1) {
		coarseValue = 1;
	}
	else if (coarseValue > 1023) {
		coarseValue = 1023;
	}

	biasValue.coarseValue = U16T(coarseValue);

	// Calculate coarse current value based on value going to device.
	// This is the maximum for the fine divider.
	double coarseCurrent = (1000000.0 * (double) coarseValue) / 1023.0;

	double fineValueDouble = round(((1023.0 * (double) picoAmps) / coarseCurrent));
	int32_t fineValue      = I32T(fineValueDouble);

	// Ensure within range.
	if (fineValue < 1) {
		fineValue = 1;
	}
	else if (fineValue > 1023) {
		fineValue = 1023;
	}

	biasValue.fineValue = U16T(fineValue);

	return (biasValue);
}

uint32_t caerBiasCoarseFine1024ToCurrent(struct caer_bias_coarsefine1024 coarseFine1024Bias) {
	// Special base: disabled bias.
	if ((coarseFine1024Bias.coarseValue == 0) || (coarseFine1024Bias.fineValue == 0)) {
		return (0);
	}

	double coarseCurrent = (1000000.0 * (double) coarseFine1024Bias.coarseValue) / 1023.0;
	double fineCurrent   = (coarseCurrent * (double) coarseFine1024Bias.fineValue) / 1023.0;

	double biasCurrent = round(fineCurrent);

	return (U32T(biasCurrent));
}

//////////////////////////////////
/// FX3 Debug Transfer Support ///
//////////////////////////////////
static void allocateDebugTransfers(dvs132sHandle handle) {
	// Allocate transfers and set them up.
	for (size_t i = 0; i < DEBUG_TRANSFER_NUM; i++) {
		handle->state.fx3Support.debugTransfers[i] = libusb_alloc_transfer(0);
		if (handle->state.fx3Support.debugTransfers[i] == NULL) {
			dvs132sLog(CAER_LOG_CRITICAL, handle,
				"Unable to allocate further libusb transfers (debug channel, %zu of %" PRIu32 ").", i,
				DEBUG_TRANSFER_NUM);
			continue;
		}

		// Create data buffer.
		handle->state.fx3Support.debugTransfers[i]->length = DEBUG_TRANSFER_SIZE;
		handle->state.fx3Support.debugTransfers[i]->buffer = malloc(DEBUG_TRANSFER_SIZE);
		if (handle->state.fx3Support.debugTransfers[i]->buffer == NULL) {
			dvs132sLog(CAER_LOG_CRITICAL, handle,
				"Unable to allocate buffer for libusb transfer %zu (debug channel). Error: %d.", i, errno);

			libusb_free_transfer(handle->state.fx3Support.debugTransfers[i]);
			handle->state.fx3Support.debugTransfers[i] = NULL;

			continue;
		}

		// Initialize Transfer.
		handle->state.fx3Support.debugTransfers[i]->dev_handle = handle->state.usbState.deviceHandle;
		handle->state.fx3Support.debugTransfers[i]->endpoint   = DEBUG_ENDPOINT;
		handle->state.fx3Support.debugTransfers[i]->type       = LIBUSB_TRANSFER_TYPE_INTERRUPT;
		handle->state.fx3Support.debugTransfers[i]->callback   = &libUsbDebugCallback;
		handle->state.fx3Support.debugTransfers[i]->user_data  = handle;
		handle->state.fx3Support.debugTransfers[i]->timeout    = 0;
		handle->state.fx3Support.debugTransfers[i]->flags      = LIBUSB_TRANSFER_FREE_BUFFER;

		if ((errno = libusb_submit_transfer(handle->state.fx3Support.debugTransfers[i])) == LIBUSB_SUCCESS) {
			atomic_fetch_add(&handle->state.fx3Support.activeDebugTransfers, 1);
		}
		else {
			dvs132sLog(CAER_LOG_CRITICAL, handle,
				"Unable to submit libusb transfer %zu (debug channel). Error: %s (%d).", i, libusb_strerror(errno),
				errno);

			// The transfer buffer is freed automatically here thanks to
			// the LIBUSB_TRANSFER_FREE_BUFFER flag set above.
			libusb_free_transfer(handle->state.fx3Support.debugTransfers[i]);
			handle->state.fx3Support.debugTransfers[i] = NULL;
		}
	}

	if (atomic_load(&handle->state.fx3Support.activeDebugTransfers) == 0) {
		// Didn't manage to allocate any USB transfers, log failure.
		dvs132sLog(CAER_LOG_CRITICAL, handle, "Unable to allocate any libusb transfers (debug channel).");
	}
}

static void cancelAndDeallocateDebugTransfers(dvs132sHandle handle) {
	// Wait for all transfers to go away.
	struct timespec waitForTerminationSleep = {.tv_sec = 0, .tv_nsec = 1000000};

	while (atomic_load(&handle->state.fx3Support.activeDebugTransfers) > 0) {
		// Continue trying to cancel all transfers until there are none left.
		// It seems like one cancel pass is not enough and some hang around.
		for (size_t i = 0; i < DEBUG_TRANSFER_NUM; i++) {
			if (handle->state.fx3Support.debugTransfers[i] != NULL) {
				errno = libusb_cancel_transfer(handle->state.fx3Support.debugTransfers[i]);
				if ((errno != LIBUSB_SUCCESS) && (errno != LIBUSB_ERROR_NOT_FOUND)) {
					dvs132sLog(CAER_LOG_CRITICAL, handle,
						"Unable to cancel libusb transfer %zu (debug channel). Error: %s (%d).", i,
						libusb_strerror(errno), errno);
					// Proceed with trying to cancel all transfers regardless of errors.
				}
			}
		}

		// Sleep for 1ms to avoid busy loop.
		thrd_sleep(&waitForTerminationSleep, NULL);
	}

	// No more transfers in flight, deallocate them all here.
	for (size_t i = 0; i < DEBUG_TRANSFER_NUM; i++) {
		if (handle->state.fx3Support.debugTransfers[i] != NULL) {
			libusb_free_transfer(handle->state.fx3Support.debugTransfers[i]);
			handle->state.fx3Support.debugTransfers[i] = NULL;
		}
	}
}

static void LIBUSB_CALL libUsbDebugCallback(struct libusb_transfer *transfer) {
	dvs132sHandle handle = transfer->user_data;

	// Completed or cancelled transfers are what we expect to handle here, so
	// if they do have data attached, try to parse them.
	if (((transfer->status == LIBUSB_TRANSFER_COMPLETED) || (transfer->status == LIBUSB_TRANSFER_CANCELLED))
		&& (transfer->actual_length > 0)) {
		// Handle debug data.
		debugTranslator(handle, transfer->buffer, (size_t) transfer->actual_length);
	}

	if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
		// Submit transfer again.
		if (libusb_submit_transfer(transfer) == LIBUSB_SUCCESS) {
			return;
		}
	}

	// Cannot recover (cancelled, no device, or other critical error).
	// Signal this by adjusting the counter and exiting.
	// Freeing the transfers is taken care of by cancelAndDeallocateDebugTransfers().
	atomic_fetch_sub(&handle->state.fx3Support.activeDebugTransfers, 1);
}

static void debugTranslator(dvs132sHandle handle, const uint8_t *buffer, size_t bytesSent) {
	// Check if this is a debug message (length 7-64 bytes).
	if ((bytesSent >= 7) && (buffer[0] == 0x00)) {
		// Debug message, log this.
		dvs132sLog(CAER_LOG_ERROR, handle, "Error message: '%s' (code %u at time %u).", &buffer[6], buffer[1],
			*((const uint32_t *) &buffer[2]));
	}
	else {
		// Unknown/invalid debug message, log this.
		dvs132sLog(CAER_LOG_WARNING, handle, "Unknown/invalid debug message.");
	}
}
