#ifndef LIBCAER_SRC_DVS132S_H_
#define LIBCAER_SRC_DVS132S_H_

#include "libcaer/devices/device_discover.h"
#include "libcaer/devices/dvs132s.h"

#include "container_generation.h"
#include "data_exchange.h"
#include "usb_utils.h"

#define IMU_TYPE_TEMP   0x01
#define IMU_TYPE_GYRO   0x02
#define IMU_TYPE_ACCEL  0x04
#define IMU_TOTAL_COUNT 14

#define IMU6_EVENT_PKT_POS  2
#define DVS132S_EVENT_TYPES 3

#define DVS132S_POLARITY_DEFAULT_SIZE 4096
#define DVS132S_SPECIAL_DEFAULT_SIZE  128
#define DVS132S_IMU_DEFAULT_SIZE      64

#define DVS132S_DEVICE_NAME "DVS132S"

#define DVS132S_DEVICE_PID                 0x841E
#define DVS132S_REQUIRED_LOGIC_VERSION     18
#define DVS132S_REQUIRED_LOGIC_PATCH_LEVEL 1
#define DVS132S_REQUIRED_FIRMWARE_VERSION  6

#define DEBUG_ENDPOINT      0x81
#define DEBUG_TRANSFER_NUM  4
#define DEBUG_TRANSFER_SIZE 64

struct dvs132s_state {
	// Per-device log-level
	atomic_uint_fast8_t deviceLogLevel;
	// Data Acquisition Thread -> Mainloop Exchange
	struct data_exchange dataExchange;
	// USB Device State
	struct usb_state usbState;
	// Timestamp fields
	struct timestamps_state_new_logic timestamps;
	struct {
		// DVS specific fields
		uint16_t lastY;
		uint16_t lastX;
		int16_t sizeX;
		int16_t sizeY;
		bool invertXY;
	} dvs;
	struct {
		// IMU specific fields
		bool ignoreEvents;
		bool flipX;
		bool flipY;
		bool flipZ;
		uint8_t type;
		uint8_t count;
		uint8_t tmpData;
		float accelScale;
		float gyroScale;
		// Current composite events, for later copy, to not loose them on commits.
		struct caer_imu6_event currentEvent;
	} imu;
	// Packet Container state
	struct container_generation container;
	struct {
		// Polarity Packet state
		caerPolarityEventPacket polarity;
		int32_t polarityPosition;
		// IMU6 Packet state
		caerIMU6EventPacket imu6;
		int32_t imu6Position;
		// Special Packet state
		caerSpecialEventPacket special;
		int32_t specialPosition;
	} currentPackets;
	// Device timing data.
	struct {
		uint16_t logicClock;
		uint16_t usbClock;
		uint16_t clockDeviationFactor;
		float logicClockActual;
		float usbClockActual;
	} deviceClocks;
	struct {
		// Debug transfer support (FX3 only).
		struct libusb_transfer *debugTransfers[DEBUG_TRANSFER_NUM];
		atomic_uint_fast32_t activeDebugTransfers;
	} fx3Support;
};

typedef struct dvs132s_state *dvs132sState;

struct dvs132s_handle {
	uint16_t deviceType;
	// Information fields
	struct caer_dvs132s_info info;
	// State for data management, common to all DVS132S.
	struct dvs132s_state state;
};

typedef struct dvs132s_handle *dvs132sHandle;

ssize_t dvs132sFind(caerDeviceDiscoveryResult *discoveredDevices);

caerDeviceHandle dvs132sOpen(
	uint16_t deviceID, uint8_t busNumberRestrict, uint8_t devAddressRestrict, const char *serialNumberRestrict);
bool dvs132sClose(caerDeviceHandle cdh);

bool dvs132sSendDefaultConfig(caerDeviceHandle cdh);
// Negative addresses are used for host-side configuration.
// Positive addresses (including zero) are used for device-side configuration.
bool dvs132sConfigSet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t param);
bool dvs132sConfigGet(caerDeviceHandle cdh, int8_t modAddr, uint8_t paramAddr, uint32_t *param);

bool dvs132sDataStart(caerDeviceHandle handle, void (*dataNotifyIncrease)(void *ptr),
	void (*dataNotifyDecrease)(void *ptr), void *dataNotifyUserPtr, void (*dataShutdownNotify)(void *ptr),
	void *dataShutdownUserPtr);
bool dvs132sDataStop(caerDeviceHandle handle);
caerEventPacketContainer dvs132sDataGet(caerDeviceHandle handle);

#endif /* LIBCAER_SRC_DVS132S_H_ */
