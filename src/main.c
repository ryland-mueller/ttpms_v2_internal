/* includes that will always be required */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>

/* includes for debugging/temporary */
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ttpms);

// IMPORTANT: define which corner the sensor will go
// options = TTPMS_IFL, TTPMS_IFR, TTPMS_IRL, TTPMS_IRR
#define TTPMS_IFL


int temp_value = 69;
int pressure_value = 420;


/* --- BLUETOOTH SHIT START --- */

#define BLE_ADV_INTERVAL_MIN 1200	// 1200 * 0.625ms = 0.75s
#define BLE_ADV_INTERVAL_MAX 2000	// 2000 * 0.625ms = 1.25s

// First hex char must be C for random static address
#ifdef TTPMS_IFL
#define TTPMS_SENSOR_BT_ID "CA:69:F1:F1:42:42"
char bt_device_name[28] = "TTPMS Internal Sensor FL";
#endif
#ifdef TTPMS_IFR
#define TTPMS_SENSOR_BT_ID "CA:69:F1:F1:43:43"
char bt_device_name[28] = "TTPMS Internal Sensor FR";
#endif
#ifdef TTPMS_IRL
#define TTPMS_SENSOR_BT_ID "CA:69:F1:F1:44:44"
char bt_device_name[28] = "TTPMS Internal Sensor RL";
#endif
#ifdef TTPMS_IRR
#define TTPMS_SENSOR_BT_ID "CA:69:F1:F1:45:45"
char bt_device_name[28] = "TTPMS Internal Sensor RR";
#endif

// Custom BT Service UUIDs
#define BT_UUID_CUSTOM_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

static struct bt_uuid_128 primary_service_uuid = BT_UUID_INIT_128(
	BT_UUID_CUSTOM_SERVICE_VAL);

static struct bt_uuid_128 temp_characteristic_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1));

static struct bt_uuid_128 pressure_characteristic_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef2));

static struct bt_le_adv_param adv_param;

static ssize_t read_temp(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
	// what would actually go here is submitting sensor measurement work to system workqueue,
	// then waiting for the work to be completed before passing on the temp value below
	int *value = &temp_value;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 sizeof(temp_value));
}

static ssize_t read_pressure(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
	int *value = &pressure_value;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 sizeof(temp_value));
}

// Vendor Primary Service Declaration
BT_GATT_SERVICE_DEFINE(primary_service,
	BT_GATT_PRIMARY_SERVICE(&primary_service_uuid),
	BT_GATT_CHARACTERISTIC(&temp_characteristic_uuid.uuid,
			       BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ,
			       read_temp, NULL, NULL),
	BT_GATT_CHARACTERISTIC(&pressure_characteristic_uuid.uuid,
			       BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ,
			       read_pressure, NULL, NULL),
);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_CUSTOM_SERVICE_VAL),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_WRN("Connection failed (err 0x%02x)", err);
	} else {
		LOG_INF("Connected");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason 0x%02x)", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected
};

/* --- BLUETOOTH SHIT END --- */


void main(void)
{
	LOG_INF("Running ttpms_v2_internal");

	int err;

	bt_addr_le_t addr;

	err = bt_addr_le_from_str(TTPMS_SENSOR_BT_ID, "random", &addr);
	if (err) {
		LOG_WRN("Invalid BT address (err %d)", err);
	}

	err = bt_id_create(&addr, NULL);
	if (err < 0) {
		LOG_WRN("Creating new BT ID failed (err %d)", err);
	}

	err = bt_enable(NULL);
	if (err) {
		LOG_WRN("Bluetooth init failed (err %d)", err);
	} else {
		LOG_INF("Bluetooth initialized");
	}

	err = bt_set_name(bt_device_name);
	if (err) {
		LOG_WRN("Failed to set BT Device name (err %d)", err);
	} else {
		LOG_INF("Successfully set BT device name");
	}

	adv_param = *BT_LE_ADV_CONN_NAME;
	adv_param.interval_min = BLE_ADV_INTERVAL_MIN;
	adv_param.interval_max = BLE_ADV_INTERVAL_MAX;

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_WRN("Advertising failed to start (err %d)", err);
	} else {
		LOG_INF("Advertising successfully started");
	}

	while (1) {
		k_sleep(K_FOREVER);
	}
}
