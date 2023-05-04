/* includes that will always be required */
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/drivers/gpio.h>
#include <math.h>

#include "ttpms_common.h"
#include "I2C_Functions.h"
#include "MLX90641_API.h"

/* includes for debugging/temporary */
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ttpms);


// IMPORTANT: define which corner the sensor will go
// options = TTPMS_IFL, TTPMS_IFR, TTPMS_IRL, TTPMS_IRR
#define TTPMS_IFL


// frequency of temp sensor updates (0x00 = 0.5Hz, 0x01 = 1Hz, 0x02 = 2Hz, 0x03 = 4Hz, 0x04 = 8Hz, 0x05 = 16Hz (default), 0x06 = 32Hz, 0x07 = 64Hz)
// Unlike the MLX90640, each MLX90641 subpage contains new data for every pixel
// have not tested max refresh rate with MLX90641 yet
// best way for more speed is to modify the MLX90641_API.c functions to only read and process relevant pixels
#define TEMP_FREQ 0x04


// haven't tuned these
#define TEMP_THREAD_STACKSIZE 8192
#define TEMP_THREAD_PRIORITY 4

void temp_thread(void *dummy1, void *dummy2, void *dummy3);
K_THREAD_STACK_DEFINE(temp_stack_area, TEMP_THREAD_STACKSIZE);
struct k_thread temp_thread_data;
k_tid_t temp_thread_id;


// How we keep track of state.
// Use Zephyr atomic set, clear, test functions.
ATOMIC_DEFINE(flags, 2);
#define TEMP_ENABLED_FLAG		0
#define PRESSURE_ENABLED_FLAG	1


// The actual temp data we will send to receiver
// Temp values have 0.5 scale, 0 offset (ie 0xAF = 175 = 87.5 C)
uint8_t tire_temp[16];


// Tune the below with testing
const float ta_shift = 8;		// Default shift for MLX90641 in open air
const float emissivity = 0.95;


static float mlx90641To[192];	// Processed temp data from MLX functions
paramsMLX90641 mlx90641;		// Used for MLX functions

float Vdd;	// From MLX, used by MLX functions
float Ta;	// From MLX, used by MLX functions


static const struct gpio_dt_spec mlx_power =
	GPIO_DT_SPEC_GET_OR(DT_NODELABEL(mlx_power), gpios, {0});



/* --- BLUETOOTH STUFF START --- */

#define BLE_ADV_INTERVAL_MIN 1200	// 1200 * 0.625ms = 0.75s
#define BLE_ADV_INTERVAL_MAX 2000	// 2000 * 0.625ms = 1.25s

#ifdef TTPMS_IFL
	#define TTPMS_SENSOR_BT_ID TTPMS_IFL_BT_ID
	char bt_device_name[28] = "TTPMS Internal Sensor FL";
#endif
#ifdef TTPMS_IFR
	#define TTPMS_SENSOR_BT_ID TTPMS_IFR_BT_ID
	char bt_device_name[28] = "TTPMS Internal Sensor FR";
#endif
#ifdef TTPMS_IRL
	#define TTPMS_SENSOR_BT_ID TTPMS_IRL_BT_ID
	char bt_device_name[28] = "TTPMS Internal Sensor RL";
#endif
#ifdef TTPMS_IRR
	#define TTPMS_SENSOR_BT_ID TTPMS_IRR_BT_ID
	char bt_device_name[28] = "TTPMS Internal Sensor RR";
#endif


static struct bt_uuid_128 primary_service_uuid = BT_UUID_INIT_128(TTPMS_SERVICE_BASE_UUID);
static struct bt_uuid_128 temp_characteristic_uuid = BT_UUID_INIT_128(TTPMS_SERVICE_TEMP_UUID);


static struct bt_le_adv_param adv_param;

static struct bt_conn *connection;


// this is a callback for if CCC is changed (could be subscribe or unsubscribe)
static void subscribe_temp(const struct bt_gatt_attr *attr, uint16_t value)
{
	const bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);	// not sure why this is assigned to this of just checked in the if()

	if (notif_enabled) {	
		atomic_set_bit(flags, TEMP_ENABLED_FLAG);	// enable temperature measurements and start temp thread
		temp_thread_id = k_thread_create(&temp_thread_data, temp_stack_area,
                                 K_THREAD_STACK_SIZEOF(temp_stack_area),
                                 temp_thread,
                                 NULL, NULL, NULL,
                                 TEMP_THREAD_PRIORITY, 0, K_NO_WAIT);
		LOG_INF("Notifications subscribed, temp enabled");
	} else {
		atomic_clear_bit(flags, TEMP_ENABLED_FLAG); // disable temperature measurements
		LOG_INF("Notifications unsubscribed, temp disabled");
	}
}


// Primary Service Declaration
BT_GATT_SERVICE_DEFINE(primary_service,
	BT_GATT_PRIMARY_SERVICE(&primary_service_uuid),
		BT_GATT_CHARACTERISTIC(&temp_characteristic_uuid.uuid, BT_GATT_CHRC_NOTIFY, NULL, NULL, NULL, NULL),
			BT_GATT_CCC(subscribe_temp, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, TTPMS_SERVICE_BASE_UUID),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_WRN("Connection failed (err 0x%02x)", err);
	} else {
		LOG_INF("Connected");
		connection = conn;
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected (reason 0x%02x)", reason);
	atomic_clear_bit(flags, TEMP_ENABLED_FLAG);	// disable temperature measurements
	connection = NULL;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected
};

/* --- BLUETOOTH STUFF END --- */


void main(void)
{
	LOG_INF("Running ttpms_v2_internal build for %s", CONFIG_BOARD);

	int err;
	
	if (!gpio_is_ready_dt(&mlx_power)) {
		LOG_ERR("The load switch pin GPIO port is not ready.");
		return;
	}

	err = gpio_pin_configure_dt(&mlx_power, GPIO_OUTPUT_INACTIVE);
	if (err != 0) {
		LOG_ERR("Configuring GPIO pin failed: %d", err);
		return;
	}

	bt_addr_le_t addr;

	err = bt_addr_le_from_str(TTPMS_SENSOR_BT_ID, "random", &addr);
	if (err) {
		LOG_ERR("Invalid BT address (err %d)", err);
	}

	err = bt_id_create(&addr, NULL);
	if (err < 0) {
		LOG_ERR("Creating new BT ID failed (err %d)", err);
	}

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
	} else {
		LOG_INF("Bluetooth initialized");
	}

	err = bt_set_name(bt_device_name);
	if (err) {
		LOG_ERR("Failed to set BT Device name (err %d)", err);
	} else {
		LOG_INF("Successfully set BT device name");
	}

	adv_param = *BT_LE_ADV_CONN_NAME;
	adv_param.interval_min = BLE_ADV_INTERVAL_MIN;
	adv_param.interval_max = BLE_ADV_INTERVAL_MAX;

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
	} else {
		LOG_INF("Advertising successfully started");
	}

}



void temp_thread(void *dummy1, void *dummy2, void *dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);

	int status;

	status = gpio_pin_set_dt(&mlx_power, 1);	// turn on MLX power MOSFET
	if (status != 0) {
		LOG_ERR("Setting GPIO pin level failed: %d", status);
	}

	//k_sleep(K_MSEC(80));	// wait 80ms after MLX90641 POR, then:
	//k_sleep(K_MSEC(500));	// wait one refresh rate? default refresh rate?
	k_sleep(K_MSEC(1000));	//datasheet is weird so just wait 1 second to be safe

	uint16_t eeMLX90641[832];

	status = MLX90641_DumpEE(MLX90641_ADDR, eeMLX90641);
	if (status != 0) {
		LOG_ERR("Failed to load system parameters, MLX90641_DumpEE() returned %d", status);
	} else {
		LOG_INF("Dumped MLX EE");
	}

	status = MLX90641_ExtractParameters(eeMLX90641, &mlx90641);
	if (status != 0) {
		LOG_ERR("Parameter extraction failed, MLX90641_ExtractParameters() returned %d", status);
	} else {
		LOG_INF("Extracted MLX parameters");
	}

	status = MLX90641_SetRefreshRate(MLX90641_ADDR, TEMP_FREQ);
	if (status != 0) {
		LOG_ERR("Setting refresh rate failed, MLX90641_SetRefreshRate() returned %d", status);
	} else {
		LOG_INF("Set MLX refresh rate");
	}

	status = MLX90641_SynchFrame(MLX90641_ADDR);
	if (status != 0) {
		LOG_ERR("Synchronizing MLX frame failed, MLX90641_SynchFrame() returned %d", status);
	} else {
		LOG_INF("Synchronized MLX frame");
	}

	
	while (atomic_test_bit(flags, TEMP_ENABLED_FLAG))	// atomic babbyyyy
	{

		uint16_t frame[834];

		status = MLX90641_GetFrameData(MLX90641_ADDR, frame);	// this function waits until data is available from MLX
		if (status < 0) {
			LOG_ERR("Getting MLX frame data failed, MLX90641_GetFrameData() returned %d", status);
		}

		Vdd = MLX90641_GetVdd(frame, &mlx90641);

		Ta = MLX90641_GetTa(frame, &mlx90641);

		float tr = Ta - ta_shift; // Reflected temperature based on the sensor ambient temperature

		//LOG_INF("started temp processing");
		MLX90641_CalculateTo(frame, &mlx90641, emissivity, tr, mlx90641To);
		//LOG_INF("finished temp processing");

		for (int i = 0; i < 16; i++)
		{
			tire_temp[i] = CLAMP(round((mlx90641To[64+i]+mlx90641To[80+i]+mlx90641To[96+i]+mlx90641To[112+i])/2), 0, 255);
		}
		
		//LOG_INF("First pixel: %.1f 16th pixel: %.1f, ((float)tire_temp[0]/2), ((float)tire_temp[15]/2));

		if((connection != NULL) && atomic_test_bit(flags, TEMP_ENABLED_FLAG)) {	// MLX processing above takes a while, check if we are still connected and temp is enabled
			status = bt_gatt_notify(connection, &primary_service.attrs[1], &tire_temp, sizeof(tire_temp));
			if(status != 0) {
				LOG_ERR("Failed to notify, bt_gatt_notify returned: %d", status);
			} else {
				//LOG_INF("Notified");
			}
		}
	}

	status = gpio_pin_set_dt(&mlx_power, 0);	// turn off MLX power MOSFET
	if (status != 0) {
		LOG_ERR("Setting GPIO pin level failed: %d", status);
	}

}