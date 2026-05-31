#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_zigbee_core.h"
#include "onewire_bus.h"
#include "ds18b20.h"

#define TAG "ZB_TEMP"

// ---------- CONFIGURATION ----------
#define TEMP_ENDPOINT          1
#define DS_GPIO                4
#define MEASURE_INTERVAL_MS    (5 * 60 * 1000) 
#define TEMP_DELTA             50              

// ---------- GLOBAL VARIABLES ----------
static int16_t temperature_value = 0;
static int16_t last_reported = -10000;

static ds18b20_device_handle_t ds18b20 = NULL;
static onewire_bus_handle_t bus = NULL;

// ---------- SENSOR LOGIC ----------
static void ds18b20_init_sensor(void) {
    // Basic bus configuration
    onewire_bus_config_t bus_config = { 
        .bus_gpio_num = DS_GPIO 
    };

    // RMT backend configuration (Required for ESP32-H2)
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10, 
    };

    // Initialize the 1-Wire bus using RMT
    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));

    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_device;
    ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
    
    ESP_LOGI(TAG, "Searching for DS18B20 sensor...");
    if (onewire_device_iter_get_next(iter, &next_device) == ESP_OK) {
        ds18b20_config_t ds_cfg = {}; 
        ESP_ERROR_CHECK(ds18b20_new_device_from_enumeration(&next_device, &ds_cfg, &ds18b20));
        ESP_LOGI(TAG, "Found sensor: %08llx", next_device.address);
    } else {
        ESP_LOGE(TAG, "No OneWire devices found on GPIO %d", DS_GPIO);
    }
    onewire_del_device_iter(iter);

    if (ds18b20) {
        ESP_ERROR_CHECK(ds18b20_set_resolution(ds18b20, DS18B20_RESOLUTION_12B));
        ESP_LOGI(TAG, "DS18B20 initialized successfully");
    }
}

static float ds18b20_read_temperature(void) {
    float temp = 0;
    if (!ds18b20) return -127.0;
    if (ds18b20_trigger_temperature_conversion(ds18b20) != ESP_OK) return -127.0;
    vTaskDelay(pdMS_TO_TICKS(750)); 
    if (ds18b20_get_temperature(ds18b20, &temp) != ESP_OK) return -127.0;
    return temp;
}

// ---------- ZIGBEE REPORTING ----------
static void report_temperature(void) {
    // Update local attribute storage inside Zigbee stack
    esp_zb_zcl_set_attribute_val(
        TEMP_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        &temperature_value,
        false);

    // Prepare report command structure
    esp_zb_zcl_report_attr_cmd_t report_req = {
        .zcl_basic_cmd = {
            .src_endpoint = TEMP_ENDPOINT,
        },
        // Configured to send to whatever destination ZHA dynamically binds to this cluster
        .address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT, 
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        .attributeID = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        .direction = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    };
    
    esp_zb_zcl_report_attr_cmd_req(&report_req);
    last_reported = temperature_value;
    ESP_LOGI(TAG, "ZCL Report Sent: %d", temperature_value);
}

// ---------- SENSOR TASK ----------
static void sensor_task(void *pvParameters) {
    while (1) {
        float temp = ds18b20_read_temperature();
        if (temp > -100) {
            temperature_value = (int16_t)(temp * 100);
            if (abs(temperature_value - last_reported) >= TEMP_DELTA) {
                report_temperature();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(MEASURE_INTERVAL_MS));
    }
}

// ---------- ZIGBEE EVENT HANDLER ----------
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
    esp_zb_app_signal_type_t sig_type = *(esp_zb_app_signal_type_t *)signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Joined network successfully");
        } else {
            ESP_LOGW(TAG, "Join failed, status: %d", err_status);
        }
        break;
    default:
        break;
    }
}

// ---------- ZIGBEE INITIALIZATION ----------
static void zigbee_init(void) {
    esp_zb_cfg_t zb_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,
        .install_code_policy = false,
        .nwk_cfg.zed_cfg = {
            .ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_64MIN,
            .keep_alive = 3000,
        }
    };
    esp_zb_init(&zb_cfg);

    /* 1. Create Basic Cluster */
    esp_zb_attribute_list_t *basic_attr_list = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    esp_zb_basic_cluster_add_attr(basic_attr_list, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, "Espressif");
    esp_zb_basic_cluster_add_attr(basic_attr_list, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, "ESP32H2_Temp_Sensor");

    /* 2. Create Identify Cluster */
    uint8_t identify_id = 0;
    esp_zb_attribute_list_t *identify_attr_list = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    esp_zb_identify_cluster_add_attr(identify_attr_list, ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &identify_id);

    /* 3. Create Temperature Measurement Cluster USING THE STANDARD INITIALIZER */
    // This automatically configures required min/max boundaries so the stack validation won't crash.
    esp_zb_temperature_meas_cluster_cfg_t temp_cfg = {
        .measured_value = 0,
        .min_value = -5000,  // -50.00 °C
        .max_value = 10000,  // 100.00 °C
    };
    esp_zb_attribute_list_t *temp_attr_list = esp_zb_temperature_meas_cluster_create(&temp_cfg);

    /* 4. Combine Clusters into a Cluster List */
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(cluster_list, identify_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list, temp_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* 5. Create Endpoint and add the Cluster List to it */
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t ep_cfg = {
        .endpoint = TEMP_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
    };
    esp_zb_ep_list_add_ep(ep_list, cluster_list, ep_cfg);

    esp_zb_device_register(ep_list);
    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);
    
    esp_zb_set_rx_on_when_idle(false);
}

// ---------- MAIN ----------
void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ds18b20_init_sensor();
    zigbee_init();

    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);

    esp_zb_start(false);
    esp_zb_stack_main_loop(); 
}