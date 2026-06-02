#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_zigbee_core.h"
#include "onewire_bus.h"
#include "ds18b20.h"
#include "led_strip.h" // Required for integrated WS2812B LED on ESP32-H2 Super Mini

#define TAG "ZB_TEMP"

// ---------- CONFIGURATION ----------
#define SENSOR_1_ENDPOINT      1
#define DS_GPIO                4

// Calibration Offset in Degrees Celsius (adjust as needed)
#define SENSOR_OFFSET         0.0  

// Super Mini On-Board WS2812B Pin Configuration
#define NEOPIXEL_GPIO          8   
#define NEOPIXEL_STRIP_LEN     1

#define MEASURE_INTERVAL_MS    (5 * 60 * 1000)  // 5 minutes
#define MAX_REPORT_INTERVAL_MS (30 * 60 * 1000) // 30 minutes heartbeat
#define TEMP_DELTA             50               // 0.5°C change trigger

// ---------- GLOBAL VARIABLES ----------
static int16_t sensor1_temp_value = 0;
static int16_t sensor1_last_reported = -10000;

static onewire_bus_handle_t bus = NULL;
static ds18b20_device_handle_t ds18b20_s1 = NULL;

// NeoPixel State Control
static led_strip_handle_t led_strip;
static bool zigbee_connected = false;
static TaskHandle_t red_led_task_handle = NULL;

// ---------- WS2812B NEOPIXEL LOGIC ----------
static void init_neopixel(void) {
    led_strip_config_t strip_config = {
        .strip_gpio_num = NEOPIXEL_GPIO,
        .max_leds = NEOPIXEL_STRIP_LEN,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // Broad version compatibility format
        .flags.invert_out = false,
    };
    
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);
}

// Startup Sequence: Cycles through Green, Blue, Red on boot
static void neopixel_startup_test_cycle(void) {
    ESP_LOGI(TAG, "Starting LED Diagnostic Cycle...");

    // 1. Solid Green (R=0, G=48, B=0)
    led_strip_set_pixel(led_strip, 0, 0, 48, 0);
    led_strip_refresh(led_strip);
    vTaskDelay(pdMS_TO_TICKS(500));

    // 2. Solid Blue (R=0, G=0, B=64)
    led_strip_set_pixel(led_strip, 0, 0, 0, 64);
    led_strip_refresh(led_strip);
    vTaskDelay(pdMS_TO_TICKS(500));

    // 3. Solid Red (R=64, G=0, B=0)
    led_strip_set_pixel(led_strip, 0, 64, 0, 0);
    led_strip_refresh(led_strip);
    vTaskDelay(pdMS_TO_TICKS(500));

    // Clear it out before normal background loops kick in
    led_strip_clear(led_strip);
}

// Background task that pulses Red when disconnected
static void red_led_blink_task(void *pvParameters) {
    while (1) {
        if (!zigbee_connected) {
            // Heartbeat double-blink pattern (Red color: R=64, G=0, B=0)
            led_strip_set_pixel(led_strip, 0, 64, 0, 0);
            led_strip_refresh(led_strip);
            vTaskDelay(pdMS_TO_TICKS(100));
            
            led_strip_clear(led_strip);
            vTaskDelay(pdMS_TO_TICKS(1500));
        } else {
            led_strip_clear(led_strip);
            vTaskSuspend(NULL); // Suspend self when connected
        }
    }
}

static void update_connection_leds(bool connected) {
    zigbee_connected = connected;
    if (connected) {
        if (red_led_task_handle) {
            vTaskSuspend(red_led_task_handle);
        }
        // Solid Green color (R=0, G=48, B=0)
        led_strip_set_pixel(led_strip, 0, 0, 48, 0);
        led_strip_refresh(led_strip);
    } else {
        if (red_led_task_handle) {
            vTaskResume(red_led_task_handle); // Wake up the red pulsing routine
        }
    }
}

static void flash_blue_led(void) {
    // Briefly overwrite pixel to standard Blue (R=0, G=0, B=128)
    led_strip_set_pixel(led_strip, 0, 0, 0, 128);
    led_strip_refresh(led_strip);
    
    vTaskDelay(pdMS_TO_TICKS(150)); // Flash hold length

    // Revert to current connectivity state color
    if (zigbee_connected) {
        led_strip_set_pixel(led_strip, 0, 0, 48, 0); // Back to solid Green
    } else {
        led_strip_clear(led_strip); // Back to blinking cycle
    }
    led_strip_refresh(led_strip);
}

// ---------- SENSOR LOGIC ----------
static void ds18b20_init_sensors(void) {
    onewire_bus_config_t bus_config = { .bus_gpio_num = DS_GPIO };
    onewire_bus_rmt_config_t rmt_config = { .max_rx_bytes = 10 };

    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));

    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_device;
    ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));

    ESP_LOGI(TAG, "Searching for DS18B20 sensor on 1-Wire bus...");
    
    if (onewire_device_iter_get_next(iter, &next_device) == ESP_OK) {
        ds18b20_config_t ds_cfg = {};
        ESP_ERROR_CHECK(ds18b20_new_device_from_enumeration(&next_device, &ds_cfg, &ds18b20_s1));
        ESP_LOGI(TAG, "Found Sensor Address: %08llx", next_device.address);
        ESP_ERROR_CHECK(ds18b20_set_resolution(ds18b20_s1, DS18B20_RESOLUTION_12B));
    } else {
        ESP_LOGW(TAG, "No 1-Wire devices found!");
    }
    
    onewire_del_device_iter(iter);
}

// ---------- ZIGBEE REPORTING ----------
static void report_temperature(uint8_t endpoint, int16_t value) {
    esp_zb_lock_acquire(portMAX_DELAY);

    esp_zb_zcl_set_attribute_val(
        endpoint,
        ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        &value,
        false);

    esp_zb_zcl_report_attr_cmd_t report_req = {
        .zcl_basic_cmd = { .src_endpoint = endpoint },
        .address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT,
        .clusterID = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        .attributeID = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        .direction = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    };

    esp_zb_zcl_report_attr_cmd_req(&report_req);
    esp_zb_lock_release();
    
    ESP_LOGI(TAG, "ZCL Report Sent from Endpoint %d: %d", endpoint, value);
    
    // Flash the NeoPixel Blue dynamically on data transmission
    flash_blue_led();
}

// ---------- SENSOR TASK ----------
static void sensor_task(void *pvParameters) {
    uint32_t s1_last_report_time = 0;

    while (1) {
        if (!ds18b20_s1) {
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }

        ds18b20_trigger_temperature_conversion(ds18b20_s1);
        vTaskDelay(pdMS_TO_TICKS(750)); 

        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

        float temp1 = 0;
        if (ds18b20_get_temperature(ds18b20_s1, &temp1) == ESP_OK && temp1 > -100) {
            
            // --- CALIBRATION BIAS STEP ---
            // Applies the preset software correction offset before formatting calculations
            float calibrated_temp1 = temp1 + SENSOR_OFFSET;
            
            sensor1_temp_value = (int16_t)(calibrated_temp1 * 100);
            
            if ((abs(sensor1_temp_value - sensor1_last_reported) >= TEMP_DELTA) ||
                (current_time - s1_last_report_time >= MAX_REPORT_INTERVAL_MS) ||
                (sensor1_last_reported == -10000)) {
                
                report_temperature(SENSOR_1_ENDPOINT, sensor1_temp_value);
                sensor1_last_reported = sensor1_temp_value;
                s1_last_report_time = current_time;
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
        update_connection_leds(false); 
        esp_zb_lock_acquire(portMAX_DELAY);
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        esp_zb_lock_release();
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Joined network successfully");
            update_connection_leds(true); 
        } else {
            update_connection_leds(false); 
            vTaskDelay(pdMS_TO_TICKS(10000));
            esp_zb_lock_acquire(portMAX_DELAY);
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            esp_zb_lock_release();
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
        .nwk_cfg.zed_cfg = { .ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_64MIN, .keep_alive = 3000 }
    };
    esp_zb_init(&zb_cfg);

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

    esp_zb_attribute_list_t *basic_attr_list = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    esp_zb_basic_cluster_add_attr(basic_attr_list, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, "Espressif");
    esp_zb_basic_cluster_add_attr(basic_attr_list, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, "ESP32H2_Temp_Sensor");

    uint8_t identify_id = 0;
    esp_zb_attribute_list_t *identify_attr_list = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    esp_zb_identify_cluster_add_attr(identify_attr_list, ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &identify_id);

    esp_zb_temperature_meas_cluster_cfg_t temp_cfg1 = { .measured_value = 0, .min_value = -5000, .max_value = 10000 };
    esp_zb_attribute_list_t *temp_attr_list1 = esp_zb_temperature_meas_cluster_create(&temp_cfg1);

    esp_zb_cluster_list_t *cluster_list1 = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(cluster_list1, basic_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(cluster_list1, identify_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list1, temp_attr_list1, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_endpoint_config_t ep_cfg1 = { .endpoint = SENSOR_1_ENDPOINT, .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID, .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID };
    esp_zb_ep_list_add_ep(ep_list, cluster_list1, ep_cfg1);

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

    // 1. Initialize RGB Driver
    init_neopixel();
    
    // 2. Fire the Green -> Blue -> Red Diagnostic Sequence 
    neopixel_startup_test_cycle();
    
    // 3. Spawn the background red-status network monitor thread
    xTaskCreate(red_led_blink_task, "red_led_task", 2560, NULL, 2, &red_led_task_handle);

    ds18b20_init_sensors();
    zigbee_init();

    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);

    esp_zb_start(false);
    esp_zb_stack_main_loop();
}