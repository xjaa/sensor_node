/**
 * @file main.c
 * @brief ROBUST: ESP32-S3 Sensor Node in Broadcaster Mode
 *
 * This device reads sensors and broadcasts the data in a non-connectable
 * advertising packet. It is hardened against I2C sensor failures.
 * If a sensor is not connected at boot or fails during operation, it broadcasts
 * an error code for that specific value without crashing or affecting other sensor readings.
 */

#include <stdio.h>
#include <string.h>
#include <limits.h> // For INT16_MAX etc.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/i2c.h"
#include "esp_mac.h"

// 传感器库
#include "sht4x.h"
#include "bh1750.h" 
#include "i2cdev.h" 

// NimBLE 协议栈
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"

#define TAG "SENSOR_BROADCASTER"

// I2C 配置
#define I2C_MASTER_SCL_IO           GPIO_NUM_4
#define I2C_MASTER_SDA_IO           GPIO_NUM_5
#define I2C_MASTER_NUM              I2C_NUM_0

// 自定义制造商ID
#define CUSTOM_MANU_ID              0x02E5 // Espressif Inc.

// 定义广播数据包的结构
#pragma pack(push, 1)
typedef struct {
    uint16_t manu_id;
    uint8_t  node_id;
    int16_t  temperature;
    uint16_t humidity;
    uint16_t illuminance;
} adv_sensor_data_t;
#pragma pack(pop)

// 定义错误码
#define TEMP_ERROR_VAL      INT16_MAX
#define HUMI_ERROR_VAL      UINT16_MAX
#define LUX_ERROR_VAL       UINT16_MAX

// 全局变量，用于存储要广播的数据
static adv_sensor_data_t g_adv_data;

/**
 * @brief 设置并更新蓝牙广播数据
 */
static void ble_app_set_adv_data(void)
{
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.mfg_data = (uint8_t*)&g_adv_data;
    fields.mfg_data_len = sizeof(g_adv_data);

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting advertising data; rc=%d", rc);
    }
}

/**
 * @brief 配置并开始蓝牙广播
 */
static void ble_app_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.conn_mode = BLE_GAP_CONN_MODE_NON;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    
    int rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, NULL, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error starting advertising; rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "Advertising started (non-connectable)");
}

/**
 * @brief NimBLE协议栈同步完成后的回调
 */
static void ble_app_on_sync(void)
{
    ble_app_set_adv_data();
    ble_app_advertise();
}

/**
 * @brief NimBLE 主机任务
 */
void ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/**
 * @brief FreeRTOS 任务，用于读取传感器数据并更新广播包
 */
static void sensor_task(void *pvParameters)
{
    sht4x_t sht40_dev;
    i2c_dev_t bh1750_dev; 
    
    bool sht40_ok = false;
    bool bh1750_ok = false;

    float temperature, humidity;
    uint16_t illuminance_raw; 

    // *** 核心修改：移除致命的 ESP_ERROR_CHECK，改为温和的错误处理 ***
    // 初始化 SHT40
    memset(&sht40_dev, 0, sizeof(sht40_dev));
    if (sht4x_init_desc(&sht40_dev, I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO) == ESP_OK &&
        sht4x_init(&sht40_dev) == ESP_OK) {
        ESP_LOGI(TAG, "SHT40 sensor initialized successfully");
        sht40_ok = true;
    } else {
        ESP_LOGE(TAG, "SHT40 sensor init failed (is it connected?)");
    }

    // 初始化 BH1750
    memset(&bh1750_dev, 0, sizeof(i2c_dev_t));
    if (bh1750_init_desc(&bh1750_dev, BH1750_ADDR_LO, I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO) == ESP_OK &&
        bh1750_setup(&bh1750_dev, BH1750_MODE_CONTINUOUS, BH1750_RES_HIGH) == ESP_OK) {
        ESP_LOGI(TAG, "BH1750 sensor configured successfully");
        bh1750_ok = true;
    } else {
        ESP_LOGE(TAG, "BH1750 sensor init failed (is it connected?)");
    }

    while (1) {
        // 读取 SHT40
        if (sht40_ok && sht4x_measure(&sht40_dev, &temperature, &humidity) == ESP_OK) {
            g_adv_data.temperature = (int16_t)(temperature * 100.0f);
            g_adv_data.humidity = (uint16_t)(humidity * 100.0f);
        } else {
            if (sht40_ok) ESP_LOGE(TAG, "Failed to read from SHT40 sensor");
            g_adv_data.temperature = TEMP_ERROR_VAL;
            g_adv_data.humidity = HUMI_ERROR_VAL;
        }

        // 读取 BH1750
        if (bh1750_ok && bh1750_read(&bh1750_dev, &illuminance_raw) == ESP_OK) {
            g_adv_data.illuminance = illuminance_raw;
        } else {
            if (bh1750_ok) ESP_LOGE(TAG, "Failed to read from BH1750 sensor");
            g_adv_data.illuminance = LUX_ERROR_VAL;
        }

        ESP_LOGI(TAG, "Updating advertising data...");
        
        ble_app_set_adv_data();
        
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(i2cdev_init());

    // 初始化时就填入错误码，防止重启后广播0
    g_adv_data.manu_id = CUSTOM_MANU_ID;
    g_adv_data.temperature = TEMP_ERROR_VAL;
    g_adv_data.humidity = HUMI_ERROR_VAL;
    g_adv_data.illuminance = LUX_ERROR_VAL;
    
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    g_adv_data.node_id = mac[5];
    ESP_LOGI(TAG, "This device's Node ID is: %d (from MAC: ...%02X)", g_adv_data.node_id, mac[5]);

    nimble_port_init();
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(ble_host_task);

    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
}
