#include "ble_file_transfer.h"

#include <esp_log.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
#include <esp_bt_device.h>
#include <esp_gap_ble_api.h>
#include <esp_gatts_api.h>
#include <esp_gatt_defs.h>
#include <esp_gatt_common_api.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <fstream>

#define TAG "BLEFileTransfer"
#define GATTS_APP_ID 0x55
#define FILE_SERVICE_UUID 0xFFF0
#define FILE_CHAR_UUID    0xFFF1
#define MAX_ATTR_NUM 4

static const uint16_t primary_service_uuid        = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid  = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid= ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t  char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_READ |
                                                    ESP_GATT_CHAR_PROP_BIT_WRITE |
                                                    ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static uint8_t config_descriptor[2] = {0x00, 0x00};

static const uint16_t file_service_uuid = FILE_SERVICE_UUID;
static const uint16_t file_char_uuid    = FILE_CHAR_UUID;

static esp_gatt_if_t g_gatts_if = ESP_GATT_IF_NONE;
static uint16_t g_service_handle = 0;
static uint16_t g_char_handle = 0;
static uint16_t g_conn_id = 0;
static bool g_connected = false;
static std::function<void(bool)> transfer_done_cb;

static const esp_gatts_attr_db_t g_gatt_db[MAX_ATTR_NUM] = {
    // Service
    [0] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t*)&primary_service_uuid, ESP_GATT_PERM_READ,
         sizeof(uint16_t), sizeof(file_service_uuid), (uint8_t*)&file_service_uuid}
    },
    // Characteristic Declaration
    [1] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ,
         sizeof(uint8_t), sizeof(uint8_t),
         (uint8_t*)&char_prop_read_write_notify}
    },
    // Characteristic Value
    [2] = {
        {ESP_GATT_RSP_BY_APP},
        {ESP_UUID_LEN_16, (uint8_t*)&file_char_uuid,
         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         512, 0, NULL}
    },
    // Client Characteristic Configuration (for notify)
    [3] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t*)&character_client_config_uuid,
         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(uint16_t), sizeof(uint16_t), (uint8_t*)&config_descriptor}
    },
};

static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT: {
        g_gatts_if = gatts_if;
        ESP_LOGI(TAG, "GATTS REG EVT - gatts_if: %d", gatts_if);

        // Set device name
        esp_err_t ret = esp_ble_gap_set_device_name("BLEFileXfer");
        ESP_LOGI(TAG, "Set device name 'BLEFileXfer': %s", esp_err_to_name(ret));

        // Configure and start advertising
        {
            esp_ble_adv_params_t adv_params = {};
            adv_params.adv_int_min        = 0x20;      // 20ms
            adv_params.adv_int_max        = 0x40;      // 40ms
            adv_params.adv_type           = ADV_TYPE_IND;
            adv_params.own_addr_type      = BLE_ADDR_TYPE_PUBLIC;
            adv_params.channel_map        = ADV_CHNL_ALL;
            adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;

            ret = esp_ble_gap_start_advertising(&adv_params);
            ESP_LOGI(TAG, "Start advertising: %s", esp_err_to_name(ret));
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "BLE device 'BLEFileXfer' should now be discoverable!");
            }
        }

        // Create GATT attribute table
        ret = esp_ble_gatts_create_attr_tab(g_gatt_db, gatts_if, MAX_ATTR_NUM, 0);
        ESP_LOGI(TAG, "Create attribute table: %s", esp_err_to_name(ret));
        break;
    }
    case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
        if (param->add_attr_tab.status == ESP_GATT_OK) {
            g_service_handle = param->add_attr_tab.handles[0];
            g_char_handle = param->add_attr_tab.handles[2];
            esp_err_t ret = esp_ble_gatts_start_service(g_service_handle);
            ESP_LOGI(TAG, "Service started: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGE(TAG, "Create attr table failed: %d", param->add_attr_tab.status);
        }
        break;
    }
    case ESP_GATTS_CONNECT_EVT: {
        g_connected = true;
        g_conn_id = param->connect.conn_id;
        ESP_LOGI(TAG, "Client connected - conn_id: %d", g_conn_id);
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT: {
        g_connected = false;
        ESP_LOGI(TAG, "Client disconnected - conn_id: %d", param->disconnect.conn_id);

        // Restart advertising after disconnection
        esp_ble_adv_params_t adv_params = {};
        adv_params.adv_int_min        = 0x20;
        adv_params.adv_int_max        = 0x40;
        adv_params.adv_type           = ADV_TYPE_IND;
        adv_params.own_addr_type      = BLE_ADDR_TYPE_PUBLIC;
        adv_params.channel_map        = ADV_CHNL_ALL;
        adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
        esp_err_t ret = esp_ble_gap_start_advertising(&adv_params);
        ESP_LOGI(TAG, "Restart advertising after disconnect: %s", esp_err_to_name(ret));
        break;
    }
    // Handle other GATTS events (not used in our simple file transfer implementation)
    case ESP_GATTS_READ_EVT:
    case ESP_GATTS_WRITE_EVT:
    case ESP_GATTS_EXEC_WRITE_EVT:
    case ESP_GATTS_MTU_EVT:
    case ESP_GATTS_CONF_EVT:
    case ESP_GATTS_UNREG_EVT:
    case ESP_GATTS_CREATE_EVT:
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
    case ESP_GATTS_ADD_CHAR_EVT:
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
    case ESP_GATTS_DELETE_EVT:
    case ESP_GATTS_START_EVT:
    case ESP_GATTS_STOP_EVT:
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    case ESP_GATTS_RESPONSE_EVT:
    case ESP_GATTS_SET_ATTR_VAL_EVT:
    case ESP_GATTS_SEND_SERVICE_CHANGE_EVT:
        // These events are not needed for our simple file transfer service
        break;
    default:
        ESP_LOGD(TAG, "Unhandled GATTS event: %d", event);
        break;
    }
}

void BleFileTransfer::Init(std::function<void(bool)> on_transfer_done) {
    transfer_done_cb = on_transfer_done;
    ESP_LOGI(TAG, "Starting BLE initialization...");

    // Step 1: Release classic BT memory
    esp_err_t ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    ESP_LOGI(TAG, "Release classic BT memory: %s", esp_err_to_name(ret));

    // Step 2: Initialize BT controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(ret));
        if (transfer_done_cb) transfer_done_cb(false);
        return;
    }
    ESP_LOGI(TAG, "BT controller init: %s", esp_err_to_name(ret));

    // Step 3: Enable BLE mode
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(ret));
        if (transfer_done_cb) transfer_done_cb(false);
        return;
    }
    ESP_LOGI(TAG, "BT controller enable: %s", esp_err_to_name(ret));

    // Step 4: Initialize Bluedroid
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        if (transfer_done_cb) transfer_done_cb(false);
        return;
    }
    ESP_LOGI(TAG, "Bluedroid init: %s", esp_err_to_name(ret));

    // Step 5: Enable Bluedroid
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        if (transfer_done_cb) transfer_done_cb(false);
        return;
    }
    ESP_LOGI(TAG, "Bluedroid enable: %s", esp_err_to_name(ret));

    // Step 6: Register GATTS callback
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GATTS register callback failed: %s", esp_err_to_name(ret));
        if (transfer_done_cb) transfer_done_cb(false);
        return;
    }
    ESP_LOGI(TAG, "GATTS register callback: %s", esp_err_to_name(ret));

    // Step 7: Register GATTS application
    ret = esp_ble_gatts_app_register(GATTS_APP_ID);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GATTS app register failed: %s", esp_err_to_name(ret));
        if (transfer_done_cb) transfer_done_cb(false);
        return;
    }
    ESP_LOGI(TAG, "GATTS app register: %s, waiting for callback...", esp_err_to_name(ret));
}

void BleFileTransfer::SendFile(const std::string& filepath) {
    if (!g_connected || g_char_handle == 0) {
        ESP_LOGW(TAG, "Not connected or char not ready");
        if (transfer_done_cb) transfer_done_cb(false);
        return;
    }
    std::ifstream f(filepath, std::ios::binary);
    if (!f.is_open()) {
        ESP_LOGE(TAG, "Open failed %s", filepath.c_str());
        if (transfer_done_cb) transfer_done_cb(false);
        return;
    }
    std::vector<uint8_t> buf(512);
    int idx = 0;
    while (f) {
        f.read((char*)buf.data(), buf.size());
        size_t n = f.gcount();
        if (n == 0) break;
        esp_ble_gatts_send_indicate(g_gatts_if, g_conn_id, g_char_handle, n, buf.data(), false);
        ESP_LOGI(TAG, "Sent pkt %d size=%zu", idx++, n);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    f.close();
    if (transfer_done_cb) transfer_done_cb(true);
    ESP_LOGI(TAG, "Transfer done");
}

bool BleFileTransfer::IsConnected() const { return g_connected; }