#include "esp_bt.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "ble_gatts.h"


#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"

#include "wifi_conn.h"

#define GATTS_TABLE_TAG "SEC_GATTS_DEMO"

#undef ESP_LOGE
#undef ESP_LOGD
#undef ESP_LOGI
#define ESP_LOGE(...)
#define ESP_LOGD(...)
#define ESP_LOGI(...)

#define WIFI_BT_PROFILE_NUM 1
#define WIFI_BT_PROFILE_APP_IDX 0
#define ESP_WIFI_BT_APP_ID 0x55
#define WIFI_BT_SVC_INST_ID 0

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

#define ADV_CONFIG_FLAG (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

enum {
  WIFI_BT_SVC,

  WIFI_BT_MEAS_CHAR,
  WIFI_BT_UUID_MEAS_VAL,
  WIFI_BT_PASS_CHAR,
  WIFI_BT_PASS_MEAS_VAL,

  WIFI_BT_NB,
};

char ssid[32];     /**< SSID of target AP. */
char password[64]; /**< Password of target AP. */
static bool isEnabled = false;

static char bt_device_name[ESP_BLE_ADV_NAME_LEN_MAX] = "ESP_HM3301_GPS";
static uint8_t adv_config_done = 0;
static uint16_t wifi_bt_handle_table[WIFI_BT_NB];
static uint8_t test_manufacturer[3] = {'E', 'S', 'P'};
static uint8_t sec_service_uuid[16] = {0x21, 0x4e, 0x0a, 0x8f, 0x9b, 0x5c,
                                       0xd2, 0xa1, 0x8f, 0x4e, 0x64, 0x3b,
                                       0x9a, 0x1c, 0x2f, 0x7e};

// config adv data
static esp_ble_adv_data_t wifi_bt_adv_config = {
    .set_scan_rsp = false,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(sec_service_uuid),
    .p_service_uuid = sec_service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// config scan response data
static esp_ble_adv_data_t wifi_bt_scan_rsp_config = {
    .set_scan_rsp = true,
    .include_name = true,
    .manufacturer_len = sizeof(test_manufacturer),
    .p_manufacturer_data = test_manufacturer,
};

static esp_ble_adv_params_t wifi_bt_adv_params = {
    .adv_int_min = 0x100,
    .adv_int_max = 0x100,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_RPA_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
  esp_gatts_cb_t gatts_cb;
  uint16_t gatts_if;
  uint16_t app_id;
  uint16_t conn_id;
  uint16_t service_handle;
  esp_gatt_srvc_id_t service_id;
  uint16_t char_handle;
  esp_bt_uuid_t char_uuid;
  esp_gatt_perm_t perm;
  esp_gatt_char_prop_t property;
  uint16_t descr_handle;
  esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t *param);

static struct gatts_profile_inst wifi_bt_profile_tab[WIFI_BT_PROFILE_NUM] = {
    [WIFI_BT_PROFILE_APP_IDX] =
        {
            .gatts_cb = gatts_profile_event_handler,
            .gatts_if = ESP_GATT_IF_NONE,
        },

};

/*
 *  WiFi BT PROFILE ATTRIBUTES
 */
static const uint16_t wifi_bt_svc = 0x104;
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t char_decl_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t ssid_uuid = 0x104A;
static const uint16_t password_uuid = 0x104B;
static const uint8_t char_prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE;

static const esp_gatts_attr_db_t heart_rate_gatt_db[WIFI_BT_NB] = {
    [WIFI_BT_SVC] = {{ESP_GATT_AUTO_RSP},
                     {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid,
                      ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(wifi_bt_svc),
                      (uint8_t *)&wifi_bt_svc}},

    [WIFI_BT_MEAS_CHAR] = {{ESP_GATT_AUTO_RSP},
                           {ESP_UUID_LEN_16,
                            (uint8_t *)&character_declaration_uuid,
                            ESP_GATT_PERM_READ, sizeof(uint8_t),
                            sizeof(uint8_t), (uint8_t *)&char_prop_write}},

    [WIFI_BT_UUID_MEAS_VAL] = {{ESP_GATT_AUTO_RSP},
                               {ESP_UUID_LEN_16, (uint8_t *)&ssid_uuid,
                                ESP_GATT_PERM_WRITE, sizeof(uint8_t) * 32, 0,
                                NULL}},

    [WIFI_BT_PASS_CHAR] = {{ESP_GATT_AUTO_RSP},
                           {ESP_UUID_LEN_16, (uint8_t *)&char_decl_uuid,
                            ESP_GATT_PERM_READ, sizeof(uint8_t),
                            sizeof(uint8_t), (uint8_t *)&char_prop_write}},

    [WIFI_BT_PASS_MEAS_VAL] = {{ESP_GATT_AUTO_RSP},
                               {ESP_UUID_LEN_16, (uint8_t *)&password_uuid,
                                ESP_GATT_PERM_WRITE, sizeof(uint8_t) * 64, 0,
                                NULL}},
};

/*********************************************************************************/

static void show_bonded_devices(void) {
  int dev_num = esp_ble_get_bond_device_num();
  if (dev_num == 0) {
    ESP_LOGI(GATTS_TABLE_TAG, "Bonded devices number zero\n");
    return;
  }

  esp_ble_bond_dev_t *dev_list =
      (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
  if (!dev_list) {
    ESP_LOGI(GATTS_TABLE_TAG, "malloc failed, return\n");
    return;
  }
  esp_ble_get_bond_device_list(&dev_num, dev_list);
  ESP_LOGI(GATTS_TABLE_TAG, "Bonded devices number %d", dev_num);
  for (int i = 0; i < dev_num; i++) {
    ESP_LOGI(GATTS_TABLE_TAG, "[%u] addr_type %u, addr " ESP_BD_ADDR_STR "", i,
             dev_list[i].bd_addr_type, ESP_BD_ADDR_HEX(dev_list[i].bd_addr));
  }

  free(dev_list);
}

static void remove_all_bonded_devices(void) {
  int dev_num = esp_ble_get_bond_device_num();
  if (dev_num == 0) {
    ESP_LOGI(GATTS_TABLE_TAG, "Bonded devices number zero\n");
    return;
  }

  esp_ble_bond_dev_t *dev_list =
      (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
  if (!dev_list) {
    ESP_LOGI(GATTS_TABLE_TAG, "malloc failed, return\n");
    return;
  }
  esp_ble_get_bond_device_list(&dev_num, dev_list);
  for (int i = 0; i < dev_num; i++) {
    esp_ble_remove_bond_device(dev_list[i].bd_addr);
  }

  free(dev_list);
}

uint8_t isBTEnabled() { return isEnabled ? 1 : 0; }

void disable_bt() {
  isEnabled = false;
  remove_all_bonded_devices();
  ESP_ERROR_CHECK(esp_bluedroid_disable());
  ESP_ERROR_CHECK(esp_bluedroid_deinit());
  ESP_ERROR_CHECK(esp_bt_controller_disable());
  ESP_ERROR_CHECK(esp_bt_controller_deinit());
  ESP_LOGI("DEBUG BT", "Bluetooth disabled and deinitialized.");
}

static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param) {
  switch (event) {
  case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
    adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
    if (adv_config_done == 0) {
      esp_ble_gap_start_advertising(&wifi_bt_adv_params);
    }
    break;
  case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
    adv_config_done &= (~ADV_CONFIG_FLAG);
    if (adv_config_done == 0) {
      esp_ble_gap_start_advertising(&wifi_bt_adv_params);
    }
    break;
  case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
    if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
      ESP_LOGE(GATTS_TABLE_TAG, "Advertising start failed, status %x",
               param->adv_start_cmpl.status);
      break;
    }
    ESP_LOGI(GATTS_TABLE_TAG, "Advertising start successfully");
    break;
  case ESP_GAP_BLE_OOB_REQ_EVT: {
    ESP_LOGI(GATTS_TABLE_TAG, "OOB request");
    uint8_t tk[16] = {1};
    esp_ble_oob_req_reply(param->ble_security.ble_req.bd_addr, tk, sizeof(tk));
    break;
  }
  case ESP_GAP_BLE_NC_REQ_EVT:
    esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
    ESP_LOGI(GATTS_TABLE_TAG, "Numeric Comparison request, passkey %" PRIu32,
             param->ble_security.key_notif.passkey);
    break;
  case ESP_GAP_BLE_SEC_REQ_EVT:
    esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
    break;
  case ESP_GAP_BLE_AUTH_CMPL_EVT: {
    esp_bd_addr_t bd_addr;
    memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr,
           sizeof(esp_bd_addr_t));
    ESP_LOGI(GATTS_TABLE_TAG,
             "Authentication complete, addr_type %u, addr " ESP_BD_ADDR_STR "",
             param->ble_security.auth_cmpl.addr_type, ESP_BD_ADDR_HEX(bd_addr));
    if (!param->ble_security.auth_cmpl.success) {
      ESP_LOGI(GATTS_TABLE_TAG, "Pairing failed, reason 0x%x",
               param->ble_security.auth_cmpl.fail_reason);
    }
    show_bonded_devices();
    break;
  }
  case ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT: {
    ESP_LOGD(GATTS_TABLE_TAG,
             "Bond device remove, status %d, device " ESP_BD_ADDR_STR "",
             param->remove_bond_dev_cmpl.status,
             ESP_BD_ADDR_HEX(param->remove_bond_dev_cmpl.bd_addr));
    break;
  }
  case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
    if (param->local_privacy_cmpl.status != ESP_BT_STATUS_SUCCESS) {
      ESP_LOGE(GATTS_TABLE_TAG, "Local privacy config failed, status %x",
               param->local_privacy_cmpl.status);
      break;
    }
    ESP_LOGI(GATTS_TABLE_TAG, "Local privacy config successfully");

    esp_err_t ret = esp_ble_gap_config_adv_data(&wifi_bt_adv_config);
    if (ret) {
      ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
    } else {
      adv_config_done |= ADV_CONFIG_FLAG;
    }

    ret = esp_ble_gap_config_adv_data(&wifi_bt_scan_rsp_config);
    if (ret) {
      ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
    } else {
      adv_config_done |= SCAN_RSP_CONFIG_FLAG;
    }

    break;
  default:
    break;
  }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t *param) {
  ESP_LOGV(GATTS_TABLE_TAG, "event = %x", event);
  switch (event) {
  case ESP_GATTS_REG_EVT:
    ESP_LOGI(GATTS_TABLE_TAG,
             "GATT server register, status %d, app_id %d, gatts_if %d",
             param->reg.status, param->reg.app_id, gatts_if);
    esp_ble_gap_set_device_name(bt_device_name);
    esp_ble_gap_config_local_privacy(true);
    esp_ble_gatts_create_attr_tab(heart_rate_gatt_db, gatts_if, WIFI_BT_NB,
                                  WIFI_BT_SVC_INST_ID);
    break;
  case ESP_GATTS_READ_EVT:
    break;
  case ESP_GATTS_WRITE_EVT:
    ESP_LOGI(GATTS_TABLE_TAG, "Characteristic write, value ");
    ESP_LOG_BUFFER_HEX(GATTS_TABLE_TAG, param->write.value, param->write.len);

    if (param->write.handle == wifi_bt_handle_table[WIFI_BT_UUID_MEAS_VAL]) {
      sprintf(ssid, "%.*s", param->write.len, param->write.value);
    } else if (param->write.handle ==
               wifi_bt_handle_table[WIFI_BT_PASS_MEAS_VAL]) {
      sprintf(password, "%.*s", param->write.len, param->write.value);
    }
    break;
  case ESP_GATTS_CONNECT_EVT:
    ESP_LOGI(
        GATTS_TABLE_TAG, "Connected, conn_id %u, remote " ESP_BD_ADDR_STR "",
        param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));
    /* start security connect with peer device when receive the connect event
     * sent by the master */
    esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);
    break;
  case ESP_GATTS_DISCONNECT_EVT:
    ESP_LOGI(GATTS_TABLE_TAG,
             "Disconnected, remote " ESP_BD_ADDR_STR ", reason 0x%x",
             ESP_BD_ADDR_HEX(param->disconnect.remote_bda),
             param->disconnect.reason);
    /* start advertising again when missing the connect */
    esp_ble_gap_start_advertising(&wifi_bt_adv_params);
    break;
  case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
    if (param->create.status == ESP_GATT_OK) {
      if (param->add_attr_tab.num_handle == WIFI_BT_NB) {
        ESP_LOGI(GATTS_TABLE_TAG,
                 "Attribute table create successfully, num_handle %x",
                 param->add_attr_tab.num_handle);
        memcpy(wifi_bt_handle_table, param->add_attr_tab.handles,
               sizeof(wifi_bt_handle_table));
        esp_ble_gatts_start_service(wifi_bt_handle_table[WIFI_BT_SVC]);
      }
    }
    break;
  }

  default:
    break;
  }
}

static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param) {
  if (event == ESP_GATTS_REG_EVT) {
    if (param->reg.status == ESP_GATT_OK) {
      wifi_bt_profile_tab[WIFI_BT_PROFILE_APP_IDX].gatts_if = gatts_if;
    } else {
      ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d",
               param->reg.app_id, param->reg.status);
      return;
    }
  }

  do {
    int idx;
    for (idx = 0; idx < WIFI_BT_PROFILE_NUM; idx++) {
      if (gatts_if == ESP_GATT_IF_NONE ||
          gatts_if == wifi_bt_profile_tab[idx].gatts_if) {
        if (wifi_bt_profile_tab[idx].gatts_cb) {
          wifi_bt_profile_tab[idx].gatts_cb(event, gatts_if, param);
        }
      }
    }
  } while (0);
}

void start_bt() {
  esp_err_t ret;
  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

  esp_bt_controller_status_t bt_status;
  bt_status = esp_bt_controller_get_status();

  if (bt_status == ESP_BT_CONTROLLER_STATUS_IDLE) {
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
      ESP_LOGE(GATTS_TABLE_TAG, "%s init controller failed: %s", __func__,
               esp_err_to_name(ret));
      return;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
      ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__,
               esp_err_to_name(ret));
      return;
    }
  } else {
    ESP_LOGI(GATTS_TABLE_TAG, "Bluetooth controller is already initialized, "
                              "skipping initialization.");
  }

  esp_bluedroid_status_t bt_bluedroid_status = esp_bluedroid_get_status();
  if (bt_bluedroid_status != ESP_BLUEDROID_STATUS_ENABLED) {
    ret = esp_bluedroid_init();
    if (ret) {
      ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__,
               esp_err_to_name(ret));
      return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
      ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__,
               esp_err_to_name(ret));
      return;
    }
  } else {
    ESP_LOGI(GATTS_TABLE_TAG,
             "Bluedroid is already initialized, skipping initialization.");
  }

  ESP_LOGI(GATTS_TABLE_TAG, "%s init bluetooth", __func__);

  ret = esp_ble_gatts_register_callback(gatts_event_handler);
  if (ret) {
    ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
    return;
  }
  ret = esp_ble_gap_register_callback(gap_event_handler);
  if (ret) {
    ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
    return;
  }
  ret = esp_ble_gatts_app_register(ESP_WIFI_BT_APP_ID);
  if (ret) {
    ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
    return;
  }

  isEnabled = true;

  esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_BOND;
  esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
  uint8_t key_size = 16;
  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
  uint8_t oob_support = ESP_BLE_OOB_DISABLE;
  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req,
                                 sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap,
                                 sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size,
                                 sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH,
                                 &auth_option, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support,
                                 sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key,
                                 sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key,
                                 sizeof(uint8_t));
}
