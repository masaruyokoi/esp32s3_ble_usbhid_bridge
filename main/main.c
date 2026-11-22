#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "nvs_flash.h"

/* TinyUSB */
#include "tinyusb.h"
#include "class/hid/hid_device.h"

/* NimBLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "store/config/ble_store_config.h" // NVSへのペアリング鍵保存用

static const char *TAG = "BLE_USB_HID";

/* =========================================================================
 * 1. USB HID (TinyUSB) 設定とディスクリプタ
 * ========================================================================= */
#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)

static const uint8_t s_hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD))
};

static const char *s_hid_string_descriptor[5] = {
    (char[]){0x09, 0x04},       // 0: English (0x0409)
    "CustomHID",                // 1: Manufacturer
    "BLE-USB HID Bridge",       // 2: Product
    "20260001",                 // 3: Serials
    "HID Keyboard Interface",   // 4: HID Interface
};

static const uint8_t s_hid_configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(s_hid_report_descriptor), 0x81, 16, 10),
};

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
    (void)instance;
    return s_hid_report_descriptor;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen) {
    (void)instance; (void)report_id; (void)report_type; (void)buffer; (void)reqlen;
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize) {
    (void)instance; (void)report_id; (void)report_type; (void)buffer; (void)bufsize;
}

/* =========================================================================
 * 2. ASCII -> US配列 HIDキーコード変換テーブル & タイピング処理
 * ========================================================================= */
typedef struct {
    uint8_t modifier;
    uint8_t keycode;
} hid_key_map_t;

/* ASCII 0x20 (' ') から 0x7E ('~') までのマッピング (US配列) */
static const hid_key_map_t s_ascii_to_hid[95] = {
    {0, HID_KEY_SPACE},                        // 0x20 ' '
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_1},  // 0x21 '!'
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_APOSTROPHE}, // 0x22 '"'
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_3},  // 0x23 '#'
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_4},  // 0x24 '$'
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_5},  // 0x25 '%'
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_7},  // 0x26 '&'
    {0, HID_KEY_APOSTROPHE},                   // 0x27 '\''
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_9},  // 0x28 '('
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_0},  // 0x29 ')'
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_8},  // 0x2A '*'
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_EQUAL}, // 0x2B '+'
    {0, HID_KEY_COMMA},                        // 0x2C ','
    {0, HID_KEY_MINUS},                        // 0x2D '-'
    {0, HID_KEY_PERIOD},                       // 0x2E '.'
    {0, HID_KEY_SLASH},                        // 0x2F '/'
    {0, HID_KEY_0}, {0, HID_KEY_1}, {0, HID_KEY_2}, {0, HID_KEY_3}, {0, HID_KEY_4},
    {0, HID_KEY_5}, {0, HID_KEY_6}, {0, HID_KEY_7}, {0, HID_KEY_8}, {0, HID_KEY_9},
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_SEMICOLON}, // 0x3A ':'
    {0, HID_KEY_SEMICOLON},                    // 0x3B ';'
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_COMMA},     // 0x3C '<'
    {0, HID_KEY_EQUAL},                        // 0x3D '='
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_PERIOD},    // 0x3E '>'
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_SLASH},     // 0x3F '?'
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_2},  // 0x40 '@'
    /* 'A' - 'Z' (0x41 - 0x5A) */
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_A}, {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_B},
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_C}, {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_D},
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_E}, {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_F},
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_G}, {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_H},
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_I}, {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_J},
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_K}, {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_L},
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_M}, {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_N},
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_O}, {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_P},
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_Q}, {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_R},
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_S}, {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_T},
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_U}, {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_V},
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_W}, {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_X},
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_Y}, {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_Z},
    {0, HID_KEY_BRACKET_LEFT},                 // 0x5B '['
    {0, HID_KEY_BACKSLASH},                    // 0x5C '\\'
    {0, HID_KEY_BRACKET_RIGHT},                // 0x5D ']'
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_6},  // 0x5E '^'
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_MINUS}, // 0x5F '_'
    {0, HID_KEY_GRAVE},                        // 0x60 '`'
    /* 'a' - 'z' (0x61 - 0x7A) */
    {0, HID_KEY_A}, {0, HID_KEY_B}, {0, HID_KEY_C}, {0, HID_KEY_D}, {0, HID_KEY_E},
    {0, HID_KEY_F}, {0, HID_KEY_G}, {0, HID_KEY_H}, {0, HID_KEY_I}, {0, HID_KEY_J},
    {0, HID_KEY_K}, {0, HID_KEY_L}, {0, HID_KEY_M}, {0, HID_KEY_N}, {0, HID_KEY_O},
    {0, HID_KEY_P}, {0, HID_KEY_Q}, {0, HID_KEY_R}, {0, HID_KEY_S}, {0, HID_KEY_T},
    {0, HID_KEY_U}, {0, HID_KEY_V}, {0, HID_KEY_W}, {0, HID_KEY_X}, {0, HID_KEY_Y},
    {0, HID_KEY_Z},
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_BRACKET_LEFT},  // 0x7B '{'
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_BACKSLASH},     // 0x7C '|'
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_BRACKET_RIGHT}, // 0x7D '}'
    {KEYBOARD_MODIFIER_LEFTSHIFT, HID_KEY_GRAVE},         // 0x7E '~'
};

#define MAX_PAYLOAD_LEN 256
typedef struct {
    size_t len;
    char data[MAX_PAYLOAD_LEN];
} hid_msg_t;

static QueueHandle_t s_hid_queue = NULL;

static void send_hid_key(uint8_t modifier, uint8_t keycode) {
    uint8_t keycodes[6] = {keycode, 0, 0, 0, 0, 0};

    while (!tud_hid_ready()) {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, modifier, keycodes);
    vTaskDelay(pdMS_TO_TICKS(15));

    while (!tud_hid_ready()) {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, NULL);
    vTaskDelay(pdMS_TO_TICKS(15));
}

static void hid_typing_task(void *arg) {
    (void)arg;
    hid_msg_t msg;

    while (1) {
        if (xQueueReceive(s_hid_queue, &msg, portMAX_DELAY) == pdTRUE) {
            if (!tud_mounted()) {
                ESP_LOGW(TAG, "USB HID not mounted. Dropping %d bytes.", (int)msg.len);
                continue;
            }

            ESP_LOGI(TAG, "Typing %d characters via USB HID...", (int)msg.len);
            for (size_t i = 0; i < msg.len; i++) {
                uint8_t ch = (uint8_t)msg.data[i];
                if (ch >= 0x20 && ch <= 0x7E) {
                    hid_key_map_t map = s_ascii_to_hid[ch - 0x20];
                    send_hid_key(map.modifier, map.keycode);
                } else if (ch == '\n' || ch == '\r') {
                    send_hid_key(0, HID_KEY_ENTER);
                } else if (ch == '\t') {
                    send_hid_key(0, HID_KEY_TAB);
                } else {
                    ESP_LOGW(TAG, "Unsupported char: 0x%02X", ch);
                }
            }
            ESP_LOGI(TAG, "Typing complete.");
        }
    }
}

/* =========================================================================
 * 3. BLE (NimBLE) GATT サービス & アドバタイズ設定
 * ========================================================================= */
/* カスタムサービス UUID: 12345678-1234-5678-1234-56789abcdef0 */
static const ble_uuid128_t gatt_svr_svc_uuid =
    BLE_UUID128_INIT(0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
                     0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

/* カスタムキャラクタリスティック UUID: 12345678-1234-5678-1234-56789abcdef1 */
static const ble_uuid128_t gatt_svr_chr_uuid =
    BLE_UUID128_INIT(0xf1, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
                     0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static uint8_t s_own_addr_type;
static void ble_app_advertise(void);

static int gatt_chr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg) {
    (void)conn_handle; (void)attr_handle; (void)arg;

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t om_len = OS_MBUF_PKTLEN(ctxt->om);
        if (om_len == 0 || om_len > MAX_PAYLOAD_LEN) {
            ESP_LOGW(TAG, "Invalid write length: %d", om_len);
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }

        hid_msg_t msg = {0};
        uint16_t copied = 0;
        int rc = ble_hs_mbuf_to_flat(ctxt->om, msg.data, sizeof(msg.data), &copied);
        if (rc != 0) {
            return BLE_ATT_ERR_UNLIKELY;
        }
        msg.len = copied;

        if (xQueueSend(s_hid_queue, &msg, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGE(TAG, "HID queue full!");
            return BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

/* -Werror=missing-field-initializers 対応のため全フィールドを明示初期化 */
static const struct ble_gatt_svc_def s_gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uuid.u,
        .includes = NULL,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &gatt_svr_chr_uuid.u,
                .access_cb = gatt_chr_access_cb,
                .arg = NULL,
                .descriptors = NULL,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_WRITE_ENC,
                .min_key_size = 0,
                .val_handle = NULL,
                .cpfd = NULL,
            },
            {
                .uuid = NULL,
                .access_cb = NULL,
                .arg = NULL,
                .descriptors = NULL,
                .flags = 0,
                .min_key_size = 0,
                .val_handle = NULL,
                .cpfd = NULL,
            }
        },
    },
    {
        .type = 0,
        .uuid = NULL,
        .includes = NULL,
        .characteristics = NULL,
    }
};

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg) {
    (void)arg;
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ESP_LOGI(TAG, "BLE Connected (handle=%d)", event->connect.conn_handle);
            } else {
                ESP_LOGW(TAG, "BLE Connect failed; status=%d", event->connect.status);
                ble_app_advertise();
            }
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "BLE Disconnected; reason=%d", event->disconnect.reason);
            ble_app_advertise();
            break;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ble_app_advertise();
            break;

	case BLE_GAP_EVENT_ENC_CHANGE:
            ESP_LOGI(TAG, "Encryption change event; status=%d", event->enc_change.status);
            rc = ble_gap_conn_find(event->enc_change.conn_handle, &desc);
            if (rc == 0) {
                ESP_LOGI(TAG, "Encrypted=%d, Bonded=%d, KeySize=%d",
                         desc.sec_state.encrypted,
                         desc.sec_state.bonded,
                         desc.sec_state.key_size);
            }
            break;

        // ★追加: PC側でペアリング解除後に再接続された場合、古いボンディング鍵を削除して再ペアリングを許可
        case BLE_GAP_EVENT_REPEAT_PAIRING:
            rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
            if (rc == 0) {
                ble_store_util_delete_peer(&desc.peer_id_addr);
            }
            return BLE_GAP_REPEAT_PAIRING_RETRY;

        default:
            break;
    }
    return 0;
}

static void ble_app_advertise(void) {
    struct ble_gap_adv_params adv_params = {0};
    struct ble_hs_adv_fields fields = {0};

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    const char *name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting adv fields; rc=%d", rc);
        return;
    }

    /* スキャンレスポンスに128bitサービスUUIDを載せる (アドバタイズパケット31バイト制限回避) */
    struct ble_hs_adv_fields rsp_fields = {0};
    rsp_fields.uuids128 = (ble_uuid128_t *)&gatt_svr_svc_uuid;
    rsp_fields.num_uuids128 = 1;
    rsp_fields.uuids128_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error setting scan rsp fields; rc=%d", rc);
        return;
    }

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error starting adv; rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "BLE Advertising started as '%s'", name);
    }
}

static void ble_app_on_sync(void) {
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "Ensure addr failed; rc=%d", rc);
        return;
    }
    rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Infer addr type failed; rc=%d", rc);
        return;
    }
    ble_app_advertise();
}

static void nimble_host_task(void *param) {
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// declare the internal function
void ble_store_config_init(void);

/* main */

void app_main(void) {
    /* NVS 初期化 (BLE PHY キャリブレーションデータ保存に必須) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* HID送信キュー・タスク作成 */
    s_hid_queue = xQueueCreate(8, sizeof(hid_msg_t));
    configASSERT(s_hid_queue != NULL);
    xTaskCreate(hid_typing_task, "hid_typing_task", 4096, NULL, 5, NULL);

    /* TinyUSB 初期化 */
    ESP_LOGI(TAG, "Initializing TinyUSB HID...");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = s_hid_string_descriptor,
        .string_descriptor_count = sizeof(s_hid_string_descriptor) / sizeof(s_hid_string_descriptor[0]),
        .external_phy = false,
        .configuration_descriptor = s_hid_configuration_descriptor,
        .self_powered = false,
        .vbus_monitor_io = -1,
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    /* NimBLE 初期化 */
    ESP_LOGI(TAG, "Initializing NimBLE...");
    ESP_ERROR_CHECK(nimble_port_init());

    ble_hs_cfg.sync_cb = ble_app_on_sync;

    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;   // ディスプレイ・入力キーなし (Just Works)
    ble_hs_cfg.sm_bonding = 1;                    // ペアリング鍵の保存を有効化
    ble_hs_cfg.sm_mitm = 0;                       // NoInputNoOutput のため MITM 保護はオフ
    ble_hs_cfg.sm_sc = 1;                         // LE Secure Connections (ECDH暗号化) を有効化
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    ble_store_config_init();

    ble_svc_gap_init();
    ble_svc_gatt_init();

    ESP_ERROR_CHECK(ble_svc_gap_device_name_set("ESP32-HID-Bridge"));
    ESP_ERROR_CHECK(ble_gatts_count_cfg(s_gatt_svr_svcs));
    ESP_ERROR_CHECK(ble_gatts_add_svcs(s_gatt_svr_svcs));

    nimble_port_freertos_init(nimble_host_task);
}
