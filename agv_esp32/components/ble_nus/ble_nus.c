#include "ble_nus.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_att.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "BLE_NUS";

/* Nordic UART Service 128-bit UUIDs (Little-Endian)
 * Base UUID: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 */
static const ble_uuid128_t gatt_svr_svc_nus_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);

/* RX Characteristic (Write): 6E400002-B5A3-F393-E0A9-E50E24DCCA9E */
static const ble_uuid128_t gatt_svr_chr_nus_rx_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);

/* TX Characteristic (Notify): 6E400003-B5A3-F393-E0A9-E50E24DCCA9E */
static const ble_uuid128_t gatt_svr_chr_nus_tx_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

static ble_nus_config_t s_config;
static bool s_is_initialized = false;
static uint8_t s_own_addr_type;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_nus_tx_val_handle;
static bool s_notify_enabled = false;
static uint16_t s_current_mtu = BLE_ATT_MTU_DFLT; // Default 23 bytes

static int ble_gap_event(struct ble_gap_event *event, void *arg);
static void ble_nus_advertise(void);

static void log_connection_params(uint16_t conn_handle, const char *stage) {
    struct ble_gap_conn_desc desc;
    int rc = ble_gap_conn_find(conn_handle, &desc);
    if (rc == 0) {
        float itvl_ms = desc.conn_itvl * 1.25f;
        uint32_t timeout_ms = desc.supervision_timeout * 10;
        ESP_LOGI(TAG, "[BLE %s] Handle: %d | Peer: %02X:%02X:%02X:%02X:%02X:%02X (%s) | Itvl: %.2f ms | Latency: %d | Timeout: %u ms",
                 stage, conn_handle,
                 desc.peer_id_addr.val[5], desc.peer_id_addr.val[4],
                 desc.peer_id_addr.val[3], desc.peer_id_addr.val[2],
                 desc.peer_id_addr.val[1], desc.peer_id_addr.val[0],
                 (desc.peer_id_addr.type == BLE_ADDR_PUBLIC) ? "PUBLIC" : "RANDOM",
                 itvl_ms, desc.conn_latency, (unsigned)timeout_ms);
    }
}

static int gatt_svr_chr_access_nus(uint16_t conn_handle, uint16_t attr_handle,
                                   struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t len = 0;
        uint8_t rx_buf[512];

        int rc = ble_hs_mbuf_to_flat(ctxt->om, rx_buf, sizeof(rx_buf) - 1, &len);
        if (rc != 0) {
            ESP_LOGE(TAG, "Failed to read incoming mbuf data: %d", rc);
            return BLE_ATT_ERR_UNLIKELY;
        }

        rx_buf[len] = '\0';

        // Trim trailing CRLF for clean log display
        int print_len = (int)len;
        while (print_len > 0 && (rx_buf[print_len - 1] == '\n' || rx_buf[print_len - 1] == '\r')) {
            print_len--;
        }
        if (print_len > 0) {
            ESP_LOGI(TAG, "[BLE RX] (%u bytes) %.*s", (unsigned)len, print_len, rx_buf);
        }

        if (s_config.rx_callback != NULL) {
            s_config.rx_callback(rx_buf, len, s_config.user_data);
        }

        return 0;
    }

    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_nus_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                /* RX Characteristic - Central writes to ESP32 */
                .uuid = &gatt_svr_chr_nus_rx_uuid.u,
                .access_cb = gatt_svr_chr_access_nus,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                /* TX Characteristic - ESP32 notifies Central */
                .uuid = &gatt_svr_chr_nus_tx_uuid.u,
                .access_cb = gatt_svr_chr_access_nus,
                .val_handle = &s_nus_tx_val_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            {
                0, /* No more characteristics */
            }
        },
    },
    {
        0, /* No more services */
    },
};

static void ble_nus_advertise(void) {
    // 1. Primary Advertising Packet: Flags + 128-bit NUS Service UUID (3 + 18 = 21 bytes <= 31 bytes)
    struct ble_hs_adv_fields adv_fields;
    memset(&adv_fields, 0, sizeof(adv_fields));

    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv_fields.uuids128 = (ble_uuid128_t *)&gatt_svr_svc_nus_uuid;
    adv_fields.num_uuids128 = 1;
    adv_fields.uuids128_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set adv fields: rc=%d", rc);
        return;
    }

    // 2. Scan Response Packet: Complete Device Name (11 bytes <= 31 bytes)
    struct ble_hs_adv_fields rsp_fields;
    memset(&rsp_fields, 0, sizeof(rsp_fields));

    const char *name = (s_config.device_name != NULL) ? s_config.device_name : "AGV_ESP32";
    rsp_fields.name = (uint8_t *)name;
    rsp_fields.name_len = strlen(name);
    rsp_fields.name_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set scan response fields: rc=%d", rc);
        return;
    }

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "Failed to start advertising: rc=%d", rc);
        return;
    }

    ESP_LOGI(TAG, "BLE advertising started (Device Name: \"%s\")", name);
}

static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                s_conn_handle = event->connect.conn_handle;
                s_current_mtu = BLE_ATT_MTU_DFLT;
                ESP_LOGI(TAG, "BLE Connected (conn_handle=%d)", s_conn_handle);
                log_connection_params(s_conn_handle, "CONNECTED");
            } else {
                ESP_LOGW(TAG, "BLE Connection failed: status=%d", event->connect.status);
                ble_nus_advertise();
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "BLE Disconnected: reason=%d", event->disconnect.reason);
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            s_notify_enabled = false;
            s_current_mtu = BLE_ATT_MTU_DFLT;
            ble_nus_advertise();
            return 0;

        case BLE_GAP_EVENT_CONN_UPDATE:
            if (event->conn_update.status == 0) {
                log_connection_params(event->conn_update.conn_handle, "PARAMS UPDATED");
            } else {
                ESP_LOGW(TAG, "BLE Connection update failed: status=%d", event->conn_update.status);
            }
            return 0;

        case BLE_GAP_EVENT_SUBSCRIBE:
            if (event->subscribe.attr_handle == s_nus_tx_val_handle) {
                s_notify_enabled = event->subscribe.cur_notify;
                ESP_LOGI(TAG, "[BLE NUS] Notifications %s (conn_handle=%d)",
                         s_notify_enabled ? "ENABLED" : "DISABLED", event->subscribe.conn_handle);
            }
            return 0;

        case BLE_GAP_EVENT_MTU:
            s_current_mtu = event->mtu.value;
            ESP_LOGI(TAG, "[BLE MTU] Negotiated ATT MTU: %d bytes | Max NUS Payload: %d bytes (channel: %d)",
                     event->mtu.value, event->mtu.value - 3, event->mtu.channel_id);
            return 0;

        case BLE_GAP_EVENT_ADV_COMPLETE:
            ble_nus_advertise();
            return 0;

        default:
            return 0;
    }
}

static void ble_nus_on_sync(void) {
    int rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to determine BLE address type: rc=%d", rc);
        return;
    }

    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(s_own_addr_type, addr_val, NULL);
    if (rc == 0) {
        ESP_LOGI(TAG, "BLE MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
                 addr_val[5], addr_val[4], addr_val[3], addr_val[2], addr_val[1], addr_val[0]);
    }

    ble_nus_advertise();
}

static void ble_nus_on_reset(int reason) {
    ESP_LOGW(TAG, "NimBLE host reset: reason=%d", reason);
}

static void ble_host_task(void *param) {
    ESP_LOGI(TAG, "BLE NimBLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t ble_nus_init(const ble_nus_config_t *config) {
    if (s_is_initialized) {
        ESP_LOGW(TAG, "BLE NUS service already initialized");
        return ESP_OK;
    }

    if (config != NULL) {
        s_config = *config;
    } else {
        const ble_nus_config_t default_cfg = BLE_NUS_CONFIG_DEFAULT();
        s_config = default_cfg;
    }

    if (s_config.device_name == NULL) {
        s_config.device_name = "AGV_ESP32";
    }

    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize nimble port: %s", esp_err_to_name(ret));
        return ret;
    }

    int mtu_rc = ble_att_set_preferred_mtu(256);
    if (mtu_rc != 0) {
        ESP_LOGW(TAG, "Failed to set preferred MTU to 256: rc=%d", mtu_rc);
    }

    ble_svc_gap_init();
    ble_svc_gatt_init();

    ret = ble_svc_gap_device_name_set(s_config.device_name);
    if (ret != 0) {
        ESP_LOGE(TAG, "Failed to set device name: %d", ret);
        return ESP_FAIL;
    }

    int rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to count GATT svcs: rc=%d", rc);
        return ESP_FAIL;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to add GATT svcs: rc=%d", rc);
        return ESP_FAIL;
    }

    ble_hs_cfg.reset_cb = ble_nus_on_reset;
    ble_hs_cfg.sync_cb = ble_nus_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    nimble_port_freertos_init(ble_host_task);

    s_is_initialized = true;
    ESP_LOGI(TAG, "BLE Nordic UART Service (NUS) initialized");
    return ESP_OK;
}

int ble_nus_send(const uint8_t *data, size_t len) {
    if (!s_is_initialized || data == NULL || len == 0) {
        return 0;
    }

    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || !s_notify_enabled) {
        return 0;
    }

    // Print clean log
    int print_len = (int)len;
    while (print_len > 0 && (data[print_len - 1] == '\n' || data[print_len - 1] == '\r')) {
        print_len--;
    }
    if (print_len > 0) {
        ESP_LOGD(TAG, "[BLE TX] (%u bytes) %.*s", (unsigned)len, print_len, data);
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (om == NULL) {
        ESP_LOGE(TAG, "Failed to allocate mbuf for notify");
        return -1;
    }

    int rc = ble_gatts_notify_custom(s_conn_handle, s_nus_tx_val_handle, om);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to send BLE notification: rc=%d", rc);
        return -1;
    }

    return (int)len;
}

int ble_nus_send_str(const char *str) {
    if (str == NULL) {
        return 0;
    }
    return ble_nus_send((const uint8_t *)str, strlen(str));
}

bool ble_nus_is_connected(void) {
    return (s_conn_handle != BLE_HS_CONN_HANDLE_NONE && s_notify_enabled);
}
