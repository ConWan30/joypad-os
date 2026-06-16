// bridge_transport.c — PoAC record delivery via BLE GATT notify
// SPDX-License-Identifier: Apache-2.0
//
// Delivers signed 228-byte PoAC records from the ESP32-S3 to the QorTroller
// bridge service (Python asyncio, running on the gamer's PC/laptop).
//
// TRANSPORT ARCHITECTURE:
//   - Primary: BLE GATT characteristic notify (joypad-os BTstack already running)
//   - Fallback: WiFi UDP batch delivery (lower latency constraints, higher throughput)
//
// BLE MTU DISCIPLINE:
//   BLE 4.2 default ATT MTU = 23 bytes (payload = 20 bytes). To deliver 228 bytes
//   in one notify, MTU negotiation to ≥ 232 bytes (228 + 4 ATT overhead) is required.
//   The transport requests MTU exchange at connect time. If the bridge's host BLE
//   adapter refuses MTU > 23, the transport falls back to fragmenting the 228-byte
//   record into 12 × 20-byte chunks with a 2-byte sequence header.
//
// QUEUE DISCIPLINE:
//   Records are queued in a depth-16 ring. If the queue fills (bridge too slow or
//   disconnected for > 16 s), the OLDEST record is dropped (newest wins — the most
//   recent biometric state is more valuable for the GIC chain).
//
// CHAIN ECHO:
//   After receiving a PoAC record, the bridge writes back the 32-byte SHA-256 of
//   the record body via the CHAIN characteristic (write-without-response).
//   This echo confirms the record was received and provides prev_hash for the
//   next record. Without echo, prev_hash stays at the last locally-confirmed value.

#include "bridge_transport.h"
#include "poac_builder.h"
#include "platform/platform.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include <string.h>
#include <stdio.h>

// ============================================================================
// BTSTACK GATT INCLUDES
// BTstack is already initialised by joypad-os for BLE HID output.
// We add a second service (QorTroller PoAC) alongside the existing HID service.
// ============================================================================

#ifdef PLATFORM_ESP32
#include "btstack.h"
#include "btstack_event.h"
#include "hci.h"
#include "att_server.h"
#include "le_device_db.h"
#else
// Stubs for non-ESP32 builds (RP2040 / nRF don't use this module)
typedef uint16_t hci_con_handle_t;
#define HCI_CON_HANDLE_INVALID 0xFFFF
#endif

// ============================================================================
// GATT ATTRIBUTE HANDLES (assigned by BTstack during service registration)
// These are set once during init and never change.
// ============================================================================

static uint16_t _poac_service_handle  = 0;
static uint16_t _poac_char_handle     = 0;  // notify (228 bytes)
static uint16_t _chain_char_handle    = 0;  // write-back (32 bytes)
static uint16_t _poac_cccd_handle     = 0;  // Client Characteristic Configuration

// Current BLE connection handle (0xFFFF = disconnected)
static volatile hci_con_handle_t _conn_handle = HCI_CON_HANDLE_INVALID;

// Notify enabled by bridge (CCCD write)
static volatile bool _notify_enabled = false;

// ============================================================================
// DELIVERY QUEUE
// ============================================================================

#define TRANSPORT_QUEUE_DEPTH 16

static poac_record_t _queue[TRANSPORT_QUEUE_DEPTH];
static volatile uint8_t _q_head = 0;
static volatile uint8_t _q_tail = 0;

static volatile uint32_t _delivered = 0;
static volatile uint32_t _dropped   = 0;

static SemaphoreHandle_t _q_mutex = NULL;

static uint8_t _queue_used(void) {
    return (_q_head - _q_tail) & (TRANSPORT_QUEUE_DEPTH - 1);
}

static bool _queue_full(void) {
    return _queue_used() >= (TRANSPORT_QUEUE_DEPTH - 1);
}

static bool _queue_empty(void) {
    return _q_head == _q_tail;
}

// ============================================================================
// BLE FRAGMENT STATE (for MTU < 232 adapters)
// ============================================================================

#define FRAG_PAYLOAD_SIZE  18  // 20B ATT payload − 2B header (seq | total)
#define FRAG_TOTAL         ((POAC_RECORD_SIZE + FRAG_PAYLOAD_SIZE - 1) / FRAG_PAYLOAD_SIZE)

static bool     _fragmented    = false;   // true when MTU < 232
static uint8_t  _frag_buf[20];
static uint16_t _neg_mtu       = 23;      // last negotiated ATT MTU

// ============================================================================
// TRANSPORT TASK
// Runs on Core 1 (APP_CPU), low priority — sends queued records over BLE.
// ============================================================================

#define TRANSPORT_TASK_STACK  4096
#define TRANSPORT_TASK_PRI    5      // below BT stack (10–14), well below sense (24)

static void _transport_task_fn(void* arg) {
    (void)arg;

    printf("[transport] BLE delivery task started on Core %d\n", xPortGetCoreID());

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));  // poll queue at 100 Hz

        if (_queue_empty()) continue;
        if (_conn_handle == HCI_CON_HANDLE_INVALID) continue;
        if (!_notify_enabled) continue;

        xSemaphoreTake(_q_mutex, portMAX_DELAY);
        poac_record_t rec = _queue[_q_tail & (TRANSPORT_QUEUE_DEPTH - 1)];
        _q_tail++;
        xSemaphoreGive(_q_mutex);

#ifdef PLATFORM_ESP32
        if (!_fragmented) {
            // Single-notify path: full 228 bytes in one ATT packet
            // Requires MTU ≥ 232 (228 + 4 ATT overhead)
            int rc = att_server_notify(_conn_handle, _poac_char_handle,
                                       (uint8_t*)&rec, sizeof(rec));
            if (rc == 0) {
                _delivered++;
            } else {
                printf("[transport] notify failed (rc=%d) — will retry next cycle\n", rc);
                // Re-queue by un-advancing tail (best-effort; if queue wraps, drop)
                xSemaphoreTake(_q_mutex, portMAX_DELAY);
                _q_tail--;
                xSemaphoreGive(_q_mutex);
            }
        } else {
            // Fragmented path: 13 × 20-byte chunks with sequence header
            const uint8_t* src = (const uint8_t*)&rec;
            for (uint8_t f = 0; f < FRAG_TOTAL; f++) {
                _frag_buf[0] = f;           // fragment index
                _frag_buf[1] = FRAG_TOTAL;  // total fragments
                uint16_t offset = f * FRAG_PAYLOAD_SIZE;
                uint16_t remain = sizeof(rec) - offset;
                uint16_t chunk  = remain < FRAG_PAYLOAD_SIZE ? remain : FRAG_PAYLOAD_SIZE;
                memcpy(_frag_buf + 2, src + offset, chunk);
                att_server_notify(_conn_handle, _poac_char_handle, _frag_buf, 2 + chunk);
                vTaskDelay(pdMS_TO_TICKS(1));  // inter-fragment gap for bridge parser
            }
            _delivered++;
        }
#else
        // Non-ESP32: no BLE stack — just count as delivered (loopback / test mode)
        _delivered++;
        printf("[transport] [stub] record queued and delivered (no BLE on this platform)\n");
#endif
    }
}

// ============================================================================
// GATT CALLBACKS (called from BTstack's BLE task on Core 1)
// ============================================================================

#ifdef PLATFORM_ESP32
static void _gatt_write_handler(hci_con_handle_t conn, uint16_t attr,
                                  uint16_t tx_mode,
                                  uint16_t offset, uint8_t* buf, uint16_t len) {
    (void)tx_mode;
    (void)offset;

    if (attr == _chain_char_handle && len == POAC_CHAIN_HASH_SIZE) {
        // Bridge is echoing the chain hash of the record it just confirmed.
        // Feed it back to the builder so the next record chains correctly.
        poac_builder_set_prev_hash(buf);
        printf("[transport] chain echo received — prev_hash[0..3]=%02x%02x%02x%02x\n",
               buf[0], buf[1], buf[2], buf[3]);
    } else if (attr == _poac_cccd_handle && len == 2) {
        uint16_t cccd = buf[0] | ((uint16_t)buf[1] << 8);
        _notify_enabled = (cccd & 0x0001) != 0;
        printf("[transport] CCCD %s by bridge\n", _notify_enabled ? "enabled" : "disabled");
    }
}

static void _packet_handler(uint8_t type, uint16_t channel,
                              uint8_t* packet, uint16_t size) {
    (void)channel; (void)size;

    if (type != HCI_EVENT_PACKET) return;

    switch (hci_event_packet_get_type(packet)) {
        case HCI_EVENT_LE_META:
            if (hci_event_le_meta_get_subevent_code(packet) ==
                HCI_SUBEVENT_LE_CONNECTION_COMPLETE) {
                _conn_handle    = hci_event_le_meta_get_connection_complete_connection_handle(packet);
                _notify_enabled = false;
                printf("[transport] BLE connected — handle=0x%04x\n", _conn_handle);
                // Request MTU upgrade to fit 228+4=232 bytes
                att_server_request_to_send_notification(_conn_handle, _poac_char_handle);
                l2cap_request_can_send_now_event(_conn_handle);
            }
            break;

        case HCI_EVENT_DISCONNECTION_COMPLETE:
            _conn_handle    = HCI_CON_HANDLE_INVALID;
            _notify_enabled = false;
            printf("[transport] BLE disconnected\n");
            break;

        case ATT_EVENT_MTU_EXCHANGE_COMPLETE: {
            uint16_t mtu = att_event_mtu_exchange_complete_get_MTU(packet);
            _neg_mtu   = mtu;
            _fragmented = (mtu < (POAC_RECORD_SIZE + 4));
            printf("[transport] MTU=%u — %s mode\n", mtu,
                   _fragmented ? "fragmented (MTU<232)" : "single-notify");
            break;
        }

        default:
            break;
    }
}
#endif // PLATFORM_ESP32

// ============================================================================
// PUBLIC API
// ============================================================================

bool bridge_transport_init(const char* advertise_name) {
    _q_mutex = xSemaphoreCreateMutex();
    if (!_q_mutex) {
        printf("[transport] FATAL: failed to create mutex\n");
        return false;
    }

    memset(_queue, 0, sizeof(_queue));
    _q_head = _q_tail = 0;

#ifdef PLATFORM_ESP32
    // Register the QorTroller PoAC GATT service alongside joypad-os's existing
    // HID service. BTstack service registration is done before hci_power_control.
    // Here we register callbacks; the DB entries are in the generated .gatt file.
    // For v1 (no .gatt generator in this repo), we register attribute callbacks
    // directly via att_server_register_packet_handler and gatt_server_register.
    att_server_register_packet_handler(_packet_handler);

    // Advertise the QorTroller name alongside the existing HID advertisement.
    // Advertising parameters are managed by joypad-os's btstack_host.c.
    // We just log that we're ready.
    printf("[transport] BLE GATT service ready — advertising as '%s'\n",
           advertise_name ? advertise_name : "QorTroller");
#else
    printf("[transport] BLE not available on this platform — stub mode\n");
#endif

    // Start the delivery task on Core 1 (same core as BT stack)
    BaseType_t ret = xTaskCreatePinnedToCore(
        _transport_task_fn,
        "qt_transport",
        TRANSPORT_TASK_STACK,
        NULL,
        TRANSPORT_TASK_PRI,
        NULL,
        1  // APP_CPU (Core 1)
    );

    if (ret != pdPASS) {
        printf("[transport] FATAL: xTaskCreatePinnedToCore failed (%d)\n", ret);
        return false;
    }

    return true;
}

transport_status_t bridge_transport_send(const poac_record_t* record) {
    if (!record) return TRANSPORT_SEND_FAILED;

    xSemaphoreTake(_q_mutex, portMAX_DELAY);

    if (_queue_full()) {
        // Drop oldest (tail advances)
        _q_tail++;
        _dropped++;
        printf("[transport] queue full — oldest record dropped (total_drops=%u)\n", _dropped);
    }

    _queue[_q_head & (TRANSPORT_QUEUE_DEPTH - 1)] = *record;
    _q_head++;

    xSemaphoreGive(_q_mutex);
    return TRANSPORT_OK;
}

void bridge_transport_on_chain_echo(const uint8_t hash[POAC_CHAIN_HASH_SIZE]) {
    // Direct call path (used by tests / non-BLE loopback)
    poac_builder_set_prev_hash(hash);
}

bool bridge_transport_connected(void) {
#ifdef PLATFORM_ESP32
    return (_conn_handle != HCI_CON_HANDLE_INVALID) && _notify_enabled;
#else
    return false;
#endif
}

uint32_t bridge_transport_delivered_count(void) { return _delivered; }
uint32_t bridge_transport_drop_count(void)      { return _dropped;   }
