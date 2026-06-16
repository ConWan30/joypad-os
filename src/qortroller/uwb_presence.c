// uwb_presence.c — Qorvo QM35825 UWB presence reader (UART, hardware-gated)
// SPDX-License-Identifier: Apache-2.0
//
// HARDWARE-GATED: QM35825DK-05 eval kit + 1.8V↔3.3V level-shifter required
// (QM35825 I/O is 1.8V; ESP32-S3 GPIO is 3.3V). Without hardware, the task
// starts, UART init fails, and presence stays at UWB_PRESENCE_NO_SENSOR.
//
// PRIVACY GATE (mandatory — non-negotiable per TRACK1-LESSON-003):
//   This module requests PRESENCE_MODE only (binary human/absent flag).
//   If a frame arrives with the vital-sign flag set (UWB_FLAG_VITALSIGN),
//   the frame is DISCARDED, the violation counter increments, and an error
//   is logged. The sensor is disabled for the session. This is a hard safety
//   rail — vital-sign capture requires independent privacy review.
//
// FRAME FORMAT (v1 placeholder — replace with actual QM35825 protocol bytes
// from the Qorvo FAE datasheet once confirmed via Qorvo reg outreach 2026-06-14):
//   [0] 0xAA  magic0
//   [1] 0x55  magic1
//   [2] 0x01  frame_type = PRESENCE
//   [3] flags (0x01 = vital-sign active — MUST be 0)
//   [4] presence (0x00=absent, 0x01=present)
//   [5] confidence 0–255
//   [6] reserved
//   [7] XOR checksum of bytes [0..6]

#include "uwb_presence.h"
#include "platform/platform.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>
#include <stdio.h>

// ============================================================================
// ESP-IDF UART INCLUDES
// ============================================================================

#ifdef PLATFORM_ESP32
#include "driver/uart.h"
#include "driver/gpio.h"
#else
// Non-ESP32 stub
typedef int uart_port_t;
#define uart_driver_install(p,b,t,q,e,f)  (-1)
#define uart_set_pin(p,tx,rx,r,c)         (-1)
#define uart_read_bytes(p,b,l,t)          (-1)
#endif

// ============================================================================
// PRESENCE STATE
// ============================================================================

static volatile uwb_presence_state_t _state      = UWB_PRESENCE_NO_SENSOR;
static volatile uint8_t              _confidence  = 0;
static volatile uint32_t             _frame_count = 0;
static volatile uint32_t             _vs_violations = 0;

// Set to true if vital-sign violation detected — sensor disabled for session
static volatile bool _sensor_disabled = false;

// UART port in use
static uart_port_t _uart_port = 0;
static bool        _uart_ready = false;

// ============================================================================
// FRAME PARSER
// ============================================================================

static void _parse_frame(const uint8_t* buf) {
    // Validate checksum: XOR of bytes [0..6] must equal byte [7]
    uint8_t xor = 0;
    for (int i = 0; i < UWB_PRESENCE_FRAME_LEN - 1; i++) xor ^= buf[i];
    if (xor != buf[UWB_PRESENCE_FRAME_LEN - 1]) {
        // Checksum mismatch — discard (common during UART sync)
        return;
    }

    // Validate magic
    if (buf[0] != UWB_FRAME_MAGIC_0 || buf[1] != UWB_FRAME_MAGIC_1) return;
    if (buf[2] != UWB_FRAME_TYPE_PRESENCE) return;  // not a presence frame

    // PRIVACY GATE: vital-sign flag must be 0
    if (buf[3] & UWB_FLAG_VITALSIGN) {
        _vs_violations++;
        printf("[uwb] PRIVACY VIOLATION: vital-sign flag set in frame %u — "
               "sensor disabled for session!\n", _frame_count);
        _sensor_disabled = true;
        _state = UWB_PRESENCE_NO_SENSOR;
        qortroller_uwb_update(UWB_PRESENCE_NO_SENSOR, 0);
        return;
    }

    // Parse presence field
    uwb_presence_state_t pres;
    switch (buf[4]) {
        case 0x00: pres = UWB_PRESENCE_ABSENT;  break;
        case 0x01: pres = UWB_PRESENCE_PRESENT; break;
        default:   pres = UWB_PRESENCE_NO_SENSOR; break;
    }

    uint8_t conf = buf[5];

    _state      = pres;
    _confidence = conf;
    _frame_count++;

    // Push to qortroller_hook for embedding in PoAC records
    qortroller_uwb_update(pres, conf);

    if (_frame_count % 100 == 1) {  // log every 10 s (100 frames @ 100 ms/frame)
        printf("[uwb] frame %u — %s (conf=%u)\n",
               _frame_count,
               pres == UWB_PRESENCE_PRESENT ? "PRESENT" :
               pres == UWB_PRESENCE_ABSENT  ? "ABSENT"  : "NO_SENSOR",
               conf);
    }
}

// ============================================================================
// UWB PRESENCE TASK
// ============================================================================

#define UWB_TASK_STACK  2048
#define UWB_TASK_PRI    8     // above transport (5), below sense (24)

static void _uwb_task_fn(void* arg) {
    (void)arg;

    if (!_uart_ready) {
        printf("[uwb] UART not ready — task exiting (no QM35825 detected)\n");
        vTaskDelete(NULL);
        return;
    }

    printf("[uwb] presence reader task started on Core %d\n", xPortGetCoreID());

    // Frame sync buffer (sliding window over UART stream)
    uint8_t buf[UWB_PRESENCE_FRAME_LEN];
    uint8_t sync[2] = {UWB_FRAME_MAGIC_0, UWB_FRAME_MAGIC_1};
    uint8_t sync_idx = 0;

    while (!_sensor_disabled) {
#ifdef PLATFORM_ESP32
        uint8_t b;
        int n = uart_read_bytes(_uart_port, &b, 1,
                                pdMS_TO_TICKS(UWB_FRAME_TIMEOUT_MS));
        if (n <= 0) continue;

        // State machine: search for magic bytes, then capture full frame
        if (sync_idx < 2) {
            if (b == sync[sync_idx]) {
                buf[sync_idx] = b;
                sync_idx++;
            } else {
                sync_idx = (b == sync[0]) ? 1 : 0;
            }
            continue;
        }

        // Collect remaining frame bytes
        uint8_t remaining = UWB_PRESENCE_FRAME_LEN - 2;
        int r = uart_read_bytes(_uart_port, buf + 2, remaining,
                                pdMS_TO_TICKS(UWB_FRAME_TIMEOUT_MS));
        if (r == remaining) {
            _parse_frame(buf);
        }
        sync_idx = 0;  // reset for next frame
#else
        // Non-ESP32 stub: stay quiet (no UART)
        vTaskDelay(pdMS_TO_TICKS(UWB_POLL_INTERVAL_MS));
#endif
    }

    printf("[uwb] task exiting (sensor disabled)\n");
    vTaskDelete(NULL);
}

// ============================================================================
// PUBLIC API
// ============================================================================

bool uwb_presence_start(int uart_port, int tx_pin, int rx_pin) {
    _state        = UWB_PRESENCE_NO_SENSOR;
    _confidence   = 0;
    _frame_count  = 0;
    _vs_violations = 0;
    _sensor_disabled = false;
    _uart_ready   = false;

#ifdef PLATFORM_ESP32
    _uart_port = (uart_port_t)uart_port;

    // Configure UART at 115200 baud (QM35825 default)
    uart_config_t cfg = {
        .baud_rate  = UWB_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };

    if (uart_driver_install(_uart_port, 256, 0, 0, NULL, 0) != 0) {
        printf("[uwb] UART%d install failed — QM35825 not connected\n", uart_port);
        return false;
    }

    if (uart_param_config(_uart_port, &cfg) != 0 ||
        uart_set_pin(_uart_port, tx_pin, rx_pin, -1, -1) != 0) {
        printf("[uwb] UART%d config failed\n", uart_port);
        return false;
    }

    _uart_ready = true;
    printf("[uwb] QM35825 UART%d ready (TX=%d, RX=%d)\n", uart_port, tx_pin, rx_pin);
#else
    (void)uart_port; (void)tx_pin; (void)rx_pin;
    printf("[uwb] UWB presence reader: no UART on this platform — stub mode\n");
    return false;
#endif

    BaseType_t ret = xTaskCreatePinnedToCore(
        _uwb_task_fn,
        "qt_uwb",
        UWB_TASK_STACK,
        NULL,
        UWB_TASK_PRI,
        NULL,
        1  // APP_CPU (Core 1) — no timing criticality
    );

    if (ret != pdPASS) {
        printf("[uwb] FATAL: xTaskCreatePinnedToCore failed (%d)\n", ret);
        return false;
    }

    return true;
}

uwb_presence_state_t uwb_presence_get(void)    { return _state;       }
uint8_t              uwb_presence_confidence(void) { return _confidence; }
uint32_t             uwb_presence_frame_count(void){ return _frame_count;}
uint32_t             uwb_presence_vitalsign_violations(void) { return _vs_violations; }
