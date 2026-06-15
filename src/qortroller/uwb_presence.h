// uwb_presence.h — QM35825 UWB radar presence reader (UART interface)
// SPDX-License-Identifier: Apache-2.0
//
// The Qorvo QM35825 exposes its presence/ranging output over SPI or UART.
// QorTroller v1 uses UART (simpler wiring, sufficient for binary presence flag).
//
// VERIFIED SPECS (Qorvo product page, 2026-06-14):
//   Interface: 2× hi-speed SPI slave (40 MHz) + SPI master + UART + 25 GPIO
//   Supply: 1.14–3.6 V (level-shift required from ESP32-S3 3.3 V UART)
//   On-chip: Cortex-M33 + Secure Enclave + HW RSA/SHA/AES/TRNG
//   Modes: motion / presence / people-counting / vital-sign (vital-sign DISABLED
//          per Surface 9 privacy rider — TRACK1-LESSON-003 applies)
//
// PRIVACY GATE (mandatory — non-negotiable):
//   QorTroller v1 requests PRESENCE_MODE only (binary human/absent).
//   VITAL_SIGN_MODE must be off. Any future firmware that enables vital-sign
//   capture requires a full privacy review and cannot ship without it.
//   See: wiki/methodology/sensor_stack_v2_3_uwb_presence_design_note.md §PRIVACY

#ifndef QORTROLLER_UWB_PRESENCE_H
#define QORTROLLER_UWB_PRESENCE_H

#include "qortroller_hook.h"
#include <stdbool.h>
#include <stdint.h>

// ============================================================================
// QM35825 UART PROTOCOL (application-layer, simplified for v1)
// ============================================================================
// The QM35825 sends ASCII or binary frames over UART at 115200 baud.
// v1 parses only the presence field — everything else is ignored.
// Full protocol spec lives in the QM35825 datasheet (request from Qorvo FAE).

#define UWB_UART_BAUD     115200
#define UWB_FRAME_TIMEOUT_MS  50  // Max wait for a complete frame
#define UWB_POLL_INTERVAL_MS  100 // Presence update rate from QM35825

// Frame magic (placeholder — replace with actual QM35825 protocol bytes from datasheet)
#define UWB_FRAME_MAGIC_0  0xAA
#define UWB_FRAME_MAGIC_1  0x55

// v1 presence frame (8 bytes total — placeholder layout):
// [0]   0xAA  magic0
// [1]   0x55  magic1
// [2]   0x01  frame type = PRESENCE
// [3]   0x00  flags (0x01 = vital-sign mode active — MUST be 0 in v1)
// [4]   presence (0x00=absent, 0x01=present)
// [5]   confidence 0–255
// [6]   reserved
// [7]   XOR checksum of [0..6]
#define UWB_PRESENCE_FRAME_LEN  8
#define UWB_FRAME_TYPE_PRESENCE 0x01
#define UWB_FLAG_VITALSIGN      0x01  // MUST be 0 — privacy gate

// ============================================================================
// UWB PRESENCE TASK API
// ============================================================================

// Initialize and start the UWB presence reader task.
// uart_port: ESP-IDF UART port number (0, 1, or 2).
// tx_pin / rx_pin: GPIO pin numbers for the QM35825 UART connection.
// Returns true if UART initialized and task started successfully.
// Returns false if QM35825 is not connected (presence stays NO_SENSOR).
bool uwb_presence_start(int uart_port, int tx_pin, int rx_pin);

// Get the latest presence reading (updated by the UWB task from UART frames).
// Thread-safe: reads are atomic on Xtensa LX7.
uwb_presence_state_t uwb_presence_get(void);
uint8_t              uwb_presence_confidence(void);

// Diagnostic: number of frames received since start
uint32_t uwb_presence_frame_count(void);

// Diagnostic: number of frames with vital-sign flag set (should always be 0)
// Non-zero = QM35825 misconfigured → log error + disable the sensor in the hook
uint32_t uwb_presence_vitalsign_violations(void);

// ============================================================================
// BOARD CONFIGURATION (define in board config or CMakeLists)
// ============================================================================
// These defaults assume the QM35825DK-05 dev kit connected to ESP32-S3
// UART1 with a 3.3V ↔ 1.8V level-shift (QM35825 I/O is 1.8V on the DK).
//
// Override by defining before #include or in idf_component.yml:
//   CONFIG_UWB_UART_PORT  = 1
//   CONFIG_UWB_UART_TX    = GPIO_NUM_17
//   CONFIG_UWB_UART_RX    = GPIO_NUM_18

#ifndef CONFIG_UWB_UART_PORT
#define CONFIG_UWB_UART_PORT   1
#endif

#ifndef CONFIG_UWB_UART_TX
#define CONFIG_UWB_UART_TX    17
#endif

#ifndef CONFIG_UWB_UART_RX
#define CONFIG_UWB_UART_RX    18
#endif

#endif // QORTROLLER_UWB_PRESENCE_H
