// sense_task.h — QorTroller Core-0-pinned 1 kHz biometric sense task
// SPDX-License-Identifier: Apache-2.0
//
// This FreeRTOS task is the Core 0 owner of the biometric pipeline.
// It runs at 1 ms cadence using vTaskDelayUntil, pinned to PRO_CPU (Core 0)
// to guarantee deterministic timing independent of the BT/WiFi workload on
// APP_CPU (Core 1 — where joypad-os's main FreeRTOS loop runs).
//
// Task responsibilities:
//   1. Poll PCC (capture health) state via ADC poll-rate CV
//   2. Drive the raw-hook registration (qortroller_hook)
//   3. Trigger poac_builder_build() once per POAC_CYCLE_SAMPLES samples
//   4. Pass completed records to atca_signer for ATECC608B signing
//   5. Feed signed records to bridge_transport for BLE/WiFi delivery
//   6. Update WS2812 LED with capture state (NOMINAL=green, DEGRADED=amber, etc.)

#ifndef QORTROLLER_SENSE_TASK_H
#define QORTROLLER_SENSE_TASK_H

#include "poac_types.h"
#include <stdbool.h>
#include <stdint.h>

// ============================================================================
// TASK CONFIGURATION
// ============================================================================

#define SENSE_TASK_STACK_SIZE  8192   // bytes (large for FFT stack frames)
#define SENSE_TASK_PRIORITY      24   // above BT/WiFi (10-20), below ISR (25+)
#define SENSE_TASK_CORE           0   // PRO_CPU — pinned for deterministic timing
#define SENSE_TASK_PERIOD_MS      1   // 1 kHz cadence

// PCC thresholds (mirrors bridge/vapi_bridge/capture_continuity.py)
#define PCC_NOMINAL_HZ     900.0f    // min Hz for NOMINAL state
#define PCC_DEGRADED_HZ    200.0f    // min Hz before DEGRADED
#define PCC_STABLE_WINDOW   60       // samples for CV computation
#define PCC_CONTESTED_CV     0.40f   // CV ≥ this → CONTESTED (dual-host)
#define PCC_EXCLUSIVE_CV     0.20f   // CV < this → EXCLUSIVE_USB

// GIC warmup: require this many consecutive NOMINAL samples before counting
#define SENSE_WARMUP_SAMPLES  30000  // 30 s at 1 kHz

// ============================================================================
// CAPTURE HEALTH STATE (mirrors bridge CaptureHealthMonitor)
// ============================================================================

typedef enum {
    PCC_STATE_NOMINAL      = 0,
    PCC_STATE_DEGRADED     = 1,
    PCC_STATE_DISCONNECTED = 2,
} pcc_state_t;

typedef enum {
    PCC_HOST_EXCLUSIVE_USB = 0,
    PCC_HOST_CONTESTED     = 1,
    PCC_HOST_EXCLUSIVE_BT  = 2,
    PCC_HOST_UNKNOWN       = 3,
} pcc_host_state_t;

typedef struct {
    pcc_state_t      state;
    pcc_host_state_t host_state;
    float            poll_rate_hz;
    float            poll_rate_cv;
    bool             grind_ready;
    uint32_t         consecutive_nominal_samples;
} pcc_status_t;

// ============================================================================
// SENSE TASK API
// ============================================================================

// Initialize and start the sense task on Core 0.
// device_id: 32-byte ATECC608B-derived device identity hash.
// Returns true if the FreeRTOS task was created successfully.
bool sense_task_start(const uint8_t device_id[POAC_DEVICE_ID_SIZE]);

// Get the current PCC status (thread-safe read).
pcc_status_t sense_task_get_pcc(void);

// Signal a GIC chain break (called by bridge transport on tamper detection).
// The sense task stops emitting new records until sense_task_gic_reset() is called.
void sense_task_gic_break(void);

// Reset the GIC break flag (operator-triggered after investigation).
void sense_task_gic_reset(void);

// Get the count of PoAC records shipped this session.
uint32_t sense_task_records_shipped(void);

#endif // QORTROLLER_SENSE_TASK_H
