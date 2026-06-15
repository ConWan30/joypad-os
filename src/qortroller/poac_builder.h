// poac_builder.h — PoAC record builder and cognition-cycle manager
// SPDX-License-Identifier: Apache-2.0
//
// Accumulates raw samples from the hook, computes L4 biometric features,
// and assembles the 228-byte PoAC record for ATECC608B signing.
//
// Architecture: this module runs on ESP32-S3 Core 0, pinned via
// xTaskCreatePinnedToCore at priority 24 (above BT/WiFi, below ISR).
// The 1 kHz cadence is maintained by vTaskDelayUntil(1ms).

#ifndef QORTROLLER_POAC_BUILDER_H
#define QORTROLLER_POAC_BUILDER_H

#include "poac_types.h"
#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// COGNITION CYCLE CONFIGURATION
// ============================================================================

// Samples per PoAC record (1000 Hz × 1 s = 1000 samples = one cognition cycle)
#define POAC_CYCLE_SAMPLES   1000

// Ring buffer depth for raw samples (2 cycles of headroom)
#define POAC_RING_DEPTH      2048

// Tremor FFT window (power of 2, fits in 1 cycle at 1 kHz)
#define POAC_TREMOR_FFT_WIN   512

// L4 feature count (must match bridge/vapi_bridge L4 calibration)
#define POAC_L4_FEATURE_COUNT  11

// ============================================================================
// BUILDER STATUS
// ============================================================================

typedef enum {
    POAC_BUILD_OK            = 0,  // Record assembled successfully
    POAC_BUILD_INSUFFICIENT  = 1,  // Not enough samples yet
    POAC_BUILD_NO_PRESENCE   = 2,  // UWB: human absent (skipped)
    POAC_BUILD_MENU_DETECTED = 3,  // No trigger activity (menu, not gameplay)
    POAC_BUILD_ATCA_ERROR    = 4,  // ATECC608B signing failed
    POAC_BUILD_CHAIN_BROKEN  = 5,  // GIC chain integrity lost
} poac_build_status_t;

// ============================================================================
// BUILDER API
// ============================================================================

// Initialize the PoAC builder. Call once at startup, before registering
// the raw hook. Allocates the ring buffer and initializes L4 accumulators.
// device_id: 32-byte SHA-256 of (ATECC608B_pubkey || ATECC608B_serial)
void poac_builder_init(const uint8_t device_id[POAC_DEVICE_ID_SIZE]);

// Submit one raw sample (called by the hook registered with qortroller_hook.h).
// Lock-free ring buffer insert. If the ring is full, the oldest sample is dropped
// and a counter increments — not a fatal error (1 dropped sample per 2048 is
// within L4 tolerance; 10+ per cycle degrades tremor FFT accuracy).
void poac_builder_on_sample(const poac_raw_sample_t* sample);

// Attempt to build one PoAC record from accumulated samples.
// Called once per second by the sense task (or by the bridge telemetry task
// after transport queues the previous record).
// On POAC_BUILD_OK: record is populated and ready for ATECC signing.
poac_build_status_t poac_builder_build(poac_record_t* out);

// Set the previous chain hash (updated after each signed record is confirmed
// by the bridge transport layer). The next record's prev_hash field is set
// from this value.
void poac_builder_set_prev_hash(const uint8_t hash[POAC_CHAIN_HASH_SIZE]);

// Reset all accumulators (e.g., after a chain break / GIC reset).
void poac_builder_reset(void);

// Diagnostic: number of samples accumulated since last build
uint32_t poac_builder_sample_count(void);

// Diagnostic: number of dropped samples since last build (ring overflows)
uint32_t poac_builder_drop_count(void);

// ============================================================================
// L4 FEATURE COMPUTATION (exposed for testing / bridge calibration sync)
// ============================================================================

// Compute L4 features from a completed accumulator.
// Called internally by poac_builder_build(); exposed for unit tests.
void poac_compute_l4_features(const poac_l4_features_t* acc,
                               float out_features[POAC_L4_FEATURE_COUNT]);

// Compute Mahalanobis distance given features and a calibration matrix.
// calib_mean / calib_cov_inv: loaded from flash (written by bridge during
// enrollment / calibration ceremony). If not yet loaded, returns 0.0f.
float poac_mahalanobis(const float features[POAC_L4_FEATURE_COUNT]);

// Humanity probability from Mahalanobis distance (sigmoid, same formula
// as bridge's humanity_probability()).
float poac_humanity_prob(float mahalanobis_dist);

#endif // QORTROLLER_POAC_BUILDER_H
