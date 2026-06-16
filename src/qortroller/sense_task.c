// sense_task.c — QorTroller Core-0-pinned 1 kHz biometric sense task
// SPDX-License-Identifier: Apache-2.0
//
// Runs on ESP32-S3 PRO_CPU (Core 0) via xTaskCreatePinnedToCore.
// Drives the biometric pipeline at exactly 1 ms cadence using vTaskDelayUntil.
// APP_CPU (Core 1) runs the joypad-os BT/WiFi/USB main loop — Core 0 is
// exclusively owned by this task and FreeRTOS idle.
//
// Pipeline per tick:
//   [hook already submitted raw sample from pad_input.c in Core 1 context]
//   → poac_builder accumulates sample (incremental, lock-free)
//   → every POAC_CYCLE_SAMPLES (1000 ticks = 1 s): build + sign + ship

#include "sense_task.h"
#include "poac_builder.h"
#include "qortroller_hook.h"
#include "atca_signer.h"
#include "bridge_transport.h"
#include "uwb_presence.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "platform/platform.h"

#include <string.h>
#include <stdio.h>

// ============================================================================
// TASK STATE
// ============================================================================

static volatile pcc_status_t _pcc = {
    .state      = PCC_STATE_DISCONNECTED,
    .host_state = PCC_HOST_UNKNOWN,
    .poll_rate_hz   = 0.0f,
    .poll_rate_cv   = 1.0f,
    .grind_ready    = false,
    .consecutive_nominal_samples = 0,
};

// PCC rolling rate window (60 samples = last 60 ms)
static uint64_t _rate_window[PCC_STABLE_WINDOW];
static uint8_t  _rate_idx  = 0;
static bool     _rate_full = false;

// GIC chain break flag
static volatile bool _gic_broken = false;

// Records shipped this session
static volatile uint32_t _records_shipped = 0;

// Warmup counter (samples since last DISCONNECTED → NOMINAL transition)
static uint32_t _warmup_count = 0;

// Pending raw sample from the hook (single-producer, single-consumer)
// pad_input.c runs on Core 1; sense task runs on Core 0.
// We use a double-buffer: hook writes to the back buffer, sense task reads
// from the front. A single atomic swap replaces a mutex for this 1 kHz path.
static poac_raw_sample_t _sample_front;
static poac_raw_sample_t _sample_back;
static volatile bool     _sample_ready = false;

// ============================================================================
// RAW HOOK CALLBACK
// Called from Core 1 (pad_input.c poll) at ~1 kHz via qortroller_hook_submit.
// Must be fast and lock-free.
// ============================================================================

static void _sense_hook(const poac_raw_sample_t* s) {
    if (!s) return;
    _sample_back  = *s;
    _sample_ready = true;
}

// ============================================================================
// PCC UPDATE — called once per tick from the sense task
// Infers capture health from inter-sample interval statistics.
// ============================================================================

static void _pcc_update(uint64_t now_us) {
    static uint64_t _prev_us = 0;

    if (_prev_us == 0) {
        _prev_us = now_us;
        return;
    }

    uint64_t dt = now_us - _prev_us;
    _prev_us = now_us;

    // Insert inter-sample interval (µs) into rolling window
    _rate_window[_rate_idx] = dt;
    _rate_idx = (_rate_idx + 1) % PCC_STABLE_WINDOW;
    if (_rate_idx == 0) _rate_full = true;

    uint8_t n = _rate_full ? PCC_STABLE_WINDOW : _rate_idx;
    if (n < 4) return;

    // Compute mean and CV
    float sum = 0.0f;
    for (int i = 0; i < n; i++) sum += _rate_window[i];
    float mean_us = sum / n;
    float hz = (mean_us > 0.0f) ? (1e6f / mean_us) : 0.0f;

    float var = 0.0f;
    for (int i = 0; i < n; i++) {
        float d = (float)_rate_window[i] - mean_us;
        var += d * d;
    }
    float cv = (mean_us > 0.0f) ? (sqrtf(var / n) / mean_us) : 1.0f;

    // Classify state
    pcc_state_t state;
    if (hz < PCC_DEGRADED_HZ) {
        state = PCC_STATE_DISCONNECTED;
        _warmup_count = 0;
    } else if (hz < PCC_NOMINAL_HZ) {
        state = PCC_STATE_DEGRADED;
        _warmup_count = 0;
    } else {
        state = PCC_STATE_NOMINAL;
        _warmup_count++;
    }

    // Classify host state from CV
    pcc_host_state_t host;
    if (state == PCC_STATE_NOMINAL && cv < PCC_EXCLUSIVE_CV) {
        host = PCC_HOST_EXCLUSIVE_USB;
    } else if (cv >= PCC_CONTESTED_CV) {
        host = PCC_HOST_CONTESTED;
        _warmup_count = 0;  // contested resets warmup
    } else {
        host = PCC_HOST_UNKNOWN;
    }

    bool grind_ready = (state == PCC_STATE_NOMINAL) &&
                       (host == PCC_HOST_EXCLUSIVE_USB || host == PCC_HOST_UNKNOWN) &&
                       (_warmup_count >= SENSE_WARMUP_SAMPLES);

    // Write back (no mutex needed — pcc_status_t is small enough for aligned writes
    // to be atomic on Xtensa LX7; readers call sense_task_get_pcc() which copies)
    _pcc.state       = state;
    _pcc.host_state  = host;
    _pcc.poll_rate_hz  = hz;
    _pcc.poll_rate_cv  = cv;
    _pcc.grind_ready   = grind_ready;
    _pcc.consecutive_nominal_samples = _warmup_count;
}

// ============================================================================
// SENSE TASK BODY
// ============================================================================

static void _sense_task_fn(void* arg) {
    (void)arg;

    printf("[sense] task started on Core %d at priority %d\n",
           xPortGetCoreID(), uxTaskPriorityGet(NULL));

    TickType_t wake = xTaskGetTickCount();
    uint32_t   tick = 0;

    while (1) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(SENSE_TASK_PERIOD_MS));
        tick++;

        // Consume pending sample from hook (Core 1 → Core 0)
        if (_sample_ready) {
            _sample_front = _sample_back;
            _sample_ready = false;

            // Update PCC from sample timestamp
            _pcc_update(_sample_front.timestamp_us);

            // Feed sample into PoAC builder
            poac_builder_on_sample(&_sample_front);
        }

        // Every POAC_CYCLE_SAMPLES ticks: build + sign + ship
        if (tick % POAC_CYCLE_SAMPLES != 0) continue;

        // PCC gate: must be NOMINAL to emit a record
        if (_pcc.state != PCC_STATE_NOMINAL) {
            printf("[sense] cycle skipped — PCC not NOMINAL (state=%d)\n", _pcc.state);
            poac_builder_reset();
            continue;
        }

        // GIC gate: must not be broken
        if (_gic_broken) {
            printf("[sense] cycle skipped — GIC chain broken\n");
            poac_builder_reset();
            continue;
        }

        // Build the PoAC record
        static poac_record_t _rec;
        poac_build_status_t bs = poac_builder_build(&_rec);
        if (bs != POAC_BUILD_OK) {
            printf("[sense] build skipped (status=%d)\n", bs);
            continue;
        }

        // Patch capture flags from live PCC state
        uint8_t flags = _rec.body.capture_flags[0];
        if (_pcc.state == PCC_STATE_NOMINAL)          flags |=  POAC_FLAG_CAPTURE_NOMINAL;
        else                                           flags &= ~POAC_FLAG_CAPTURE_NOMINAL;
        if (_pcc.host_state == PCC_HOST_EXCLUSIVE_USB) flags |=  POAC_FLAG_HOST_EXCLUSIVE;
        else                                            flags &= ~POAC_FLAG_HOST_EXCLUSIVE;
        if (!_gic_broken)                              flags |=  POAC_FLAG_GIC_INTACT;
        else                                           flags &= ~POAC_FLAG_GIC_INTACT;
        _rec.body.capture_flags[0] = flags;

        // Sign via ATECC608B (blocking, ~60–100 ms — acceptable here; this is
        // a per-second event, not a per-tick event)
        atca_sign_status_t ss = atca_signer_sign(&_rec);
        if (ss == ATCA_SIGN_OK) {
            _rec.body.capture_flags[0] |= POAC_FLAG_ATCA_SIGNED;
        } else {
            printf("[sense] ATCA sign failed (status=%d) — shipping unsigned\n", ss);
            // Unsigned records are still valid for local L4; bridge marks them
            // as unverified (POAC_FLAG_ATCA_SIGNED not set).
        }

        // Deliver to bridge
        transport_status_t ts = bridge_transport_send(&_rec);
        if (ts == TRANSPORT_OK) {
            _records_shipped++;
            printf("[sense] record %u shipped (dist=%.2f)\n",
                   _records_shipped,
                   // Decode humanity_prob from body for the log line
                   (float)_rec.body.humanity_prob_be[0]); // crude — full decode in bridge
        } else {
            printf("[sense] transport failed (status=%d)\n", ts);
        }
    }
}

// ============================================================================
// PUBLIC API
// ============================================================================

bool sense_task_start(const uint8_t device_id[POAC_DEVICE_ID_SIZE]) {
    // Initialize pipeline modules
    poac_builder_init(device_id);

    // Register the raw-sensor hook so pad_input.c delivers samples here
    qortroller_hook_register(_sense_hook);

    // Create the Core-0-pinned task
    BaseType_t ret = xTaskCreatePinnedToCore(
        _sense_task_fn,
        "qt_sense",
        SENSE_TASK_STACK_SIZE,
        NULL,
        SENSE_TASK_PRIORITY,
        NULL,
        SENSE_TASK_CORE
    );

    if (ret != pdPASS) {
        printf("[sense] FATAL: xTaskCreatePinnedToCore failed (%d)\n", ret);
        qortroller_hook_register(NULL);  // unregister on failure
        return false;
    }

    printf("[sense] task created — 1 kHz biometric pipeline active\n");
    return true;
}

pcc_status_t sense_task_get_pcc(void) {
    // Snapshot copy (all fields are small; struct copy is atomic enough on LX7
    // for a status read that tolerates a single-sample lag)
    return _pcc;
}

void sense_task_gic_break(void) {
    _gic_broken = true;
    printf("[sense] GIC chain break signalled — pipeline gated\n");
}

void sense_task_gic_reset(void) {
    _gic_broken = false;
    poac_builder_reset();
    printf("[sense] GIC chain reset — pipeline resuming\n");
}

uint32_t sense_task_records_shipped(void) {
    return _records_shipped;
}
