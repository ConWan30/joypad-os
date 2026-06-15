// qortroller_hook.c — Raw-sensor hook implementation
// SPDX-License-Identifier: Apache-2.0
//
// Minimal, lock-free hook dispatch. The hook function is registered once at
// startup and never changes during a session — no mutex needed for the read
// path (only the register path writes it, before any ADC polling begins).

#include "qortroller_hook.h"
#include <stddef.h>
#include <stdint.h>

// ============================================================================
// HOOK STATE
// ============================================================================

static volatile qortroller_raw_hook_fn _hook_fn = NULL;

// UWB presence state (updated by uwb_presence.c via UART ISR)
static volatile uwb_presence_state_t _uwb_state      = UWB_PRESENCE_NO_SENSOR;
static volatile uint8_t              _uwb_confidence  = 0;

// ============================================================================
// HOOK API IMPLEMENTATION
// ============================================================================

void qortroller_hook_register(qortroller_raw_hook_fn fn) {
    _hook_fn = fn;
}

bool qortroller_hook_enabled(void) {
    return _hook_fn != NULL;
}

void qortroller_hook_submit(const poac_raw_sample_t* sample) {
    qortroller_raw_hook_fn fn = _hook_fn;
    if (fn && sample) {
        fn(sample);
    }
}

// ============================================================================
// UWB PRESENCE API IMPLEMENTATION
// ============================================================================

void qortroller_uwb_update(uwb_presence_state_t state, uint8_t confidence) {
    _uwb_state      = state;
    _uwb_confidence = confidence;
}

uwb_presence_state_t qortroller_uwb_get_state(void) {
    return _uwb_state;
}

uint8_t qortroller_uwb_get_confidence(void) {
    return _uwb_confidence;
}
