// qortroller_hook.h — Raw-sensor tap-point API for joypad-os
// SPDX-License-Identifier: Apache-2.0
//
// This header is the ONLY interface joypad-os core code needs to know about.
// It defines a single callback registration function and the raw-sample struct.
//
// DESIGN PRINCIPLE: additive, zero-cost when disabled.
//   - If no hook is registered: pad_input.c behaves byte-identically to upstream.
//   - If a hook is registered: it receives the raw ADC values BEFORE normalization,
//     deadzone, debounce, or any joypad-os output mapping.
//   - joypad-os output (USB HID / BLE) is NEVER affected by the hook path.
//
// USAGE in pad_input.c (the only caller):
//
//   #include "qortroller/qortroller_hook.h"
//
//   // After reading ADC, BEFORE normalization:
//   if (qortroller_hook_enabled()) {
//       poac_raw_sample_t s = {0};
//       s.adc_raw[POAC_AXIS_LX]  = raw_lx;
//       s.adc_raw[POAC_AXIS_LY]  = raw_ly;
//       s.adc_raw[POAC_AXIS_L2]  = raw_lt;
//       s.adc_raw[POAC_AXIS_R2]  = raw_rt;
//       s.buttons_raw             = pre_debounce_buttons;
//       s.timestamp_us            = platform_get_time_us();
//       qortroller_hook_submit(&s);
//   }

#ifndef QORTROLLER_HOOK_H
#define QORTROLLER_HOOK_H

#include "poac_types.h"
#include <stdbool.h>

// ============================================================================
// HOOK API — called by pad_input.c at the raw ADC read site
// ============================================================================

// Register the QorTroller raw-sample callback.
// Called once at startup by qortroller_sense_task_init().
// Pass NULL to unregister (disables the hook path entirely).
typedef void (*qortroller_raw_hook_fn)(const poac_raw_sample_t* sample);
void qortroller_hook_register(qortroller_raw_hook_fn fn);

// Returns true if a hook is registered (used as a fast-path guard in pad_input.c
// so the sample struct is never built when QorTroller is disabled).
bool qortroller_hook_enabled(void);

// Submit one raw sample to the registered hook.
// Thread-safe: safe to call from any FreeRTOS task or ISR context.
// If hook is NULL (not registered), this is a no-op.
void qortroller_hook_submit(const poac_raw_sample_t* sample);

// ============================================================================
// UWB PRESENCE API — fed from uwb_presence.c (QM35825 UART reader)
// ============================================================================

// Presence states from the QM35825 UWB radar SoC
typedef enum {
    UWB_PRESENCE_NO_SENSOR  = 0xFF,  // QM35825 not installed / not responding
    UWB_PRESENCE_ABSENT     = 0x00,  // No human detected
    UWB_PRESENCE_PRESENT    = 0x01,  // Human physically present at controller
} uwb_presence_state_t;

// Update the presence state (called by uwb_presence.c when QM35825 reports)
void qortroller_uwb_update(uwb_presence_state_t state, uint8_t confidence);

// Read the current presence state (called by poac_builder.c when building record)
uwb_presence_state_t qortroller_uwb_get_state(void);
uint8_t              qortroller_uwb_get_confidence(void);

#endif // QORTROLLER_HOOK_H
