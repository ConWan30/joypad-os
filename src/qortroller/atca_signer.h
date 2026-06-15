// atca_signer.h — ATECC608B signing interface for QorTroller PoAC records
// SPDX-License-Identifier: Apache-2.0
//
// Wraps Microchip CryptoAuthLib (atca_*) for signing the 164-byte PoAC body
// with the ATECC608B secure element on the QorTroller dev-kit (BOM C2).
//
// PATH A ARC 2 — this module is HARDWARE-GATED until the ATECC608B breakout
// is physically wired to the ESP32-S3 I2C bus (SDA/SCL, BOM C2).
// Before hardware lands: build compiles, atca_signer_sign() returns
// ATCA_NOT_LOCKED with sig bytes zeroed (honest fail-open).
//
// ATECC608B FAMILY NOTES (from Path A manufacturing spec + HWFL-1 Cycle 16):
//   - ATECC608A is NRND — do not spec for new designs
//   - ATECC608B is Active drop-in per Microchip AN2237
//   - ATECC608C-TFLXTLS is the TrustFLEX pre-provisioned target (Rung 3)
//   - CryptoAuthLib polling-based timing REQUIRED (forward-compatible with family)
//   - I2C address: 0x60 (ATECC608B default, configurable via config zone)

#ifndef QORTROLLER_ATCA_SIGNER_H
#define QORTROLLER_ATCA_SIGNER_H

#include "poac_types.h"
#include <stdbool.h>
#include <stdint.h>

// ============================================================================
// SIGNING STATUS
// ============================================================================

typedef enum {
    ATCA_SIGN_OK          = 0,  // Record signed successfully
    ATCA_NOT_LOCKED       = 1,  // ATECC608B not connected / config zone not locked
    ATCA_I2C_ERROR        = 2,  // I2C bus error (wire, pull-up, address)
    ATCA_SIGN_FAILED      = 3,  // CryptoAuthLib atcab_sign() returned error
    ATCA_VERIFY_FAILED    = 4,  // Post-sign verify failed (signature corrupt)
    ATCA_KEY_SLOT_INVALID = 5,  // Key slot not provisioned (Rung 2 gate open)
} atca_sign_status_t;

// ============================================================================
// KEY SLOT CONFIGURATION
// ============================================================================
// ATECC608B has 16 key slots (0–15). QorTroller v1 uses:
//   Slot 0: device identity private key (ECDSA-P256, Path A §2 Rung 1)
//   Slot 1: manufacturer root CA public key (for cert verification)
//   Slot 2: device certificate (DER, Path A §4.1 birth cert)
// Slots 3–15: reserved for future arc expansion

#define ATCA_SLOT_DEVICE_KEY    0
#define ATCA_SLOT_MFG_PUBKEY    1
#define ATCA_SLOT_DEVICE_CERT   2

// I2C address of the ATECC608B (default — matches Path A manufacturing spec)
#define ATCA_I2C_ADDR  0x60

// ============================================================================
// SIGNER API
// ============================================================================

// Initialize the ATECC608B on the specified I2C bus.
// sda_pin / scl_pin: GPIO pins for the I2C bus (shared with display if present).
// Returns true if the chip is detected and config zone is locked.
// Returns false if chip not present — all subsequent sign calls return ATCA_NOT_LOCKED.
bool atca_signer_init(int sda_pin, int scl_pin);

// Check if the signer is ready (chip detected and provisioned)
bool atca_signer_ready(void);

// Get the 32-byte device identity (SHA-256 of public key || serial number).
// Used by poac_builder_init() to embed device_id in every PoAC record.
// Returns false if chip not provisioned.
bool atca_signer_get_device_id(uint8_t device_id_out[POAC_DEVICE_ID_SIZE]);

// Sign a completed PoAC body (164 bytes → SHA-256 → ECDSA-P256 sign via slot 0).
// On success: out->sig[0:64] is populated and POAC_FLAG_ATCA_SIGNED is set.
// On failure: sig bytes are zeroed, flag is NOT set, status returned.
// NOTE: this is a BLOCKING call (~60–100 ms for sign + verify on ATECC608B).
//       Call from a task that can tolerate this latency (bridge transport task,
//       NOT the 1 kHz sense task).
atca_sign_status_t atca_signer_sign(poac_record_t* record);

// Verify a signed record against the device public key.
// Returns true if the ECDSA-P256 signature over the 164-byte body is valid.
bool atca_signer_verify(const poac_record_t* record);

// Read the device certificate from slot 2 (DER format, Path A §4.1).
// Used by bridge at enrollment to establish the device identity chain.
bool atca_signer_get_cert(uint8_t* cert_buf, size_t* cert_len, size_t buf_size);

#endif // QORTROLLER_ATCA_SIGNER_H
