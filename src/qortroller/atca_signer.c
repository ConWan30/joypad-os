// atca_signer.c — ATECC608B signing implementation (hardware-gated)
// SPDX-License-Identifier: Apache-2.0
//
// PATH A ARC 2 — hardware-gated until the ATECC608B/608C-class breakout is
// physically wired to the ESP32-S3 I2C bus (BOM C2, SDA/SCL shared with display).
//
// BUILD BEHAVIOR:
//   Without CRYPTOAUTHLIB_PRESENT defined:
//     All functions compile cleanly. atca_signer_init() returns false,
//     atca_signer_sign() returns ATCA_NOT_LOCKED with sig bytes zeroed.
//     This is the HONEST FAIL-OPEN posture: the firmware runs, the pipeline
//     runs, records are delivered unsigned — the bridge marks POAC_FLAG_ATCA_SIGNED
//     as absent and treats the record as a software-signed fallback.
//
//   With CRYPTOAUTHLIB_PRESENT defined (Arc 2 hardware lands):
//     Real CryptoAuthLib atcab_* calls are compiled in.
//     POLLING-BASED TIMING IS MANDATORY per Path A manufacturing spec §2
//     (ATECC608B/608C family — latency differs by variant; polling is
//     forward-compatible where fixed-delay firmware is not).
//
// ATECC608B FAMILY DISCIPLINE (HWFL-1 Cycle 16 spec amendment):
//   - ATECC608A: NRND — do NOT spec for new designs
//   - ATECC608B: Active, drop-in per Microchip AN2237
//   - ATECC608C-TFLXTLS: Active, TrustFLEX pre-provisioned (Rung 3 target)
//   - Key slot 0: device identity P-256 private key (never readable, sign-only)
//   - Key slot 1: manufacturer root CA public key
//   - Key slot 2: device birth certificate (DER, Path A §4.1)

#include "atca_signer.h"
#include "platform/platform.h"
#include <string.h>
#include <stdio.h>

// ============================================================================
// CRYPTOAUTHLIB SHIM
// When CryptoAuthLib is not linked (pre-Arc-2), all atcab_* calls are stubs
// that return a "not connected" error code. The public API of this file is
// identical in both cases — callers never see the difference.
// ============================================================================

#ifdef CRYPTOAUTHLIB_PRESENT
#include "cryptoauthlib.h"
#else
// Stub type and error codes matching CryptoAuthLib's actual values
typedef int ATCA_STATUS;
#define ATCA_SUCCESS           ((ATCA_STATUS)0x00)
#define ATCA_NOT_LOCKED        ((ATCA_STATUS)0x01)
#define ATCA_UNIMPLEMENTED     ((ATCA_STATUS)0xF1)

// No-op stubs so the linker is satisfied without the library
static ATCA_STATUS atcab_init(void* cfg)       { (void)cfg; return ATCA_NOT_LOCKED; }
static ATCA_STATUS atcab_info(uint8_t* r)      { (void)r;   return ATCA_NOT_LOCKED; }
static ATCA_STATUS atcab_sha(uint16_t len, const uint8_t* d, uint8_t* digest) {
    (void)len; (void)d; (void)digest; return ATCA_NOT_LOCKED;
}
static ATCA_STATUS atcab_sign(uint16_t slot, const uint8_t* digest, uint8_t* sig) {
    (void)slot; (void)digest; (void)sig; return ATCA_NOT_LOCKED;
}
static ATCA_STATUS atcab_verify_extern(const uint8_t* m, const uint8_t* s,
                                        const uint8_t* pub, bool* ok) {
    (void)m; (void)s; (void)pub; (void)ok; return ATCA_NOT_LOCKED;
}
static ATCA_STATUS atcab_get_pubkey(uint16_t slot, uint8_t* pub) {
    (void)slot; (void)pub; return ATCA_NOT_LOCKED;
}
static ATCA_STATUS atcab_read_serial_number(uint8_t* sn) {
    (void)sn; return ATCA_NOT_LOCKED;
}
static ATCA_STATUS atcab_read_bytes_zone(uint8_t zone, uint16_t slot,
                                          size_t offset, uint8_t* data, size_t len) {
    (void)zone; (void)slot; (void)offset; (void)data; (void)len;
    return ATCA_NOT_LOCKED;
}
#endif // CRYPTOAUTHLIB_PRESENT

// ============================================================================
// SIGNER STATE
// ============================================================================

static bool _ready    = false;  // chip detected and config zone locked
static bool _has_key  = false;  // slot 0 provisioned

// 32-byte device identity: SHA-256(pubkey_64B || serial_9B)
static uint8_t _device_id[POAC_DEVICE_ID_SIZE];

// ============================================================================
// PRIVATE HELPERS
// ============================================================================

// Compute device_id = SHA-256(ATECC608B public key [64B] || serial number [9B])
// This is the same formula as bridge/vapi_bridge/device_birth_cert.py DeviceBirthCertificate
static bool _compute_device_id(void) {
    uint8_t pubkey[64] = {0};
    uint8_t serial[9]  = {0};

    ATCA_STATUS s1 = atcab_get_pubkey(ATCA_SLOT_DEVICE_KEY, pubkey);
    ATCA_STATUS s2 = atcab_read_serial_number(serial);

    if (s1 != ATCA_SUCCESS || s2 != ATCA_SUCCESS) {
        return false;
    }

    // SHA-256(pubkey || serial) — using CryptoAuthLib's on-chip SHA engine
    uint8_t preimage[73];
    memcpy(preimage,      pubkey, 64);
    memcpy(preimage + 64, serial, 9);

    ATCA_STATUS s3 = atcab_sha(sizeof(preimage), preimage, _device_id);
    return (s3 == ATCA_SUCCESS);
}

// ============================================================================
// PUBLIC API
// ============================================================================

bool atca_signer_init(int sda_pin, int scl_pin) {
    _ready   = false;
    _has_key = false;
    memset(_device_id, 0, sizeof(_device_id));

#ifdef CRYPTOAUTHLIB_PRESENT
    // Build ESP32 I2C CryptoAuthLib config with POLLING-BASED timing (mandatory)
    ATCAIfaceCfg cfg = cfg_ateccx08a_i2c_default;
    cfg.atcai2c.address = ATCA_I2C_ADDR;
    cfg.atcai2c.bus     = 0;  // ESP-IDF I2C port 0
    cfg.atcai2c.baud    = 400000;
    cfg.wake_delay      = 1500;
    cfg.rx_retries      = 20;

    ATCA_STATUS s = atcab_init(&cfg);
    if (s != ATCA_SUCCESS) {
        printf("[atca] init failed (0x%02x) — chip not detected on I2C 0x%02x\n",
               s, ATCA_I2C_ADDR);
        return false;
    }

    // Verify chip is alive
    uint8_t info[4] = {0};
    s = atcab_info(info);
    if (s != ATCA_SUCCESS) {
        printf("[atca] info command failed (0x%02x)\n", s);
        return false;
    }
    printf("[atca] ATECC608B detected — revision 0x%02x%02x%02x%02x\n",
           info[0], info[1], info[2], info[3]);

    // Compute device identity from public key + serial
    if (!_compute_device_id()) {
        printf("[atca] WARNING: device_id not computable (key slot 0 not provisioned)\n");
        _has_key = false;
    } else {
        _has_key = true;
        printf("[atca] device_id[0..3]=%02x%02x%02x%02x\n",
               _device_id[0], _device_id[1], _device_id[2], _device_id[3]);
    }

    _ready = true;
#else
    (void)sda_pin;
    (void)scl_pin;
    printf("[atca] CryptoAuthLib not present — stub mode (ATCA_NOT_LOCKED)\n");
    printf("[atca] Path A Arc 2 hardware-gated: wire ATECC608B to I2C SDA/SCL\n");
#endif

    return _ready;
}

bool atca_signer_ready(void) {
    return _ready && _has_key;
}

bool atca_signer_get_device_id(uint8_t device_id_out[POAC_DEVICE_ID_SIZE]) {
    if (!_ready || !_has_key) return false;
    memcpy(device_id_out, _device_id, POAC_DEVICE_ID_SIZE);
    return true;
}

atca_sign_status_t atca_signer_sign(poac_record_t* record) {
    if (!record) return ATCA_SIGN_FAILED;

    // Zero the signature field before any return
    memset(record->sig, 0, POAC_SIG_SIZE);

    if (!_ready) {
        return ATCA_NOT_LOCKED;
    }

#ifdef CRYPTOAUTHLIB_PRESENT
    // Hash the 164-byte body (SHA-256 on-chip for consistency with off-chip verifier)
    uint8_t digest[32] = {0};
    ATCA_STATUS s = atcab_sha(POAC_BODY_SIZE, (const uint8_t*)&record->body, digest);
    if (s != ATCA_SUCCESS) {
        printf("[atca] SHA failed (0x%02x)\n", s);
        return ATCA_SIGN_FAILED;
    }

    // ECDSA-P256 sign via key slot 0 (polling-based timing — forward-compatible)
    uint8_t sig_raw[64] = {0};
    s = atcab_sign(ATCA_SLOT_DEVICE_KEY, digest, sig_raw);
    if (s != ATCA_SUCCESS) {
        printf("[atca] sign failed (0x%02x)\n", s);
        return ATCA_SIGN_FAILED;
    }

    // Verify before committing (belt-and-suspenders against transient I2C errors)
    uint8_t pubkey[64] = {0};
    atcab_get_pubkey(ATCA_SLOT_DEVICE_KEY, pubkey);
    bool verified = false;
    s = atcab_verify_extern(digest, sig_raw, pubkey, &verified);
    if (s != ATCA_SUCCESS || !verified) {
        printf("[atca] post-sign verify failed (0x%02x, verified=%d)\n", s, verified);
        return ATCA_VERIFY_FAILED;
    }

    memcpy(record->sig, sig_raw, POAC_SIG_SIZE);
    return ATCA_SIGN_OK;
#else
    // Honest stub: signature bytes stay zero, ATCA_NOT_LOCKED returned.
    // The bridge checks POAC_FLAG_ATCA_SIGNED; unsigned records are still
    // valid for L4 biometric analysis, just not silicon-rooted.
    return ATCA_NOT_LOCKED;
#endif
}

bool atca_signer_verify(const poac_record_t* record) {
    if (!record || !_ready) return false;

#ifdef CRYPTOAUTHLIB_PRESENT
    uint8_t digest[32] = {0};
    atcab_sha(POAC_BODY_SIZE, (const uint8_t*)&record->body, digest);

    uint8_t pubkey[64] = {0};
    atcab_get_pubkey(ATCA_SLOT_DEVICE_KEY, pubkey);

    bool verified = false;
    ATCA_STATUS s = atcab_verify_extern(digest, record->sig, pubkey, &verified);
    return (s == ATCA_SUCCESS && verified);
#else
    return false;
#endif
}

bool atca_signer_get_cert(uint8_t* cert_buf, size_t* cert_len, size_t buf_size) {
    if (!cert_buf || !cert_len || !_ready) return false;

#ifdef CRYPTOAUTHLIB_PRESENT
    // DER certificate in slot 2 — read up to buf_size bytes
    // CryptoAuthLib stores compressed certs; full DER needs reconstruction
    // via atcacert_read_cert() (atcacert.h). Stub here for Arc 2.
    *cert_len = 0;
    ATCA_STATUS s = atcab_read_bytes_zone(ATCA_ZONE_DATA, ATCA_SLOT_DEVICE_CERT,
                                           0, cert_buf, buf_size);
    if (s != ATCA_SUCCESS) return false;
    *cert_len = buf_size;  // actual length from atcacert_read_cert in Arc 2
    return true;
#else
    *cert_len = 0;
    return false;
#endif
}
