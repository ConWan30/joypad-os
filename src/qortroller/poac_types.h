// poac_types.h — QorTroller Proof-of-Autonomous-Cognition wire types
// SPDX-License-Identifier: Apache-2.0
//
// The 228-byte PoAC record is the FROZEN cryptographic atom of the V.A.P.I.
// protocol. NEVER modify POAC_RECORD_SIZE or the field layout — doing so
// constitutes a hard-fork that invalidates all on-chain verifiers.
//
// Layout: 164-byte signed body || 64-byte ECDSA-P256 signature
// Chain link hash: SHA-256( body[0:164] )  ← 164B only, NOT full 228B

#ifndef QORTROLLER_POAC_TYPES_H
#define QORTROLLER_POAC_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// ============================================================================
// FROZEN CONSTANTS — DO NOT MODIFY
// ============================================================================

#define POAC_RECORD_SIZE     228   // Total wire bytes (body + sig)
#define POAC_BODY_SIZE       164   // Signed body
#define POAC_SIG_SIZE         64   // ECDSA-P256 signature (r || s, 32B each)
#define POAC_DEVICE_ID_SIZE   32   // SHA-256 device identity hash
#define POAC_CHAIN_HASH_SIZE  32   // SHA-256 of previous body

// Domain tag — FROZEN-v1 family identifier, embedded in every record
#define POAC_DOMAIN_TAG      "VAPI-POAC-v1"
#define POAC_DOMAIN_TAG_SIZE  12

// ============================================================================
// RAW SENSOR SAMPLE — captured BEFORE joypad-os normalization
// This is the biometric-fidelity atom. joypad-os never sees this struct;
// it flows from the raw hook → PoAC builder → attestation chain.
// ============================================================================

// Axis identifiers (mirrors joypad-os ANALOG_* but for raw ADC)
typedef enum {
    POAC_AXIS_LX  = 0,   // Left stick X
    POAC_AXIS_LY  = 1,   // Left stick Y
    POAC_AXIS_RX  = 2,   // Right stick X
    POAC_AXIS_RY  = 3,   // Right stick Y
    POAC_AXIS_L2  = 4,   // Left trigger (adaptive force-curve — primary L4 discriminator)
    POAC_AXIS_R2  = 5,   // Right trigger (adaptive force-curve)
    POAC_AXIS_COUNT = 6
} poac_axis_t;

// One raw ADC sample captured at the pre-normalization tap point
typedef struct {
    uint16_t adc_raw[POAC_AXIS_COUNT];  // 12-bit ADC counts (0–4095)
    uint32_t buttons_raw;               // Pre-debounce button bitmask
    int16_t  accel_x;                   // IMU accelerometer X (raw LSB)
    int16_t  accel_y;                   // IMU accelerometer Y (raw LSB)
    int16_t  accel_z;                   // IMU accelerometer Z (raw LSB)
    int16_t  gyro_x;                    // IMU gyroscope X (raw LSB)
    int16_t  gyro_y;                    // IMU gyroscope Y (raw LSB)
    int16_t  gyro_z;                    // IMU gyroscope Z (raw LSB)
    uint64_t timestamp_us;              // Monotonic microseconds since boot
} poac_raw_sample_t;

// ============================================================================
// L4 FEATURE ACCUMULATOR — rolling biometric fingerprint per cognition cycle
// Updated incrementally from raw samples; finalized once per PoAC record.
// ============================================================================

typedef struct {
    // Trigger force-curve features (PRIMARY DISCRIMINATOR per Sensor Stack v2.1)
    float    l2_onset_velocity;         // dADC/dt at trigger onset (ADC/ms)
    float    r2_onset_velocity;         // dADC/dt at trigger onset (ADC/ms)

    // Micro-tremor (IMU accelerometer variance, 4–15 Hz band)
    float    accel_tremor_variance;     // Variance of 64-sample ring buffer
    float    tremor_peak_hz;            // Peak frequency in tremor band (FFT)
    float    tremor_band_power;         // Power in 4–15 Hz band

    // Stick autocorrelation (motor-pattern fingerprint)
    float    stick_autocorr_lag1;       // lag-1 autocorr of L+R stick magnitude
    float    stick_autocorr_lag5;       // lag-5 autocorr

    // Postural gravity fingerprint (AIT surface — circular encoding)
    float    roll_cos;                  // cos(roll) from mean gravity vector
    float    roll_sin;                  // sin(roll)
    float    pitch_cos;                 // cos(pitch)

    // Button timing jitter
    float    press_jitter_variance;     // Normalized IBI variance (human 0.001–0.05)

    // Grip asymmetry (L vs R trigger energy differential)
    float    grip_asymmetry;

    // Sample count this cycle
    uint32_t sample_count;
    uint32_t cycle_start_us;
    uint32_t cycle_end_us;
} poac_l4_features_t;

// ============================================================================
// POAC BODY — 164-byte signed payload (FROZEN layout)
// ============================================================================

typedef struct __attribute__((packed)) {
    // [0:12]  Domain tag
    uint8_t  domain_tag[POAC_DOMAIN_TAG_SIZE];

    // [12:44] Device identity (SHA-256 of ATECC608B public key || serial)
    uint8_t  device_id[POAC_DEVICE_ID_SIZE];

    // [44:76] Chain link to previous record
    uint8_t  prev_hash[POAC_CHAIN_HASH_SIZE];

    // [76:84] Cognition cycle timestamp (ns since Unix epoch, big-endian)
    uint8_t  timestamp_ns_be[8];

    // [84:88] Cycle sequence number
    uint8_t  cycle_seq_be[4];

    // [88:96] L4 Mahalanobis distance (float32, big-endian IEEE 754)
    uint8_t  l4_distance_be[4];
    // [92:96] Humanity probability [0.0, 1.0] (float32, big-endian)
    uint8_t  humanity_prob_be[4];

    // [96:140] Quantized L4 feature vector (11 × float32 big-endian)
    uint8_t  l4_features_be[44];

    // [140:148] Trigger onset velocities L2 || R2 (float32 × 2, big-endian)
    uint8_t  trigger_onset_be[8];

    // [148:156] Tremor peak Hz || tremor band power (float32 × 2)
    uint8_t  tremor_be[8];

    // [156:160] Gameplay activity flag + capture state byte + 2 reserved
    uint8_t  capture_flags[4];

    // [160:164] UWB presence flag (0x01=present, 0x00=absent, 0xFF=no_sensor)
    //           + UWB confidence [0–255] + 2 reserved
    uint8_t  uwb_presence[4];
} poac_body_t;

// Verify at compile time that the body is exactly 164 bytes
_Static_assert(sizeof(poac_body_t) == POAC_BODY_SIZE,
               "poac_body_t must be exactly 164 bytes — FROZEN wire format");

// ============================================================================
// FULL POAC RECORD — 228 bytes
// ============================================================================

typedef struct __attribute__((packed)) {
    poac_body_t body;                  // [0:164]  signed payload
    uint8_t     sig[POAC_SIG_SIZE];   // [164:228] ECDSA-P256 sig (ATECC608B)
} poac_record_t;

_Static_assert(sizeof(poac_record_t) == POAC_RECORD_SIZE,
               "poac_record_t must be exactly 228 bytes — FROZEN wire format");

// ============================================================================
// CAPTURE STATE FLAGS (capture_flags[0])
// ============================================================================

#define POAC_FLAG_GAMEPLAY_ACTIVE   0x01  // trigger_active_fraction > 0
#define POAC_FLAG_CAPTURE_NOMINAL   0x02  // PCC state = NOMINAL
#define POAC_FLAG_HOST_EXCLUSIVE    0x04  // host_state = EXCLUSIVE_USB
#define POAC_FLAG_GIC_INTACT        0x08  // GIC chain_intact = true
#define POAC_FLAG_UWB_ACTIVE        0x10  // QM35825 present and reading
#define POAC_FLAG_ATCA_SIGNED       0x80  // ATECC608B signature valid

#endif // QORTROLLER_POAC_TYPES_H
