// poac_builder.c — PoAC record builder implementation
// SPDX-License-Identifier: Apache-2.0
//
// Runs on ESP32-S3 Core 0, pinned FreeRTOS task.
// All floating-point math uses the ESP32-S3's hardware FPU (Xtensa LX7).

#include "poac_builder.h"
#include "qortroller_hook.h"
#include "platform/platform.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

// ============================================================================
// RING BUFFER
// ============================================================================

static poac_raw_sample_t _ring[POAC_RING_DEPTH];
static volatile uint32_t _ring_head = 0;  // write index (producer: hook ISR/task)
static volatile uint32_t _ring_tail = 0;  // read index  (consumer: build task)
static volatile uint32_t _drop_count = 0;

static inline uint32_t _ring_used(void) {
    return (_ring_head - _ring_tail) & (POAC_RING_DEPTH - 1);
}

static inline bool _ring_full(void) {
    return _ring_used() >= (POAC_RING_DEPTH - 1);
}

// ============================================================================
// BUILDER STATE
// ============================================================================

static uint8_t  _device_id[POAC_DEVICE_ID_SIZE];
static uint8_t  _prev_hash[POAC_CHAIN_HASH_SIZE];
static uint32_t _cycle_seq     = 0;
static uint32_t _sample_count  = 0;
static uint32_t _drop_snapshot = 0;

// L4 accumulators (rolling per cognition cycle)
static poac_l4_features_t _acc;

// Calibration parameters (loaded from NVS flash after enrollment)
static bool  _calib_loaded = false;
static float _calib_mean[POAC_L4_FEATURE_COUNT];
static float _calib_cov_inv[POAC_L4_FEATURE_COUNT][POAC_L4_FEATURE_COUNT];

// Last L2/R2 ADC values for onset-velocity computation
static uint16_t _prev_l2_adc = 0;
static uint16_t _prev_r2_adc = 0;
static uint64_t _prev_ts_us  = 0;

// IMU tremor ring (for FFT — stores accel magnitude detrended)
static float    _tremor_ring[POAC_TREMOR_FFT_WIN];
static uint32_t _tremor_idx  = 0;
static float    _tremor_sum  = 0.0f;  // running sum for mean subtraction

// Button inter-press intervals for jitter computation
#define MAX_IBI 128
static uint32_t _ibi_buf[MAX_IBI];
static uint8_t  _ibi_count  = 0;
static uint32_t _prev_press_ts = 0;
static uint32_t _prev_buttons  = 0;

// ============================================================================
// PUBLIC API
// ============================================================================

void poac_builder_init(const uint8_t device_id[POAC_DEVICE_ID_SIZE]) {
    memcpy(_device_id, device_id, POAC_DEVICE_ID_SIZE);
    memset(_prev_hash, 0, POAC_CHAIN_HASH_SIZE);
    memset(&_acc, 0, sizeof(_acc));
    memset(_tremor_ring, 0, sizeof(_tremor_ring));
    _cycle_seq    = 0;
    _ring_head    = 0;
    _ring_tail    = 0;
    _drop_count   = 0;
    _sample_count = 0;
    printf("[poac] builder init — device_id[0..3]=%02x%02x%02x%02x\n",
           device_id[0], device_id[1], device_id[2], device_id[3]);
}

void poac_builder_on_sample(const poac_raw_sample_t* sample) {
    if (!sample) return;

    // Ring buffer insert (lock-free single-producer)
    if (_ring_full()) {
        _drop_count++;
        return;
    }
    _ring[_ring_head & (POAC_RING_DEPTH - 1)] = *sample;
    _ring_head++;
    _sample_count++;

    // -----------------------------------------------------------------------
    // Incremental L4 feature accumulation (done here so the build step is fast)
    // -----------------------------------------------------------------------

    // 1. Trigger onset velocity — dADC/dt at the first rising edge
    uint64_t dt_us = sample->timestamp_us - _prev_ts_us;
    if (dt_us > 0 && dt_us < 5000) {  // ignore gaps > 5 ms (bridge disconnect)
        float dt_ms = dt_us / 1000.0f;

        int16_t d_l2 = (int16_t)sample->adc_raw[POAC_AXIS_L2] - (int16_t)_prev_l2_adc;
        int16_t d_r2 = (int16_t)sample->adc_raw[POAC_AXIS_R2] - (int16_t)_prev_r2_adc;

        if (d_l2 > 50 && _prev_l2_adc < 200) {
            // Rising edge on L2 — record onset velocity (fastest observed wins)
            float v = (float)d_l2 / dt_ms;
            if (v > _acc.l2_onset_velocity) _acc.l2_onset_velocity = v;
        }
        if (d_r2 > 50 && _prev_r2_adc < 200) {
            float v = (float)d_r2 / dt_ms;
            if (v > _acc.r2_onset_velocity) _acc.r2_onset_velocity = v;
        }
    }
    _prev_l2_adc = sample->adc_raw[POAC_AXIS_L2];
    _prev_r2_adc = sample->adc_raw[POAC_AXIS_R2];
    _prev_ts_us  = sample->timestamp_us;

    // 2. Micro-tremor (accel magnitude into ring)
    float ax = sample->accel_x / 16384.0f;  // normalise to g (ICM-42688-P)
    float ay = sample->accel_y / 16384.0f;
    float az = sample->accel_z / 16384.0f;
    float mag = sqrtf(ax*ax + ay*ay + az*az);

    _tremor_sum -= _tremor_ring[_tremor_idx];
    _tremor_ring[_tremor_idx] = mag;
    _tremor_sum += mag;
    _tremor_idx = (_tremor_idx + 1) & (POAC_TREMOR_FFT_WIN - 1);

    // Running variance (Welford's online)
    float mean = _tremor_sum / POAC_TREMOR_FFT_WIN;
    float delta = mag - mean;
    _acc.accel_tremor_variance += delta * delta / POAC_TREMOR_FFT_WIN;

    // 3. Stick autocorrelation accumulation — deferred to build step
    //    (requires full cycle buffer; too expensive per-sample)

    // 4. Button inter-press interval
    uint32_t pressed_now  = sample->buttons_raw & ~_prev_buttons;  // new presses
    if (pressed_now && _prev_press_ts > 0 && _ibi_count < MAX_IBI) {
        uint32_t ibi = (uint32_t)(sample->timestamp_us - _prev_press_ts);
        _ibi_buf[_ibi_count++] = ibi;
    }
    if (pressed_now) _prev_press_ts = (uint32_t)sample->timestamp_us;
    _prev_buttons = sample->buttons_raw;

    // 5. Postural gravity (running mean of accel unit vector)
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm > 0.5f) {
        float gx = ax / norm, gy = ay / norm, gz = az / norm;
        // Accumulate for mean; finalize in build step
        _acc.roll_cos  += gx;
        _acc.roll_sin  += gy;
        _acc.pitch_cos += gz;
    }

    _acc.sample_count++;
}

void poac_builder_set_prev_hash(const uint8_t hash[POAC_CHAIN_HASH_SIZE]) {
    memcpy(_prev_hash, hash, POAC_CHAIN_HASH_SIZE);
}

void poac_builder_reset(void) {
    memset(&_acc, 0, sizeof(_acc));
    memset(_tremor_ring, 0, sizeof(_tremor_ring));
    _tremor_idx   = 0;
    _tremor_sum   = 0.0f;
    _sample_count = 0;
    _drop_count   = 0;
    _ibi_count    = 0;
    _prev_press_ts = 0;
    printf("[poac] builder reset\n");
}

uint32_t poac_builder_sample_count(void) { return _sample_count; }
uint32_t poac_builder_drop_count(void)   { return _drop_count;   }

// ============================================================================
// L4 FEATURE COMPUTATION
// ============================================================================

void poac_compute_l4_features(const poac_l4_features_t* acc,
                               float out[POAC_L4_FEATURE_COUNT]) {
    uint32_t n = acc->sample_count > 0 ? acc->sample_count : 1;

    out[0]  = acc->l2_onset_velocity;
    out[1]  = acc->r2_onset_velocity;
    out[2]  = acc->accel_tremor_variance;
    out[3]  = acc->tremor_peak_hz;
    out[4]  = acc->tremor_band_power;
    out[5]  = acc->stick_autocorr_lag1;
    out[6]  = acc->stick_autocorr_lag5;
    out[7]  = acc->roll_cos  / n;    // mean postural gravity
    out[8]  = acc->roll_sin  / n;
    out[9]  = acc->pitch_cos / n;
    out[10] = acc->press_jitter_variance;
}

float poac_mahalanobis(const float features[POAC_L4_FEATURE_COUNT]) {
    if (!_calib_loaded) return 0.0f;

    float d[POAC_L4_FEATURE_COUNT];
    for (int i = 0; i < POAC_L4_FEATURE_COUNT; i++) {
        d[i] = features[i] - _calib_mean[i];
    }

    float dist = 0.0f;
    for (int i = 0; i < POAC_L4_FEATURE_COUNT; i++) {
        float inner = 0.0f;
        for (int j = 0; j < POAC_L4_FEATURE_COUNT; j++) {
            inner += _calib_cov_inv[i][j] * d[j];
        }
        dist += d[i] * inner;
    }
    return sqrtf(fabsf(dist));
}

float poac_humanity_prob(float dist) {
    // Matches bridge formula: sigmoid centered at calibration threshold
    // anomaly_threshold = 7.009 (Phase 57 L4 calibration baseline)
    static const float ANOMALY_THRESHOLD = 7.009f;
    float z = (dist - ANOMALY_THRESHOLD) / 2.0f;
    return 1.0f / (1.0f + expf(z));
}

// ============================================================================
// RECORD ASSEMBLY
// ============================================================================

// Write a float32 as 4 big-endian bytes
static void write_be_f32(uint8_t* dst, float v) {
    uint32_t bits;
    memcpy(&bits, &v, 4);
    dst[0] = (bits >> 24) & 0xFF;
    dst[1] = (bits >> 16) & 0xFF;
    dst[2] = (bits >>  8) & 0xFF;
    dst[3] = (bits      ) & 0xFF;
}

// Write a uint64 as 8 big-endian bytes
static void write_be_u64(uint8_t* dst, uint64_t v) {
    for (int i = 7; i >= 0; i--) {
        dst[i] = v & 0xFF;
        v >>= 8;
    }
}

// Write a uint32 as 4 big-endian bytes
static void write_be_u32(uint8_t* dst, uint32_t v) {
    dst[0] = (v >> 24) & 0xFF;
    dst[1] = (v >> 16) & 0xFF;
    dst[2] = (v >>  8) & 0xFF;
    dst[3] = (v      ) & 0xFF;
}

poac_build_status_t poac_builder_build(poac_record_t* out) {
    if (!out) return POAC_BUILD_INSUFFICIENT;
    if (_acc.sample_count < POAC_CYCLE_SAMPLES / 2) {
        return POAC_BUILD_INSUFFICIENT;
    }

    // UWB presence gate (advisory — only block if sensor is installed AND absent)
    uwb_presence_state_t pres = qortroller_uwb_get_state();
    if (pres == UWB_PRESENCE_ABSENT) {
        printf("[poac] cycle %u skipped — UWB reports human absent\n", _cycle_seq);
        return POAC_BUILD_NO_PRESENCE;
    }

    // Gameplay gate (skip pure-menu cycles: no trigger activity)
    bool gameplay_active = (_acc.l2_onset_velocity > 0.0f || _acc.r2_onset_velocity > 0.0f);

    // Finalize IBI variance
    if (_ibi_count >= 4) {
        float ibi_mean = 0.0f;
        for (int i = 0; i < _ibi_count; i++) ibi_mean += _ibi_buf[i];
        ibi_mean /= _ibi_count;
        float ibi_var = 0.0f;
        for (int i = 0; i < _ibi_count; i++) {
            float d = _ibi_buf[i] - ibi_mean;
            ibi_var += d * d;
        }
        _acc.press_jitter_variance = (ibi_var / _ibi_count) / (ibi_mean * ibi_mean + 1.0f);
    }

    // Compute L4 features
    float features[POAC_L4_FEATURE_COUNT];
    poac_compute_l4_features(&_acc, features);

    float dist  = poac_mahalanobis(features);
    float hprob = poac_humanity_prob(dist);

    // Assemble body
    poac_body_t* body = &out->body;
    memset(out, 0, sizeof(*out));

    memcpy(body->domain_tag, POAC_DOMAIN_TAG, POAC_DOMAIN_TAG_SIZE);
    memcpy(body->device_id,  _device_id,      POAC_DEVICE_ID_SIZE);
    memcpy(body->prev_hash,  _prev_hash,       POAC_CHAIN_HASH_SIZE);

    uint64_t now_ns = (uint64_t)platform_get_time_us() * 1000ULL;
    write_be_u64(body->timestamp_ns_be, now_ns);
    write_be_u32(body->cycle_seq_be,   _cycle_seq);
    write_be_f32(body->l4_distance_be, dist);
    write_be_f32(body->humanity_prob_be, hprob);

    // Pack 11 L4 features (4B each, big-endian) into 44-byte slot
    for (int i = 0; i < POAC_L4_FEATURE_COUNT; i++) {
        write_be_f32(&body->l4_features_be[i * 4], features[i]);
    }

    write_be_f32(&body->trigger_onset_be[0], _acc.l2_onset_velocity);
    write_be_f32(&body->trigger_onset_be[4], _acc.r2_onset_velocity);
    write_be_f32(&body->tremor_be[0],        _acc.tremor_peak_hz);
    write_be_f32(&body->tremor_be[4],        _acc.tremor_band_power);

    body->capture_flags[0] =
        (gameplay_active               ? POAC_FLAG_GAMEPLAY_ACTIVE : 0) |
        POAC_FLAG_CAPTURE_NOMINAL |  // updated by sense task from PCC state
        POAC_FLAG_HOST_EXCLUSIVE  |  // updated by sense task from host inference
        POAC_FLAG_GIC_INTACT;        // updated by sense task from chain status

    body->uwb_presence[0] = (uint8_t)pres;
    body->uwb_presence[1] = qortroller_uwb_get_confidence();
    if (pres != UWB_PRESENCE_NO_SENSOR) {
        body->capture_flags[0] |= POAC_FLAG_UWB_ACTIVE;
    }

    // Signature is filled by atca_signer.c after this call returns
    // body->capture_flags[0] |= POAC_FLAG_ATCA_SIGNED;  // set by signer

    // Advance cycle
    _cycle_seq++;
    _drop_snapshot = _drop_count;
    poac_builder_reset();

    printf("[poac] cycle %u built — samples=%u drops=%u dist=%.3f hprob=%.3f\n",
           _cycle_seq - 1, _acc.sample_count, _drop_snapshot, dist, hprob);

    return POAC_BUILD_OK;
}
