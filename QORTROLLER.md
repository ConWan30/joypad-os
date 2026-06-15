# QorTroller × joypad-os — Novel Interoperability

**ConWan30/joypad-os** is the QorTroller fork of the open-source
[joypad-os](https://github.com/joypad-ai/joypad-os) firmware platform.
It adds the **V.A.P.I. biometric pipeline** — making every ESP32-S3 board running
this firmware a potential **DePIN controller node** that cryptographically proves
human presence and gaming biometric identity on-chain.

---

## What joypad-os does

Runs on RP2040 / ESP32-S3 / nRF52840. Reads game controllers (USB HID, BT Classic,
BLE, native SNES/N64/GC), normalizes inputs, routes to output protocols (USB HID,
XInput, BLE, retro-console PIO). Excellent multi-target firmware platform, Apache-2.0.

## What QorTroller adds

A **parallel biometric sense pipeline** that taps the raw ADC values *before*
joypad-os normalizes them — without touching joypad-os's output path at all.

```
Physical sensors (ADC/IMU/GPIO)
         │
         ├──▶ [RAW TAP]  ──▶  QorTroller biometric pipeline
         │         │                │
         │    qortroller_hook.h     │
         │                    poac_builder.c
         │                          │
         │                    228-byte PoAC record
         │                          │
         │                    ATECC608B sign (I2C)
         │                          │
         │                    BLE/WiFi → bridge
         │
         └──▶ joypad-os normalization → USB HID / BLE output (UNCHANGED)
```

The PS5 / host sees a **normal HID device**. QorTroller's pipeline is a pure
side-channel — additive, zero-impact on gameplay.

---

## The 228-byte Proof-of-Autonomous-Cognition (PoAC)

Every cognition cycle (1 second at 1 kHz), the firmware assembles a **228-byte
cryptographic record** and signs it with the ATECC608B secure element:

```
[0:12]   Domain tag    "VAPI-POAC-v1"  (FROZEN-v1 family)
[12:44]  Device ID     SHA-256(ATECC608B pubkey ‖ serial)
[44:76]  Chain hash    SHA-256(previous body[0:164])
[76:84]  Timestamp     nanoseconds, big-endian
[84:88]  Cycle seq     record counter
[88:96]  L4 distance + humanity probability
[96:140] L4 features   11 × float32 (trigger, tremor, grip, posture, jitter)
[140:156] Trigger + tremor telemetry
[156:164] Capture + UWB presence flags
───────── 164-byte signed body ─────────
[164:228] ECDSA-P256 signature (ATECC608B, slot 0)
```

**FROZEN** — this layout never changes. On-chain verifiers (`VAPIPoEPRegistry`,
`TournamentGateV3`, `isFullyEligible()` on IoTeX testnet 4690) depend on it.

---

## Why the raw tap matters (the biometric-fidelity argument)

joypad-os normalizes ADC values to 0–255 with deadzone and debounce. That's
correct for gameplay. But QorTroller's L4 biometric features live in the
pre-normalization signal:

| Feature | Where it lives | Lost after joypad-os normalization? |
|---|---|---|
| Trigger onset velocity (dADC/dt) | Raw 12-bit ADC edge | **YES** — clipped to 8-bit |
| Micro-tremor variance (4–15 Hz) | Raw IMU LSBs | **YES** — not exposed at all |
| Press timing jitter (IBI CV) | Pre-debounce edge timestamps | **YES** — debounced away |
| Stick autocorrelation | Raw ADC sequence | **PARTIALLY** — deadzone erases center |
| AIT postural gravity | Raw IMU accel vector | **YES** — not in joypad-os |

The separation ratio 1.199 (N=37, IoTeX testnet AIT corpus) was measured on raw
USB HID data from a DualSense Edge at 1002 Hz. Biometric fidelity requires the
same signal quality from the native device — which is why the tap fires
pre-normalization.

---

## Novel interoperability: what this combination enables

| joypad-os provides | QorTroller adds | Combined capability |
|---|---|---|
| ESP32-S3 ESP-IDF build system | PoAC wire format + signing | Native DePIN controller node |
| MAX3421E USB host driver | Raw ADC pre-norm tap | Read DualSense Edge raw HID + capture biometrics simultaneously |
| I2C bus (display / expanders) | ATECC608B CryptoAuthLib | Silicon-rooted per-device identity on existing I2C bus |
| UART peer transport | QM35825 UART presence frame | UWB radar embodied-presence gate (Sensor Stack v2.3 Surface 9) |
| BTstack + WiFi stack | BLE GATT PoAC characteristic | Wireless PoAC delivery to bridge (no USB tether required) |
| WS2812 RGB LED | PCC capture health states | Real-time visual: green=NOMINAL, amber=DEGRADED, red=DISCONNECTED |
| Flash config storage (NVS) | L4 calibration matrix + chain hash | Per-device enrollment persistence across power cycles |
| Multi-target (RP2040/ESP32-S3/nRF52840) | `CONFIG_QORTROLLER` flag | Enable on any target without breaking others |

---

## Architecture: how to enable

QorTroller's pipeline is **opt-in via a single CMake flag**. All upstream
joypad-os builds are unaffected.

### ESP32-S3 (primary QorTroller target)

In `esp/sdkconfig.defaults` or your board config:
```
CONFIG_QORTROLLER=y
```

In `esp/main/CMakeLists.txt`, add the qortroller component:
```cmake
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../../src/qortroller")
```

Then build as normal:
```bash
cd esp
source env.sh
idf.py build
```

### What compiles in vs. what stays dormant

| Module | Status | Gate |
|---|---|---|
| `qortroller_hook.c` + `poac_types.h` | **BUILDS** | CONFIG_QORTROLLER=y |
| `poac_builder.c` | **BUILDS** | CONFIG_QORTROLLER=y |
| `sense_task.c` (Core-0 1 kHz loop) | STUB — implement next | CONFIG_QORTROLLER=y |
| `atca_signer.c` (ATECC608B) | STUB — hardware-gated | Physical breakout wired |
| `uwb_presence.c` (QM35825) | STUB — hardware-gated | QM35825DK-05 eval kit |
| `bridge_transport.c` (BLE GATT) | STUB — implement next | BLE MTU ≥ 244 (ATT_MTU=244) |

---

## Build-forward steps

1. **`sense_task.c`** — FreeRTOS task `xTaskCreatePinnedToCore(..., 0)` at 1 ms cadence,
   Core 0 (PRO_CPU). Drives the hook registration and calls `poac_builder_build()` once
   per cycle. ~300 lines.

2. **`bridge_transport.c`** — BLE GATT characteristic (UUID in `bridge_transport.h`)
   that notifies the bridge with the 228-byte PoAC record. The bridge's Python asyncio
   already parses 228-byte records; only the socket type changes (BLE vs USB).
   ~250 lines.

3. **`atca_signer.c`** — CryptoAuthLib `atcab_sign()` wrapper. Hardware-gated on ATECC608B
   breakout wired to ESP32-S3 I2C. ~150 lines (mostly CryptoAuthLib boilerplate).

4. **`uwb_presence.c`** — UART frame parser for QM35825 presence frames.
   Hardware-gated on QM35825DK-05 eval kit (Qorvo partner request submitted 2026-06-14).
   ~120 lines. **Privacy gate: vital-sign mode MUST stay disabled** (see `uwb_presence.h`).

---

## Protocol stack on IoTeX testnet

| Layer | Component | Status |
|---|---|---|
| L1 chain | IoTeX testnet (chainId 4690) | LIVE |
| Contracts | 66 deployed / 58 active (VAPIPoEPRegistry, TournamentGateV3, VAPIProtocolLens) | LIVE |
| Bridge service | Python asyncio + SQLite | LIVE |
| Biometric corpus | AIT separation ratio 1.199, N=37 | LIVE |
| Hardware capture | DualSense Edge USB/hidapi (today) | LIVE |
| Native firmware | **this repo** (ESP32-S3) | **IN BUILD** |

---

## Honest status

This firmware is a **build-forward proposal in active development**, not a shipping
product. Every claim above is graded:

- **VERIFIED**: joypad-os ESP32-S3 chassis, I2C/UART transports, MAX3421E driver
- **BUILT**: raw hook tap, PoAC types, poac_builder L4 accumulator
- **ASPIRATIONAL**: sense_task (1 kHz Core-0), atca_signer (hardware), uwb_presence (hardware)
- **MEASUREMENT-PENDING**: Stage A empirical unknowns #1 (trigger Mahalanobis > 1.0)
  and #4 (stick same-batch separability ≥ 20%) must pass before LIVE claims

See `docs/qortroller-joypad-os-integration-analysis.md` (in the parent repo) for the
full graded analysis.

---

## License

joypad-os upstream: Apache-2.0 (Robert Dale Smith / joypad-ai)  
QorTroller additions (`src/qortroller/`): Apache-2.0 (ConWan30 / QorTroller)  
Protocol: V.A.P.I. — Verifiable Autonomous Physical Intelligence
