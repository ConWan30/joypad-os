// bridge_transport.h — PoAC record delivery to the QorTroller bridge
// SPDX-License-Identifier: Apache-2.0
//
// Delivers the signed 228-byte PoAC record from the ESP32-S3 to the
// QorTroller bridge service (Python asyncio, running on the gamer's PC/laptop)
// via BLE characteristic notify or WiFi UDP packet.
//
// The bridge is already expecting 228-byte records (228-byte PoAC FROZEN format).
// This module handles the transport layer only — no format change.
//
// TRANSPORT PRIORITY (v1):
//   1. BLE GATT characteristic notify (lower latency, no routing needed)
//   2. WiFi UDP fallback (higher throughput for batch delivery)
//
// The bridge's Python side (bridge/vapi_bridge/main.py) already handles
// the 228-byte record on the USB/hidapi path. The wireless path adds a thin
// reception endpoint — same parse logic, different socket.

#ifndef QORTROLLER_BRIDGE_TRANSPORT_H
#define QORTROLLER_BRIDGE_TRANSPORT_H

#include "poac_types.h"
#include <stdbool.h>
#include <stdint.h>

// ============================================================================
// BLE GATT CONFIGURATION
// ============================================================================
// QorTroller BLE service (custom UUID derived from the V.A.P.I. domain tag)
// UUID format: 128-bit little-endian

// Service UUID: VAPI-POAC-SERVICE-v1
#define QORTROLLER_BLE_SERVICE_UUID \
    { 0x56, 0x41, 0x50, 0x49, 0x50, 0x4F, 0x41, 0x43, \
      0x53, 0x56, 0x43, 0x31, 0x00, 0x00, 0x00, 0x00 }

// Characteristic UUID: POAC record (notify, 228 bytes per notification)
#define QORTROLLER_BLE_POAC_CHAR_UUID \
    { 0x56, 0x41, 0x50, 0x49, 0x50, 0x4F, 0x41, 0x43, \
      0x52, 0x45, 0x43, 0x31, 0x00, 0x00, 0x00, 0x00 }

// Characteristic UUID: Chain hash echo (write-back from bridge after confirm)
#define QORTROLLER_BLE_CHAIN_CHAR_UUID \
    { 0x56, 0x41, 0x50, 0x49, 0x50, 0x4F, 0x41, 0x43, \
      0x43, 0x48, 0x41, 0x4E, 0x00, 0x00, 0x00, 0x00 }

// ============================================================================
// WIFI UDP CONFIGURATION (fallback / high-throughput batch path)
// ============================================================================

#define BRIDGE_UDP_PORT       5748    // QorTroller bridge UDP port
#define BRIDGE_UDP_MAGIC      0x5641  // "VA" — magic for bridge UDP parser
#define BRIDGE_UDP_BATCH_MAX    16    // Max records per UDP packet

// ============================================================================
// TRANSPORT STATUS
// ============================================================================

typedef enum {
    TRANSPORT_OK           = 0,
    TRANSPORT_NOT_CONNECTED = 1,   // No BLE/WiFi connection to bridge
    TRANSPORT_QUEUE_FULL   = 2,    // Delivery queue full (bridge too slow)
    TRANSPORT_SEND_FAILED  = 3,    // BLE notify or UDP send failed
    TRANSPORT_MTU_ERROR    = 4,    // BLE MTU too small for 228-byte payload
} transport_status_t;

// ============================================================================
// TRANSPORT API
// ============================================================================

// Initialize the transport layer. Call after BLE/WiFi stack is up.
// advertise_name: BLE device name shown to the bridge ("QorTroller-XXXX").
// Returns true if BLE advertising started.
bool bridge_transport_init(const char* advertise_name);

// Queue a signed PoAC record for delivery.
// Returns immediately (non-blocking). The transport task sends in background.
// On TRANSPORT_QUEUE_FULL: the oldest queued record is dropped (newest wins).
transport_status_t bridge_transport_send(const poac_record_t* record);

// Called by the BLE GATT stack when the bridge writes a chain hash echo
// (bridge confirms receipt of the previous record's chain hash).
// Updates poac_builder's prev_hash so the next record chains correctly.
void bridge_transport_on_chain_echo(const uint8_t hash[POAC_CHAIN_HASH_SIZE]);

// True if the bridge is currently connected (BLE or WiFi).
bool bridge_transport_connected(void);

// Diagnostic: total records delivered this session
uint32_t bridge_transport_delivered_count(void);

// Diagnostic: total records dropped (queue full or send failed)
uint32_t bridge_transport_drop_count(void);

#endif // QORTROLLER_BRIDGE_TRANSPORT_H
