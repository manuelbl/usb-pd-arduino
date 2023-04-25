//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#pragma once

#include <stdint.h>

/// USB PD message type
enum class PDMessageType : uint16_t {
    controlGoodCrc = 0x01,
    controlGotoMin = 0x02,
    controlAccept = 0x03,
    controlReject = 0x04,
    controlPing = 0x05,
    controlPsReady = 0x06,
    controlGetSourceCap = 0x07,
    controlGetSinkCap = 0x08,
    controlDrSwap = 0x09,
    controlPrSwap = 0x0a,
    controlVconnSwap = 0x0b,
    controlWait = 0x0c,
    controlSoftReset = 0x0d,
    controlDataReset = 0x0e,
    controlDataResetComplete = 0x0f,
    controlNotSupported = 0x10,
    controlGetSourceCapExtended = 0x11,
    controlGetStatus = 0x12,
    controlFrSwap = 0x13,
    controlGetPpsStatus = 0x14,
    controlGetCountryCodes = 0x15,
    controlGetSinkCapExtended = 0x16,
    dataSourceCapabilities = 0x81,
    dataRequest = 0x82,
    dataBist = 0x83,
    dataSinkCapabilities = 0x84,
    dataBatteryStatus = 0x85,
    dataAlert = 0x86,
    dataGetCountryInfo = 0x87,
    dataEnterUsb = 0x88,
    dataVendorDefined = 0x8f
};

/// USB PD start of packet sequence
enum class PDSOPSequence : uint8_t {
    /// Invalid sequence
    invalid = 0,
    /// SOP (start of packet)
    sop = 1,
    /// SOP' (start of packet prime) 
    sop1 = 2,
    /// SOP'' (start of packet double prime)
    sop2 = 3,
    /// SOP' Debug (start of packet prime debug)
    sop1Debug = 4,
    /// SOP'' Debug (start of packet dobule prime debug)
    sop2Debug = 5,
    /// Hard reset
    hardReset = 6,
    /// Cable reset
    cableReset = 7
};

/**
 * USB Power Delivery Message.
 *
 * This struct is defined with a maximum of 64 data objects.
 * But the actual length of the struct might be shorter.
 * Do not access data objects beyond the number of data objects
 * indicated in the header.
 */
struct PDMessage {
    /// CC line
    uint8_t cc;
    /// Start of packet sequence
    PDSOPSequence sopSequence;
    /// Message Header
    uint16_t header;
    /// Data objects
    uint32_t objects[64];

    /// Gets the message type
    PDMessageType type() const { return static_cast<PDMessageType>(((numObjects() != 0) << 7) | (header & 0x1f)); }
    /// Gets the number of data objects
    int numObjects() const { return (header >> 12) & 0x07; }
    /// Gets the message ID
    int messageId() const { return (header >> 9) & 0x07; }
    /// Gets the specification revision
    int specRev() const { return ((header >> 6) & 0x03) + 1; }
    /// Indicates an extended message
    bool hasExtended() const { return (header & 0x8000) != 0; }
    /// Start of payload (header and objects)
    uint8_t* payload() { return reinterpret_cast<uint8_t*>(&header); }
    /// Start of payload (header and objects)
    const uint8_t* payload() const { return reinterpret_cast<const uint8_t*>(&header); }
    /// Size of payload (header + objects) in bytes
    int payloadSize() const { return 2 + 4 * numObjects(); }
    /// End address of message (word aligned) 
    uint8_t* end() { return word_aligned(reinterpret_cast<uint8_t*>(&objects[0]) + numObjects() * 4); }

    /// Intializes the header for a control message
    void initControl(PDMessageType messageType, int rev = 1) {
        sopSequence = PDSOPSequence::sop;
        header = ((uint16_t)messageType & 0x1f) | 0x40 | ((rev - 1) << 6);
    }
    /// Initializes the header for a data message
    void initData(PDMessageType messageType, int numObjects, int rev = 1) {
        sopSequence = PDSOPSequence::sop;
        header = ((numObjects & 0x07) << 12) | ((uint16_t)messageType & 0x1f) | 0x40 | ((rev - 1) << 6);
    }
    /// Sets the message ID
    void setMessageId(int messageId) { header = (header & ~0x0e00) | ((messageId << 9) & 0x0e00); }

    template <class T> static T* word_aligned(T* addr) {
        return reinterpret_cast<T*>((reinterpret_cast<uint32_t>(addr) + 3) & ~3);
    }

} __attribute__((packed));
