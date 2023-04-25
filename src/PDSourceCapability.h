//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#pragma once

#include "PDMessage.h"

/// Power supply type
enum class PDSupplyType {
    /// Fixed supply (Vmin = Vmax)
    fixed = 0,
    /// Battery
    battery = 1,
    /// Variable supply (non-battery)
    variable = 2,
    /// Programmable power supply
    pps = 3
};

/// Power source capability
struct PDSourceCapability {
    /// Supply type (fixed, batttery, variable etc.)
    PDSupplyType supplyType;
    /// Position within message (don't touch)
    uint8_t objPos;
    /// Maximum current (in mA)
    uint16_t maxCurrent;
    /// Maximum voltage (in mV)
    uint16_t maxVoltage;
    /// Minimum voltage (in mV)
    uint16_t minVoltage;

    /// Parses a 'Source Capabilities' message and stores result in 'capabilities'
    static void parseMessage(const PDMessage* message, int& numCapabilities, PDSourceCapability* capabilities);
    /// Creates a data object to for a 'Request' message. 'voltage' and 'maxCurrent' are changed to the effectively
    /// requestd values.
    static uint32_t powerRequestObject(int& index, int& voltage, int& maxCurrent, int numCapabilities,
                                       const PDSourceCapability* capabilities);
    /// Creates a data object to for a 'Request' message. 'voltage' and 'maxCurrent' are changed to the effectively
    /// requestd values.
    static uint32_t powerRequestObjectForCapability(int index, int& voltage, int& maxCurrent, int numCapabilities,
                                                    const PDSourceCapability* capabilities);
    /// Creates a 'Fixed or Variable Request' data object. 'voltage' and 'maxCurrent' are changed to the effectively
    /// requestd values.
    static uint32_t fixedRequestObject(int objPos, int& current);
    /// Creates a 'Programmable Request' data object. 'voltage' and 'maxCurrent' are changed to the effectively requestd
    /// values.
    static uint32_t programmableRequestObject(int objPos, int& voltage, int& current);
};
