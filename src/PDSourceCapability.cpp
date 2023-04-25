//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#include "PDSourceCapability.h"

void PDSourceCapability::parseMessage(const PDMessage* message, int& numCapabilities,
                                      PDSourceCapability* capabilities) {
    int n = 0;
    int numDataObjects = message->numObjects();
    // isUnconstrained = false;
    // supportsExtMessage = false;

    for (int index = 0; index < numDataObjects; index++) {
        if (index >= 7)
            break;

        uint32_t capability = message->objects[index];

        PDSupplyType type = static_cast<PDSupplyType>(capability >> 30);
        uint16_t maxCurrent = (capability & 0x3ff) * 10;
        uint16_t minVoltage = ((capability >> 10) & 0x03ff) * 50;
        uint16_t maxVoltage = ((capability >> 20) & 0x03ff) * 50;

        if (type == PDSupplyType::fixed) {
            maxVoltage = minVoltage;

            // Fixed 5V capability contains additional information
            // if (maxVoltage == 5000) {
            //     isUnconstrained = (capability & (1 << 27)) != 0;
            //     supportsExtMessage = (capability & (1 << 24)) != 0;
            // }

        } else if (type == PDSupplyType::pps) {
            if ((capability & (3 << 28)) != 0)
                continue;

            maxCurrent = (capability & 0x007f) * 50;
            minVoltage = ((capability >> 8) & 0x00ff) * 100;
            maxVoltage = ((capability >> 17) & 0x00ff) * 100;
        }

        capabilities[n] = {
            .supplyType = type,
            .objPos = static_cast<uint8_t>(index + 1),
            .maxCurrent = maxCurrent,
            .maxVoltage = maxVoltage,
            .minVoltage = minVoltage,
        };
        n++;
    }

    numCapabilities = n;
}

uint32_t PDSourceCapability::powerRequestObject(int& index, int& voltage, int& maxCurrent, int numCapabilities,
                                                const PDSourceCapability* capabilities) {
    // Lookup fixed voltage capabilities first
    index = -1;
    for (int i = 0; i < numCapabilities; i++) {
        auto cap = capabilities + i;
        if (cap->supplyType == PDSupplyType::fixed && voltage >= cap->minVoltage && voltage <= cap->maxVoltage) {
            index = i;
            if (maxCurrent == 0)
                maxCurrent = cap->maxCurrent;
            break;
        }
    }

    // Lookup PPS capabilites next
    if (index == -1) {
        for (int i = 0; i < numCapabilities; i++) {
            auto cap = capabilities + i;
            if (cap->supplyType == PDSupplyType::pps && voltage >= cap->minVoltage && voltage <= cap->maxVoltage) {
                if (maxCurrent == 0) {
                    maxCurrent = cap->maxCurrent;
                    index = i;
                    break;
                } else if (maxCurrent >= 25 && maxCurrent <= cap->maxCurrent) {
                    index = i;
                    break;
                }
            }
        }
    }

    if (index == -1)
        return 0;

    return powerRequestObjectForCapability(index, voltage, maxCurrent, numCapabilities, capabilities);
}

uint32_t PDSourceCapability::powerRequestObjectForCapability(int index, int& voltage, int& maxCurrent,
                                                             int numCapabilities,
                                                             const PDSourceCapability* capabilities) {
    if (index < 0 || index >= numCapabilities)
        return 0;
    auto cap = capabilities + index;
    if (cap->supplyType != PDSupplyType::fixed && cap->supplyType != PDSupplyType::pps)
        return 0;
    if (voltage < cap->minVoltage || voltage > cap->maxVoltage)
        return 0;
    if (maxCurrent < 25 || maxCurrent > cap->maxCurrent)
        return 0;

    // Create data object for 'request' message
    if (cap->supplyType == PDSupplyType::fixed) {
        return fixedRequestObject(cap->objPos, maxCurrent);
    } else {
        return programmableRequestObject(cap->objPos, voltage, maxCurrent);
    }
}

uint32_t PDSourceCapability::fixedRequestObject(int objPos, int& current) {
    constexpr uint32_t noUsbSuspend = 1 << 24;

    // current in multiples of 10mA
    uint32_t current10 = (current + 5) / 10;
    if (current10 > 0x3ff)
        current10 = 0x3ff;

    uint32_t object = (objPos & 0x07) << 28; // object position
    object |= noUsbSuspend;
    object |= current10 << 10; // operating current
    object |= current10;       // maximum operating current

    current = current10 * 10;

    return object;
}

uint32_t PDSourceCapability::programmableRequestObject(int objPos, int& voltage, int& current) {
    constexpr uint32_t noUsbSuspend = 1 << 24;

    // voltage in multiples of 20mV
    uint32_t voltage20 = (voltage + 10) / 20;
    if (voltage20 > 0x7ff)
        voltage20 = 0x7ff;

    // current in multiples of 50mA
    uint32_t current50 = (current + 25) / 50;
    if (current50 > 0x7f)
        current50 = 0x7f;

    uint32_t object = (objPos & 0x07) << 28; // object position
    object |= noUsbSuspend;
    object |= voltage20 << 9; // output voltage
    object |= current50;      // operating current

    voltage = voltage20 * 20;
    current = current50 * 50;

    return object;
}
