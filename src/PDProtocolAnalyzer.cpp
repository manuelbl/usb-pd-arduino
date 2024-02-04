//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#include <Arduino.h>
#include "PDProtocolAnalyzer.h"
#include "PDController.h"

static const char* const ControlMessageNames[] = {
    [0] = nullptr,
    [(int)PDMessageType::controlGoodCrc] = "GoodCRC",
    [(int)PDMessageType::controlGotoMin] = "GotoMin",
    [(int)PDMessageType::controlAccept] = "Accept",
    [(int)PDMessageType::controlReject] = "Reject",
    [(int)PDMessageType::controlPing] = "Ping",
    [(int)PDMessageType::controlPsReady] = "PS_Ready",
    [(int)PDMessageType::controlGetSourceCap] = "Get_Source_Cap",
    [(int)PDMessageType::controlGetSinkCap] = "Get_Sink_Cap",
    [(int)PDMessageType::controlDrSwap] = "DR_Swap",
    [(int)PDMessageType::controlPrSwap] = "PR_Swap",
    [(int)PDMessageType::controlVconnSwap] = "VCONN_Swap",
    [(int)PDMessageType::controlWait] = "Wait",
    [(int)PDMessageType::controlSoftReset] = "Soft Reset",
    [(int)PDMessageType::controlDataReset] = "Data_Reset",
    [(int)PDMessageType::controlDataResetComplete] = "Data_Reset_Complete",
    [(int)PDMessageType::controlNotSupported] = "Not_Supported",
    [(int)PDMessageType::controlGetSourceCapExtended] = "Get_Source_Cap_Ext",
    [(int)PDMessageType::controlGetStatus] = "Get_Status",
    [(int)PDMessageType::controlFrSwap] = "FR_Swap",
    [(int)PDMessageType::controlGetPpsStatus] = "Get_PPS_Status",
    [(int)PDMessageType::controlGetCountryCodes] = "Get_Country_Codes",
    [(int)PDMessageType::controlGetSinkCapExtended] = "Get_Sink_Cap_Ext",
};

static const char* const DataMessageNames[] = {
    [0] = nullptr,
    [(int)PDMessageType::dataSourceCapabilities - 0x80] = "Source Capabilities",
    [(int)PDMessageType::dataRequest - 0x80] = "Request",
    [(int)PDMessageType::dataBist - 0x80] = "BIST",
    [(int)PDMessageType::dataSinkCapabilities - 0x80] = "Sink Capabilities",
    [(int)PDMessageType::dataBatteryStatus - 0x80] = "Battery_Status",
    [(int)PDMessageType::dataAlert - 0x80] = "Alert",
    [(int)PDMessageType::dataGetCountryInfo - 0x80] = "Get_Country_Info",
    [(int)PDMessageType::dataEnterUsb - 0x80] = "Enter_USB",
    [0x09] = nullptr,
    [0x0a] = nullptr,
    [0x0b] = nullptr,
    [0x0c] = nullptr,
    [0x0d] = nullptr,
    [0x0e] = nullptr,
    [(int)PDMessageType::dataVendorDefined - 0x80] = "Vendor Defined",
};

static const char* const SOPSequenceNames[] = {
    [0] = "INV",
    [(int)PDSOPSequence::sop] = "SOP",
    [(int)PDSOPSequence::sop1] = "SOP'",
    [(int)PDSOPSequence::sop2] = "SOP''",
    [(int)PDSOPSequence::sop1Debug] = "SOP1D",
    [(int)PDSOPSequence::sop2Debug] = "SOP2D",
};


USBPDProtocolAnalyzer PDProtocolAnalyzer;

USBPDProtocolAnalyzer::USBPDProtocolAnalyzer() {
    memset(&capabilities, 0, sizeof(capabilities));
}

void USBPDProtocolAnalyzer::poll() {
    auto logEntry = PowerController.popLogEntry();
    if (logEntry == nullptr)
        return;

    Serial.printf("%9lu  ", logEntry->time);

    switch (logEntry->type) {
    case PDLogEntryType::messageReceived:
        Serial.print("RX: ");
        printMessage(logEntry->message);
        break;
    case PDLogEntryType::sinkSourceConnected:
        Serial.printf("Connected: CC%d", PowerController.ccPin);
        Serial.println();
        break;
    case PDLogEntryType::sinkSourceDisconnected:
        Serial.println("Disconnected");
        break;
    case PDLogEntryType::error:
        Serial.println("Error");
        break;
    case PDLogEntryType::hardReset:
        Serial.println("--- Hard Reset");
        break;
    case PDLogEntryType::cableReset:
        Serial.println("--- Cable Reset");
        break;
    case PDLogEntryType::transmissionStarted:
        Serial.print("TX: ");
        printMessage(logEntry->message);
        break;
    case PDLogEntryType::transmissionCompleted:
        Serial.println("TX: completed");
        break;
    case PDLogEntryType::transmissionFailed:
        Serial.println("TX: failed");
        break;
    }
}

void USBPDProtocolAnalyzer::printMessage(const PDMessage* message) {
    Serial.printf("CC%d %-5s %-7s %-20s %d  %04x",
            message->cc, getSOPSequenceName(message->sopSequence),
            getSender(message),getMessageName(message->type()),
            message->messageId(), message->header);

    int numObjects = message->numObjects();
    for (int i = 0; i < numObjects; i++)
        Serial.printf(" %08x", message->objects[i]);
    
    Serial.println();

    if (numObjects > 0) {
        switch (message->type()) {
            case PDMessageType::dataRequest:
                printRequestDetails(message);
                break;
            case PDMessageType::dataSourceCapabilities:
                printCapabilitiesDetails(message);
                break;
            default:
                ;
        }
    }
}

void USBPDProtocolAnalyzer::printCapabilitiesDetails(const PDMessage* message) {
    auto numbObjects = message->numObjects();

    // remember source capabilities
    capabilities.header = message->header;
    memcpy(&capabilities.objects, &message->objects, numbObjects * 4);

    for (int i = 0; i < numbObjects; i += 1) {
        auto object = message->objects[i];
        int supplyType = (object >> 30) & 0x03;

        Serial.printf("  %2d:", i + 1);

        if (supplyType == 0) {
            // fixed supply: Vmin = Vmax
            int maxCurrent = (object & 0x3ff) * 10;
            int maxVoltage = ((object >> 10) & 0x3ff) * 50;
            Serial.printf("  Fixed                   %6dmV (fix) %6dmA (max)  ", maxVoltage, maxCurrent);
            if ((object & (1 << 24)) != 0) Serial.print("extended msg, ");
            if ((object & (1 << 25)) != 0) Serial.print("dual-role data, ");
            if ((object & (1 << 26)) != 0) Serial.print("USB comm capable, ");
            if ((object & (1 << 27)) != 0) Serial.print("unconstrained power, ");
            if ((object & (1 << 28)) != 0) Serial.print("USB suspend, ");
            if ((object & (1 << 29)) != 0) Serial.print("dual-role power");

        } else if (supplyType == 1) {
            // battery
            int maxCurrent = (object & 0x3ff) * 10;
            int minVoltage = ((object >> 10) & 0x3ff) * 50;
            int maxVoltage = ((object >> 20) & 0x3ff) * 50;
            Serial.printf("  Battery  %6dmV (min) %6dmV (max) %6dmA (max)", minVoltage, maxVoltage, maxCurrent);

        } else if (supplyType == 2) {
            // variable
            int maxPower = (object & 0x3ff) * 250;
            int minVoltage = ((object >> 10) & 0x3ff) * 50;
            int maxVoltage = ((object >> 20) & 0x3ff) * 50;
            Serial.printf("  Variable %6dmV (min) %6dmV (max) %6dmW (max)", minVoltage, maxVoltage, maxPower);

        } else {
            // APDO
            if (((object >> 28) & 0x03) == 0) {
                // PPS
                int maxCurrent = (object & 0x7f) * 50;
                int minVoltage = ((object >> 8) & 0xff) * 100;
                int maxVoltage = ((object >> 17) & 0xff) * 100;
                Serial.printf("  PPS      %6dmV (min) %6dmV (max) %6dmA (max)", minVoltage, maxVoltage, maxCurrent);
            }
        }

        Serial.println();
    } 
}

void USBPDProtocolAnalyzer::printRequestDetails(const PDMessage* message) {
    auto object = message->objects[0];
    int objPos = (object >> 28) & 0x07;
    bool giveBack = (object & (1 << 27)) != 0;
    auto capObject = capabilities.numObjects() >= objPos ? capabilities.objects[objPos - 1] : 0;
    int supplyType = (capObject >> 30) & 0x03;

    if (supplyType == 0 || supplyType == 2) {
        int current0 = (object & 0x3ff) * 10;
        int current1 = ((object >> 10) & 0x3ff) * 10;
        Serial.printf("  Capability: %d  %6dmA (oper)  %6dmA (%s)   ", objPos, current1, current0, giveBack ? "min" : "max");

    } else if (supplyType == 1) {
        int power0 = (object & 0x3ff) * 250;
        int power1 = ((object >> 10) & 0x3ff) * 250;
        Serial.printf("  Capability: %d  %6dmW (oper)  %6dmW (%s)   ", objPos, power1, power0, giveBack ? "min" : "max");

    } else {
        int current = (object & 0x7f) * 50;
        int voltage = ((object >> 9) & 0x7ff) * 20;
        Serial.printf("  Capability: %d  %6dmV (oper)  %6dmA (oper)  ", objPos, voltage, current);
    }

    if ((object & (1 << 23)) != 0) Serial.print("extended msg, ");
    if ((object & (1 << 24)) != 0) Serial.print("USB suspend, ");
    if ((object & (1 << 25)) != 0) Serial.print("USB comm capable, ");
    if (giveBack) Serial.print("give back");
    Serial.println();
}

const char* USBPDProtocolAnalyzer::getMessageName(PDMessageType messageType) {
    unsigned int index = (unsigned int)messageType;
    const char* name = nullptr;
    if (index < 0x80) {
        constexpr unsigned int arraySize = sizeof(ControlMessageNames) / sizeof(ControlMessageNames[0]);
        if (index < arraySize)
            name = ControlMessageNames[index];
    } else {
        index -= 0x80;
        constexpr unsigned int arraySize = sizeof(DataMessageNames) / sizeof(DataMessageNames[0]);
        if (index < arraySize)
            name = DataMessageNames[index];
    }
    return name != nullptr ? name : "<unknown>";
}

const char* USBPDProtocolAnalyzer::getSOPSequenceName(PDSOPSequence sequence) {
    constexpr unsigned int arraySize = sizeof(SOPSequenceNames) / sizeof(SOPSequenceNames[0]);
    unsigned int index = (unsigned int)sequence;
    if (index >= arraySize)
        index = arraySize;
    return SOPSequenceNames[index];
}

const char* USBPDProtocolAnalyzer::getSender(const PDMessage* message) {
    auto seq = message->sopSequence;
    if (seq == PDSOPSequence::sop) {
        return (message->header & 0x0100) != 0 ? "Source" : "Sink";
    } else if (seq == PDSOPSequence::sop1 || seq == PDSOPSequence::sop2) {
        return (message->header & 0x0100) != 0 ? "Cable" : "Port";
    }

    return "";
}
