//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#include <Arduino.h>
#include "USBPDProtocolAnalyzer.h"
#include "PDSourceCapability.h"
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

static const char* getMessageName(PDMessageType messageType) {
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

static const char* const SupplyTypeNames[] = {
    [(int)PDSupplyType::fixed] = "Fixed",
    [(int)PDSupplyType::battery] = "Battery",
    [(int)PDSupplyType::variable] = "Variable",
    [(int)PDSupplyType::pps] = "PPS",
};

static const char* getSupplyTypeName(PDSupplyType type) {
    constexpr unsigned int arraySize = sizeof(SupplyTypeNames) / sizeof(SupplyTypeNames[0]);
    if ((unsigned int)type < arraySize)
        return SupplyTypeNames[(unsigned int)type];
    else
        return "<unknown>";
}

static const char* const SOPSequenceNames[] = {
    [0] = "INV",
    [(int)PDSOPSequence::sop] = "SOP",
    [(int)PDSOPSequence::sop1] = "SOP'",
    [(int)PDSOPSequence::sop2] = "SOP''",
    [(int)PDSOPSequence::sop1Debug] = "SOP1D",
    [(int)PDSOPSequence::sop2Debug] = "SOP2D",
};

static const char* getSOPSequenceName(PDSOPSequence sequence) {
    constexpr unsigned int arraySize = sizeof(SOPSequenceNames) / sizeof(SOPSequenceNames[0]);
    unsigned int index = (unsigned int)sequence;
    if (index >= arraySize)
        index = arraySize;
    return SOPSequenceNames[index];
}

static const char* getSender(const PDMessage* message) {
    auto seq = message->sopSequence;
    if (seq == PDSOPSequence::sop) {
        return (message->header & 0x0100) != 0 ? "Source" : "Sink";
    } else if (seq == PDSOPSequence::sop1 || seq == PDSOPSequence::sop2) {
        return (message->header & 0x0100) != 0 ? "Cable" : "Port";
    }

    return "";
}

static void printMessage(const PDMessage* message) {
    Serial.printf("CC%d %-5s %-7s %-20s %d  %04x",
            message->cc, getSOPSequenceName(message->sopSequence),
            getSender(message),getMessageName(message->type()),
            message->messageId(), message->header);
    int numObjects = message->numObjects();
    for (int i = 0; i < numObjects; i++)
        Serial.printf(" %08x", message->objects[i]);
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
        Serial.println();
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
        Serial.println();
        break;
    case PDLogEntryType::transmissionCompleted:
        Serial.println("TX: completed");
        break;
    case PDLogEntryType::transmissionFailed:
        Serial.println("TX: failed");
        break;
    }
}
