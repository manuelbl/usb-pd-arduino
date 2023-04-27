//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#pragma once

#include "PDMessage.h"

struct USBPDProtocolAnalyzer {
    USBPDProtocolAnalyzer();
    void poll();

private:
    void printMessage(const PDMessage* message);
    void printCapabilitiesDetails(const PDMessage* message);
    void printRequestDetails(const PDMessage* message);
    const char* getMessageName(PDMessageType messageType);
    const char* getSOPSequenceName(PDSOPSequence sequence);
    const char* getSender(const PDMessage* message);

    PDMessage capabilities;
};

extern USBPDProtocolAnalyzer PDProtocolAnalyzer;