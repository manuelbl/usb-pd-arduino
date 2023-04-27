//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

//
// --- Test program switching between the available fixed voltages
//

#include <Arduino.h>
#include "USBPowerDelivery.h"

static void handleEvent(PDSinkEventType eventType);
static void switchVoltage();
static bool hasExpired(uint32_t time);

static bool isUSBPDSource = false;
static uint32_t nextVoltageChangeTime = 0;
static int voltageIndex = 0;

void setup() {
    Serial.begin(115200);
    PowerSink.start(handleEvent);
    Serial.println();
    Serial.println("USB Power Delivery");
}

void loop() {
    PowerSink.poll();
    PDProtocolAnalyzer::poll();

    if (isUSBPDSource && hasExpired(nextVoltageChangeTime))
        switchVoltage();
}

void switchVoltage() {
    // select next fixed voltage
    do {
        voltageIndex += 1;
        if (voltageIndex >= PowerSink.numSourceCapabilities)
            voltageIndex = 0;
    } while (PowerSink.sourceCapabilities[voltageIndex].supplyType != PDSupplyType::fixed);

    PowerSink.requestPower(PowerSink.sourceCapabilities[voltageIndex].maxVoltage);
    nextVoltageChangeTime += 3000;
}

void handleEvent(PDSinkEventType eventType) {
    switch (eventType) {
    case PDSinkEventType::sourceCapabilitiesChanged:
        if (PowerSink.isConnected()) {
            Serial.println("New source capabilities (USB PD supply)");
            isUSBPDSource = true;
            voltageIndex = 0;
            nextVoltageChangeTime = millis() + 2000;
        } else {
            isUSBPDSource = false;
            Serial.println("New source capabilities (no USB PD supply connected)");
        }
        break;

    case PDSinkEventType::voltageChanged:
        Serial.printf("Voltage changd: %5dmV  %5dmA (max)", PowerSink.activeVoltage, PowerSink.activeCurrent);
        Serial.println();
        break;

    case PDSinkEventType::powerRejected:
        Serial.println("Power request rejected");
        break;
    }
}

bool hasExpired(uint32_t time) {
    return (int32_t)(time - millis()) <= 0;
}
