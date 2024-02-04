//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

//
// --- USB PD Protocol Analyzer
//

#include <Arduino.h>
#include "USBPowerDelivery.h"

void setup() {
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println("USB PD for Arduino - Protocol Analyzer");

    PowerController.startMonitor();

    #if defined(SNK1M1_SHIELD)
        NucleoSNK1MK1.init();
    #endif
}

void loop() {
    PDProtocolAnalyzer.poll();
}
