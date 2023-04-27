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
  PowerController.startMonitor();
}

void loop() {
  PDProtocolAnalyzer.poll();
}
