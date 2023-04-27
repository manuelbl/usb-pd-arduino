//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

//
// Monitors the USB PD communication between two USB C devices
// and prints the decoded messages.
//
// Please see https://github.com/manuelbl/usb-pd-arduino/wiki for
// instructions how to wire your specific board to a USB C intermediate
// adapter. For this sketch, "Monitor Mode" is relevant.
//

#include "USBPowerDelivery.h"

void setup() {
  Serial.begin(115200);
  PowerController.startMonitor();
}

void loop() {
  PDProtocolAnalyzer.poll();
}
