//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

//
// Simple trigger board selecting 12V at 1A when the power supply is
// connected and if 12V is available.
//
// Please see https://github.com/manuelbl/usb-pd-arduino/wiki for
// instructions how to wire your specific board to a USB C connector.
// For this sketch, "Sink Mode" is relevant.
//

#include "USBPowerDelivery.h"

void setup() {
  PowerSink.start();
  // request 12V @ 1A once power supply is connected
  PowerSink.requestPower(12000, 1000);
}

void loop() {
}
