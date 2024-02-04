//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

//
// Selects 12V if available, and 15V otherwise.

// Please see https://github.com/manuelbl/usb-pd-arduino/wiki for
// instructions how to wire your specific board to a USB C connector.
// For this sketch, "Sink Mode" is relevant.
//

#include "USBPowerDelivery.h"

void setup() {
  Serial.begin(115200);
  PowerSink.start(handleEvent);

  // Uncomment if using X-NUCLEO-SNK1MK1 shield
  // NucleoSNK1MK1.init();
}

void loop() {
  PowerSink.poll();
}

void handleEvent(PDSinkEventType eventType) {

  if (eventType == PDSinkEventType::sourceCapabilitiesChanged) {
    // source capabilities have changed
    if (PowerSink.isConnected()) {
      // USB PD supply is connected
      requestVoltage();
  
    } else {
      // no supply or no USB PD capable supply is connected
      PowerSink.requestPower(5000); // reset to 5V
    }

  } else if (eventType == PDSinkEventType::voltageChanged) {
    // voltage has changed
    if (PowerSink.activeVoltage != 0) {
      Serial.printf("Voltage: %d mV @ %d mA (max)", PowerSink.activeVoltage, PowerSink.activeCurrent);
      Serial.println();
    } else {
      Serial.println("Disconnected");
    }

  } else if (eventType == PDSinkEventType::powerRejected) {
    // rare case: power supply rejected requested power
    Serial.println("Power request rejected");
    Serial.printf("Voltage: %d mV @ %d mA (max)", PowerSink.activeVoltage, PowerSink.activeCurrent);
  }
}

void requestVoltage() {
  // check if 12V is supported
  for (int i = 0; i < PowerSink.numSourceCapabilities; i += 1) {
    if (PowerSink.sourceCapabilities[i].minVoltage <= 12000
        && PowerSink.sourceCapabilities[i].maxVoltage >= 12000) {
      PowerSink.requestPower(12000);
      return;
    }
  }

  // check if 15V is supported
  for (int i = 0; i < PowerSink.numSourceCapabilities; i += 1) {
    if (PowerSink.sourceCapabilities[i].minVoltage <= 15000
        && PowerSink.sourceCapabilities[i].maxVoltage >= 15000) {
      PowerSink.requestPower(15000);
      return;
    }
  }

  Serial.println("Neither 12V nor 15V is supported");
}

