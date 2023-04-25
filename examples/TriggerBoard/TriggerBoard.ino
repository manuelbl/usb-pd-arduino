#include "USBPowerDelivery.h"

void setup() {
  PowerSink.start();
  // request 12V @ 1A once power supply is connected
  PowerSink.requestPower(12000, 1000);
}

void loop() {
}
