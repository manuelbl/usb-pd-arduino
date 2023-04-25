#include "USBPowerDelivery.h"

void setup() {
  Serial.begin(115200);
  PowerController.startMonitor();
}

void loop() {
  ProtocolAnalyzer::poll();
}
