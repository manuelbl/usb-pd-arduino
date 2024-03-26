//
// USB Power Delivery for Arduino
// Copyright (c) 2024 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#if defined(ARDUINO_NUCLEO_G474RE) || defined(ARDUINO_NUCLEO_G071RB)

#include <Arduino.h>
#include "NucleoSNK1MK1.h"


void SNK1MK1Controller::init() {

#if defined(ARDUINO_NUCLEO_G474RE)
    // Pin PB1: DB_OUT
    // Pin PC10: VCC_OUT
    pinMode(PB1, OUTPUT);
    digitalWrite(PB1, HIGH);
    pinMode(PC10, OUTPUT);
    digitalWrite(PC10, HIGH);
#elif defined(ARDUINO_NUCLEO_G071RB)
    // Pin PB6: DB_OUT
    // Pin PC10: VCC_OUT
    pinMode(PB6, OUTPUT);
    digitalWrite(PB6, HIGH);
    pinMode(PC10, OUTPUT);
    digitalWrite(PC10, HIGH);
#endif
}

SNK1MK1Controller NucleoSNK1MK1;

#endif
