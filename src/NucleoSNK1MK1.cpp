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
    // Pin PB1: DB_OUT -> PIN_A9
    // Pin PC10: VCC_OUT -> 16
    pinMode(PIN_A9, OUTPUT);
    digitalWrite(PIN_A9, HIGH);
    pinMode(16, OUTPUT);
    digitalWrite(16, HIGH);
#elif defined(ARDUINO_NUCLEO_G071RB)
    // Pin PB6: DB_OUT -> 46
    // Pin PC10: VCC_OUT -> 16
    pinMode(46, OUTPUT);
    digitalWrite(46, HIGH);
    pinMode(16, OUTPUT);
    digitalWrite(16, HIGH);
#endif
}

SNK1MK1Controller NucleoSNK1MK1;

#endif
