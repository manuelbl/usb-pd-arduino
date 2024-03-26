//
// USB Power Delivery for Arduino
// Copyright (c) 2024 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#pragma once

#if defined(ARDUINO_NUCLEO_G474RE) || defined(ARDUINO_NUCLEO_G071RB)

/**
 * Controller for X-NUCLEO-SNK1MK1 shield for Nucleo-64.
 */
class SNK1MK1Controller {
public:
    /// Constructor
    SNK1MK1Controller() { }

    /**
     * Initializes the shield.
     * 
     * This will activate the TCPP01-M12 chip and connect the CC1/CC2 pins to the STM32.
     */
    void init();
};

extern SNK1MK1Controller NucleoSNK1MK1;

#endif
