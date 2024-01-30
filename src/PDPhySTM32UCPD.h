//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#pragma once

#if defined(STM32G431xx)

#include "PDPhy.h"

extern "C" void UCPD1_IRQHandler();

/**
 * @brief Physical layer for USB PD communication.
 * 
 */
struct PDPhySTM32UCPD : PDPhy {
private:
    static void init(bool isMonitor);
    static void enableCommunication(int cc);
    static void disableCommunication();
    static void enableRead();
    static PDSOPSequence mapSOPSequence(uint32_t orderedSet);

    static void handleInterrupt();

    friend class PDPhy;
    friend void UCPD1_IRQHandler();
};

#endif
