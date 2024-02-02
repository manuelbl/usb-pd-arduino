//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#pragma once

#if defined(STM32G0xx) || defined(STM32G4xx)

#include "PDPhy.h"

extern "C" void UCPD1_IRQHandler();
extern "C" void UCPD1_2_IRQHandler();

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
    friend void UCPD1_2_IRQHandler();
};

#endif
