//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#pragma once

#if defined(STM32F401xC)

#include "PDPhy.h"

extern "C" void ADC_IRQHandler();
extern "C" void DMA2_Stream3_IRQHandler();

/**
 * @brief Physical layer for USB PD communication.
 * 
 */
struct PDPhySTM32F4 : PDPhy {
private:
    enum class RxConfig {
        monitorCC,
        rxWaitForMessage,
        rxProcessMessage,
        pausedForTx
    };

    static void initRx();

    static int decodeChunk(int currentIndex);
    static void processData();
    static void switchCC();
    static void switchAfterDebounce(uint8_t cc);
    static void configureRx(RxConfig config);
    static void onEXTIInterrupt();

    static void initTx();
    static bool transmitMessage(const PDMessage* msg);
    static void messageTransmitted();

    friend class PDPhy;
    friend void ADC_IRQHandler();
    friend void DMA2_Stream3_IRQHandler();
};

#endif
