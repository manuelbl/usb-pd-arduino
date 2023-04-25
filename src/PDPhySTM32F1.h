//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#pragma once

#if defined(STM32F103xB)

#include "PDPhy.h"

extern "C" void ADC1_2_IRQHandler();
extern "C" void DMA1_Channel3_IRQHandler();

/**
 * @brief Physical layer for USB PD communication.
 * 
 */
struct PDPhySTM32F1 : PDPhy {
private:
    enum class RxConfig {
        monitorCC,
        rxWaitForMessage,
        rxProcessMessage,
        pausedForTx
    };

    static void initRx();
    static void initTx();
    static bool transmitMessage(const PDMessage* msg);

    static void mergeTimeStamps(int currentIndex);
    static int decodeChunk(int currentIndex);
    static void processData();
    static void switchCC();
    static void switchAfterDebounce(uint8_t cc);
    static void messageTransmitted();
    static void configureRx(RxConfig config);
    static void onEXTIInterrupt();

    friend class PDPhy;
    friend void ADC1_2_IRQHandler();
    friend void DMA1_Channel3_IRQHandler();
};

#endif
