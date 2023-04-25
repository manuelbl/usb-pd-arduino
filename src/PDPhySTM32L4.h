//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#pragma once

#if defined(STM32L4xx)

#include "PDPhy.h"

extern "C" void COMP_IRQHandler();
extern "C" void ADC1_IRQHandler();
extern "C" void DMA1_Channel3_IRQHandler();
extern "C" void DMA2_Channel2_IRQHandler();

/**
 * @brief Physical layer for USB PD communication.
 * 
 */
struct PDPhySTM32L4 : PDPhy {
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

    static int decodeChunk(int currentIndex);
    static void processData();
    static void switchCC();
    static void switchAfterDebounce(uint8_t cc);
    static void messageTransmitted();
    static void configureRx(RxConfig config);
    static void enablePDCommunication();
    static void disablePDCommunication();

    friend class PDPhy;
    friend void COMP_IRQHandler();
    friend void ADC1_IRQHandler();
    friend void DMA1_Channel3_IRQHandler();
    friend void DMA2_Channel2_IRQHandler();
};

#endif
