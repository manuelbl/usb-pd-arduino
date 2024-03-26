//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#if defined(STM32G0xx) || defined(STM32G4xx)

#include <Arduino.h>
#include "stm32yyxx_ll_bus.h"
#include "stm32yyxx_ll_dma.h"
#include "stm32yyxx_ll_pwr.h"
#include "stm32yyxx_ll_ucpd.h"
#include "stm32yyxx_ll_system.h"
#include "PDController.h"
#include "PDPhySTM32UCPD.h"

#if defined(STM32G4xx)
    // STM32G4 family: CC1 -> PB6, CC2 -> PB4
    #define GPIO_CC1 GPIOB
    #define PIN_CC1 LL_GPIO_PIN_6
    #define ARDUINO_PIN_CC1 PB6
    #define GPIO_CC2 GPIOB
    #define PIN_CC2 LL_GPIO_PIN_4
    #define ARDUINO_PIN_CC2 PB4
    #define DMA_RX DMA1
    #define DMA_CHANNEL_RX LL_DMA_CHANNEL_1
    #define DMA_TX DMA1
    #define DMA_CHANNEL_TX LL_DMA_CHANNEL_2
    #define UCPD_IRQ UCPD1_IRQn

#elif defined(STM32G0xx)
    // STM32G0 family: CC1 -> PA8, CC2 -> PB15
    #define GPIO_CC1 GPIOA
    #define PIN_CC1 LL_GPIO_PIN_8
    #define ARDUINO_PIN_CC1 PA8
    #define GPIO_CC2 GPIOB
    #define PIN_CC2 LL_GPIO_PIN_15
    #define ARDUINO_PIN_CC2 PB15
    #define DMA_RX DMA1
    #define DMA_CHANNEL_RX LL_DMA_CHANNEL_1
    #define DMA_TX DMA1
    #define DMA_CHANNEL_TX LL_DMA_CHANNEL_2
    #define UCPD_IRQ UCPD1_2_IRQn
#endif


static PDMessage* rxMessage;
static int ccActive;


void PDPhy::initMonitor() {
    PDPhySTM32UCPD::init(true);
}

void PDPhy::initSink() {
    PDPhySTM32UCPD::init(false);
}

void PDPhySTM32UCPD::init(bool isMonitor) {
    // clocks
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    #if defined(STM32G4xx)
        LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);
    #else
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    #endif

    // Use Arduino function for basic pin configuration so the Arduino library is aware
    // if the most important settings (such as GPIO clock initialization).
    pinMode(ARDUINO_PIN_CC1, INPUT_ANALOG);
    pinMode(ARDUINO_PIN_CC2, INPUT_ANALOG);

    // initialize UCPD1
    LL_UCPD_InitTypeDef ucpdInit = {};
    LL_UCPD_StructInit(&ucpdInit);
    LL_UCPD_Init(UCPD1, &ucpdInit);

    LL_GPIO_InitTypeDef pinCc1Init = {
        .Pin = PIN_CC1,
        .Mode = LL_GPIO_MODE_ANALOG,
        .Speed = LL_GPIO_SPEED_FREQ_LOW,
        .OutputType = LL_GPIO_OUTPUT_OPENDRAIN,
        .Pull = LL_GPIO_PULL_NO,
        .Alternate = LL_GPIO_AF_0,
    };
    LL_GPIO_Init(GPIO_CC1, &pinCc1Init);

    LL_GPIO_InitTypeDef pinCc2Init = {
        .Pin = PIN_CC2,
        .Mode = LL_GPIO_MODE_ANALOG,
        .Speed = LL_GPIO_SPEED_FREQ_LOW,
        .OutputType = LL_GPIO_OUTPUT_OPENDRAIN,
        .Pull = LL_GPIO_PULL_NO,
        .Alternate = LL_GPIO_AF_0,
    };
    LL_GPIO_Init(GPIO_CC2, &pinCc2Init);

    // configure DMA for USB PD RX
    LL_DMA_InitTypeDef rxDmaInit = {
        .PeriphOrM2MSrcAddress = (uint32_t)&UCPD1->RXDR,
        .MemoryOrM2MDstAddress = 0,
        .Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY,
        .Mode = LL_DMA_MODE_NORMAL,
        .PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT,
        .MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT,
        .PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE,
        .MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE,
        .NbData = 0,
        .PeriphRequest = LL_DMAMUX_REQ_UCPD1_RX,
        .Priority = LL_DMA_PRIORITY_LOW,
    };
    LL_DMA_Init(DMA_RX, DMA_CHANNEL_RX, &rxDmaInit);

    if (!isMonitor) {
        // configure DMA for USB PD TX
        LL_DMA_InitTypeDef txDmaInit = {
            .PeriphOrM2MSrcAddress = (uint32_t)&UCPD1->TXDR,
            .MemoryOrM2MDstAddress = 0,
            .Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH,
            .Mode = LL_DMA_MODE_NORMAL,
            .PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT,
            .MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT,
            .PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE,
            .MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE,
            .NbData = 0,
            .PeriphRequest = LL_DMAMUX_REQ_UCPD1_TX,
            .Priority = LL_DMA_PRIORITY_LOW
        };
        LL_DMA_Init(DMA_TX, DMA_CHANNEL_TX, &txDmaInit);
    }

    #if defined(STM32G4xx)
        // turn off dead battery detection
        LL_PWR_DisableUCPDDeadBattery();
    #endif

    // configure ordered sets
    LL_UCPD_SetRxOrderSet(UCPD1, LL_UCPD_ORDERSET_SOP | LL_UCPD_ORDERSET_SOP1 | LL_UCPD_ORDERSET_SOP2 |
                                     LL_UCPD_ORDERSET_CABLERST | LL_UCPD_ORDERSET_HARDRST);

    // enable interrupts
    LL_UCPD_EnableIT_TypeCEventCC1(UCPD1);
    LL_UCPD_EnableIT_TypeCEventCC2(UCPD1);

    // enable
    LL_UCPD_Enable(UCPD1);

    // configure as sink (when enabled)
    LL_UCPD_SetSNKRole(UCPD1);
    LL_UCPD_SetRpResistor(UCPD1, isMonitor ? LL_UCPD_RESISTOR_DEFAULT : LL_UCPD_RESISTOR_NONE);
    LL_UCPD_SetccEnable(UCPD1, LL_UCPD_CCENABLE_CC1CC2);
    #if defined(STM32G0xx)
        if (!isMonitor) {
            LL_SYSCFG_DisableDBATT(LL_SYSCFG_UCPD1_STROBE);
        }
    #endif

    // enable DMA
    LL_UCPD_RxDMAEnable(UCPD1);
    if (!isMonitor)
        LL_UCPD_TxDMAEnable(UCPD1);

    // same interrupt priority as timer 7 so they don't interrupt each other (code is not re-entrant)
    NVIC_SetPriority(UCPD_IRQ, NVIC_GetPriority(TIM7_IRQn));
    // enable interrupt handler
    NVIC_EnableIRQ(UCPD_IRQ);
}

void PDPhy::prepareRead(PDMessage* msg) {
    rxMessage = msg;
    if (ccActive != 0)
        PDPhySTM32UCPD::enableRead();
}

void PDPhySTM32UCPD::enableRead() {
    // enable RX DMA
    LL_DMA_SetMemoryAddress(DMA_RX, DMA_CHANNEL_RX, reinterpret_cast<uint32_t>(&rxMessage->header));
    LL_DMA_SetDataLength(DMA_RX, DMA_CHANNEL_RX, 30);
    LL_UCPD_RxEnable(UCPD1);
    LL_DMA_EnableChannel(DMA_RX, DMA_CHANNEL_RX);
}

bool PDPhy::transmitMessage(const PDMessage* msg) {
    // configure DMA request
    LL_DMA_SetMemoryAddress(DMA_TX, DMA_CHANNEL_TX, reinterpret_cast<uint32_t>(&msg->header));
    LL_DMA_SetDataLength(DMA_TX, DMA_CHANNEL_TX, msg->payloadSize());
    LL_DMA_EnableChannel(DMA_TX, DMA_CHANNEL_TX);

    // start transmitting
    LL_UCPD_WriteTxOrderSet(UCPD1, LL_UCPD_ORDERED_SET_SOP);
    LL_UCPD_WriteTxPaySize(UCPD1, msg->payloadSize());
    LL_UCPD_SendMessage(UCPD1);

    return true;
}

void PDPhySTM32UCPD::enableCommunication(int cc) {
    // set pin for communication
    LL_UCPD_SetCCPin(UCPD1, cc == 1 ? LL_UCPD_CCPIN_CC1 : LL_UCPD_CCPIN_CC2);

    // enable interrupts
    LL_UCPD_EnableIT_RxHRST(UCPD1);
    LL_UCPD_EnableIT_RxMsgEnd(UCPD1);
    LL_UCPD_EnableIT_TxMSGSENT(UCPD1);
    LL_UCPD_EnableIT_TxMSGDISC(UCPD1);
    LL_UCPD_EnableIT_TxMSGABT(UCPD1);

    enableRead();
}

void PDPhySTM32UCPD::disableCommunication() {

    // cancel read
    LL_DMA_DisableChannel(DMA_RX, DMA_CHANNEL_RX);
    LL_UCPD_RxDisable(UCPD1);

    // cancel transmit
    LL_DMA_DisableChannel(DMA_TX, DMA_CHANNEL_TX);

    // disable interrupts
    LL_UCPD_DisableIT_RxMsgEnd(UCPD1);
    LL_UCPD_DisableIT_RxHRST(UCPD1);
    LL_UCPD_DisableIT_TxMSGSENT(UCPD1);
    LL_UCPD_DisableIT_TxMSGDISC(UCPD1);
    LL_UCPD_DisableIT_TxMSGABT(UCPD1);
}

// interrupt handler
extern "C" void UCPD1_IRQHandler() {
    PDPhySTM32UCPD::handleInterrupt();
}

extern "C" void UCPD1_2_IRQHandler() {
    PDPhySTM32UCPD::handleInterrupt();
}

void PDPhySTM32UCPD::handleInterrupt() {

    uint32_t status = UCPD1->SR;

    // voltage changed on CC1 or CC2 pin
    if ((status & (UCPD_SR_TYPECEVT1 | UCPD_SR_TYPECEVT2)) != 0) {
        LL_UCPD_ClearFlag_TypeCEventCC1(UCPD1);
        LL_UCPD_ClearFlag_TypeCEventCC2(UCPD1);

        int cc = 0;
        if (LL_UCPD_GetTypeCVstateCC1(UCPD1) != 0) {
            cc = 1;
        } else if (LL_UCPD_GetTypeCVstateCC2(UCPD1) != 0) {
            cc = 2;
        }
        if (cc != ccActive) {
            ccActive = cc;
            if (cc != 0)
                enableCommunication(cc);
            else
                disableCommunication();

            PowerController.onVoltageChanged(cc);
        }
    }

    // hard reset received
    if ((status & UCPD_SR_RXHRSTDET) != 0) {
        LL_UCPD_ClearFlag_RxHRST(UCPD1);
        PowerController.onReset(PDSOPSequence::hardReset);
    }

    // message received
    if ((status & UCPD_SR_RXMSGEND) != 0) {
        LL_UCPD_ClearFlag_RxMsgEnd(UCPD1);
        LL_DMA_DisableChannel(DMA_RX, DMA_CHANNEL_RX);
        if ((status & UCPD_SR_RXERR) == 0) {
            uint32_t orderedSet = LL_UCPD_ReadRxOrderSet(UCPD1);
            rxMessage->sopSequence = mapSOPSequence(orderedSet);
            rxMessage->cc = ccActive;
            PowerController.onMessageReceived(rxMessage);

        } else {
            PowerController.onError();
        }
    }

    // message sent
    if ((status & UCPD_SR_TXMSGSENT) != 0) {
        LL_UCPD_ClearFlag_TxMSGSENT(UCPD1);
        LL_DMA_DisableChannel(DMA_TX, DMA_CHANNEL_TX);
        PowerController.onMessageTransmitted(true);
    }

    // message aborted
    if ((status & UCPD_SR_TXMSGABT) != 0) {
        LL_UCPD_ClearFlag_TxMSGABT(UCPD1);
        LL_DMA_DisableChannel(DMA_TX, DMA_CHANNEL_TX);
        PowerController.onMessageTransmitted(false);
    }

    // message discarded
    if ((status & UCPD_SR_TXMSGDISC) != 0) {
        LL_UCPD_ClearFlag_TxMSGDISC(UCPD1);
        LL_DMA_DisableChannel(DMA_TX, DMA_CHANNEL_TX);
        PowerController.onMessageTransmitted(false);
    }
}

PDSOPSequence PDPhySTM32UCPD::mapSOPSequence(uint32_t orderedSet) {
    switch (orderedSet) {
    case LL_UCPD_RXORDSET_SOP:
        return PDSOPSequence::sop;
    case LL_UCPD_RXORDSET_SOP1:
        return PDSOPSequence::sop1;
    case LL_UCPD_RXORDSET_SOP2:
        return PDSOPSequence::sop2;
    case LL_UCPD_RXORDSET_SOP1_DEBUG:
        return PDSOPSequence::sop1Debug;
    case LL_UCPD_RXORDSET_SOP2_DEBUG:
        return PDSOPSequence::sop2Debug;
    case LL_UCPD_RXORDSET_CABLE_RESET:
        return PDSOPSequence::cableReset;
    default:
        return PDSOPSequence::invalid;
    }
}

#endif
