//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#if defined(STM32G431xx)

#include <Arduino.h>
#include "stm32yyxx_ll_bus.h"
#include "stm32yyxx_ll_dma.h"
#include "stm32yyxx_ll_pwr.h"
#include "stm32yyxx_ll_ucpd.h"
#include "PDController.h"
#include "PDPhySTM32UCPD.h"

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
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1);

    // initialize UCPD1
    LL_UCPD_InitTypeDef ucpdInit = {0};
    LL_UCPD_StructInit(&ucpdInit);
    LL_UCPD_Init(UCPD1, &ucpdInit);

    // configure pins (CC1 -> PB6, CC2 -> PB4)
    pinMode(D12, INPUT_ANALOG);
    LL_GPIO_InitTypeDef pb4Init = {
        .Pin = LL_GPIO_PIN_4,
        .Mode = LL_GPIO_MODE_ANALOG,
        .Pull = LL_GPIO_PULL_NO
    };
    LL_GPIO_Init(GPIOB, &pb4Init);

    pinMode(D6, INPUT_ANALOG);
    LL_GPIO_InitTypeDef pb6Init = {
        .Pin = LL_GPIO_PIN_6,
        .Mode = LL_GPIO_MODE_ANALOG,
        .Pull = LL_GPIO_PULL_NO
    };
    LL_GPIO_Init(GPIOB, &pb6Init);

    // configure DMA for USB PD RX
    LL_DMA_InitTypeDef rxDmaInit = {
        .PeriphOrM2MSrcAddress = (uint32_t)&UCPD1->RXDR,
        .Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY,
        .Mode = LL_DMA_MODE_NORMAL,
        .PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT,
        .MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT,
        .PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE,
        .MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE,
        .PeriphRequest = LL_DMAMUX_REQ_UCPD1_RX,
        .Priority = LL_DMA_PRIORITY_LOW
    };
    LL_DMA_Init(DMA1, LL_DMA_CHANNEL_1, &rxDmaInit);

    if (!isMonitor) {
        // configure DMA for USB PD TX
        LL_DMA_InitTypeDef txDmaInit = {
            .PeriphOrM2MSrcAddress = (uint32_t)&UCPD1->TXDR,
            .Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH,
            .Mode = LL_DMA_MODE_NORMAL,
            .PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT,
            .MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT,
            .PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE,
            .MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE,
            .PeriphRequest = LL_DMAMUX_REQ_UCPD1_TX,
            .Priority = LL_DMA_PRIORITY_LOW
        };
        LL_DMA_Init(DMA1, LL_DMA_CHANNEL_2, &txDmaInit);
    }

    // turn off dead battery detection
    LL_PWR_DisableUCPDDeadBattery();

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

    // enable DMA
    LL_UCPD_RxDMAEnable(UCPD1);
    if (!isMonitor)
        LL_UCPD_TxDMAEnable(UCPD1);

    // same interrupt priority as timer 7 so they don't interrupt each other (code is not re-entrant)
    NVIC_SetPriority(UCPD1_IRQn, NVIC_GetPriority(TIM7_IRQn));
    // enable interrupt handler
    NVIC_EnableIRQ(UCPD1_IRQn);
}

void PDPhy::prepareRead(PDMessage* msg) {
    rxMessage = msg;
    if (ccActive != 0)
        PDPhySTM32UCPD::enableRead();
}

void PDPhySTM32UCPD::enableRead() {
    // enable RX DMA
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, reinterpret_cast<uint32_t>(&rxMessage->header));
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 30);
    LL_UCPD_RxEnable(UCPD1);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
}

bool PDPhy::transmitMessage(const PDMessage* msg) {
    // configure DMA request
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, reinterpret_cast<uint32_t>(&msg->header));
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, msg->payloadSize());
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);

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
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
    LL_UCPD_RxDisable(UCPD1);

    // cancel transmit
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);

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
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
        if ((status & UCPD_SR_RXERR) == 0) {
            uint32_t orderedSet = LL_UCPD_ReadRxOrderSet(UCPD1);
            rxMessage->sopSequence = orderedSet <= LL_UCPD_ORDERED_SET_SOP2_DEBUG
                ? static_cast<PDSOPSequence>(orderedSet) : PDSOPSequence::cableReset;
            rxMessage->cc = ccActive;
            PowerController.onMessageReceived(rxMessage);

        } else {
            PowerController.onError();
        }
    }

    // message sent
    if ((status & UCPD_SR_TXMSGSENT) != 0) {
        LL_UCPD_ClearFlag_TxMSGSENT(UCPD1);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
        PowerController.onMessageTransmitted(true);
    }

    // message aborted
    if ((status & UCPD_SR_TXMSGABT) != 0) {
        LL_UCPD_ClearFlag_TxMSGABT(UCPD1);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
        PowerController.onMessageTransmitted(false);
    }

    // message discarded
    if ((status & UCPD_SR_TXMSGDISC) != 0) {
        LL_UCPD_ClearFlag_TxMSGDISC(UCPD1);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
        PowerController.onMessageTransmitted(false);
    }
}

#endif
