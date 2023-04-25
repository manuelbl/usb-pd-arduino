//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#if defined(STM32F103xB)

#include <Arduino.h>
#include <string.h>
#include "CMSISHelper.h"
#include "PDController.h"
#include "PDMessageDecoder.h"
#include "PDMessageEncoder.h"
#include "PDPhySTM32F1.h"
#include "TaskScheduler.h"

static constexpr uint8_t CC1_ADC_CHAN = 1; // PA1
static constexpr uint8_t CC2_ADC_CHAN = 0; // PA0
static constexpr uint32_t ADC_SQR3_CC1 = (CC1_ADC_CHAN << ADC_SQR3_SQ1_Pos); // Single conversion of channel 1 (PA1)
static constexpr uint32_t ADC_SQR3_CC2 = (CC2_ADC_CHAN << ADC_SQR3_SQ1_Pos); // Single conversion of channel 0 (PA0)

static constexpr int TIMER_FREQ = 2400000; // Hz
static constexpr int RX_PROCESS_INTERVAL = 89; // µs
static constexpr uint32_t CC_SAMPLE_INTERVAL = 467 * (TIMER_FREQ / 1000) / 1000; // about 500 µs interval

static constexpr int DEBOUNCE_TIME = 40000; // µs

// Voltage tresholds between unconnected (sink), connected, unconnected (source).
// They work both on the sink and source side, but are insufficient to detect
// active cable and different current capabilities. Finer treshold would be possible
// if the role (sink or source) was known.
static constexpr uint32_t CC_TRESHOLD_LOW = 300 * 4095 / 3300; // 300mV
static constexpr uint32_t CC_TRESHOLD_HIGH = 2200 * 4095 / 3300; // 2.2V

static HardwareTimer timer(TIM2);
static PDMessageDecoder decoder;

// Sufficient samples for 200µs at a rate of 600kbps
static constexpr int TIMESTAMPS_SIZE = 130;
static constexpr int TIMESTAMPS_EDGE_SIZE = TIMESTAMPS_SIZE / 2;
static uint16_t timestamps[TIMESTAMPS_SIZE];
static uint16_t timestampsRising[TIMESTAMPS_EDGE_SIZE];
static uint16_t timestampsFalling[TIMESTAMPS_EDGE_SIZE];

static int mergedIndex; // index into `timestamps` array
static int processedIndex; // index into `timestamps` array
static uint32_t nextProcessingTime;
static PDMessage* currentMessage;
static uint8_t ccActive;
static uint8_t ccAfterDebounce;
static int ccLevel[2]; // -1: unknown: 0: 0V/unconnected, 1: connected, 2: 3.3V/unconnected
static volatile bool hasRxActivity;
static volatile bool isTransmitting;
static uint8_t txBitStream[57];
static int txBitStreamLength;


// --- Debugging ---

// PB3 can be configured as debug output pin. It indicates certain activities such as
// ADC conversion complete, EXTI interrupt, processing of received data.

#if USBPD_DEBUG_PIN
static constexpr bool useDebugOut = true;
#else
static constexpr bool useDebugOut = false;
#endif
static inline void debugOutOn() { if (useDebugOut) gpioSetOutputHigh(GPIOB, 3); }
static inline void debugOutOff() { if (useDebugOut) gpioSetOutputLow(GPIOB, 3); }

// --- Initialization ---

void PDPhy::initMonitor() {
    PDPhySTM32F1::initRx();
}

void PDPhy::initSink() {
    PDPhySTM32F1::initRx();
    PDPhySTM32F1::initTx();
}


// --- Voltage monitoring ---

// The voltage on CC1 and CC2 is regularly measured to detect the connection,
// disconnection and role change of power sink and source. The analog-to-
// digital conversion start is triggered from timer 2 / channel 2. Once it is
// complete, an interrupt is triggered to process the result and schedule the
// next conversion. CC1 and CC2 are measured alternately.

static inline int ccLevelFromADC(uint32_t adcValue) {
    return adcValue < CC_TRESHOLD_LOW ? 0
        : (adcValue > CC_TRESHOLD_HIGH ? 2 : 1);
}

void PDPhySTM32F1::switchCC() {
    // callback after debouncing period
    ccActive = ccAfterDebounce;

    if (ccActive != 0) {
        configureRx(RxConfig::rxWaitForMessage);
    } else {
        configureRx(RxConfig::monitorCC);
        Scheduler.cancelTask(processData);
    }

    if (currentMessage != nullptr)
        currentMessage->cc = ccActive;

    PowerController.onVoltageChanged(ccActive);
}

void PDPhySTM32F1::switchAfterDebounce(uint8_t cc) {
    if (ccAfterDebounce == ccActive) {
        ccAfterDebounce = cc;
        Scheduler.scheduleTaskAfter(switchCC, DEBOUNCE_TIME);

    } else if (cc == ccActive) {
        Scheduler.cancelTask(switchCC);
        ccAfterDebounce = cc;

    } else { // cc ≠ ccActive ≠ ccAfterDebounce
        Scheduler.cancelTask(switchCC);
        ccAfterDebounce = cc;
        Scheduler.scheduleTaskAfter(switchCC, DEBOUNCE_TIME);
    }
}

extern "C" void ADC1_2_IRQHandler() {
    // CC measurement complete
    debugOutOn();

    int cc = ADC1->SQR3 == ADC_SQR3_CC1 ? 1 : 2;
    uint16_t value = ADC1->DR;
    ccLevel[cc - 1] = ccLevelFromADC(value);

    uint8_t newCC = 0;
    if (ccLevel[0] == 1 && (ccLevel[1] & 1) == 0) {
        newCC = 1;
    } else if ((ccLevel[0] & 1) == 0 && ccLevel[1] == 1) {
        newCC = 2;
    }

    if (newCC != ccAfterDebounce)
        PDPhySTM32F1::switchAfterDebounce(newCC);

    // prepare and schedule next measurment
    setRegBits(TIM2->CCMR1, 0b100 << TIM_CCMR1_OC2M_Pos, TIM_CCMR1_OC2M_Msk); // clear OC2REF
    setRegBits(TIM2->CCMR1, 0b001 << TIM_CCMR1_OC2M_Pos, TIM_CCMR1_OC2M_Msk); // set active on event
    uint32_t cnt = TIM2->CNT;
    TIM2->CCR2 = (cnt + CC_SAMPLE_INTERVAL) & 0xffff;
    ADC1->SQR3 = ADC1->SQR3 == ADC_SQR3_CC1 ? ADC_SQR3_CC2 : ADC_SQR3_CC1;

    debugOutOff();
}

// --- RX ---

// The CC pins must externally be level-shifted, inverted and combined. It can
// be done with a dual comparator with open collector output. The level-
// shifted signal is then connected to both PA2 and PA15.
//
// PA2 and PA15 are connected to timer 2 channel 1 and 3 respectively. Both
// channels are configured in input capture mode, channel 1 for the rising
// edge, channel 2 for the falling edge. Using two DMA channels, the
// captured timer values (basically timestamps) are written to two circular
// buffers. These two buffer are then combined into a single buffer in
// software. Two timer and DMA channels are needed as timer channels of the
// STM32F1 family cannot capture on both the rising and falling edge, only on
// one kind of edge at a time.
//
// Additionally, an EXTI interrupt is set up on PA15. See "Data processing"
// below for more information.

void PDPhySTM32F1::initRx() {
    // enable clock for GPIOA, GPIOB, AFIO and ADC1
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_ADC1EN;
    // enable clock for DMA1
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    
    // wait for clock
    asm("nop");
    asm("nop");

    pinMode(PA15, INPUT);
    pinMode(PA2, INPUT);
    setRegBits(AFIO->MAPR, 0b01 << AFIO_MAPR_TIM2_REMAP_Pos, AFIO_MAPR_TIM2_REMAP_Msk); // remap PA15 to TIM2_CH1

    // configure TIM2
    timer.setPrescaleFactor((timer.getTimerClkFreq() + TIMER_FREQ / 2) / TIMER_FREQ);
    timer.setOverflow(0x10000); // full 16-bit range

    // channel 1 in input capture mode, triggered by PA15 on the rising edge
    // channel 2 in output compare mode (for triggering the ADC)
    // channel 3 in input capture mode, triggered by PA2 on the falling edge
    timer.setMode(1, TIMER_INPUT_CAPTURE_RISING);
    timer.setMode(2, TIMER_OUTPUT_COMPARE_ACTIVE);
    timer.setMode(3, TIMER_INPUT_CAPTURE_FALLING);
    setRegBits(TIM2->DIER, TIM_DIER_CC1DE | TIM_DIER_CC3DE, TIM_DIER_CC1DE_Msk | TIM_DIER_CC3DE_Msk); // enable DMA requests on trigger

    timer.refresh();
    timer.resume();

    // configure DMA1 / channel 5 (connected to timer 2 / channel 1)
    DMA1_Channel5->CCR = 
        DMA_CCR_CIRC // circular mode
        | DMA_CCR_MINC // increment memory
        | (0b01 << DMA_CCR_PSIZE_Pos) // peripheral size 16 bit
        | (0b01 << DMA_CCR_MSIZE_Pos) // memory size 16 bit
        | (0b01 << DMA_CCR_PL_Pos); // channel priority medium
    DMA1_Channel5->CNDTR = TIMESTAMPS_EDGE_SIZE;
    DMA1_Channel5->CPAR = (uint32_t) &TIM2->CCR1;
    DMA1_Channel5->CMAR = (uint32_t) &timestampsRising;

    // configure DMA1 / channel 1 (connected to timer 2 / channel 3)
    DMA1_Channel1->CCR = 
        DMA_CCR_CIRC // circular mode
        | DMA_CCR_MINC // increment memory
        | (0b01 << DMA_CCR_PSIZE_Pos) // peripheral size 16 bit
        | (0b01 << DMA_CCR_MSIZE_Pos) // memory size 16 bit
        | (0b01 << DMA_CCR_PL_Pos); // channel priority medium
    DMA1_Channel1->CNDTR = TIMESTAMPS_EDGE_SIZE;
    DMA1_Channel1->CPAR = (uint32_t) &TIM2->CCR3;
    DMA1_Channel1->CMAR = (uint32_t) &timestampsFalling;

    // setup EXTI interrupt for PA15
    attachInterrupt(PA15, onEXTIInterrupt, RISING);
    clearRegBits(EXTI->IMR, EXTI_IMR_MR15_Msk); // disable interrupt for now

    // same priority as timer (so they don't interrupt each other)
    NVIC_SetPriority(EXTI15_10_IRQn, NVIC_GetPriority(TIM2_IRQn));

    // startup ADC
    ADC1->CR1 = 0; // default config for ADC
    ADC1->CR2 = ADC_CR2_ADON; // enable ADC
    delayMicroseconds(2);

    // calibrate ADC
    ADC1->CR2 |= ADC_CR2_CAL;
    while ((ADC1->CR2 & ADC_CR2_CAL) != 0)
        ;

    // configure PA1 (for CC1 on ADC1 channel 1) and PA0 (for CC2 on ADC1 channel 0) as inputs
    pinMode(PA1, INPUT_ANALOG);
    pinMode(PA0, INPUT_ANALOG);

    // configure ADC
    setRegBits(ADC1->CR2, (0b011 << ADC_CR2_EXTSEL_Pos) | ADC_CR2_EXTTRIG, ADC_CR2_EXTSEL_Msk | ADC_CR2_EXTTRIG_Msk); // triggered by timer 2 / channel 2
    ADC1->SQR1 = 0 << ADC_SQR1_L_Pos; // a single conversion
    ADC1->SQR3 = 1 << ADC_SQR3_SQ1_Pos; // convert channel 0

    // enable ADC1 interrupt (trigger after AD conversion has completed)
    NVIC_SetPriority(ADC1_2_IRQn, NVIC_GetPriority(TIM2_IRQn));
    NVIC_EnableIRQ(ADC1_2_IRQn);

    if (useDebugOut) {
        // configure PB3 as debug output
        debugOutOff();
        setRegBits(GPIOB->CRL, GPIO_CRL_MODE_OUTPUT_50MHZ(3) | GPIO_CRL_CNF_OUTPUT(3), GPIO_CRL_MODE_MASK(3) | GPIO_CRL_CNF_MASK(3));
    }

    ccLevel[0] = -1;
    ccLevel[1] = -1;

    configureRx(RxConfig::monitorCC);
}

void PDPhySTM32F1::configureRx(RxConfig config) {

    if (config == RxConfig::monitorCC || config == RxConfig::rxWaitForMessage) {
        // enable voltage measurement
        uint32_t cnt = TIM2->CNT;
        TIM2->CCR2 = (cnt + CC_SAMPLE_INTERVAL) & 0xffff;
        setRegBits(ADC1->CR1, ADC_CR1_EOSIE, ADC_CR1_EOSIE_Msk); // enable interrupt
    
    } else {
        // disable voltage measurement
        clearRegBits(ADC1->CR1, ADC_CR1_EOSIE_Msk); // disable interrupt
    }

    if (config == RxConfig::rxWaitForMessage) {
        // enable EXTI interrupt
        setRegBits(EXTI->IMR, EXTI_IMR_MR15, EXTI_IMR_MR15_Msk);
    
    } else {
        // disable EXTI interrupt
        clearRegBits(EXTI->IMR, EXTI_IMR_MR15_Msk);
    }

    // clear EXTI interrupts
    EXTI->PR = EXTI_PR_PR15;

    if (config == RxConfig::rxWaitForMessage) {
        // disable DMA to reconfigure it
        clearRegBits(DMA1_Channel5->CCR, DMA_CCR_EN_Msk);
        clearRegBits(DMA1_Channel1->CCR, DMA_CCR_EN_Msk);
        mergedIndex = 0;
        processedIndex = 0;
        DMA1_Channel5->CNDTR = TIMESTAMPS_EDGE_SIZE;
        DMA1_Channel1->CNDTR = TIMESTAMPS_EDGE_SIZE;

        // enable DMA
        setRegBits(DMA1_Channel5->CCR, DMA_CCR_EN, DMA_CCR_EN_Msk);
        setRegBits(DMA1_Channel1->CCR, DMA_CCR_EN, DMA_CCR_EN_Msk);

    } else if (config != RxConfig::rxProcessMessage) {
        // disable DMA for time stamps
        clearRegBits(DMA1_Channel5->CCR, DMA_CCR_EN_Msk);
        clearRegBits(DMA1_Channel1->CCR, DMA_CCR_EN_Msk);
    }
}

void PDPhySTM32F1::onEXTIInterrupt() {
    debugOutOn();

    hasRxActivity = true;

    configureRx(RxConfig::rxProcessMessage);

    // recheck in about 100µs
    nextProcessingTime = micros() + RX_PROCESS_INTERVAL;
    Scheduler.scheduleTaskAt(processData, nextProcessingTime);

    debugOutOff();
}


// --- Message decoding ---

// The shortest message requires about 500 bytes of time stamps. To limit
// memory requirements, the time stamps are processed about every 100µs,
// immediately reducing the message to its actual size (about 10 bytes for the
// shortest message).
//
// The EXTI interrupt starts the processing. It disables itself and instead
// schedules a task for further processing after about 100µs. The processing
// task reschedules itself if no complete message has yet been decoded and if
// no error has occurred. Once a message has been decoded, an error has
// occurred or no more time stamps have been recorded for about 20µs, the task
// re-enables the comparator interrupt instead of rescheduling itself.
//
// The entire message decoding happens in interrupt handlers. While it
// slightly increases the complexity for the USB PD code, it makes the code
// much more robust with regards to timing (USB PD is very timing sensitive).
// It also simplifies writing Arduino applications considerably. The
// application code does not need to worry about timing issues.

void PDPhy::prepareRead(PDMessage* msg) {
    currentMessage = msg;
    currentMessage->cc = ccActive;
    decoder.setMessage(currentMessage);
}

void PDPhySTM32F1::processData() {
    // called by scheduler - process chunk of data

    // There are two modes of operation: monitoring and power sink.
    //
    // In monitoring mode, a chunk of data can contain parts of two messages
    // (end of one and start of next one). So after a message has been
    // received, decoding needs to continue.
    //
    // In power sink mode, there a chunk will likely only contain parts of a
    // single message. Once a complete message has been decoded, the device
    // will likely start to transmit a GoodCRC message (in the
    // `onMessageReceived` callback). The rest of the chunk – if any – can be
    // discarded.

    debugOutOn();
    int currentIndex = (TIMESTAMPS_EDGE_SIZE - DMA1_Channel1->CNDTR) * 2;

    while (currentIndex != processedIndex) {

        mergeTimeStamps(currentIndex);
        auto result = static_cast<PDMessageDecoder::Result>(decodeChunk(currentIndex));

        if (result != PDMessageDecoder::Result::incomplete) {

            // event relevant for power controller has been decoded
            hasRxActivity = false; // allow switch to transmission

            switch (result) {
                case PDMessageDecoder::Result::completeMessage:
                    PowerController.onMessageReceived(currentMessage);
                    break;
                case PDMessageDecoder::Result::invalid:
                    PowerController.onError();
                    break;
                case PDMessageDecoder::Result::hardReset:
                    PowerController.onReset(PDSOPSequence::hardReset);
                    break;
                case PDMessageDecoder::Result::cableReset:
                    PowerController.onReset(PDSOPSequence::cableReset);
                    break;
            }

            if (isTransmitting) {
                // discard rest of message
                processedIndex = (TIMESTAMPS_EDGE_SIZE - DMA1_Channel1->CNDTR) * 2;
                mergedIndex = processedIndex;
                decoder.discardMessage();
                return;
            }

            if (result == PDMessageDecoder::Result::hardReset && result == PDMessageDecoder::Result::cableReset) {
                // discard rest of chunk and reset decoding
                processedIndex = (TIMESTAMPS_EDGE_SIZE - DMA1_Channel1->CNDTR) * 2;
                mergedIndex = processedIndex;
                decoder.discardMessage();
                hasRxActivity = true;
                break;
            }

            // continue with decoding
            hasRxActivity = true;
        }
    }
    
    // check for timeout (a gap without new data)
    int index = (TIMESTAMPS_EDGE_SIZE - DMA1_Channel1->CNDTR) * 2;
    uint16_t diff2 = ((uint16_t)TIM2->CNT) - decoder.lastTimeStamp();
    bool hasTimeout = processedIndex == index && diff2 > 50;

    if (!hasTimeout) {
        // schedule next data processing
        nextProcessingTime += RX_PROCESS_INTERVAL;
        Scheduler.scheduleTaskAt(processData, nextProcessingTime);

    } else {
        // done with message, wait for next one
        bool likelyError = decoder.discardMessage();
        hasRxActivity = false;

        if (likelyError)
            PowerController.onError();

        if (!isTransmitting)
            configureRx(RxConfig::rxWaitForMessage);
    }

    debugOutOff();
}

void PDPhySTM32F1::mergeTimeStamps(int currentIndex) {
    // merge time stamps of rising and falling edges into a single array

    if (mergedIndex > currentIndex) {
        // from mergedIndex to end of array
        uint16_t* src1 = &timestampsRising[mergedIndex / 2];
        uint16_t* src2 = &timestampsFalling[mergedIndex / 2];
        uint16_t* target = &timestamps[mergedIndex];
        int n = (TIMESTAMPS_SIZE - mergedIndex) / 2;
        for (int i = 0; i < n; i += 1) {
            *target++ = *src1++;
            *target++ = *src2++;
        }
        mergedIndex = 0;
    }

    if (mergedIndex < currentIndex) {
        // from mergedIndex to currentIndex
        uint16_t* src1 = &timestampsRising[mergedIndex / 2];
        uint16_t* src2 = &timestampsFalling[mergedIndex / 2];
        uint16_t* target = &timestamps[mergedIndex];
        int n = (currentIndex - mergedIndex) / 2;
        for (int i = 0; i < n; i += 1) {
            *target++ = *src1++;
            *target++ = *src2++;
        }
        mergedIndex = currentIndex;
    }
}

int PDPhySTM32F1::decodeChunk(int currentIndex) {

    if (currentIndex > processedIndex) {
        processedIndex += decoder.decodeChunk(&timestamps[processedIndex], currentIndex - processedIndex);

    } else {
        processedIndex += decoder.decodeChunk(&timestamps[processedIndex], TIMESTAMPS_SIZE - processedIndex);
        if (processedIndex == TIMESTAMPS_SIZE)
            processedIndex = 0;
    }

    return static_cast<int>(decoder.result());
}

// --- TX ---

// To transmit a message, a bit stream is created and transmitted using the
// SPI peripheral. Only MOSI is configured. The data is fed to SPI using DMA.
//
// In order to create the correct transmission voltages, the CC line is
// connected to the middle point of a voltage divider. The upper side connects
// the CC line through a 200Ω resistor to MOSI. The lower side connects it
// through a 100Ω resistor to an output pin pulling low.  When no transmission
// is on-going, both pins are in high-impedence/input mode and have no effect
// on the CC line.

void PDPhySTM32F1::initTx() {
    // enable clock for SPI1
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // Configure PB5 (MOSI/TX for CC1) and PA7 (MOSI/TX for CC2) - will be switched between input and alternate output
    gpioSetOutputHigh(GPIOB, 5);
    gpioSetOutputHigh(GPIOA, 7);
    setRegBits(GPIOB->CRL, GPIO_CRL_CNF_FLOATING(5) | GPIO_CRL_MODE_INPUT(5), GPIO_CRL_CNF_MASK(5) | GPIO_CRL_MODE_MASK(5));
    setRegBits(GPIOA->CRL, GPIO_CRL_CNF_FLOATING(7) | GPIO_CRL_MODE_INPUT(7), GPIO_CRL_CNF_MASK(7) | GPIO_CRL_MODE_MASK(7));

    // pull PA8 (CC1_TX_EN) and PA6 (CC2_TX_EN) high (to initiate a new PD communication)
    gpioSetOutputHigh(GPIOA, 6);
    gpioSetOutputHigh(GPIOA, 8);
    setRegBits(GPIOA->CRH, GPIO_CRH_CNF_OUTPUT(8) | GPIO_CRH_MODE_OUTPUT_2MHZ(8), GPIO_CRH_CNF_MASK(8) | GPIO_CRH_MODE_MASK(8));
    setRegBits(GPIOA->CRL, GPIO_CRL_CNF_OUTPUT(6) | GPIO_CRL_MODE_OUTPUT_2MHZ(6), GPIO_CRL_CNF_MASK(6) | GPIO_CRL_MODE_MASK(6));
    delay(10);

    // deactivate PA8 (CC1_TX_EN) and PA6 (CC2_TX_EN) (configure as inputs)
    setRegBits(GPIOA->CRH, GPIO_CRH_CNF_FLOATING(8) | GPIO_CRH_MODE_INPUT(8), GPIO_CRH_CNF_MASK(8) | GPIO_CRH_MODE_MASK(8));
    setRegBits(GPIOA->CRL, GPIO_CRL_CNF_FLOATING(6) | GPIO_CRL_MODE_INPUT(6), GPIO_CRL_CNF_MASK(6) | GPIO_CRL_MODE_MASK(6));
    gpioSetOutputLow(GPIOA, 6);
    gpioSetOutputLow(GPIOA, 8);

    // Configure SPI1 in TX-only mode, with 562.5 kbps
    SPI1->CR1 = (0b110 << SPI_CR1_BR_Pos) | SPI_CR1_MSTR | SPI_CR1_LSBFIRST | SPI_CR1_SSI | SPI_CR1_SSM;
    SPI1->CR2 = 0;

    // configure DMA1 / channel 3 (for SPI1_TX)
    DMA1_Channel3->CCR =
        (0b01 << DMA_CCR_PL_Pos) // priority medium
        | (0b00 << DMA_CCR_MSIZE_Pos) // memory size 8 bit
        | (0b00 << DMA_CCR_PSIZE_Pos) // peripheral size 8 bit
        | DMA_CCR_MINC // increment memory
        | (0b1 << DMA_CCR_DIR_Pos) // read from memory, write to peripheral
        | DMA_CCR_TCIE; // transfer complete interrupt
    DMA1_Channel3->CPAR = (uint32_t) &SPI1->DR;
    DMA1_Channel3->CMAR = (uint32_t) &txBitStream;

    // enable DMA1 / channel 3 interrupt
    NVIC_SetPriority(DMA1_Channel3_IRQn, NVIC_GetPriority(TIM2_IRQn));
    NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

bool PDPhy::transmitMessage(const PDMessage* msg) {
    return PDPhySTM32F1::transmitMessage(msg);
}

bool PDPhySTM32F1::transmitMessage(const PDMessage* msg) {
    debugOutOn();

    __disable_irq();
    if (hasRxActivity || isTransmitting || ccActive == 0) {
        // Cannot currently send the message
        __enable_irq();
        debugOutOff();
        return false;
    }

    isTransmitting = true;
    __enable_irq();

    txBitStreamLength = PDMessageEncoder::encode(msg, txBitStream);

    __disable_irq();
    // test again for RX activity
    if (hasRxActivity) {
        // Cannot currently send the message
        __enable_irq();
        debugOutOff();
        return false;
    }

    configureRx(RxConfig::pausedForTx);

    debugOutOff();

    if (ccActive == 1) {
        setRegBits(AFIO->MAPR, 0b1 << AFIO_MAPR_SPI1_REMAP_Pos, AFIO_MAPR_SPI1_REMAP_Msk); // remap PB5 to MOSI

        // enable PB5 as MOSI for CC1
        setRegBits(GPIOB->CRL, GPIO_CRL_CNF_ALTERNATE(5) | GPIO_CRL_MODE_OUTPUT_2MHZ(5), GPIO_CRL_CNF_MASK(5) | GPIO_CRL_MODE_MASK(5));

        // enable PA8 as CC1_TX_EN
        setRegBits(GPIOA->CRH, GPIO_CRH_CNF_OUTPUT(8) | GPIO_CRH_MODE_OUTPUT_2MHZ(8), GPIO_CRH_CNF_MASK(8) | GPIO_CRH_MODE_MASK(8));

    } else {
        setRegBits(AFIO->MAPR, 0b0 << AFIO_MAPR_SPI1_REMAP_Pos, AFIO_MAPR_SPI1_REMAP_Msk); // remap PA7 to MOSI

        // enable PA7 as MOSI for CC2
        setRegBits(GPIOA->CRL, GPIO_CRL_CNF_ALTERNATE(7) | GPIO_CRL_MODE_OUTPUT_2MHZ(7), GPIO_CRL_CNF_MASK(7) | GPIO_CRL_MODE_MASK(7));

        // enable PA6 as CC2_TX_EN
        setRegBits(GPIOA->CRL, GPIO_CRL_CNF_OUTPUT(6) | GPIO_CRL_MODE_OUTPUT_2MHZ(6), GPIO_CRL_CNF_MASK(6) | GPIO_CRL_MODE_MASK(6));
    }
    
    DMA1_Channel3->CNDTR = txBitStreamLength;
    setRegBits(DMA1_Channel3->CCR, DMA_CCR_EN, DMA_CCR_EN_Msk);
    setRegBits(SPI1->CR2, SPI_CR2_TXDMAEN, SPI_CR2_TXDMAEN_Msk);
    setRegBits(SPI1->CR1, SPI_CR1_SPE, SPI_CR1_SPE);

    __enable_irq();
    return true;
}

extern "C" void DMA1_Channel3_IRQHandler() {
    // SPI1 TX has completed
    PDPhySTM32F1::messageTransmitted();
}

void PDPhySTM32F1::messageTransmitted() {
    // disable DMA
    clearRegBits(DMA1_Channel3->CCR, DMA_CCR_EN_Msk);
    DMA1->IFCR = DMA_IFCR_CTCIF3; // clear interrupt

    // Wait until the last byte has been transmitted
    while ((SPI1->SR & SPI_SR_BSY_Msk) != 0)
        ;

    // restore all pins as inputs
    if (ccActive == 1) {
        setRegBits(GPIOB->CRL, GPIO_CRL_CNF_FLOATING(5) | GPIO_CRL_MODE_INPUT(5), GPIO_CRL_CNF_MASK(5) | GPIO_CRL_MODE_MASK(5));
        setRegBits(GPIOA->CRH, GPIO_CRH_CNF_FLOATING(8) | GPIO_CRH_MODE_INPUT(8), GPIO_CRH_CNF_MASK(8) | GPIO_CRH_MODE_MASK(8));
    } else {
        setRegBits(GPIOA->CRL, GPIO_CRL_CNF_FLOATING(7) | GPIO_CRL_MODE_INPUT(7), GPIO_CRL_CNF_MASK(7) | GPIO_CRL_MODE_MASK(7));
        setRegBits(GPIOA->CRL, GPIO_CRL_CNF_FLOATING(6) | GPIO_CRL_MODE_INPUT(6), GPIO_CRL_CNF_MASK(6) | GPIO_CRL_MODE_MASK(6));
    }

    clearRegBits(SPI1->CR1, SPI_CR1_SPE);
    clearRegBits(SPI1->CR2, SPI_CR2_TXDMAEN_Msk);

    isTransmitting = false;
    // wait until PA15 (CC_3V3) is low (otherwise falling and rising timestamp can be swapped)
    while ((GPIOA->IDR & (1 << 15)) != 0)
        ;
    configureRx(RxConfig::rxWaitForMessage);

    PowerController.onMessageTransmitted(true);
}

#endif
