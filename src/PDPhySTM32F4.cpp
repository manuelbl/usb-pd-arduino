//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#if defined(STM32F401xC)

#include <Arduino.h>
#include <string.h>
#include "CMSISHelper.h"
#include "PDController.h"
#include "PDMessageDecoder.h"
#include "PDMessageEncoder.h"
#include "PDPhySTM32F4.h"
#include "TaskScheduler.h"

static constexpr uint8_t CC1_ADC_CHAN = 3; // PA3
static constexpr uint8_t CC2_ADC_CHAN = 2; // PA2
static constexpr uint32_t ADC_SQR3_CC1 = (CC1_ADC_CHAN << ADC_SQR3_SQ1_Pos); // Single conversion of channel 3 (PA3)
static constexpr uint32_t ADC_SQR3_CC2 = (CC2_ADC_CHAN << ADC_SQR3_SQ1_Pos); // Single conversion of channel 2 (PA2)

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
static uint16_t timestamps[TIMESTAMPS_SIZE];

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
    PDPhySTM32F4::initRx();
}

void PDPhy::initSink() {
    PDPhySTM32F4::initRx();
    PDPhySTM32F4::initTx();
}


// --- Voltage monitoring ---

// The voltage on CC1 and CC2 is regularly measured to detect the connection,
// disconnection and role change of power sink and source. The analog-to-
// digital conversion start is triggered from timer 2 / channel 3. Once it is
// complete, an interrupt is triggered to process the result and schedule the
// next conversion. CC1 and CC2 are measured alternately.

static inline int ccLevelFromADC(uint32_t adcValue) {
    return adcValue < CC_TRESHOLD_LOW ? 0
        : (adcValue > CC_TRESHOLD_HIGH ? 2 : 1);
}

void PDPhySTM32F4::switchCC() {
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

void PDPhySTM32F4::switchAfterDebounce(uint8_t cc) {
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

extern "C" void ADC_IRQHandler() {
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
        PDPhySTM32F4::switchAfterDebounce(newCC);

    // prepare and schedule next measurment
    uint32_t cnt = TIM2->CNT;
    TIM2->CCR3 = (cnt + CC_SAMPLE_INTERVAL) & 0xffff;
    ADC1->SQR3 = ADC1->SQR3 == ADC_SQR3_CC1 ? ADC_SQR3_CC2 : ADC_SQR3_CC1;

    debugOutOff();
}

// --- RX ---

// The CC pins must externally be level-shifted, inverted and combined. It can
// be done with a dual comparator with open collector output. The level-
// shifted signal is then connected to PA1.
//
// PA1 is connected to timer 2 channel 2. Then channel is configured in input
// capture mode. Using DMA, the captured timer values (basically timestamps)
// are written to a circular buffer.
//
// Additionally, an EXTI interrupt is set up on PA1. See "Data processing"
// below for more information.

void PDPhySTM32F4::initRx() {
    // enable clock for GPIOA, GPIOB and DMA
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_DMA1EN;
    // enable clock for DMA1
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    
    // wait for clock
    asm("nop");
    asm("nop");

    pinMode(PA1, INPUT);

    // setup EXTI interrupt for PA1
    attachInterrupt(PA1, onEXTIInterrupt, RISING);
    clearRegBits(EXTI->IMR, EXTI_IMR_MR1_Msk); // disable interrupt for now

    // configure TIM2
    timer.setPrescaleFactor((timer.getTimerClkFreq() + TIMER_FREQ / 2) / TIMER_FREQ);
    timer.setOverflow(0x10000); // full 16-bit range

    // channel 2 in input capture mode, triggered by PA1 on the rising and falling edge
    // channel 3 in output compare mode (for triggering the ADC)
    timer.setMode(2, TIMER_INPUT_CAPTURE_BOTHEDGE, PA1);
    setRegBits(GPIOA->AFR[0], GPIO_AFRL(1, 1), GPIO_AFRL_Msk(1)); // PA1 should be alternate function 1, not 2
    timer.setMode(3, TIMER_OUTPUT_COMPARE_TOGGLE);
    setRegBits(TIM2->DIER, TIM_DIER_CC2DE, TIM_DIER_CC2DE_Msk); // enable DMA requests on trigger

    timer.refresh();
    timer.resume();

    // configure DMA1 / stream 6 (connected to timer 2 / channel 2)
    DMA1_Stream6->CR =
          DMA_SxCR_CIRC   // circular mode
        | DMA_SxCR_MINC   // increment memory
        | (0b01 << DMA_SxCR_PSIZE_Pos)  // peripheral size 16 bit
        | (0b01 << DMA_SxCR_MSIZE_Pos)  // memory size 16 bit
        | (0b01 << DMA_SxCR_PL_Pos   )  // medium priority
        | (   3 << DMA_SxCR_CHSEL_Pos); // DMA channel 3

    DMA1_Stream6->NDTR = TIMESTAMPS_SIZE;
    DMA1_Stream6->PAR = (uint32_t) &TIM2->CCR2;
    DMA1_Stream6->M0AR = (uint32_t) &timestamps[0];

    // same priority as timer (so they don't interrupt each other)
    NVIC_SetPriority(EXTI1_IRQn, NVIC_GetPriority(TIM2_IRQn));

    // startup ADC
    ADC1->CR1 = 0; // default config for ADC
    ADC1->CR2 = ADC_CR2_ADON; // enable ADC
    delayMicroseconds(2);

    // configure PA3 (for CC1 on ADC1 channel 3) and PA2 (for CC2 on ADC1 channel 2) as inputs
    pinMode(PA3, INPUT_ANALOG);
    pinMode(PA2, INPUT_ANALOG);

    // configure ADC
    setRegBits(ADC1->CR2, (0b0100 << ADC_CR2_EXTSEL_Pos) | (0b11 << ADC_CR2_EXTEN_Pos), ADC_CR2_EXTSEL_Msk | ADC_CR2_EXTEN_Msk); // triggered by timer 2 / channel 3
    ADC1->SQR1 = 0 << ADC_SQR1_L_Pos; // a single conversion
    ADC1->SQR3 = CC1_ADC_CHAN << ADC_SQR3_SQ1_Pos; // convert channel 3

    // enable ADC1 interrupt (trigger after AD conversion has completed)
    NVIC_SetPriority(ADC_IRQn, NVIC_GetPriority(TIM2_IRQn));
    NVIC_EnableIRQ(ADC_IRQn);

    if (useDebugOut) {
        // configure PB3 as debug output
        debugOutOff();
        pinMode(PB3, OUTPUT);
    }

    ccLevel[0] = -1;
    ccLevel[1] = -1;

    configureRx(RxConfig::monitorCC);
}

void PDPhySTM32F4::configureRx(RxConfig config) {

    if (config == RxConfig::monitorCC || config == RxConfig::rxWaitForMessage) {
        // enable voltage measurement
        uint32_t cnt = TIM2->CNT;
        TIM2->CCR3 = (cnt + CC_SAMPLE_INTERVAL) & 0xffff;
        setRegBits(ADC1->CR1, ADC_CR1_EOCIE, ADC_CR1_EOCIE_Msk); // enable interrupt
    
    } else {
        // disable voltage measurement
        clearRegBits(ADC1->CR1, ADC_CR1_EOCIE_Msk); // disable interrupt
    }

    if (config == RxConfig::rxWaitForMessage) {
        // enable EXTI interrupt
        setRegBits(EXTI->IMR, EXTI_IMR_MR1, EXTI_IMR_MR1_Msk);
    
    } else {
        // disable EXTI interrupt
        clearRegBits(EXTI->IMR, EXTI_IMR_MR1_Msk);
    }

    // clear EXTI interrupts
    EXTI->PR = EXTI_PR_PR1;

    if (config == RxConfig::rxWaitForMessage) {
        // disable DMA to reconfigure it
        clearRegBits(DMA1_Stream6->CR, DMA_SxCR_EN_Msk);
        processedIndex = 0;
        DMA1_Stream6->NDTR = TIMESTAMPS_SIZE;

        // enable DMA
        setRegBits(DMA1_Stream6->CR, DMA_SxCR_EN, DMA_SxCR_EN_Msk);

    } else if (config != RxConfig::rxProcessMessage) {
        // disable DMA for time stamps
        clearRegBits(DMA1_Stream6->CR, DMA_SxCR_EN_Msk);
    }
}

void PDPhySTM32F4::onEXTIInterrupt() {
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

void PDPhySTM32F4::processData() {
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
    int currentIndex = TIMESTAMPS_SIZE - DMA1_Stream6->NDTR;

    while (currentIndex != processedIndex) {

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
                processedIndex = TIMESTAMPS_SIZE - DMA1_Stream6->NDTR;
                decoder.discardMessage();
                return;
            }

            if (result == PDMessageDecoder::Result::hardReset && result == PDMessageDecoder::Result::cableReset) {
                // discard rest of chunk and reset decoding
                processedIndex = TIMESTAMPS_SIZE - DMA1_Stream6->NDTR;
                decoder.discardMessage();
                hasRxActivity = true;
                break;
            }

            // continue with decoding
            hasRxActivity = true;
        }
    }
    
    // check for timeout (a gap without new data)
    int index = TIMESTAMPS_SIZE - DMA1_Stream6->NDTR;
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

int PDPhySTM32F4::decodeChunk(int currentIndex) {

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

void PDPhySTM32F4::initTx() {
    // enable clock for SPI1
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    // enable clock for DMA2
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // Configure PA7 (MOSI/TX for CC1) and PB5 (MOSI/TX for CC2) - will be switched between input and alternate output
    gpioSetOutputHigh(GPIOA, 7);
    gpioSetOutputHigh(GPIOB, 5);
    setRegBits(GPIOA->MODER, GPIO_MODER_INPUT(7), GPIO_MODER_Msk(7));
    setRegBits(GPIOA->OTYPER, GPIO_OTYPER_PUSH_PULL(7), GPIO_OTYPER_Msk(7));
    setRegBits(GPIOA->OSPEEDR, GPIO_OSPEEDR_MEDIUM(7), GPIO_OSPEEDR_Msk(7));
    setRegBits(GPIOA->PUPDR, GPIO_PUPDR_NONE(7), GPIO_PUPDR_Msk(7));
    setRegBits(GPIOA->AFR[0], GPIO_AFRL(7, 5), GPIO_AFRL_Msk(7));
    setRegBits(GPIOB->MODER, GPIO_MODER_INPUT(5), GPIO_MODER_Msk(5));
    setRegBits(GPIOB->OTYPER, GPIO_OTYPER_PUSH_PULL(5), GPIO_OTYPER_Msk(5));
    setRegBits(GPIOB->OSPEEDR, GPIO_OSPEEDR_MEDIUM(5), GPIO_OSPEEDR_Msk(5));
    setRegBits(GPIOB->PUPDR, GPIO_PUPDR_NONE(5), GPIO_PUPDR_Msk(5));
    setRegBits(GPIOB->AFR[0], GPIO_AFRL(5, 5), GPIO_AFRL_Msk(5));

    // pull PA6 (CC1_TX_EN) and PB4 (CC2_TX_EN) high (to initiate a new PD communication)
    gpioSetOutputHigh(GPIOA, 6);
    gpioSetOutputHigh(GPIOB, 4);
    setRegBits(GPIOA->MODER, GPIO_MODER_OUTPUT(6), GPIO_MODER_Msk(6));
    setRegBits(GPIOA->OTYPER, GPIO_OTYPER_PUSH_PULL(6), GPIO_OTYPER_Msk(6));
    setRegBits(GPIOA->OSPEEDR, GPIO_OSPEEDR_MEDIUM(6), GPIO_OSPEEDR_Msk(6));
    setRegBits(GPIOA->PUPDR, GPIO_PUPDR_NONE(6), GPIO_PUPDR_Msk(6));
    setRegBits(GPIOB->MODER, GPIO_MODER_OUTPUT(4), GPIO_MODER_Msk(4));
    setRegBits(GPIOB->OTYPER, GPIO_OTYPER_PUSH_PULL(4), GPIO_OTYPER_Msk(4));
    setRegBits(GPIOB->OSPEEDR, GPIO_OSPEEDR_MEDIUM(4), GPIO_OSPEEDR_Msk(4));
    setRegBits(GPIOB->PUPDR, GPIO_PUPDR_NONE(4), GPIO_PUPDR_Msk(4));

    delay(10);

    // deactivate PA6 (CC1_TX_EN) and PA4 (CC2_TX_EN) (configure as inputs)
    setRegBits(GPIOA->MODER, GPIO_MODER_INPUT(6), GPIO_MODER_Msk(6));
    setRegBits(GPIOB->MODER, GPIO_MODER_INPUT(4), GPIO_MODER_Msk(4));
    gpioSetOutputLow(GPIOA, 6);
    gpioSetOutputLow(GPIOB, 4);

    // Configure SPI1 in TX-only mode, with 562.5 kbps
    SPI1->CR1 = SPI_CR1_MSTR | (0b110 << SPI_CR1_BR_Pos) | SPI_CR1_LSBFIRST | SPI_CR1_SSI | SPI_CR1_SSM;
    SPI1->CR2 = 0;

    // configure DMA2 / stream 3 (for SPI1_TX)
    DMA2_Stream3->CR =
          (0b01 << DMA_SxCR_DIR_Pos)    // from memory to peripheral
        | DMA_SxCR_MINC                 // increment memory
        | (0b00 << DMA_SxCR_PSIZE_Pos)  // peripheral size 8 bit
        | (0b00 << DMA_SxCR_MSIZE_Pos)  // memory size 8 bit
        | (0b01 << DMA_SxCR_PL_Pos   )  // medium priority
        | (   3 << DMA_SxCR_CHSEL_Pos)  // DMA channel 3
        | DMA_SxCR_TCIE;                // transfer complete interrupt

    DMA2_Stream3->NDTR = 0;
    DMA2_Stream3->PAR = (uint32_t) &SPI1->DR;
    DMA2_Stream3->M0AR = (uint32_t) &txBitStream[0];

    // enable DMA2 / stream 3 interrupt
    NVIC_SetPriority(DMA2_Stream3_IRQn, NVIC_GetPriority(TIM2_IRQn));
    NVIC_EnableIRQ(DMA2_Stream3_IRQn);
}

bool PDPhy::transmitMessage(const PDMessage* msg) {
    return PDPhySTM32F4::transmitMessage(msg);
}

bool PDPhySTM32F4::transmitMessage(const PDMessage* msg) {
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
        // enable PA7 as MOSI for CC1
        setRegBits(GPIOA->MODER, GPIO_MODER_ALTERNATE(7), GPIO_MODER_Msk(7));

        // enable PA6 as CC1_TX_EN
        setRegBits(GPIOA->MODER, GPIO_MODER_OUTPUT(6), GPIO_MODER_Msk(6));

    } else {
        // enable PB5 as MOSI for CC2
        setRegBits(GPIOB->MODER, GPIO_MODER_ALTERNATE(5), GPIO_MODER_Msk(5));

        // enable PB4 as CC2_TX_EN
        setRegBits(GPIOB->MODER, GPIO_MODER_OUTPUT(4), GPIO_MODER_Msk(4));
    }
    
    // configure and enable DMA (for SPI1_TX)
    DMA2_Stream3->NDTR = txBitStreamLength;
    setRegBits(DMA2_Stream3->CR , DMA_SxCR_EN, DMA_SxCR_EN_Msk);
    setRegBits(SPI1->CR2, SPI_CR2_TXDMAEN, SPI_CR2_TXDMAEN_Msk);
    setRegBits(SPI1->CR1, SPI_CR1_SPE, SPI_CR1_SPE_Msk);

    __enable_irq();
    return true;
}

extern "C" void DMA2_Stream3_IRQHandler() {
    // SPI1 TX has completed
    PDPhySTM32F4::messageTransmitted();
}

void PDPhySTM32F4::messageTransmitted() {
    // disable DMA
    clearRegBits(DMA2_Stream3->CR, DMA_SxCR_EN_Msk);
    DMA2->LIFCR = DMA_LIFCR_CTCIF3; // clear interrup

    // Wait until the last byte has been transmitted
    while ((SPI1->SR & SPI_SR_BSY_Msk) != 0)
        ;

    // restore all pins as inputs
    if (ccActive == 1) {
        setRegBits(GPIOA->MODER, GPIO_MODER_INPUT(7), GPIO_MODER_Msk(7));
        setRegBits(GPIOA->MODER, GPIO_MODER_INPUT(6), GPIO_MODER_Msk(6));
    } else {
        setRegBits(GPIOB->MODER, GPIO_MODER_INPUT(5), GPIO_MODER_Msk(5));
        setRegBits(GPIOB->MODER, GPIO_MODER_INPUT(4), GPIO_MODER_Msk(4));
    }

    clearRegBits(SPI1->CR1, SPI_CR1_SPE);
    clearRegBits(SPI1->CR2, SPI_CR2_TXDMAEN_Msk);

    isTransmitting = false;
    configureRx(RxConfig::rxWaitForMessage);

    PowerController.onMessageTransmitted(true);
}

#endif
