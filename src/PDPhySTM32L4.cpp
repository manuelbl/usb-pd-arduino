//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#if defined(STM32L4xx)

#include <Arduino.h>
#include <string.h>
#include "CMSISHelper.h"
#include "PDController.h"
#include "PDMessageDecoder.h"
#include "PDMessageEncoder.h"
#include "PDPhySTM32L4.h"
#include "TaskScheduler.h"


static constexpr uint8_t CC1_ADC_CHAN = 9; // PA4
static constexpr uint8_t CC2_ADC_CHAN = 10; // PA5
static constexpr uint32_t ADC_SQR1_CC1 = (CC1_ADC_CHAN << ADC_SQR1_SQ1_Pos) | (0 << ADC_SQR1_L_Pos); // Single conversion of channel 9 (PA4)
static constexpr uint32_t ADC_SQR1_CC2 = (CC2_ADC_CHAN << ADC_SQR1_SQ1_Pos) | (0 << ADC_SQR1_L_Pos); // Single conversion of channel 10 (PA5)

static constexpr int TIMER_FREQ = 2400000; // Hz
static constexpr int RX_PROCESS_INTERVAL = 89; // µs
static constexpr int CC_SAMPLE_INTERVAL = 500 * (TIMER_FREQ / 1000) / 1000;

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
static constexpr int BUF_SIZE = 130;
static uint16_t buffer[BUF_SIZE];

static int processedIndex;
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

// PA8 can be configured as debug output pin. It indicates certain activities such as
// ADC conversion complete, comparator interrupt, processing of received data.

#if USBPD_DEBUG_PIN
static constexpr bool useDebugOut = true;
#else
static constexpr bool useDebugOut = false;
#endif
static inline void debugOutOn() { if (useDebugOut) gpioSetOutputHigh(GPIOA, 8); }
static inline void debugOutOff() { if (useDebugOut) gpioSetOutputLow(GPIOA, 8); }


// --- Initialization ---

void PDPhy::initMonitor() {
    PDPhySTM32L4::initRx();
}

void PDPhy::initSink() {
    PDPhySTM32L4::initRx();
    PDPhySTM32L4::initTx();
}


// --- Voltage monitoring ---

// The voltage on CC1 and CC2 is regularly measured to detect the connection,
// disconnection and role change of power sink and source. The analog-to-
// digital conversion start is triggered from timer channel 2. Once it is
// complete, an interrupt is triggered to process the result and schedule the
// next conversion. CC1 and CC2 are measured alternately.

static inline int ccLevelFromADC(uint32_t adcValue) {
    return adcValue < CC_TRESHOLD_LOW ? 0
        : (adcValue > CC_TRESHOLD_HIGH ? 2 : 1);
}

void PDPhySTM32L4::switchCC() {
    // callback after debouncing period
    ccActive = ccAfterDebounce;

    if (ccActive != 0) {
        enablePDCommunication();
        configureRx(RxConfig::rxWaitForMessage);
    } else {
        disablePDCommunication();
        configureRx(RxConfig::monitorCC);
        Scheduler.cancelTask(processData);
    }

    if (currentMessage != nullptr)
        currentMessage->cc = ccActive;

    PowerController.onVoltageChanged(ccActive);
}

void PDPhySTM32L4::switchAfterDebounce(uint8_t cc) {
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

extern "C" void ADC1_IRQHandler() {
    // CC measurement complete
    debugOutOn();

    int cc = ADC1->SQR1 == ADC_SQR1_CC1 ? 1 : 2;
    uint16_t value = ADC1->DR;
    ccLevel[cc - 1] = ccLevelFromADC(value);

    uint8_t newCC = 0;
    if (ccLevel[0] == 1 && (ccLevel[1] & 1) == 0) {
        newCC = 1;
    } else if ((ccLevel[0] & 1) == 0 && ccLevel[1] == 1) {
        newCC = 2;
    }

    if (newCC != ccAfterDebounce)
        PDPhySTM32L4::switchAfterDebounce(newCC);

    // prepare and schedule next measurment
    ADC1->ISR = ADC_ISR_EOS;
    uint32_t cnt = TIM2->CNT;
    TIM2->CCR2 = (cnt + CC_SAMPLE_INTERVAL) & 0xffff;
    ADC1->SQR1 = ADC1->SQR1 != ADC_SQR1_CC1 ? ADC_SQR1_CC1 : ADC_SQR1_CC2;

    debugOutOff();
}


// --- RX ---

// The CC pin is connected to the input (plus) of a comparator. The other
// input of the comparator (minus) is internally connected to 1/4 of VCC,
// resulting in a reference voltage of 0.82V.
//
// The comparator output is connected to the input of a timer channel.
// Whenever the comparator output changes, the current timer value will be
// written to the channel's capture register. At the same time, a DMA
// transaction is triggered. It writes the capture register value to a circular
// buffer. That way the time stamps of all transitions on the CC line are
// saved in a buffer.

void PDPhySTM32L4::initRx() {
    // enable clock for TIM2
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
    // enable clock for COMP1/2 (SYSCFG clock)
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    // enable clock for DMA1 and CRC
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_CRCEN;
    // enable clock for GPIOA, GPIOB and ADC1
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_ADCEN;
    
    // wait for clock
    asm("nop");
    asm("nop");

    // configure comparator 1
    COMP1->CSR = 
          (0b10 << COMP_CSR_INPSEL_Pos) // select PA1 as input+
        | COMP_CSR_SCALEN // enable Vrefint scaling
        | COMP_CSR_BRGEN // enable voltage bridge
        | (0b000 << COMP_CSR_INMSEL_Pos) // 1/4 Vrefint as input-
        | (0b11 << COMP_CSR_HYST_Pos) // high hysteresis
        | ( 0 << COMP_CSR_PWRMODE_Pos); // high speed mode

    // comparator interrupt on both rising and falling edge
    setRegBits(EXTI->RTSR1, EXTI_COMP1, EXTI_COMP1_Msk);
    setRegBits(EXTI->FTSR1, EXTI_COMP1, EXTI_COMP1_Msk);

    // For debugging: configure PA6 as COMP1_OUT
    if (useDebugOut) {
        setRegBits(GPIOA->MODER, GPIO_MODER_ALTERNATE(6), GPIO_MODER_Msk(6));
        setRegBits(GPIOA->OTYPER, GPIO_OTYPER_PUSH_PULL(6), GPIO_OTYPER_Msk(6));
        setRegBits(GPIOA->OSPEEDR, GPIO_OSPEEDR_MEDIUM(6), GPIO_OSPEEDR_Msk(6));
        setRegBits(GPIOA->PUPDR, GPIO_PUPDR_NONE(6), GPIO_PUPDR_Msk(6));
        setRegBits(GPIOA->AFR[0], GPIO_AFRL(6, 6), GPIO_AFRL_Msk(6));
    }

    // configure comparator 2
    COMP2->CSR = 
          (0b10 << COMP_CSR_INPSEL_Pos) // select PA3 as input+
        | COMP_CSR_SCALEN // enable Vrefint scaling
        | COMP_CSR_BRGEN // enable voltage bridge
        | (0b000 << COMP_CSR_INMSEL_Pos) // 1/4 Vrefint as input-
        | (0b11 << COMP_CSR_HYST_Pos) // high hysteresis
        | ( 0 << COMP_CSR_PWRMODE_Pos); // high speed mode

    // comparator interrupt on both rising and falling edge
    setRegBits(EXTI->RTSR1, EXTI_COMP2, EXTI_COMP2_Msk);
    setRegBits(EXTI->FTSR1, EXTI_COMP2, EXTI_COMP2_Msk);

    // For debugging: configure PA7 as COMP2_OUT
    if (useDebugOut) {
        setRegBits(GPIOA->MODER, GPIO_MODER_ALTERNATE(7), GPIO_MODER_Msk(7));
        setRegBits(GPIOA->OTYPER, GPIO_OTYPER_PUSH_PULL(7), GPIO_OTYPER_Msk(7));
        setRegBits(GPIOA->OSPEEDR, GPIO_OSPEEDR_MEDIUM(7), GPIO_OSPEEDR_Msk(7));
        setRegBits(GPIOA->PUPDR, GPIO_PUPDR_NONE(7), GPIO_PUPDR_Msk(7));
        setRegBits(GPIOA->AFR[0], GPIO_AFRL(7, 12), GPIO_AFRL_Msk(7));
    }

    // configure TIM2
    timer.setPrescaleFactor((timer.getTimerClkFreq() + TIMER_FREQ / 2) / TIMER_FREQ);
    timer.setOverflow(0x10000); // full 16-bit range

    // channel 2 in output compare mode (for triggering the ADC)
    // channel 4 in input capture mode, triggered by comparator
    timer.setMode(2, TIMER_OUTPUT_COMPARE_TOGGLE);
    timer.setMode(4, TIMER_INPUT_CAPTURE_BOTHEDGE);
    setRegBits(TIM2->DIER, TIM_DIER_CC4DE, TIM_DIER_CC4DE_Msk); // enable DMA requests on trigger
    setRegBits(TIM2->OR1, 0b01 << TIM2_OR1_TI4_RMP_Pos, TIM2_OR1_TI4_RMP_Msk); // remap COMP1_OUT to input 4

    timer.refresh();
    timer.resume();

    // configure DMA1 / channel 7
    DMA1_Channel7->CCR =
        (0b01 << DMA_CCR_PL_Pos) // priority medium
        | (0b01 << DMA_CCR_MSIZE_Pos) // memory size 16 bit
        | (0b01 << DMA_CCR_PSIZE_Pos) // peripheral size 16 bit
        | DMA_CCR_MINC // increment memory
        | DMA_CCR_CIRC; // circular mode
    DMA1_Channel7->CNDTR = BUF_SIZE;
    DMA1_Channel7->CPAR = (uint32_t) &TIM2->CCR4;
    DMA1_Channel7->CMAR = (uint32_t) &buffer;
    setRegBits(DMA1_CSELR->CSELR, 0b0100 << DMA_CSELR_C7S_Pos, DMA_CSELR_C7S_Msk);

    // initialize CRC
    CRC->POL = 0x04C11DB7;
    CRC->INIT = 0xFFFFFFFF;

    // enable comparator interrupt
    NVIC_SetPriority(COMP_IRQn, NVIC_GetPriority(TIM2_IRQn));
    NVIC_EnableIRQ(COMP_IRQn);

    // startup ADC
    ADC1->CR = 0; // disable deep power down mode
    ADC1->CR = ADC_CR_ADVREGEN; // enable ADC regulator
    delayMicroseconds(20); // wait 20µs for the ADC regulator to stabilize

    // calibrate ADC
    ADC1_COMMON->CCR = 0b11 << ADC_CCR_CKMODE_Pos;
    ADC1->CR |= ADC_CR_ADCAL; // calibrate
    while ((ADC1->CR & ADC_CR_ADCAL) != 0)
        ;

    // configure ADC
    setRegBits(ADC1->CFGR,
        (0b11 << ADC_CFGR_EXTEN_Pos) | (0b0011 << ADC_CFGR_EXTSEL_Pos) | ADC_CFGR_OVRMOD,
        ADC_CFGR_EXTEN_Msk | ADC_CFGR_EXTSEL_Msk | ADC_CFGR_OVRMOD_Msk); // Timer 2 channel 2 as external trigger
    ADC1->CFGR2 = ADC_CFGR2_ROVSE | (0b0100 << ADC_CFGR2_OVSR_Pos) | (0b0101 << ADC_CFGR2_OVSS_Pos); // 32x oversampling (filtering)
    ADC1->SQR1 = ADC_SQR1_CC1;
    ADC1->IER |= ADC_IER_EOSIE; // enable interrupt
    ADC1->CR |= ADC_CR_ADEN; // enable ADC
    while ((ADC1->ISR & ADC_ISR_ADRDY_Msk) == 0) // wait until it is ready
        ;

    // enable ADC1 interrupt
    NVIC_SetPriority(ADC1_IRQn, NVIC_GetPriority(COMP_IRQn));
    NVIC_EnableIRQ(ADC1_IRQn);

    if (useDebugOut) {
        // configure PA8 as debug output
        debugOutOff();
        setRegBits(GPIOA->MODER, GPIO_MODER_OUTPUT(8), GPIO_MODER_Msk(8));
        setRegBits(GPIOA->OTYPER, GPIO_OTYPER_PUSH_PULL(8), GPIO_OTYPER_Msk(8));
        setRegBits(GPIOA->OSPEEDR, GPIO_OSPEEDR_MEDIUM(8), GPIO_OSPEEDR_Msk(8));
        setRegBits(GPIOA->PUPDR, GPIO_PUPDR_NONE(8), GPIO_PUPDR_Msk(8));
    }

    // configure PB4 as analog (if TX is not configured)
    setRegBits(GPIOB->MODER, GPIO_MODER_ANALOG(4), GPIO_MODER_Msk(4));

    ccLevel[0] = -1;
    ccLevel[1] = -1;

    configureRx(RxConfig::monitorCC);
}

void PDPhySTM32L4::configureRx(RxConfig config) {

    if (config == RxConfig::monitorCC || config == RxConfig::rxWaitForMessage) {
        // enable voltage measurement
        uint32_t cnt = TIM2->CNT;
        TIM2->CCR2 = (cnt + CC_SAMPLE_INTERVAL) & 0xffff;
        setRegBits(ADC1->CR, ADC_CR_ADSTART, ADC_CR_ADSTART_Msk); // start ADC
    
    } else {
        // disable voltage measurement
        setRegBits(ADC1->CR, ADC_CR_ADSTP, ADC_CR_ADSTP_Msk); // stop ADC
    }

    if (config == RxConfig::rxWaitForMessage) {
        // enable COMP interrupt
        if (ccActive == 1)
            setRegBits(EXTI->IMR1, EXTI_COMP1, EXTI_COMP1_Msk);
        else
            setRegBits(EXTI->IMR1, EXTI_COMP2, EXTI_COMP2_Msk);
    
    } else {
        // disable COMP interrupt
        clearRegBits(EXTI->IMR1, ccActive == 1 ? EXTI_COMP1_Msk : EXTI_COMP2_Msk);

    }
    // clear comparator interrupts
    EXTI->PR1 = EXTI_COMP1 | EXTI_COMP2;

    if (config == RxConfig::rxWaitForMessage || config == RxConfig::rxProcessMessage) {
        // enable DMA for time stamps
        setRegBits(DMA1_Channel7->CCR, DMA_CCR_EN, DMA_CCR_EN_Msk);

    } else {
        // disable DMA for time stamps
        clearRegBits(DMA1_Channel7->CCR, DMA_CCR_EN_Msk);
    }
}

void PDPhySTM32L4::enablePDCommunication() {
    if (ccActive == 1) {
        // enable comparator 1, DMA, timer and COMP1 interrupt
        clearRegBits(COMP2->CSR, COMP_CSR_EN_Msk);
        setRegBits(COMP1->CSR, COMP_CSR_EN, COMP_CSR_EN_Msk);
        setRegBits(TIM2->OR1, 0b01 << TIM2_OR1_TI4_RMP_Pos, TIM2_OR1_TI4_RMP_Msk); // remap COMP1_OUT to input 4
        setRegBits(TIM2->DIER, TIM_DIER_CC4DE, TIM_DIER_CC4DE_Msk); // enable DMA requests on trigger
        setRegBits(EXTI->IMR1, EXTI_COMP1, EXTI_COMP1_Msk);
        clearRegBits(EXTI->IMR1, EXTI_COMP2_Msk);

    } else {
        // enable comparator 2, DMA, timer and COMP2 interrupt
        clearRegBits(COMP1->CSR, COMP_CSR_EN_Msk);
        setRegBits(COMP2->CSR, COMP_CSR_EN, COMP_CSR_EN_Msk);
        setRegBits(TIM2->OR1, 0b10 << TIM2_OR1_TI4_RMP_Pos, TIM2_OR1_TI4_RMP_Msk); // remap COMP2_OUT to input 4
        setRegBits(TIM2->DIER, TIM_DIER_CC4DE, TIM_DIER_CC4DE_Msk); // enable DMA requests on trigger
        setRegBits(EXTI->IMR1, EXTI_COMP2, EXTI_COMP2_Msk);
        clearRegBits(EXTI->IMR1, EXTI_COMP1_Msk);
    }
}

void PDPhySTM32L4::disablePDCommunication() {
    // disable comparators, DMA, timer and interrupts
    clearRegBits(TIM2->DIER, TIM_DIER_CC4DE_Msk); // disable DMA requests on trigger
    clearRegBits(COMP1->CSR, COMP_CSR_EN_Msk);
    clearRegBits(COMP2->CSR, COMP_CSR_EN_Msk);
    clearRegBits(EXTI->IMR1, EXTI_COMP1_Msk | EXTI_COMP2_Msk);
}

extern "C" void COMP_IRQHandler() {
    debugOutOn();

    hasRxActivity = true;

    PDPhySTM32L4::configureRx(PDPhySTM32L4::RxConfig::rxProcessMessage);

    // recheck in about 100µs
    nextProcessingTime = micros() + RX_PROCESS_INTERVAL;
    Scheduler.scheduleTaskAt(PDPhySTM32L4::processData, nextProcessingTime);

    debugOutOff();
}


// --- Message decoding ---

// The shortest message requires about 500 bytes of time stamps. To limit
// memory requirements, the time stamps are processed about every 100µs,
// immediately reducing the message to its actual size (about 10 bytes for the
// shortest message).
//
// The comparator interrupt starts the processing. It disables itself and
// instead schedules a task for further processing after about 100µs. The
// processing task reschedules itself if no complete message has yet been
// decoded and if no error has occurred. Once a message has been decoded,
// an error has occurred or no more time stamps have been recorded for about
// 20µs, the task re-enables the comparator interrupt instead of rescheduling
// itself.
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

void PDPhySTM32L4::processData() {
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
    int currentIndex = BUF_SIZE - DMA1_Channel7->CNDTR;

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
                default:
                    break;
            }

            if (isTransmitting) {
                // discard rest of message
                processedIndex = BUF_SIZE - DMA1_Channel7->CNDTR;
                decoder.discardMessage();
                return;
            }

            if (result == PDMessageDecoder::Result::hardReset && result == PDMessageDecoder::Result::cableReset) {
                // discard rest of chunk and reset decoding
                processedIndex = BUF_SIZE - DMA1_Channel7->CNDTR;
                decoder.discardMessage();
                hasRxActivity = true;
                break;
            }

            // continue with decoding
            hasRxActivity = true;
        }
    }
    
    // check for timeout (a gap without new data)
    int index = BUF_SIZE - DMA1_Channel7->CNDTR;
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

int PDPhySTM32L4::decodeChunk(int currentIndex) {

    if (currentIndex > processedIndex) {
        processedIndex += decoder.decodeChunk(&buffer[processedIndex], currentIndex - processedIndex);

    } else {
        processedIndex += decoder.decodeChunk(&buffer[processedIndex], BUF_SIZE - processedIndex);
        if (processedIndex == BUF_SIZE)
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
// the CC line through a 200Ω resistor to an output pin, which is set to high
// (3.3V). The lower side connects it through a 100Ω resistor to the MOSI
// output pin, which either pulls the pin low or leaves it in open-drain mode.
// When no transmission is on-going, both pins are in high-impedence/input
// mode and have no effect on the CC line.

void PDPhySTM32L4::initTx() {
    // enable clock for SPI1
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    // enable clock for GPIOA and GPIOB
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;

    // Configure PA12 as MOSI (CC1_TX)
    setRegBits(GPIOA->MODER, GPIO_MODER_ANALOG(12), GPIO_MODER_Msk(12)); // as analog input until activated
    setRegBits(GPIOA->OTYPER, GPIO_OTYPER_PUSH_PULL(12), GPIO_OTYPER_Msk(12));
    setRegBits(GPIOA->OSPEEDR, GPIO_OSPEEDR_MEDIUM(12), GPIO_OSPEEDR_Msk(12));
    setRegBits(GPIOA->PUPDR, GPIO_PUPDR_NONE(12), GPIO_PUPDR_Msk(12));
    setRegBits(GPIOA->AFR[1], GPIO_AFRH(12, 5), GPIO_AFRH_Msk(12)); 

    // Configure PB0 as CC1_TX_EN
    gpioSetOutputHigh(GPIOB, 0); // set high (open drain)
    setRegBits(GPIOB->MODER, GPIO_MODER_OUTPUT(0), GPIO_MODER_Msk(0));
    setRegBits(GPIOB->OTYPER, GPIO_OTYPER_OPEN_DRAIN(0), GPIO_OTYPER_Msk(0));
    setRegBits(GPIOB->OSPEEDR, GPIO_OSPEEDR_LOW(0), GPIO_OSPEEDR_Msk(0));
    setRegBits(GPIOB->PUPDR, GPIO_PUPDR_NONE(0), GPIO_PUPDR_Msk(0));

    // Configure PB5 as MOSI (CC2_TX)
    setRegBits(GPIOB->MODER, GPIO_MODER_ANALOG(5), GPIO_MODER_Msk(5)); // as analog input until activated
    setRegBits(GPIOB->OTYPER, GPIO_OTYPER_PUSH_PULL(5), GPIO_OTYPER_Msk(5));
    setRegBits(GPIOB->OSPEEDR, GPIO_OSPEEDR_MEDIUM(5), GPIO_OSPEEDR_Msk(5));
    setRegBits(GPIOB->PUPDR, GPIO_PUPDR_NONE(5), GPIO_PUPDR_Msk(5));
    setRegBits(GPIOB->AFR[0], GPIO_AFRL(5, 5), GPIO_AFRL_Msk(5)); 

    // Configure PB4 as CC2_TX_EN
    gpioSetOutputHigh(GPIOB, 4); // set high (open drain)
    setRegBits(GPIOB->MODER, GPIO_MODER_OUTPUT(4), GPIO_MODER_Msk(4));
    setRegBits(GPIOB->OTYPER, GPIO_OTYPER_OPEN_DRAIN(4), GPIO_OTYPER_Msk(4));
    setRegBits(GPIOB->OSPEEDR, GPIO_OSPEEDR_LOW(4), GPIO_OSPEEDR_Msk(4));
    setRegBits(GPIOB->PUPDR, GPIO_PUPDR_NONE(4), GPIO_PUPDR_Msk(4));

    // Configure SPI1 in TX-only mode, with 500kpbs
    SPI1->CR1 = (0b110 << SPI_CR1_BR_Pos) | SPI_CR1_LSBFIRST | SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;
    SPI1->CR2 = (0b0111 << SPI_CR2_DS_Pos);

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
    setRegBits(DMA1_CSELR->CSELR, 0b0001 << DMA_CSELR_C3S_Pos, DMA_CSELR_C3S_Msk);

    // enable DMA1 / channel 3 interrupt
    NVIC_SetPriority(DMA1_Channel3_IRQn, NVIC_GetPriority(TIM2_IRQn));
    NVIC_EnableIRQ(DMA1_Channel3_IRQn);

    // pull CC1 and CC2 high for a short period to initiate new USB PD communication
    gpioSetOutputHigh(GPIOA, 12);
    setRegBits(GPIOA->MODER, GPIO_MODER_OUTPUT(12), GPIO_MODER_Msk(12));
    gpioSetOutputHigh(GPIOB, 5);
    setRegBits(GPIOB->MODER, GPIO_MODER_OUTPUT(5), GPIO_MODER_Msk(5));
    delay(10);
    setRegBits(GPIOA->MODER, GPIO_MODER_ANALOG(12), GPIO_MODER_Msk(12));
    setRegBits(GPIOB->MODER, GPIO_MODER_ANALOG(5), GPIO_MODER_Msk(5));
}

bool PDPhy::transmitMessage(const PDMessage* msg) {
    return PDPhySTM32L4::transmitMessage(msg);
}

bool PDPhySTM32L4::transmitMessage(const PDMessage* msg) {
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

    DMA1_Channel3->CNDTR = txBitStreamLength;
    setRegBits(DMA1_Channel3->CCR, DMA_CCR_EN, DMA_CCR_EN_Msk);
    setRegBits(SPI1->CR2, SPI_CR2_TXDMAEN, SPI_CR2_TXDMAEN_Msk);

    if (ccActive == 1) {
        // enable PA12 and PB0
        setRegBits(GPIOA->MODER, GPIO_MODER_ALTERNATE(12), GPIO_MODER_Msk(12));
        gpioSetOutputLow(GPIOB, 0); // pull low
    } else {
        // enable PB5 and PB4
        setRegBits(GPIOB->MODER, GPIO_MODER_ALTERNATE(5), GPIO_MODER_Msk(5));
        gpioSetOutputLow(GPIOB, 4); // pull low
    }
    
    setRegBits(SPI1->CR1, SPI_CR1_SPE, SPI_CR1_SPE);
    
    __enable_irq();
    return true;
}

extern "C" void DMA1_Channel3_IRQHandler() {
    // SPI1 TX has completed
    PDPhySTM32L4::messageTransmitted();
}

void PDPhySTM32L4::messageTransmitted() {
    // disable SPI DMA
    clearRegBits(DMA1_Channel3->CCR, DMA_CCR_EN_Msk);

    // Wait until the remaining data in the SPI TX FIFO has been transmitted
    // (busy waiting in IRQ handler is not optimal but max duration is 40µs)
    while ((SPI1->SR & (SPI_SR_FTLVL_Msk | SPI_SR_BSY_Msk)) != 0)
        ;

    if (ccActive == 1) {
        // disable PA12 and PB0
        setRegBits(GPIOA->MODER, GPIO_MODER_ANALOG(12), GPIO_MODER_Msk(12));
        gpioSetOutputHigh(GPIOB, 0); // set high (open-drain)
    } else {
        // disable PB5 and PB4
        setRegBits(GPIOB->MODER, GPIO_MODER_ANALOG(5), GPIO_MODER_Msk(5));
        gpioSetOutputHigh(GPIOB, 4); // set high (open-drain)
    }

    clearRegBits(SPI1->CR1, SPI_CR1_SPE);
    clearRegBits(SPI1->CR2, SPI_CR2_TXDMAEN_Msk);
    DMA1->IFCR = DMA_IFCR_CTCIF3;

    isTransmitting = false;
    configureRx(RxConfig::rxWaitForMessage);

    PowerController.onMessageTransmitted(true);
}

#endif
