//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#pragma once

#if defined(STM32F103xB)
#include "stm32f103xb.h"
#elif defined(STM32F401xC)
#include "stm32f401xc.h"
#elif defined(STM32L4xx)
#include "stm32l4xx.h"
#elif defined(STM32G071xx)
#include "stm32g071xx.h"
#elif defined(STM32G431xx)
#include "stm32g431xx.h"
#elif defined(STM32G474xx)
#include "stm32g474xx.h"
#elif defined(STM32G491xx)
#include "stm32g491xx.h"
#else
#pragma GCC error "This board is not supported by usb-pd-arduino. Supported boards: STM32F103xB (aka Bluepill), STM32F401xC (aka Blackpill), STM32L4 family, STM32G071, STM32G431, STM32G474."
#endif

// --- Register manipulation ---

/**
 * @brief Sets bits of a register
 * 
 * @param reg register
 * @param value values to set (must be masked to the relevant bits)
 * @param mask mask of bits to change
 */
static inline void setRegBits(volatile uint32_t& reg, uint32_t value, uint32_t mask) {
    reg = (reg & (~mask)) | (value & mask);
}

/**
 * @brief Clears bits of a register
 * 
 * @param reg register
 * @param mask mask of bits to clear
 */
static inline void clearRegBits(volatile uint32_t& reg, uint32_t mask) {
    reg = reg & ~mask;
}


#if defined(STM32L4) || defined(STM32F4xx)

// --- GPIO ---

// Helpers to build bits and masks for GPIO registers
static constexpr uint32_t GPIO_MODER_INPUT(uint8_t gpio) { return 0b00 << (gpio * 2); }
static constexpr uint32_t GPIO_MODER_OUTPUT(uint8_t gpio) { return 0b01 << (gpio * 2); }
static constexpr uint32_t GPIO_MODER_ALTERNATE(uint8_t gpio) { return 0b10 << (gpio * 2); }
static constexpr uint32_t GPIO_MODER_ANALOG(uint8_t gpio) { return 0b11 << (gpio * 2); }
static constexpr uint32_t GPIO_MODER_Msk(uint8_t gpio) { return 0b11 << (gpio * 2); }
static constexpr uint32_t GPIO_OTYPER_PUSH_PULL(uint8_t gpio) { return 0 << gpio; }
static constexpr uint32_t GPIO_OTYPER_OPEN_DRAIN(uint8_t gpio) { return 1 << gpio; }
static constexpr uint32_t GPIO_OTYPER_Msk(uint8_t gpio) { return 1 << gpio; }
static constexpr uint32_t GPIO_OSPEEDR_LOW(uint8_t gpio) { return 0b00 << (gpio * 2); }
static constexpr uint32_t GPIO_OSPEEDR_MEDIUM(uint8_t gpio) { return 0b01 << (gpio * 2); }
static constexpr uint32_t GPIO_OSPEEDR_HIGH(uint8_t gpio) { return 0b10 << (gpio * 2); }
static constexpr uint32_t GPIO_OSPEEDR_VERY_HIGH(uint8_t gpio) { return 0b11 << (gpio * 2); }
static constexpr uint32_t GPIO_OSPEEDR_Msk(uint8_t gpio) { return 0b11 << (gpio * 2); }
static constexpr uint32_t GPIO_PUPDR_NONE(uint8_t gpio) { return 0b00 << (gpio * 2); }
static constexpr uint32_t GPIO_PUPDR_PULL_UP(uint8_t gpio) { return 0b01 << (gpio * 2); }
static constexpr uint32_t GPIO_PUPDR_PULL_DOWN(uint8_t gpio) { return 0b10 << (gpio * 2); }
static constexpr uint32_t GPIO_PUPDR_Msk(uint8_t gpio) { return 0b11 << (gpio * 2); }
static constexpr uint32_t GPIO_AFRL(uint8_t gpio, uint8_t af) { return af << (gpio * 4); }
static constexpr uint32_t GPIO_AFRL_Msk(uint8_t gpio) { return 0b1111 << (gpio * 4); }
static constexpr uint32_t GPIO_AFRH(uint8_t gpio, uint8_t af) { return af << ((gpio - 8) * 4); }
static constexpr uint32_t GPIO_AFRH_Msk(uint8_t gpio) { return 0b1111 << ((gpio - 8) * 4); }
static inline void gpioSetOutputHigh(GPIO_TypeDef* port, uint8_t gpio) { port->BSRR = 1 << gpio; }
static inline void gpioSetOutputLow(GPIO_TypeDef* port, uint8_t gpio) { port->BSRR = 1 << (16 + gpio); }

#endif 


// --- EXTI ---

#if defined(STM32L4)

// Additional EXTI constants
static constexpr uint8_t EXTI_COMP1_Pos = 21;
static constexpr uint32_t EXTI_COMP1 = 1 << EXTI_COMP1_Pos;
static constexpr uint32_t EXTI_COMP1_Msk = 1 << EXTI_COMP1_Pos;
static constexpr uint8_t EXTI_COMP2_Pos = 22;
static constexpr uint32_t EXTI_COMP2 = 1 << EXTI_COMP2_Pos;
static constexpr uint32_t EXTI_COMP2_Msk = 1 << EXTI_COMP2_Pos;

#endif

#if defined(STM32F103xB)

// --- GPIO ---

// Helpers to build bits and masks for GPIO registers
static constexpr uint32_t GPIO_CRL_MODE_INPUT(uint8_t gpio) { return 0b00 << (gpio * 4); }
static constexpr uint32_t GPIO_CRL_MODE_OUTPUT_10MHZ(uint8_t gpio) { return 0b01 << (gpio * 4); }
static constexpr uint32_t GPIO_CRL_MODE_OUTPUT_2MHZ(uint8_t gpio) { return 0b10 << (gpio * 4); }
static constexpr uint32_t GPIO_CRL_MODE_OUTPUT_50MHZ(uint8_t gpio) { return 0b11 << (gpio * 4); }
static constexpr uint32_t GPIO_CRL_MODE_MASK(uint8_t gpio) { return 0b11 << (gpio * 4); }
static constexpr uint32_t GPIO_CRL_CNF_ANALOG(uint8_t gpio) { return 0b00 << ((gpio * 4) + 2); }
static constexpr uint32_t GPIO_CRL_CNF_FLOATING(uint8_t gpio) { return 0b01 << ((gpio * 4) + 2); }
static constexpr uint32_t GPIO_CRL_CNF_INPUT_PUPD(uint8_t gpio) { return 0b10 << ((gpio * 4) + 2); }
static constexpr uint32_t GPIO_CRL_CNF_OUTPUT(uint8_t gpio) { return 0b00 << ((gpio * 4) + 2); }
static constexpr uint32_t GPIO_CRL_CNF_OUTPUT_OPENDRAIN(uint8_t gpio) { return 0b01 << ((gpio * 4) + 2); }
static constexpr uint32_t GPIO_CRL_CNF_ALTERNATE(uint8_t gpio) { return 0b10 << ((gpio * 4) + 2); }
static constexpr uint32_t GPIO_CRL_CNF_ALTERNATE_OPENDRAIN(uint8_t gpio) { return 0b11 << ((gpio * 4) + 2); }
static constexpr uint32_t GPIO_CRL_CNF_MASK(uint8_t gpio) { return 0b11 << ((gpio * 4) + 2); }
static constexpr uint32_t GPIO_CRH_MODE_INPUT(uint8_t gpio) { return 0b00 << ((gpio - 8) * 4); }
static constexpr uint32_t GPIO_CRH_MODE_OUTPUT_10MHZ(uint8_t gpio) { return 0b01 << ((gpio - 8) * 4); }
static constexpr uint32_t GPIO_CRH_MODE_OUTPUT_2MHZ(uint8_t gpio) { return 0b10 << ((gpio - 8) * 4); }
static constexpr uint32_t GPIO_CRH_MODE_OUTPUT_50MHZ(uint8_t gpio) { return 0b11 << ((gpio - 8) * 4); }
static constexpr uint32_t GPIO_CRH_MODE_MASK(uint8_t gpio) { return 0b11 << ((gpio - 8) * 4); }
static constexpr uint32_t GPIO_CRH_CNF_ANALOG(uint8_t gpio) { return 0b00 << (((gpio - 8) * 4) + 2); }
static constexpr uint32_t GPIO_CRH_CNF_FLOATING(uint8_t gpio) { return 0b01 << (((gpio - 8) * 4) + 2); }
static constexpr uint32_t GPIO_CRH_CNF_INPUT_PUPD(uint8_t gpio) { return 0b10 << (((gpio - 8) * 4) + 2); }
static constexpr uint32_t GPIO_CRH_CNF_OUTPUT(uint8_t gpio) { return 0b00 << (((gpio - 8) * 4) + 2); }
static constexpr uint32_t GPIO_CRH_CNF_OUTPUT_OPENDRAIN(uint8_t gpio) { return 0b01 << (((gpio - 8) * 4) + 2); }
static constexpr uint32_t GPIO_CRH_CNF_ALTERNATE(uint8_t gpio) { return 0b10 << (((gpio - 8) * 4) + 2); }
static constexpr uint32_t GPIO_CRH_CNF_ALTERNATE_OPENDRAIN(uint8_t gpio) { return 0b11 << (((gpio - 8) * 4) + 2); }
static constexpr uint32_t GPIO_CRH_CNF_MASK(uint8_t gpio) { return 0b11 << (((gpio - 8) * 4) + 2); }
static inline void gpioSetOutputHigh(GPIO_TypeDef* port, uint8_t gpio) { port->BSRR = 1 << gpio; }
static inline void gpioSetOutputLow(GPIO_TypeDef* port, uint8_t gpio) { port->BSRR = 1 << (16 + gpio); }

#endif
