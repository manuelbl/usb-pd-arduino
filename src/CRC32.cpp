//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#include "CRC32.h"
#include "CMSISHelper.h"

#if defined(STM32L4xx)

bool CRC32::isValid(const uint8_t* data, int len) {
    CRC->CR = (0b01 << CRC_CR_REV_IN_Pos) | CRC_CR_RESET;

    for (int i = 0; i < len; i += 1)
        *((volatile uint8_t*)&CRC->DR) = data[i]; // bytewise operation

    return CRC->DR == ExpectedResidual;
}

uint32_t CRC32::compute(const uint8_t* data, int len) {
    CRC->CR = (0b01 << CRC_CR_REV_IN_Pos) | CRC_CR_REV_OUT | CRC_CR_RESET;

    for (int i = 0; i < len; i += 1)
        *((volatile uint8_t*)&CRC->DR) = data[i]; // bytewise operation

    return ~CRC->DR;
}

#else

/// look-up table
static const uint32_t CRC32Lookup[16] = {
    0x00000000, 0x1DB71064, 0x3B6E20C8, 0x26D930AC,
    0x76DC4190, 0x6B6B51F4, 0x4DB26158, 0x5005713C,
    0xEDB88320, 0xF00F9344, 0xD6D6A3E8, 0xCB61B38C,
    0x9B64C2B0, 0x86D3D2D4, 0xA00AE278, 0xBDBDF21C
};

/// Calculates CRC32
static uint32_t CRC32Calc(const uint8_t* data, int len) {
    uint32_t crc = CRC32::InitialValue;

    // nibble-wise calculation
    for (int i = 0; i < len; i += 1) {
        crc = CRC32Lookup[(crc ^  data[i]      ) & 0x0f] ^ (crc >> 4);
        crc = CRC32Lookup[(crc ^ (data[i] >> 4)) & 0x0f] ^ (crc >> 4);
    }

    return ~crc;
}


bool CRC32::isValid(const uint8_t* data, int len) {
    uint32_t crc = CRC32Calc(data, len);
    return __RBIT(~crc) == ExpectedResidual;
}

uint32_t CRC32::compute(const uint8_t* data, int len) {
    return CRC32Calc(data, len);
}

#endif
