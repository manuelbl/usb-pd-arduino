//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#pragma once

#include <stdint.h>

/**
 * @brief Computes and validates the CRC32 checksum.
 * 
 * It uses the CRC32 polynomial 0x04C11DB7, and initial value of 0xFFFFFFFF
 * and computes the checksum starting with the least-significant bit.
 * The payload data does not need to be a multiple of 32-bit.
 */
struct CRC32 {
    /**
     * @brief Validates the integrity of the specified data.
     * 
     * The checksum is expected to be at the end of the specified data.
     * 
     * @param data the data
     * @param len the length of the data (in bytes)
     * @return `true` if the data is valid, `false` otherwise
     */
    static bool isValid(const uint8_t* data, int len);

    /**
     * @brief Computes the CRC32 checksum for the specified data.
     * 
     * The checksum can be added to the data, in LSB order.
     * 
     * @param data the data
     * @param len the length of the data (in bytes)
     * @return uint32_t CRC32 checksum
     */
    static uint32_t compute(const uint8_t* data, int len);

    /// @brief Initial value for CRC32 compuation
    static constexpr uint32_t InitialValue = 0xFFFFFFFF;

    /// @brief Expected residual value when validating checksum.
    static constexpr uint32_t ExpectedResidual = 0xC704DD7B;
};
