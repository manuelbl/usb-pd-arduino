//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#pragma once

#include "PDMessage.h"

struct PDMessageEncoder {
    /**
     * @brief Encodes the message into a bit stream.
     * 
     * The bit stream, suitable for transmission at 600kbps, is stored in the
     * specified buffer.
     * 
     * The buffer size must be at least (435 + n * 40) / 8 bytes, where 'n' is
     * the number of objects in the message, or 57 bytes to be on the safe
     * side.
     * 
     * The bit stream is stored byte per byte, LSB first.
     * 
     * @param message the message to be encoded
     * @param buffer the buffer to receive the encoded message
     * @return the number of bytes written to the buffer
     */
    static int encode(const PDMessage* message, uint8_t* buffer);
};
