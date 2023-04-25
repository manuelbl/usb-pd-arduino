//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#include "CRC32.h"
#include "PDMessageEncoder.h"

constexpr static uint8_t SYNC1 = 0;
constexpr static uint8_t SYNC2 = 1;
constexpr static uint8_t SYNC3 = 2;
constexpr static uint8_t RST1 = 3;
constexpr static uint8_t RST2 = 4;
constexpr static uint8_t EOP = 5;


const uint8_t SOP_SYMBOLS[][4] = {
    // invalid
    { 0, 0, 0, 0 },
    // SOP
    { SYNC1, SYNC1, SYNC1, SYNC2 },
    // SOP'
    { SYNC1, SYNC1, SYNC3, SYNC3 },
    // SOP''
    { SYNC1, SYNC3, SYNC1, SYNC3 },
    // SOP' Debug
    { SYNC1, RST2, RST2, SYNC3 },
    // SOP'' Debug
    { SYNC1, RST2, SYNC3, SYNC2 },
    // Hard Reset
    { RST1, RST1, RST1, RST2 },
    // Cable Reset
    { RST1, SYNC1, RST1, SYNC3 }
};

// Bitstreams assume the previous bit ended with low
const uint16_t SYMBOL_BITSTREAMS[6] = {
    0x02b3, // SYNC1 -> 1100110101
    0x02cd, // SYNC2 -> 1011001101
    0x032b, // SYNC3 -> 1101010011
    0x00d5, // RST1  -> 1010101100
    0x014d, // RST2  -> 1011001010
    0x00ad  // EOP   -> 1011010100
};

// Bitstreams assume the previous bit ended with low
const uint16_t DATA_BITSTREAMS[16] = {
    0x02ab, //  0 -> 1101010101
    0x034d, //  1 -> 1011001011
    0x02d3, //  2 -> 1100101101
    0x012d, //  3 -> 1011010010
    0x034b, //  4 -> 1101001011
    0x00b5, //  5 -> 1010110100
    0x00ab, //  6 -> 1101010100
    0x0355, //  7 -> 1010101011
    0x02cb, //  8 -> 1101001101
    0x0135, //  9 -> 1010110010
    0x012b, // 10 -> 1101010010
    0x02d5, // 11 -> 1010101101
    0x014b, // 12 -> 1101001010
    0x02b5, // 13 -> 1010110101
    0x0153, // 14 -> 1100101010
    0x02ad  // 15 -> 1011010101
};

#define ADD_BITS(bits) \
    do { \
        uint32_t symbolBits = (bits) ^ xorMask; \
        word |= symbolBits << numBits; \
        numBits += 10; \
        xorMask = (symbolBits & 0x200) != 0 ? 0x3ff : 0; \
    } while (0)

#define STORE_BITS() \
    do { \
        while (numBits >= 8) { \
            buffer[index] = (uint8_t)word; \
            index += 1; \
            word >>= 8; \
            numBits -= 8; \
        } \
    } while (0)


int PDMessageEncoder::encode(const PDMessage* message, uint8_t* buffer) {

    // preamble
    for (int i = 0; i < 16; i++)
        buffer[i] = 0b10110100;
    
    int index = 16;
    uint8_t numBits = 0;
    uint32_t word = 0;
    uint32_t xorMask = 0x3ff; // 0x3ff if the last bit ended high, 0 if it ended low

    // SOP
    const uint8_t* sopSymbols = &SOP_SYMBOLS[(int)message->sopSequence][0];
    ADD_BITS(SYMBOL_BITSTREAMS[sopSymbols[0]]);
    ADD_BITS(SYMBOL_BITSTREAMS[sopSymbols[1]]);
    STORE_BITS();
    ADD_BITS(SYMBOL_BITSTREAMS[sopSymbols[2]]);
    ADD_BITS(SYMBOL_BITSTREAMS[sopSymbols[3]]);
    STORE_BITS();

    // payload (header and objects)
    const uint8_t* data = message->payload();
    int len = 2 + 4 * message->numObjects();
    for (int i = 0; i < len; i++) {
        // lower nibble        
        ADD_BITS(DATA_BITSTREAMS[data[i] & 0x0f]);

        // upper nibble
        ADD_BITS(DATA_BITSTREAMS[data[i] >> 4]);
        STORE_BITS();
    }

    // CRC
    uint32_t crc = CRC32::compute(data, len);
    const uint8_t* crcBytes = reinterpret_cast<uint8_t*>(&crc);
    for (int i = 0; i < 4; i++) {
        // lower nibble        
        ADD_BITS(DATA_BITSTREAMS[crcBytes[i] & 0x0f]);

        // upper nibble
        ADD_BITS(DATA_BITSTREAMS[crcBytes[i] >> 4]);
        STORE_BITS();
    }

    // EOP
    ADD_BITS(SYMBOL_BITSTREAMS[EOP]);

    // remaining bits
    if (xorMask == 0) {
        // add two high bits
        word |= 0b11 << numBits;
        numBits += 2;
    }

    numBits += 8;

    STORE_BITS();

    return index;
}
