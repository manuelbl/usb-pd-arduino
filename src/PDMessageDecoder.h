//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#pragma once

#include "PDMessage.h"

/**
 * @brief Decoder for USB PD message.
 * 
 * Arrays of time stamps are fed to this decoder. Each time stamp represents
 * a high-low or low-high transition in the USB PD signal.
 * 
 * After an error, the decoder will swallow all time stamps until the end of
 * a message is detected. The end of the message is indicated by a long gap
 * between time stamps.
 */
struct PDMessageDecoder {
    /// Decoding result
    enum class Result {
        /// decoding is incomplete; more chunks are needed
        incomplete,
        /// a complete message has been decoded
        completeMessage,
        /// a hard reset can been decoded
        hardReset,
        /// a cable reset has been decoded
        cableReset,
        /// invalid data was encountered
        invalid        
    };

    /// Creates a new instance
    PDMessageDecoder();

    /**
     * @brief Process a number of time stamps.
     * 
     * If an event is encoutered, not all time stamps are processed.
     * The event can be queried with `result()`.
     * 
     * @param timeStamps array of time stamps
     * @param numTimeStamps number of time stamps in array
     * @return number of processed time stamps
     */
    int decodeChunk(const uint16_t* timeStamps, int numTimeStamps);

    /**
     * @brief Discard the message being currently decoded.
     * 
     * @return `true` if a considerable amount of data was discarded
     *      (indicating an error), `false` otherwise
     */
    bool discardMessage();

    /**
     * @brief Set the message to be filled with the decoded message
     * 
     * @param msg the message
     */
    void setMessage(PDMessage* msg) {
        message = msg;
    }

    /// Get the last decoding result 
    Result result() {
        return decodingResult;
    }

    uint16_t lastTimeStamp() {
        return lastValue;
    }

private:
    enum Symbol: uint8_t {
        SymbolData = 0 << 4,
        SymbolSync1 = 1 << 4,
        SymbolSync2 = 2 << 4,
        SymbolSync3 = 3 << 4,
        SymbolRst1 = 4 << 4,
        SymbolRst2 = 5 << 4,
        SymbolEOP = 6 << 4,
        SymbolError = 7 << 4
    };

    constexpr static int PayloadLenPreamble = -5;
    constexpr static int PayloadLenSOP = -4;
    
    int decodePreamble(const uint16_t* timeStamps, int numTimeStamps);
    int decodeSymbols(const uint16_t* timeStamps, int numTimeStamps);
    PDSOPSequence decodeSOP();
    int skipToEOM(const uint16_t* timeStamps, int numTimeStamps);
    void checkMessage();
    void reset();

    static inline bool isShort(uint16_t value) {
        return value <= 6;
    }

    static inline bool isLong(uint16_t value) {
        return value >= 7 && value <= 12;
    }

    static inline bool isGap(uint16_t value) {
        return value > 12;
    }

    static inline bool isData(uint8_t symbol) {
        return (symbol & 0xf0) == SymbolData;
    }

    static inline uint8_t dataNibble(uint8_t symbol) {
        return symbol & 0x0f;
    }

    static inline bool isSOP(uint8_t symbol) {
        return (symbol >> 4) >= 1 && (symbol >> 4) <= 5;
    }

    static constexpr inline uint32_t combinedSymbols(PDMessageDecoder::Symbol sym1, PDMessageDecoder::Symbol sym2,
            PDMessageDecoder::Symbol sym3, PDMessageDecoder::Symbol sym4) {
        return sym1 | (sym2 << 8) | (sym3 << 16) | (sym4 << 24);
    }
    
    PDMessage* message;
    Result decodingResult;
    int payloadLen; // in nibbles; negative numbers are for preamble and SOP
    int preambleIndex;
    Symbol startSymbols[4];
    int numBitsOfSymbol;
    uint16_t lastValue;
    uint8_t partialSymbol;
    bool isHalfBit;

    static const uint8_t decode4b5bTab[32];
};
