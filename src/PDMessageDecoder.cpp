//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#include "CRC32.h"
#include "PDMessageDecoder.h"
#include <string.h>


// sorted from LSB to MSB
const uint8_t PDMessageDecoder::decode4b5bTab[32] = {
    SymbolError,        // 00000
    SymbolError,        // 10000
    SymbolError,        // 01000
    SymbolSync1,        // 11000
    SymbolError,        // 00100
    SymbolData | 0x02,  // 10100
    SymbolError,        // 01100
    SymbolData | 0x0e,  // 11100
    SymbolError,        // 00010
    SymbolData | 0x08,  // 10010
    SymbolData | 0x04,  // 01010
    SymbolData | 0x0c,  // 11010
    SymbolSync3,        // 00110
    SymbolData | 0x0a,  // 10110
    SymbolData | 0x06,  // 01110
    SymbolData | 0x00,  // 11110
    SymbolError,        // 00001
    SymbolSync2,        // 10001
    SymbolData | 0x01,  // 01001
    SymbolRst2,         // 11001
    SymbolError,        // 00101
    SymbolData | 0x03,  // 10101
    SymbolEOP,          // 01101
    SymbolData | 0x0f,  // 11101
    SymbolError,        // 00011
    SymbolData | 0x09,  // 10011
    SymbolData | 0x05,  // 01011
    SymbolData | 0x0d,  // 11011
    SymbolRst1,         // 00111
    SymbolData | 0x0b,  // 10111
    SymbolData | 0x07,  // 01111
    SymbolError         // 11111
};

PDMessageDecoder::PDMessageDecoder() : decodingResult(Result::incomplete), lastValue(0) {
    reset();
}

void PDMessageDecoder::reset() {
    preambleIndex = 0;
    partialSymbol = 0;
    numBitsOfSymbol = 0;
    payloadLen = PayloadLenPreamble;
    isHalfBit = false;
}

int PDMessageDecoder::decodeChunk(const uint16_t* timeStamps, int numTimeStamps) {

    int numProcessed = 0;
    if (decodingResult == Result::invalid) {
        int n = skipToEOM(timeStamps, numTimeStamps);
        numProcessed += n;
        timeStamps += n;

    } else {
        decodingResult = Result::incomplete;
    }

    while (numProcessed < numTimeStamps && decodingResult == Result::incomplete) {
        int rem = numTimeStamps - numProcessed;
        int n = payloadLen == PayloadLenPreamble ? decodePreamble(timeStamps, rem) : decodeSymbols(timeStamps, rem);
        numProcessed += n;
        timeStamps += n;
    }

    return numProcessed;
}

int PDMessageDecoder::decodePreamble(const uint16_t* timeStamps, int numTimeStamps) {
    // use local variable for efficiency
    int index = preambleIndex;
    bool halfBit = isHalfBit;
    uint16_t last = lastValue;
    
    int i = 0;
    while (i < numTimeStamps) {
        uint16_t value = timeStamps[i] - last;
        last += value;
        i += 1;
        
        if ((index & 1) == 0) {
            // even index: long pulse expected
            if (isLong(value)) {
                if (halfBit) {
                    if (index <= 2) {
                        // ignore errors at the beginning
                        index = 0;
                        continue;
                    }

                    decodingResult = Result::invalid; // invalid transition in preamble
                    break;
                }
                index += 1;
                continue;
            }
            
        } else {
            // odd index: short pulse expected
            if (isShort(value)) {
                if (halfBit)
                    index += 1;
                halfBit = !halfBit;
                continue;
            }
        }
        
        if (index <= 2) {
            // ignore errors at the beginning
            index = 0;
            continue;
        }
        
        // Test for transition to K-codes in SOP (start of packet).
        // SOP can start with either a Sync-1 or Rst-1 symbol. Possible transitions are:
        // ------- Preamble -------------|------- Start of Packet -------
        //   ... / long / short / short  |  long / long! / long / ... (Sync-1)
        //   ... / long / short / short  |  short! / short / short / ... (Rst-1)
        // The transition it detected if the sequence deviates from the preamble pattern,
        // marked with !.
        if (index >= 63 && index <= 67) {
            if ((index & 1) == 1 && isLong(value) && !halfBit) {
                // Sync-1
                numBitsOfSymbol = 2;
            } else if ((index & 1) == 0 && isShort(value) && !halfBit) {
                // Rst-1
                halfBit = true;
                numBitsOfSymbol = 0;
            } else {
                decodingResult = Result::invalid;
                break;
            }

            payloadLen = PayloadLenSOP;
            partialSymbol = 0;
            break;
        }
        
        decodingResult = Result::invalid; // invalid preamble
        break;
    }

    if (decodingResult == Result::incomplete) {
        preambleIndex = index;
        isHalfBit = halfBit;
        lastValue = last;
    } else {
        reset();
    }

    return i;
}

int PDMessageDecoder::decodeSymbols(const uint16_t* timeStamps, int numTimeStamps) {
    // use local variable for efficiency
    uint16_t last = lastValue;
    bool halfBit = isHalfBit;
    int numBits = numBitsOfSymbol;
    uint8_t symbol = partialSymbol;

    int i = 0;
    while (i < numTimeStamps) {
        uint16_t value = timeStamps[i] - last;
        last += value;
        i += 1;

        if (isShort(value)) {
            // short pulse
            if (halfBit) {
                symbol = (symbol << 1) | 1;
                numBits += 1;
            }
            halfBit = !halfBit;
            
        } else if (isLong(value)) {
            // long pulse
            if (halfBit) {
                decodingResult = Result::invalid; // invalid transition in payload
                break;
            }
            symbol = symbol << 1;
            numBits += 1;

        } else {
            // delay longer than a pulse
            decodingResult = Result::invalid; // unexpected end of message
            break;
        }

        if (numBits == 5) {
            // complete symbol

            // decode symbol
            symbol = decode4b5bTab[symbol];

            if (payloadLen >= 0) {
                // data expected
                if (isData(symbol)) {
                    uint8_t nibble = dataNibble(symbol);
                    uint8_t* payload = message->payload();
                    if ((payloadLen & 1) == 0) {
                        payload[payloadLen >> 1] = nibble;
                    } else {
                        uint8_t b = payload[payloadLen >> 1] | nibble << 4;
                        payload[payloadLen >> 1] = b;
                    }
                    payloadLen += 1;

                } else if (symbol == SymbolEOP) {
                    checkMessage();
                    break;
            
                } else {
                    // error
                    decodingResult = Result::invalid; // invalid symbol
                    break;
                }

            } else {
                // symbol valid in SOP expected
                if (isSOP(symbol)) {
                    startSymbols[payloadLen + 4] = (Symbol)symbol;
                    payloadLen += 1;

                    if (payloadLen == 0) {
                        // 4 K-codes have been received; decode them
                        PDSOPSequence seq = decodeSOP();
                        if (seq == PDSOPSequence::hardReset || seq == PDSOPSequence::cableReset) {
                            decodingResult = seq == PDSOPSequence::hardReset ? Result::hardReset : Result::cableReset;
                            break;
                        } else if (seq != PDSOPSequence::invalid) {
                            message->sopSequence = seq;
                        } else {
                            decodingResult = Result::invalid; // invalid SOP sequence
                            break;
                        }
                    }

                } else {
                    decodingResult = Result::invalid; // error in SOP
                    break;
                }
            }

            numBits = 0;
            symbol = 0;
        }
    }
    
    if (decodingResult == Result::incomplete) {
        partialSymbol = symbol;
        numBitsOfSymbol = numBits;
        isHalfBit = halfBit;
        lastValue = last;
    } else {
        reset();
    }

    return i;
}

PDSOPSequence PDMessageDecoder::decodeSOP() {
    PDSOPSequence seq;

    uint32_t symbols;
    memcpy(&symbols, startSymbols, sizeof(symbols));

    switch (symbols) {
        case combinedSymbols(SymbolSync1, SymbolSync1, SymbolSync1, SymbolSync2):
            seq = PDSOPSequence::sop;
            break;

        case combinedSymbols(SymbolSync1, SymbolSync1, SymbolSync3, SymbolSync3):
            seq = PDSOPSequence::sop1;
            break;

        case combinedSymbols(SymbolSync1, SymbolSync3, SymbolSync1, SymbolSync3):
            seq = PDSOPSequence::sop2;
            break;

        case combinedSymbols(SymbolSync1, SymbolRst2, SymbolRst2, SymbolSync3):
            seq = PDSOPSequence::sop1Debug;
            break;

        case combinedSymbols(SymbolSync1, SymbolRst2, SymbolSync3, SymbolSync2):
            seq = PDSOPSequence::sop2Debug;
            break;

        case combinedSymbols(SymbolSync1, SymbolSync1, SymbolSync1, SymbolSync3):
            seq = PDSOPSequence::sop;
            break;

        case combinedSymbols(SymbolRst1, SymbolRst1, SymbolRst1, SymbolRst2):
            seq = PDSOPSequence::hardReset;
            break;

        case combinedSymbols(SymbolRst1, SymbolSync1, SymbolRst1, SymbolSync3):
            seq = PDSOPSequence::cableReset;
            break;

        default:
            seq = PDSOPSequence::invalid;
    }

    return seq;
}

int PDMessageDecoder::skipToEOM(const uint16_t* timeStamps, int numTimeStamps) {
    uint16_t last = lastValue;
    int i = 0;
    while (i < numTimeStamps) {
        uint16_t value = timeStamps[i] - last;
        last += value;
        i += 1;

        if (isGap(value)) {
            // EOM found
            decodingResult = Result::incomplete;
            break;
        }
    }

    lastValue = last;
    return i;
}

bool PDMessageDecoder::discardMessage() {
    bool looksLikeError = decodingResult == Result::incomplete && preambleIndex > 20;
    decodingResult = Result::incomplete;
    reset();
    return looksLikeError;
}

void PDMessageDecoder::checkMessage() {
    if (payloadLen >= 12 && (payloadLen & 1) == 0) {
        if (payloadLen == 12 + message->numObjects() * 8) {
                decodingResult = CRC32::isValid(message->payload(), payloadLen >> 1)
                    ? Result::completeMessage : Result::invalid;
        } else {
            decodingResult = Result::invalid; // message length different from what header indicates
        }

    } else {
        decodingResult = Result::invalid; // unexpected EOM
    }
}
