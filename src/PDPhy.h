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
 * @brief Physical layer for USB PD communication.
 * 
 */
struct PDPhy {
    /**
     * @brief Initializes the USB PD PHY as monitor.
     * 
     * In the monitor role, the PHY will only listen to the USB PD
     * communication but not interact with it. For PHYs with 
     * controllable pull-up/down resistors, it will not activate them.
     */
    static void initMonitor();

    /**
     * @brief Initializes the USB PD PHY as a sink.
     * 
     * In the sink role, the PHY will interact in the USB PD communication.
     * For PHYs with controllable pull-up/down resistors, it will
     * activate the pull-down resistors to present itself as a power sink.
     */
    static void initSink();

    /**
     * @brief Sets the message and buffer to be used for the next incoming message.
     * 
     * @param msg the message
     */
    static void prepareRead(PDMessage* msg);

    /**
     * @brief Transmits a message.
     * 
     * The method is asynchronous. It will start the transmissoin when the CC
     * line is idle and trigger an event to report when the message has been
     * transmitted or the transmission has failed.
     * 
     * @param msg the message
     * @return `true` if transmission was started, `false` if it failed
     *      (TX activity by this or the other device, no active CC line)
     */
    static bool transmitMessage(const PDMessage* msg);
};
