//
// USB Power Delivery for Arduino
// Copyright (c) 2023 Manuel Bleichenbacher
//
// Licensed under MIT License
// https://opensource.org/licenses/MIT
//

#include <Arduino.h>
#include "PDController.h"
#include "PDPhy.h"
#include "TaskScheduler.h"

PDController PowerController{};

PDController::PDController()
: ccPin(0), isMonitorOnly(true), eventHandler(nullptr), rxMessageHead(rxBuffer), txMessage((PDMessage*)txBuffer),
    logHead(0), logTail(0), txMessageId(0), txRetryCount(0), lastRxMessageId(-1), lastSpecRev(1), lastMessage(nullptr)
{
    PDPhy::prepareRead(reinterpret_cast<PDMessage*>(rxMessageHead));
}

void PDController::startController(EventHandlerFunction handler) {
    eventHandler = handler;
    isMonitorOnly = false;
    PDPhy::initSink();
    reset();
}

void PDController::startMonitor() {
    eventHandler = nullptr;
    isMonitorOnly = true;
    PDPhy::initMonitor();
    reset();
}

const PDLogEntry* PDController::popLogEntry() {
    if (logHead == logTail)
        return nullptr;

    uint32_t index = logTail % LogSize;
    logTail += 1;
    return &logEntries[index];
}

void PDController::log(PDLogEntryType type, const PDMessage* message) {
    uint32_t index = logHead % LogSize;
    logEntries[index].type = type;
    logEntries[index].time = micros();
    logEntries[index].message = message;
    logHead += 1;
}

void PDController::reset() {
    lastRxMessageId = -1;
    txMessageId = 0;
    txRetryCount = 0;
    lastMessage = nullptr;
    Scheduler.cancelTask(noGoodCrcReceivedCallback);

    if (eventHandler != nullptr)
        eventHandler(PDControllerEvent(PDControllerEventType::reset));
}

void PDController::onReset(PDSOPSequence seq) {
    reset();
    log(seq == PDSOPSequence::hardReset ? PDLogEntryType::hardReset : PDLogEntryType::cableReset);
    if (eventHandler != nullptr)
        eventHandler(PDControllerEvent(PDControllerEventType::reset));
}

bool PDController::sendControlMessage(PDMessageType messageType) {
    if (isTransmitting())
        return false;
    txMessage->initControl(messageType, lastSpecRev);
    txRetryCount = paramNRetryCount + 1;
    return sendMessage();
}

bool PDController::sendDataMessage(PDMessageType messageType, int numObjects, const uint32_t* objects) {
    if (isTransmitting())
        return false;
    txMessage->initData(messageType, numObjects, lastSpecRev);
    memcpy(txMessage->objects, objects, 4 * numObjects);
    txRetryCount = paramNRetryCount + 1;
    return sendMessage();
}

bool PDController::sendMessage() {
    bool isGoodCrc = txMessage->type() == PDMessageType::controlGoodCrc;

    // add message ID
    txMessage->setMessageId(isGoodCrc ? lastRxMessageId : txMessageId);

    txMessage->cc = ccPin;
    if (!PDPhy::transmitMessage(txMessage))
        return false;

    log(PDLogEntryType::transmissionStarted, txMessage);
    return true;
}

void PDController::onMessageTransmitted(bool successful) {

    if (!successful) {
        log(PDLogEntryType::transmissionFailed);
        prepareNextTxMessage();
        return;
    }

    log(PDLogEntryType::transmissionCompleted);

    // For regular messages, we will start timer for the GoodCRC message (so the TX buffer is not yet changed).
    // For GoodCRC messages, the transmission is completed.

    if (txMessage->type() != PDMessageType::controlGoodCrc) {
        // scheduled timer to check for GoodCRC
        Scheduler.scheduleTaskAfter(noGoodCrcReceivedCallback, paramCRCReceiveTimer);

    } else {
        prepareNextTxMessage();

        // If a GoodCRC message for a message was successfully sent, the event handler is notified
        if (lastMessage != nullptr) {
            if (eventHandler != nullptr)
                eventHandler(PDControllerEvent(PDControllerEventType::messageReceived, lastMessage));
            lastMessage = nullptr;
        }
    }
}

void PDController::noGoodCrcReceivedCallback() {
    PowerController.onNoGoodCrcReceived();
}

void PDController::onNoGoodCrcReceived() {
    Scheduler.cancelTask(noGoodCrcReceivedCallback);

    txRetryCount -= 1;
    if (txRetryCount > 0 && txMessage->type() != PDMessageType::controlGoodCrc) {
        // retry
        sendMessage();

    } else {
        // transmission has failed - no retry
        prepareNextTxMessage();
    }
}

void PDController::onMessageReceived(PDMessage* message) {
    int messageId = message->messageId();
    PDMessageType type = message->type();
    PDSOPSequence sopSeq = message->sopSequence;
    bool reportTransmissionFailed = false;

    log(PDLogEntryType::messageReceived, message);

    // prepare for next read
    rxMessageHead = message->end();
    if (rxMessageHead + MaxMessageSize > rxBuffer + sizeof(rxBuffer))
        rxMessageHead = rxBuffer;

    PDPhy::prepareRead(reinterpret_cast<PDMessage*>(rxMessageHead));

    if (isTransmitting()) {
        // a GoodCRC message is expected
        if (type != PDMessageType::controlGoodCrc || sopSeq != PDSOPSequence::sop || messageId != txMessageId) {
            log(PDLogEntryType::transmissionFailed);
            reportTransmissionFailed = true;
        }

        Scheduler.cancelTask(noGoodCrcReceivedCallback);
        prepareNextTxMessage();
        lastMessage = nullptr;
    }

    if (type != PDMessageType::controlGoodCrc && sopSeq == PDSOPSequence::sop && !isMonitorOnly) {

        // if the message has the same message ID like the previous one,
        // 'lastMessage' is not set to prevent notifying the event handler twice
        if (messageId != lastRxMessageId) {
            lastMessage = message;
            lastRxMessageId = messageId;
            lastSpecRev = message->specRev();
        }

        // send GoodCRC to confirm received message
        sendControlMessage(PDMessageType::controlGoodCrc);
    }

    // deliberately call event handler after GoodCRC transmission has started
    if (reportTransmissionFailed && eventHandler != nullptr)
        eventHandler(PDControllerEvent(PDControllerEventType::transmissionFailed));
}

void PDController::prepareNextTxMessage() {
    txRetryCount = 0;

    if (txMessage->type() != PDMessageType:: controlGoodCrc) {
        txMessageId += 1;
        if (txMessageId >= 8)
            txMessageId = 0;
    }

    // prepare new TX message buffer
    uint8_t* head = txMessage->end();
    if (head + MaxMessageSize >= txBuffer + sizeof(txBuffer))
        head = txBuffer;
    txMessage = (PDMessage*)head;
}

void PDController::onError() {
    log(PDLogEntryType::error);

    // re-setup same buffer for reception
    PDPhy::prepareRead(reinterpret_cast<PDMessage*>(rxMessageHead));
}

void PDController::onVoltageChanged(int cc) {
    ccPin = cc;
    log(cc == 0 ? PDLogEntryType::sinkSourceDisconnected : PDLogEntryType::sinkSourceConnected);
    if (eventHandler != nullptr)
        eventHandler(PDControllerEvent(cc == 0 ? PDControllerEventType::disconnected : PDControllerEventType::connected));
}
