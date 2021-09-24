#include <xc.h>
#include "UART_Protocol.h"
#include "CB_RX1.h"
#include "CB_TX1.h"
#include "UART.h"
#include "main.h"
#include "toolbox.h"
#include "Asservissement.h"
#include "robot.h"

int msgDecodedFunction = 0;
int msgDecodedPayloadLength = 0;
unsigned char msgDecodedPayload[128];
int msgDecodedPayloadIndex = 0;
int reset;

typedef enum {
    Waiting,
    FunctionMSB,
    FunctionLSB,
    PayloadLengthMSB,
    PayloadLengthLSB,
    Payload,
    CheckSum
} StateReception;

StateReception rcvState = Waiting;

unsigned char UartCalculateChecksum(int msgFunction, int msgPayloadLength, unsigned char* msgPayload) {
    int i;
    unsigned char checksum = 0xFE;
    checksum ^= msgFunction >> 8;
    checksum ^= msgFunction >> 0;
    checksum ^= msgPayloadLength >> 8;
    checksum ^= msgPayloadLength >> 0;
    for (i = 0; i < msgPayloadLength; i++) {
        checksum ^= msgPayload[i];
    }
    return checksum;
}

void UartEncodeAndSendMessage(int msgFunction, int msgPayloadLength, unsigned char* msgPayload) {
    int pos = 0, i;
    unsigned char trame[6 + msgPayloadLength];
    trame[pos++] = 0xFE;
    trame[pos++] = msgFunction >> 8;
    trame[pos++] = msgFunction >> 0;
    trame[pos++] = msgPayloadLength >> 8;
    trame[pos++] = msgPayloadLength >> 0;
    for (i = 0; i < msgPayloadLength; i++) {
        trame[pos++] = msgPayload[i];
    }
    trame[pos++] = UartCalculateChecksum(msgFunction, msgPayloadLength, msgPayload);
    SendMessage(trame, pos);

}

void UartDecodeMessage(unsigned char c) {
    switch (rcvState) {
        case Waiting:
            if (c == 0xFE) {
                rcvState = FunctionMSB;
            }
            msgDecodedFunction = 0;
            msgDecodedPayloadIndex = 0;
            msgDecodedPayloadLength = 0;
            break;

        case FunctionMSB:
            msgDecodedFunction = (unsigned char) (c << 8);
            rcvState = FunctionLSB;
            break;

        case FunctionLSB:
            msgDecodedFunction += (unsigned char) (c << 0);
            rcvState = PayloadLengthMSB;
            break;

        case PayloadLengthMSB:
            msgDecodedPayloadLength = (unsigned char) (c << 8);
            rcvState = PayloadLengthLSB;
            break;

        case PayloadLengthLSB:
            msgDecodedPayloadLength += (unsigned char) (c << 0);
            if (msgDecodedPayloadLength != 0) {
                msgDecodedPayloadIndex = 0;
                rcvState = Payload;
            } else {
                rcvState = CheckSum;
            }
            break;

        case Payload:
            msgDecodedPayload[msgDecodedPayloadIndex] = c;
            msgDecodedPayloadIndex++;
            if (msgDecodedPayloadIndex >= msgDecodedPayloadLength) {
                rcvState = CheckSum;
            }
            break;

        case CheckSum:;
            unsigned char receivedChecksum = c;
            unsigned char calculatedChecksum = UartCalculateChecksum(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
            if (calculatedChecksum == receivedChecksum) {
                UartProcessDecodedMessage(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
            }
            rcvState = Waiting;
            break;

        default:
            rcvState = Waiting;
            break;
    }
}

void UartProcessDecodedMessage(unsigned char function, unsigned char payloadLength, unsigned char * payload) {
    switch (function) {
        case SET_ROBOT_STATE:
            SetRobotState(payload[0]);
            UartEncodeAndSendMessage(0x0053, 1, payload);
            break;

        case SET_ROBOT_MANUAL_CONTROL:
            SetRobotAutoControlState(payload[0]);
            break;

        case RESET_ODO:
            reset = 1;
            break;

        case SET_SPEED:
            SetRobotSpeed(payload[0], payload[1]);
            break;

        case SET_PID:
            if(payload[0] == 0)
                SetupPidAssservissement(&robotState.PidX, payload[1], payload[2], payload[3], payload[4], payload[5], payload[6]);
            else
                SetupPidAssservissement(&robotState.PidTheta, payload[1], payload[2], payload[3], payload[4], payload[5], payload[6]);
            break;
    }
}

