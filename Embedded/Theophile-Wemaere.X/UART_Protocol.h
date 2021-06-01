#ifndef UART_PROTOCOLE_H
#define	UART_PROTOCOLE_H

#define SET_ROBOT_STATE 0x0051
#define SET_ROBOT_MANUAL_CONTROL 0x0052
#define RESET_ODO 0x0062
unsigned char UartCalculateChecksum(int msgFunction,int msgPayloadLength, unsigned char* msgPayload);
void UartEncodeAndSendMessage(int msgFunction,int msgPayloadLength, unsigned char* msgPayload);
void UartDecodeMessage(unsigned char c);
void UartProcessDecodedMessage(unsigned char function,unsigned char payloadLength, unsigned char* payload);
extern int reset;

#endif	/* UART_PROTOCOLE_H */