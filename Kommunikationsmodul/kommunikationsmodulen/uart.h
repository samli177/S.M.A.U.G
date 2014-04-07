/*
 * uart.h
 *
 * Created: 4/7/2014 2:50:35 PM
 *  Author: marhu300
 */ 


#ifndef UART_H_
#define UART_H_


void USART_init();
uint8_t USART_CheckRxComplete();
uint8_t USART_CheckTxReady();
void USART_WriteByte(uint8_t DataByteOut);
uint8_t USART_ReadByte();
uint16_t USART_crc16(uint8_t tag, uint8_t length);
void USART_SendPacket(char tag, uint8_t length);
void USART_SendMessage(char msg[]);
void USART_SendSensors();
uint8_t USART_DecodeMessageRxFIFO();
void USART_DecodeRxFIFO();
void USART_Bounce();



#endif /* UART_H_ */