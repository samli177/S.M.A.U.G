	/*
 * usart.h
 *
 * Created: 4/8/2014 1:52:09 PM
 *  Author: samli177
 */ 


#ifndef USART_H_
#define USART_H_


#define F_CPU 16000000
#define BaudRate 115200
#define POLY 0x8408
// -- Declarations --

void USART_init();
uint8_t USART_CheckRxComplete();
uint8_t USART_CheckTxReady();
void USART_WriteByte(uint8_t DataByteOut);
uint8_t USART_ReadByte();
uint16_t USART_crc16(uint8_t tag, uint8_t length);
void USART_SendPacket(char tag, uint8_t length);
void USART_SendMessage(char msg[]);
void USART_SendSensors();
void USART_SendGyro();
void USART_send_climb_done();
void USART_send_turn_done();
uint8_t USART_DecodeMessageRxFIFO();
uint8_t USART_DecodeCommandRxFIFO();
uint8_t USART_DecodeElevationRxFIFO();
uint8_t USART_DecodeTurnRxFIFO();
uint8_t USART_DecodeClimbRxFIFO();
void USART_DecodeRxFIFO();
void USART_Bounce();
void USART_SendValue(float flo);

uint8_t USART_getRotation();
uint8_t USART_getSpeed();
uint8_t USART_getDirection();
void USART_send_ready();

uint8_t USART_get_turn_flag();
uint8_t USART_get_turn_dir();
uint16_t USART_get_turn_angle();

float USART_get_z();
uint8_t USART_elevation_flag();
uint8_t USART_climb_flag();


#endif /* USART_H_ */