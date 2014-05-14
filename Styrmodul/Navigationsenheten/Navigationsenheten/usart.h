/*
 * usart.h
 *
 * Created: 4/8/2014 1:52:09 PM
 *  Author: samli177
 */ 


#ifndef USART_H_
#define USART_H_



#define BaudRate 115200
#define POLY 0x8408
// -- Declarations --

void USART_init();
void USART_set_twi_message_destination(uint16_t address);
uint8_t USART_CheckRxComplete();
uint8_t USART_CheckTxReady();
void USART_WriteByte(uint8_t DataByteOut);
uint8_t USART_ReadByte();
uint16_t USART_crc16(uint8_t tag, uint8_t length);
void USART_SendPacket(char tag, uint8_t length);
void USART_SendMessage(char msg[]);
void USART_SendSensors();
void USART_SendCommand();
void USART_SendElevation();
void USART_SendTurn(uint16_t angle, uint8_t dir);
void USART_SendClimb();
void USART_RequestGyro();
uint8_t USART_DecodeMessageRxFIFO();
uint8_t USART_DecodeCommandRxFIFO();
uint8_t USART_DecodeGyroRxFIFO();
uint8_t USART_DecodeTurnDoneRxFIFO();
uint8_t USART_DecodeClimbDoneRxFIFO();
void USART_DecodeRxFIFO();
void USART_Bounce();
void USART_send_command_parameters(uint8_t direction, uint8_t rotation, uint8_t speed);

uint8_t USART_ready();
uint8_t USART_turn_done();
uint8_t USART_climb_done();
uint8_t USART_GyroFlag();
float USART_gyro_get_P();
float USART_gyro_get_R();
float USART_gyro_get_Y();




#endif /* USART_H_ */