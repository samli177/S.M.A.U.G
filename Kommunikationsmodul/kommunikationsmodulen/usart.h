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


/**
 * \brief 
 * Sets up uart with the baudrate defined in usart.h
 * Frame format: 8 data, no parity, 1 stop bit
 * \return void
 */
void USART_init();

/**
 * \brief 
 * Check if USART data register is ready to be read
 * NOTE: not used
 * \return uint8_t
 */
uint8_t USART_check_rx_complete();

/**
 * \brief 
 * Check if new byte can be written to USART data register
 * 
 * \return uint8_t
 * zero if data register is not ready to receive new data
 */
uint8_t USART_check_tx_ready();

/**
 * \brief 
 * Writes byte if data to USART data register, the hardware will handle the transmission
 * \param DataByteOut
 * Byte to be transmitted via UART
 * \return void
 */
void USART_write_byte(uint8_t DataByteOut);


/**
 * \brief 
 * Reads byte from USART data register
 * NOTE: not used
 * \return uint8_t
 */
uint8_t USART_read_byte();


// TODO: use this, seems to not be needed though...
uint16_t USART_crc16(uint8_t tag, uint8_t length);

/**
 * \brief 
 * Send packet over UART
 * \param tag
 * The tags are: M for message, C for command, V for value (float), P for parameters, A for autonomous settings & E for elevation settings
 * NOTE: the payload needs to be written to gTxPayload before this function is called
 * \param length
 * Number of bytes in payload
 * \return void
 */
void USART_send_packet(char tag, uint8_t length);

/**
 * \brief 
 * Transmits a char array over UART 
 * \param msg
 * Char array to be transmitted
 * \return void
 */
void USART_send_message(char msg[]);

/**
 * \brief 
 * Transmits a value (float) over UART
 * \param msg
 * The bytes bytes the float as a char array
 * \return void
 */
void USART_send_value(uint8_t msg[]);

/**
 * \brief 
 * Transmits the distance readings from the 9 sensors over USART
 * The data is read from the twi library
 * \return void
 */
void USART_send_sensors();


/**
 * \brief 
 *  Transmits the autonom settings of the robot.
 * \param settings
 *  The settings to be transmitted
 * \return void
 */
void USART_send_autonom(uint8_t settings);
/**
 * \brief 
 * Decodes received packets from USART fifo-ring-buffer
 * IMPORTANT: this function needs to be called perpetually (and preferably outside interrupts) for USART communication to function as it should.
 * \return void
 */
void USART_decode_rx_fifo();

/**
 * \brief 
 * Decodes UART messages -tag from fifo-ring buffer
 * Sends messages to display
 * \return uint8_t
 */
uint8_t USART_decode_message_rx_fifo();

/**
 * \brief 
 * Decodes UART command and sends it to Navigationsenheten over twi
 * 
 * \return uint8_t
 */
uint8_t USART_decode_command_rx_fifo();

/**
 * \brief 
 * Decodes UART command and sends it to Navigationsenheten over twi
 * NOTE: not used
 * \return uint8_t
 */
uint8_t USART_decode_parameters_rx_fifo();

/**
 * \brief 
 * Decodes UART autonomous settings and sends them to Navigationsenheten over twi
 * 
 * \return uint8_t
 */
uint8_t USART_decode_autonom_rx_fifo();

/**
 * \brief 
 * Decodes package telling if the robot should send status messages in autonomous mode.
 * 
 * \return uint8_t
 */
uint8_t USART_decode_status_rx_fifo();


/**
 * \brief 
 * For debugging UART
 * 
 * \return void
 */
void USART_bounce();




#endif /* USART_H_ */