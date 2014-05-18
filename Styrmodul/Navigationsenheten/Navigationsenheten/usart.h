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
 * Set default destination for twi transmissions
 * \param address
 * TWI address
 * \return void
 */
void USART_set_twi_message_destination(uint16_t address);

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

uint16_t USART_crc16(uint8_t tag, uint8_t length);

void USART_send_climb();

uint8_t USART_decode_climb_done_rx_fifo();



/**
 * \brief 
 * Send packet over UART
 * \param tag
 * The tags are: M for message, C for command, E for elevation, G for gyro-data request, T for turn
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
 * Transmits the distance readings from the 9 sensors over USART
 * The data is read from the twi library
 * \return void
 */
void USART_send_sensors();



/**
 * \brief 
 * Decodes received packets from USART fifo-ring-buffer
 * IMPORTANT: this function needs to be called perpetually (and preferably outside interrupts) for USART communication to function as it should.
 * \return void
 */

void USART_decode_rx_fifo();
/**
 * \brief 
 * Decodes UART messages from fifo-ring buffer
 * 
 * \return uint8_t
 */
uint8_t USART_decode_message_rx_fifo();

/**
 * \brief 
 * Decodes USART command
 * NOTE: not used
 * \return uint8_t
 */
uint8_t USART_decode_command_rx_fifo();

/**
 * \brief 
 * For debugging UART
 * 
 * \return void
 */
void USART_bounce();

// --- Navigationsenheten, extra functions ---


/**
 * \brief 
 * Transmits a command-byte over USART
 * 
 * \return void
 */
void USART_send_command();


/**
 * \brief 
 * Transmits elevation from twi over USART
 * 
 * \return void
 */
void USART_send_elevation();


/**
 * \brief 
 * Transmits command to rotate angle degrees in direction dir over UART
 * \param angle
 * \param dir
 * 
 * \return void
 */

void USART_send_turn(uint16_t angle, uint8_t dir);

/**
 * \brief 
 * Transmits request for gyro data over UART
 * 
 * \return void
 */
void USART_request_gyro();

/**
 * \brief 
 * Decodes gyro data from fifo-ring-buffer
 * 
 * \return uint8_t
 */
uint8_t USART_decode_gyro_rx_fifo();

/**
 * \brief 
 * Decodes turn-done message from fifo-ring-buffer
 * 
 * \return uint8_t
 */
uint8_t USART_decode_turn_done_rx_fifo();

/**
 * \brief 
 * Sends movement command over UART
 * \param direction
 * \param rotation
 * \param speed
 * 
 * \return void
 */
void USART_send_command_parameters(uint8_t direction, uint8_t rotation, uint8_t speed);

/**
 * \brief 
 * Check if the GangReady-flag is set. 
 * Clears the flag if it is.
 * \return uint8_t
 * Returns 1 if flag is set
 */
uint8_t USART_ready();

/**
 * \brief 
 * Check if the TurnDone-flag is set. 
 * Clears the flag if it is.
 * \return uint8_t
 * Returns 1 if flag is set
 */
uint8_t USART_turn_done();

uint8_t USART_climb_done();


/**
 * \brief 
 * Check if the Gyro-flag is set. 
 * Clears the flag if it is.
 * \return uint8_t
 * Returns 1 if flag is set
 */
uint8_t USART_GyroFlag();

/**
 * \brief 
 * Get last updated pitch from the mpu
 * 
 * \return float
 * 
 */
float USART_gyro_get_P();

/**
 * \brief 
 * Get last updated roll from the mpu
 * 
 * \return float
 * 
 */
float USART_gyro_get_R();

/**
 * \brief 
 * Get last updated yaw from the mpu
 * 
 * \return float
 * 
 */
float USART_gyro_get_Y();




#endif /* USART_H_ */