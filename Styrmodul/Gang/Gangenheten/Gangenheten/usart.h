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

uint16_t USART_crc16(uint8_t tag, uint8_t length);

/**
 * \brief 
 * Send packet over UART
 * \param tag
 * The tags are: M for message, C for command, E for elevation, G for gyro-data, T for turn, R for gang-ready
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
 * Decodes received packets from USART fifo-ring-buffer
 * IMPORTANT: this function needs to be called perpetually (and preferably outside interrupts) for USART communication to function as it should.
 * \return void
 */
void USART_decode_rx_fifo();

/**
 * \brief 
 * Decodes UART messages -tag from fifo-ring buffer
 * 
 * \return uint8_t
 */
uint8_t USART_decode_message_rx_fifo();

uint8_t USART_decode_parameters_rx_fifo();


/**
 * \brief 
 * Transmits a value (float) over UART
 * \param msg
 * The bytes bytes the float as a char array
 * \return void
 */
void USART_send_value(float flo);


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

// --- Gang, extra functions ---

/**
 * \brief 
 * Transmit last updated mpu-angles in yaw-pitch-roll format over UART as floats
 * 
 * \return void
 */
void USART_send_gyro();

/**
 * \brief 
 * Transmit "turn done" message over UART 
 * 
 * \return void
 */
void USART_send_turn_done();

// TODO: add comments
void USART_send_climb_done();

uint8_t USART_decode_climb_rx_fifo();


/**
 * \brief 
 * Transmit "ready" message over UART
 * 
 * \return void
 */
void USART_send_ready();

/**
 * \brief 
 * Decode command for changing elevation of robot body
 * Flag is set when elevation is changed
 * \return uint8_t
 */
uint8_t USART_decode_elevation_rx_fifo();

/**
 * \brief 
 * Decode command for turning a given angle
 * Flag is set when command is received
 * \return uint8_t
 */
uint8_t USART_decode_turn_rx_fifo();


/**
 * \brief 
 * Returns rotation if new rotation has been received since last called, if not returns 50
 * 
 * \return uint8_t
 */
uint8_t USART_getRotation();

/**
 * \brief 
 * Returns speed if new speed has been received since last called, if not returns 0
 * 
 * \return uint8_t
 */
uint8_t USART_getSpeed();

/**
 * \brief 
 * Returns direction if new direction has been received since last called, if not returns 0
 * 
 * \return uint8_t
 */
uint8_t USART_getDirection();


/**
 * \brief 
 * Returns and clears flag
 * 
 * \return uint8_t
 */
uint8_t USART_get_turn_flag();

/**
 * \brief 
 * Returns and clears flag
 * 
 * \return uint8_t
 */
uint8_t USART_get_turn_dir();

/**
 * \brief 
 * Returns and clears flag
 * 
 * \return uint8_t
 */
uint16_t USART_get_turn_angle();

uint8_t USART_get_climb_flag();

uint8_t USART_parameters_flag();

uint8_t USART_get_parameter(uint8_t index);


/**
 * \brief 
 * Returns z 
 * 
 * \return float
 */

float USART_get_z();

/**
 * \brief 
 * Returns and clears flag
 * 
 * \return uint8_t
 */
uint8_t USART_elevation_flag();


#endif /* USART_H_ */