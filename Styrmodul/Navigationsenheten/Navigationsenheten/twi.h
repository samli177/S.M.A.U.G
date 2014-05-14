/*
 * twi.h
 *
 * Created: 4/4/2014 2:40:42 PM
 *  Author: perjo018
 */ 

#ifndef TWI_H
#define TWI_H

//Controlbyte (TWSR)
#define CONTROL (TWSR & 0xF8)
//Master Transmitter
#define START 0x08
#define REPEATEDSTART 0x10
#define ADDRESS_W 0x18
#define NO_ADDRESS_W 0x20
#define DATA_W 0x28
#define NO_DATA_W 0x30
#define ARBITRATION 0x38
//Master Reciever (+ START + REPEATEDSTART + ARBITRATION)
#define ADDRESS_R 0x40
#define NO_ADDRESS_R 0x48
#define DATA_R 0x50
#define NO_DATA_R 0x58
//Slave Reciever
#define SLAW 0x60
#define ARBIT_SLAW 0x68
#define GENERAL 0x70
#define ARBIT_GENERAL 0x78
#define DATA_SLAW 0x80
#define NO_DATA_SLAW 0x88
#define DATA_GENERAL 0x90
#define NO_DATA_GENERAL 0x98
#define STOP 0xA0
//Slave transmitter
#define SLAR 0xA8
#define ARBIT_SLAR 0xB0
#define DATA_SLAR 0xB8
#define NO_DATA_SLAR 0xC0
#define LAST_DATA_ERROR 0xC8

//Instruction byte
#define I_COMMAND 0
#define I_SETTINGS 1
#define I_SWEEP 2
#define I_STATUS 3
#define I_ELEVATION 4
#define I_AUTONOM 5
#define I_FLOAT 6
#define I_STRING 25

//Module addresses
#define C_ADDRESS 0x80
#define S_ADDRESS 0x40
#define ST_ADDRESS 0x20
#define G_ADDRESS 0

//Declarations
/**
 * \brief 
 * Initializes the TWI bus
 * \return void
 */
void TWI_init(uint8_t moduleAdress);
/**
 * \brief 
 * Sends an I_STATUS instruction packet over the TWI bus
 * \param adr
 * address to be sent to
 * \return int
 * 1 if address was connected, 0 otherwise
 */
uint8_t TWI_send_status(uint8_t adr);
/**
 * \brief 
 * Sends control settings, navigation -> communication or vise versa.
 * \param adr
 * ST_ADDRESS or C_ADDRESS
 * \param KP
 * The current value of control setting KP to be read or set.
 * \param KI
 * The current value of control setting KI to be read or set.
 * \param KD
 * The current value of control setting KD to be read or set.
 * \return int
 * 1 if packet was successfully sent, 0 otherwise
 */
uint8_t TWI_send_control_settings(uint8_t adr, uint8_t KP,uint8_t KI,uint8_t KD);
/**
 * \brief 
 * Sends autonomous settings, navigation -> communication or vise versa.
 * \param adr
 * ST_ADDRESS or C_ADDRESS
 * \param autonom
 * The current value of autonom, to be read or set.
 * \return int
 * 1 if packet was successfully sent, 0 otherwise
 */
uint8_t TWI_send_autonom_settings(uint8_t adr,uint8_t autonom);
/**
 * \brief 
 * Sends commands, communications -> navigation
 * \param direction
 * The direction the robot shall go
 * \param rotation
 * The rotation the robot shall do
 * \param speed
 * The speed the robot shall have
 * \return int
 * 1 if packet was successfully sent, 0 otherwise
 */
uint8_t TWI_send_command(uint8_t direction, uint8_t rotation, uint8_t speed);
/**
 * \brief 
 * Sends commands, communications -> navigation
 * \param direction
 * The direction the robot shall go
 * \param rotation
 * The rotation the robot shall do
 * \param speed
 * The speed the robot shall have
 * \return int
 * 1 if packet was successfully sent, 0 otherwise
 */
uint8_t TWI_send_elevation(uint8_t elevation);
/**
 * \brief 
 * Sends sensors, sensors -> general call
 * \param sens[8]
 * The array consisting of all sensor readings
 * \param serv
 * The current position of the servo
 * \return int
 * 1 if packet was successfully sent, 0 otherwise
 */
uint8_t TWI_send_sensors(uint8_t sens[8], uint8_t serv);
/**
 * \brief 
 * Sends an order for sweep, navigation -> sensor
 * \param sweep
 * The position for the servo to sweep to.
 * \return int
 * 1 if packet was successfully sent, 0 otherwise
 */
uint8_t TWI_send_sweep(uint8_t sweep);
/**
 * \brief 
 * Sends a string, any module -> communications or sensor
 * \param adr
 * C_ADDRESS or S_ADDRESS
 * \param str
 * The char array/string to be sent
 * \return int
  * 1 if packet was successfully sent, 0 otherwise
 */
uint8_t TWI_send_string(uint8_t adr, char str[]);
/**
 * \brief 
 * Sends a fixed length of a string, any module -> communications or sensor
 * \param adr
 * C_ADDRESS or S_ADDRESS
 * \param str
 * The char array/string to be sent
 * \return int
 * 1 if packet was successfully sent, 0 otherwise
 */
uint8_t TWI_send_string_fixed_length(uint8_t adr, uint8_t str[], int length);
/**
 * \brief 
 * Sends a float, any module -> communications
 * \param adr
 * C_ADDRESS, no other module will use it. (yet?)
 * \param flo
 * Float to be sent
 * \return int
 * 1 if packet was successfully sent, 0 otherwise
 */
uint8_t TWI_send_float(uint8_t adr, float flo);
/**
 * \brief 
 * Basic send function, sends one byte anywhere anyhow.
 * \param adr
 * Address to be sent to, C_ADDRESS, ST_ADDRESS or S_ADDRESS
 * \param instruction
 * Instruction for the packet
 * \param packet
 * Information to be sent
 * \return int
 * 1 if packet was successfully sent, 0 otherwise
 */
uint8_t TWI_send_something(uint8_t adr, uint8_t instruction, uint8_t packet);


/**
 * \brief 
 * Retrieves the value of sensor #i
 * \param i
 * The sensor # to retrieve
 * \return uint8_t
 * The value of sensor #i
 */
uint8_t TWI_get_sensor(int i);
/**
 * \brief 
 * Retrieves the value of the sensor-modules servo
 * \return uint8_t
 * The value of the the sensor-modules servo
 */
uint8_t TWI_get_servo();
/**
 * \brief 
 * Retrieves the value of the sensor-modules servo to reach
 * \return uint8_t
 * The value for the sensor-modules servo to reach
 */
uint8_t TWI_get_sweep();
/**
 * \brief 
 * Retrieves a command
 * \param i
 * Which command to be retrieved. 0 -> direction, 1 -> rotation, 2 -> speed
 * \return uint8_t
 * The value of the command
 */
uint8_t TWI_get_command(int i);
/**
 * \brief 
 * Retrieves a control setting
 * \param i
 * Which setting to be retrieved. 0 -> KP, 1 -> KI, 2 -> KD
 * \return uint8_t
 * The value of the setting
 */
uint8_t TWI_get_control_setting(int i);

/**
 * \brief 
 * Retrieves the autonomous settings
 * \return uint8_t
 * the autonomous settings
 */
uint8_t TWI_get_autonom_settings();


uint8_t TWI_get_elevation();

/**
 * \brief
 * Retrieves the elevation.
 * 
 * \return uint8_t
 * The elevation
 */
uint8_t TWI_get_elevation();

/**
 * \brief 
 * Decodes a message from the FIFO buffer.
 * \return uint8_t
 * 0 if message was decoded. 1 if an error occurred
 */
uint8_t decode_message_TwiFIFO();
/**
 * \brief 
 * Adds a message to the FIFO buffer
 * \param msg
 * The message to be added to the FIFO buffer
 * \return uint8_t
 * 0 if message was added. 1 if an error occurred
 */
uint8_t write_to_TwiFIFO(char msg[]);

//Returns 1 if a new instruction of the respective type has been received.
uint8_t TWI_sensor_flag();
uint8_t TWI_command_flag();
uint8_t TWI_control_settings_flag();
uint8_t TWI_autonom_settings_flag();
uint8_t TWI_elevation_flag();
uint8_t TWI_sweep_flag();

#endif