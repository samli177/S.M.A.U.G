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
#define ADRESS_W 0x18
#define NO_ADRESS_W 0x20
#define DATA_W 0x28
#define NO_DATA_W 0x30
#define ARBITRATION 0x38
//Master Reciever (+ START + REPEATEDSTART + ARBITRATION)
#define ADRESS_R 0x40
#define NO_ADRESS_R 0x48
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

//Instructionbyte
#define I_COMMAND 0
#define I_SETTINGS 1
#define I_SWEEP 2
#define I_STATUS 3
#define I_ELEVATION 4
#define I_AUTONOM 5
#define I_FLOAT 6
#define I_STRING 25

//Module Adresses
#define C_ADRESS 0x80
#define S_ADRESS 0x40
#define ST_ADRESS 0x20
#define G_ADRESS 0

//Declarations
void TWI_init(uint8_t module_adress);

bool TWI_send_status(uint8_t status);
bool TWI_send_control_settings(uint8_t adr, uint8_t KP,uint8_t KI,uint8_t KD);
bool TWI_send_autonom_settings(uint8_t adr,uint8_t autonom);
bool TWI_send_command(uint8_t direction, uint8_t rot_elev, uint8_t speed);
bool TWI_send_elevation(uint8_t elevation);
bool TWI_send_sensors(uint8_t sens[7], uint8_t serv);
bool TWI_send_sweep(uint8_t sweep);
bool TWI_send_string(uint8_t adr, char str[]);
bool TWI_send_string_fixed_length(uint8_t adr, uint8_t str[], int length);
bool TWI_send_float(uint8_t adr, float flo);
bool TWI_send_something(uint8_t adr, uint8_t instruction, uint8_t packet);

char TWI_get_char(int i);
uint8_t TWI_get_sensor(int i);
uint8_t TWI_get_servo();
uint8_t TWI_get_sweep();
uint8_t TWI_get_command(int i);
uint8_t TWI_get_control_setting(int i);
uint8_t TWI_get_autonom_settings();
uint8_t TWI_get_message_length();

//Flags for new data, should be set false when read true
bool TWI_sensor_flag();
bool TWI_command_flag();
bool TWI_control_settings_flag();
bool TWI_autonom_settings_flag();
bool TWI_elevation_flag();
bool TWI_sweep_flag();

#endif