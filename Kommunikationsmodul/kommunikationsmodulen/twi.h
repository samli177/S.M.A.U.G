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
#define I_STRING 25

//Module Adresses
#define C_ADRESS 0x80
#define S_ADRESS 0x40
#define ST_ADRESS 0x20
#define G_ADRESS 0

//Declarations
void init_TWI(int);
void set_twi_reciever_enable();
void Error();
void start_bus();
void stop_bus();
void clear_int();
void set_data(int);
int get_data();
void send_bus();
void wait_for_bus();
bool send_status(int);
bool send_settings();
bool send_command(int,int,int);
bool send_sensors();
bool send_sweep(int);
bool send_string(int,char[]);
bool send_string_fixed_length(int adr, uint8_t str[], int len);
bool send_something(int,int,int);
void reset_TWI();

#endif