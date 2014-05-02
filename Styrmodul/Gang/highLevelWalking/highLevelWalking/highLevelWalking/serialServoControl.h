﻿/*
 * IncFile1.h
 *
 * Created: 4/5/2014 9:55:46 AM
 *  Author: samli177
 */ 


#ifndef INCFILE1_H_
#define INCFILE1_H_

#define F_CPU 16000000
#define ServoBaudRate 1000000


// -- Direction control (tristate buffer) --

#define servoDDR DDRD
#define servoDirPort PORTD
#define servoDirPin PORTD4

#define servoRx servoDirPort |= (1<<servoDirPin)
#define servoTx servoDirPort &= ~(1<<servoDirPin)

// -- Servo read modes --

#define RM_WAIT_FOR_START 0
#define RM_CHECK_FOR_SECOND_START 1
#define RM_READ_ID 2
#define RM_READ_LENGTH 3
#define RM_READ_ERROR 4
#define RM_READ_INSTRUCTION 5
#define RM_READ_PARAMETERS 6
#define RM_READ_CHECKSUM 7

//--- Control Table Address ---

//EEPROM AREA
#define P_MODEL_NUMBER_L 0
#define P_MODOEL_NUMBER_H 1
#define P_VERSION 2
#define P_ID 3
#define P_BAUD_RATE 4
#define P_RETURN_DELAY_TIME 5
#define P_CW_ANGLE_LIMIT_L 6
#define P_CW_ANGLE_LIMIT_H 7
#define P_CCW_ANGLE_LIMIT_L 8
#define P_CCW_ANGLE_LIMIT_H 9
#define P_SYSTEM_DATA2 10
#define P_LIMIT_TEMPERATURE 11
#define P_DOWN_LIMIT_VOLTAGE 12
#define P_UP_LIMIT_VOLTAGE 13
#define P_MAX_TORQUE_L 14
#define P_MAX_TORQUE_H 15
#define P_RETURN_LEVEL 16
#define P_ALARM_LED 17
#define P_ALARM_SHUTDOWN 18
#define P_OPERATING_MODE 19
#define P_DOWN_CALIBRATION_L 20
#define P_DOWN_CALIBRATION_H 21
#define P_UP_CALIBRATION_L 22
#define P_UP_CALIBRATION_H 23

//RAM AREA

#define P_TORQUE_ENABLE (24)
#define P_LED (25)
#define P_CW_COMPLIANCE_MARGIN (26)
#define P_CCW_COMPLIANCE_MARGIN (27)
#define P_CW_COMPLIANCE_SLOPE (28)
#define P_CCW_COMPLIANCE_SLOPE (29)
#define P_GOAL_POSITION_L (30)
#define P_GOAL_POSITION_H (31)
#define P_GOAL_SPEED_L (32)
#define P_GOAL_SPEED_H (33)
#define P_TORQUE_LIMIT_L (34)
#define P_TORQUE_LIMIT_H (35)
#define P_PRESENT_POSITION_L (36)
#define P_PRESENT_POSITION_H (37)
#define P_PRESENT_SPEED_L (38)
#define P_PRESENT_SPEED_H (39)
#define P_PRESENT_LOAD_L (40)
#define P_PRESENT_LOAD_H (41)
#define P_PRESENT_VOLTAGE (42)
#define P_PRESENT_TEMPERATURE (43)
#define P_REGISTERED_INSTRUCTION (44)
#define P_PAUSE_TIME (45)
#define P_MOVING (46)
#define P_LOCK (47)
#define P_PUNCH_L (48)
#define P_PUNCH_H (49)

//--- Instruction ---
#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_REG_WRITE 0x04
#define INST_ACTION 0x05
#define INST_RESET 0x06
#define INST_DIGITAL_RESET 0x07
#define INST_SYSTEM_READ 0x0C
#define INST_SYSTEM_WRITE 0x0D
#define INST_SYNC_WRITE 0x83
#define INST_SYNC_REG_WRITE 0x84

#define BROADCASTING_ID 0xfe

// -- Declarations --
struct LegData;

void SERVO_init();

// -- Private functions --

// TODO: declare static, write descriptions
uint8_t servo_check_rx_complete();
uint8_t servo_check_tx_ready();
void servo_write_byte(uint8_t DataByteOut);
uint8_t servo_read_byte(); 
void send_servo_packet(uint8_t ID, uint8_t instruction, uint8_t parametersLength);


// -- Public functions --
void SERVO_goto(uint8_t ID, double angle, uint16_t speed);
void SERVO_set_angle_limit(uint8_t ID, double minAngle, double maxAngle);
void SERVO_set_return_level(uint8_t ID, uint8_t level);
void SERVO_set_torque_limit(uint8_t ID, uint16_t maxTorque);

void SERVO_update_data(uint8_t ID);

void SERVO_action();
void SERVO_buffer_position(uint8_t ID, double angle, uint16_t speed);

void SERVO_update_EEPROM(uint8_t ID);
void SERVO_set_return_delay_time(uint8_t ID, uint8_t delay);

void SERVO_update_leg_info(struct LegData* leg);

uint16_t SERVO_get_pos();
uint16_t SERVO_get_speed();
uint16_t SERVO_get_load();
uint8_t SERVO_get_temperature();
uint8_t SERVO_get_voltage();



#endif /* INCFILE1_H_ */