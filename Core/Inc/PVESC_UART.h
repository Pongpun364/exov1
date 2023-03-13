#ifndef PVESC_UART_H
#define PVESC_UART_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include "stdint.h"
  
  
#define PACKET_MAX_PL_LEN		512 //Packet maximum Payload length


// VESC/bldc/datatypes.h -> Communication commands
typedef enum {
	COMM_FW_VERSION = 0,
	COMM_JUMP_TO_BOOTLOADER,
	COMM_ERASE_NEW_APP,
	COMM_WRITE_NEW_APP_DATA,
	COMM_GET_VALUES,
	COMM_SET_DUTY,
	COMM_SET_CURRENT,
	COMM_SET_CURRENT_BRAKE,
	COMM_SET_RPM,
	COMM_SET_POS,
	COMM_SET_HANDBRAKE,
	COMM_SET_DETECT,
	COMM_SET_SERVO_POS,
	COMM_SET_MCCONF,
	COMM_GET_MCCONF,
	COMM_GET_MCCONF_DEFAULT,
	COMM_SET_APPCONF,
	COMM_GET_APPCONF,
	COMM_GET_APPCONF_DEFAULT,
	COMM_SAMPLE_PRINT,
	COMM_TERMINAL_CMD,
	COMM_PRINT,
	COMM_ROTOR_POSITION,
	COMM_EXPERIMENT_SAMPLE,
	COMM_DETECT_MOTOR_PARAM,
	COMM_DETECT_MOTOR_R_L,
	COMM_DETECT_MOTOR_FLUX_LINKAGE,
	COMM_DETECT_ENCODER,
	COMM_DETECT_HALL_FOC,
	COMM_REBOOT,
	COMM_ALIVE,
	COMM_GET_DECODED_PPM,
	COMM_GET_DECODED_ADC,
	COMM_GET_DECODED_CHUK,
	COMM_FORWARD_CAN,
	COMM_SET_CHUCK_DATA,
	COMM_CUSTOM_APP_DATA,
	COMM_NRF_START_PAIRING
} COMM_PACKET_ID;



// VESC/bldc/crc.h -> Functions
uint16_t VESC_crc16(uint8_t *buf, uint16_t len);


uint8_t VESC_UARTsetDuty(uint8_t *messageSend, int32_t number);
//uint8_t VESC_UARTsetDuty(uint8_t *messageSend, float duty);
uint8_t VESC_UARTsetCurrent(uint8_t *messageSend, int32_t number);
//uint8_t VESC_UARTsetCurrent(uint8_t *messageSend, float current);








#ifdef __cplusplus
}
#endif

#endif /* PVESC_UART_H */