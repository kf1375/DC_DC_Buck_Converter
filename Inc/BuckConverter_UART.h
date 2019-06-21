#ifndef _BuckConverter_UART_H_
#define _BuckConverter_UART_H_

#include "stm32f1xx_hal.h"
/*----------------------------------------------------------------------------*/
#define	MY_ID											0x4B
#define DXL_NoError								0x00	//Dynamixel_Param
#define DXL_Error								  0x01	//Dynamixel_Param
#define DXL_READ_DATA      				0x02	//ReadData_Ins
#define DXL_WRITE_DATA     				0x03	//WriteData_Ins
#define BUFFER_SIZE  							10
/*----------------------------------------------------------------------------*/
typedef struct
{
	uint8_t  InputData;
	uint8_t  Buffer[BUFFER_SIZE];
	uint16_t BufferCurrentIndex;
	uint8_t  CurrentState;
	uint16_t BufferLastIndex;	
	uint8_t  Length;
}DXLLineTypeDef;
/*----------------------------------------------------------------------------*/
typedef enum
{
	SB_OK 	 = 0x01,
	SB_BUSY  = 0x02,
	SB_ERROR = 0x00
}SB_StatusTypeDef;
/*----------------------------------------------------------------------------*/
typedef enum
{
	PacketFirstHeader  = 0x00,
	PacketSecondHeader = 0x01,
	PacketID		   = 0x02,
	PacketLength	   = 0x03
}SB_DXL_InstructionPacketTypeDef;
/*----------------------------------------------------------------------------*/
extern UART_HandleTypeDef huart3;	
/*----------------------------------------------------------------------------*/
void    DXLStruct_Initializer (DXLLineTypeDef*  stct);
uint8_t Master_CheckPacket(UART_HandleTypeDef *UARTx, DXLLineTypeDef* master);
uint8_t DXL_CheckSumCalc(uint8_t *packet);
/*----------------------------------------------------------------------------*/
#endif /* _BuckConverter_UART_H_ */
