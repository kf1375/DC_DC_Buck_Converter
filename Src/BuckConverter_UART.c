#include "BuckConverter_UART.h"

void DXLStruct_Initializer(DXLLineTypeDef*  stct)
{
	stct->InputData 	  		 = 0;
	stct->Buffer[0]					 = 0;
	stct->BufferCurrentIndex = 0;
	stct->CurrentState 		 	 = 0;
	stct->BufferLastIndex 	 = 0;
	stct->Length 			 		 	 = 0;	
}

uint8_t Master_CheckPacket(UART_HandleTypeDef *UARTx, DXLLineTypeDef* master)
{
	if(HAL_UART_Receive_IT(UARTx, &master->InputData, 1) == HAL_OK)
	{
		master->Buffer[master->BufferCurrentIndex] = master->InputData;
		master->BufferCurrentIndex++;
		if(master->BufferCurrentIndex == 8){
			master->BufferCurrentIndex = 0;
		}
		switch(master->CurrentState)
		{
			case PacketFirstHeader:
				if(master->Buffer[master->BufferLastIndex + 0] == 0xFF) { master->CurrentState = PacketSecondHeader; }
				else{ master->CurrentState = PacketFirstHeader; master->BufferCurrentIndex = master->BufferLastIndex; }
			break;
			case PacketSecondHeader:
				if(master->Buffer[master->BufferLastIndex + 1] == 0xFF) { master->CurrentState = PacketID; }
				else{ master->CurrentState = PacketFirstHeader; master->BufferCurrentIndex = master->BufferLastIndex; }
			break;
			case PacketID:
				if(master->Buffer[master->BufferLastIndex + 2] != 0xFF) { master->CurrentState = PacketLength; }
				else{ master->CurrentState = PacketFirstHeader; master->BufferCurrentIndex = master->BufferLastIndex; }
			break;
			case PacketLength:
				if(master->Buffer[master->BufferLastIndex + 3] > master->Length) { master->Length++; }
				else
				{
					master->Length = 0;
					master->CurrentState = 0;
					uint8_t PacketLength = master->Buffer[master->BufferLastIndex + 3] + 4;
					return PacketLength;
				}
			break;
		}
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
uint8_t DXL_CheckSumCalc(uint8_t *packet)
{
	uint8_t i;
	uint8_t checksum =0;
	for(i = 2; i < packet[3] + 3; i++)
	{
		checksum += packet[i];
	}
	checksum =~ checksum;
	return(checksum);
}
/*----------------------------------------------------------------------------*/


