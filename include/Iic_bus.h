//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : 
//  Description: 
//  Version    Date     Author    Description
//  0.01     21/02/06   liwenz    created
//  ... 
//  ...
//--------------------------------------------------------------------------------------
#include "typedef.h"      //Data type define
#define ACK		0
#define NOACK	1

#define WRITE_MODE	0
#define READ_MODE	1

//2012-2-14
#define IIC_BUS_OK   0
#define IIC_BUS_BUSY 1

typedef struct {
	UINT8 iic_DeviceAddress;
	UINT8 iic_MemoryAddress;
	UINT8 *iic_Data;
	UINT8 iic_NumberOfByte;
}IicPack;

//2012-2-14 for IIC
void init_iic(void);
UINT8 Set_IIC_StartCondition(void);
void Set_IIC_RestartCondition(void);
void Set_IIC_StopCondition();
void IIC_ACK_INT(void);
void IIC_NACK_INT(void);
void IIC_Config_OutputPort(void);
void IIC_Config_InputPort(void);
void IIC_Master_Write(void);
void IIC_Master_Read(void);
//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xindahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------