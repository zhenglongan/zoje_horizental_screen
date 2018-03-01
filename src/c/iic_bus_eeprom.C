//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : iic_bus.c
//  Description: 
//  Version    Date     Author    Description
//  0.01     19/07/06   liwenz        created
//  ...
//--------------------------------------------------------------------------------------
#include "..\..\include\common.h"       	//Common constants definition
#if IIC_FUNCTION_ENABLE
#include "..\..\include\sfr62p.h"           //M16C/62P special function register definitions
#include "..\..\include\iic_bus_eeprom.h"   
#include "..\..\include\typedef.h"     		//Data type define
#include "..\..\include\variables.h"        //External variables declaration       
#include "..\..\include\delay.h"

#define iic_sda_d	pd7_0
#define iic_sda		p7_0
#define iic_scl_d	pd7_1
#define iic_scl		p7_1

void _WaitTime0us(void);
void _WaitTime1us(void);
void _WaitTime2us(void);

#define _Wait_tHIGH		_WaitTime1us()	/* Clock pulse width high */
#define _Wait_tLOW		_WaitTime2us()	/* Clock pulse width low */
#define _Wait_tHD_STA	_WaitTime1us()	/* Start hold time */
#define _Wait_tSU_STA	_WaitTime1us()	/* Start setup time */
#define _Wait_tHD_DAT	_WaitTime0us()	/* Data in hold time */
#define _Wait_tSU_DAT	_WaitTime1us()	/* Data in setup time */
#define _Wait_tAA		_WaitTime1us()	/* Access time */
#define _Wait_tSU_STO	_WaitTime1us()	/* Stop setup time */
#define _Wait_tBUF		_WaitTime2us()	/* Bus free time for next mode */


/************************************************************************************
 Name			: initIicBus
 Parameters		: None
 Returns		: None
 Description	: initialize I2C-BUS port
 Note			: 
************************************************************************************/
void initIicBus(void)
{
	iic_sda_d = 0;			/* SDA input ("H" state) */
	iic_scl_d = 0;			/* SCL input ("H" state) */
}


/************************************************************************************
 Name			: IicBusRead
 Parameters		: structure IicPack pointer
 Returns		: Acknowledge
 Description	: Sequential Ramdom Read Cycle (I2C-BUS)
 Note			: 
************************************************************************************/
UINT8 IicBusRead(IicPack *IicData)
{
	UINT8 i,ret;

	/* Ramdom Read Cycle / Sequential Ramdom Read Cycle */
	IicData->iic_DeviceAddress &= 0xFE;						/* WRITE Setting Device Address */
	StartCondition();										/* Start Condition */
	while (1) {
		if ((ret=ByteWrite(IicData->iic_DeviceAddress)) == NOACK)	/* WRITE Device Address */
			break;											/* NoAck Detect */
		if ((ret=ByteWrite(IicData->iic_MemoryAddress_h)) == NOACK)	/* WRITE Memory Address */
			break;											/* NoAck Detect */
		if ((ret=ByteWrite(IicData->iic_MemoryAddress_l)) == NOACK)	/* WRITE Memory Address */
			break;
		IicData->iic_DeviceAddress |= 0x01;					/* READ Setting Device Address */
		StartCondition();									/* ReStart Condition */
		if ((ret=ByteWrite(IicData->iic_DeviceAddress)) == NOACK)	/* WRITE Device Address */
			break;											/* NoAck Detect */
		for (i=1; i<IicData->iic_NumberOfByte; i++) {		/* specified bytes as loop */
			ByteRead(IicData->iic_Data, ACK);				/* Read data (Ack output) */
			IicData->iic_Data++;							/*  */
		}
		ByteRead(IicData->iic_Data, NOACK);					/* Read data (NoAck output) */
		break;
	}
	StopCondition();										/* Stop Condition */
	return(ret);
}


/************************************************************************************
 Name			: IicBusWrite
 Parameters		: structure IicPack pointer
 Returns		: Acknowledge
 Description	: Byte Write or Page Write Cycle (I2C-BUS)
 Note			: 
************************************************************************************/
UINT8 IicBusWrite(IicPack *IicData)
{
	UINT8 i,ret;

	/* Byte Write / Page Write */
	IicData->iic_DeviceAddress &= 0xFE;						/* WRITE Setting Device Address */
	StartCondition();										/* Start Condition */
	while (1) {
		if ((ret=ByteWrite(IicData->iic_DeviceAddress)) == NOACK)	/* Write Device Address */
			break;											/* NoAck Detect */
		if ((ret=ByteWrite(IicData->iic_MemoryAddress_h)) == NOACK)	/* Write Memory Addreess */
			break;											/* NoAck Detect */
		if ((ret=ByteWrite(IicData->iic_MemoryAddress_l)) == NOACK)	/* Write Memory Addreess */
			break;
		for (i=0; i<IicData->iic_NumberOfByte; i++) {		/* specified bytes as loop */
			if ((ret=ByteWrite(*(IicData->iic_Data))) == NOACK)	/* Write Data */
				break;										/* NoAck Detect */
			IicData->iic_Data++;							/*  */
		}
		break;
	}
	StopCondition();										/* Stop Condition */
	return(ret);
}


/************************************************************************************
 Name			: StartCondition
 Parameters		: None
 Returns		: None
 Description	: Output Start Condition (I2C-BUS)
 Note			: *1 adjust a wait time
************************************************************************************/
void StartCondition(void)
{
	iic_scl = 0;				/* SCL="L" */
	iic_scl_d = 1;				/* SCL output */
	_WaitTime1us();				/* wait *1 */
	iic_sda_d = 0;				/* SDA="H" */
	_WaitTime1us();				/* wait */
	_WaitTime1us();				/* wait *! */
	iic_scl = 1;				/* SCL="H" */
	_Wait_tSU_STA;				/* wait */
	iic_sda = 0;				/* SDA="L" */
	iic_sda_d = 1;				/* SDA output */
	_Wait_tHD_STA;				/* wait */
	_WaitTime1us();				/* wait *1 */
	iic_scl = 0;				/* SCL="L" */
}


/************************************************************************************
 Name			: StopCondition
 Parameters		: None
 Returns		: None
 Description	: Output Stop Condition (I2C-BUS)
 Note			: *1 adjust a wait time
************************************************************************************/
void StopCondition(void)
{
	iic_scl = 0;				/* SCL="L" */
	iic_scl_d = 1;				/* SCL output */
	_WaitTime1us();				/* wait *1 */
	iic_sda = 0;				/* SDA="L" */
	iic_sda_d = 1;				/* SDA output */
	_WaitTime1us();				/* wait *1 */
	iic_scl = 1;				/* SCL="H" */
	_Wait_tSU_STO;				/* wait */
	iic_sda_d = 0;				/* SDA="H" */
	_WaitTime1us();				/* wait */
	_WaitTime1us();				/* wait *1 */
	iic_scl = 0;				/* SCL="L" */
}


/************************************************************************************
 Name			: ByteWrite
 Parameters		: Write data
 Returns		: Acknowledge
 Description	: byte data Output (I2C-BUS)
 Note			: *1 adjust a wait time
************************************************************************************/
UINT8 ByteWrite(UINT8 iic_writeData)
{
	UINT8 maskData=0x80;		/* MSB first */
	UINT8 ret=ACK;				/* Ack/NoAck */

	while (maskData) {						/* 8times as loop */
		iic_sda = 0;						/* initialize port-latch */
		if (iic_writeData & maskData) {		/* "H" output ? */
			iic_sda_d = 0;					/*Yes SDA="H" */
		}else{
			iic_sda_d = 1;					/* No  SDA="L" */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
		}
		_Wait_tSU_DAT;						/* wait */
		_WaitTime1us();						/* wait *1 */
		iic_scl = 1;						/* SCL="H" */
		_Wait_tHIGH;						/* wait */
		_WaitTime1us();						/* wait *1 */
		iic_scl = 0;						/* SCL="L" */
		maskData >>= 1;						/* change mask data */
		_WaitTime1us();						/* wait *1 */
	}
	iic_sda_d = 0;							/* SDA input */
	_Wait_tAA;								/* wait */
	_WaitTime2us();							/* wait *1 */
	iic_scl = 1;							/* SCL="H" */
	if (iic_sda) ret=NOACK;					/* NoAck Detect */
	_Wait_tHIGH;							/* wait */
	_WaitTime1us();							/* wait *1 */
	iic_scl = 0;							/* SCL="L" */
	_Wait_tHD_DAT;							/* wait */
	return(ret);
}


/************************************************************************************
 Name			: ByteRead
 Parameters		: Read data strage location pointer, Select Ack/NoAck
 Returns		: None
 Description	: byte data input with Ack output (I2C-BUS)
 Note			: *1 adjust a wait time
************************************************************************************/
void ByteRead(UINT8 *iic_readData, UINT8 ackData)
{
	UINT8 maskData=0x80;		/* MSB first */
	UINT8 readData;

	*iic_readData = 0;						/*  */
	while (maskData) {						/* 8times as loop */
		readData = *iic_readData | maskData;	/*  */
		iic_sda_d = 0;						/* initialize port-latch */
		_Wait_tAA;							/* wait */
		iic_scl = 1;						/* SCL="H" */
		if (iic_sda) {						/* SDA="H" ? */
			*iic_readData = readData;		/* Yes  */
		}else{
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
			asm("nop");						/* wait *1 */
		}
		_Wait_tHIGH;						/* wait */
		_WaitTime1us();						/* wait *1 */
		iic_scl = 0;						/* SCL="L" */
		maskData >>= 1;						/* Change mask data */
		_WaitTime1us();						/* wait *1 */
	}
	if (!ackData) {							/* Ack output ? */
	/* Ack output */
		iic_sda = ACK;						/* Yes SDA="L" */
		iic_sda_d = 1;						/* SDA output */
	}else{
	/* NoAck output */
		iic_sda = NOACK;					/* No  SDA="H" */
		iic_sda_d = 0;						/* SDA input */
	}
	_Wait_tSU_DAT;							/* wait */
	_WaitTime1us();							/* wait *1 */
	iic_scl = 1;							/* SCL="H" */
	_Wait_tHIGH;							/* wait */
	_WaitTime1us();							/* wait *1 */
	iic_scl = 0;							/* SCL="L" */
	iic_sda_d = 0;							/* SDA input */
	_WaitTime1us();							/* wait *1 */
}


/************************************************************************************
 Name			: _WaitTime0us
 Parameters		: None
 Returns		: None
 Description	: a 0us wait
************************************************************************************/
void _WaitTime0us(void)
{
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
}


/************************************************************************************
 Name			: _WaitTime1us
 Parameters		: None
 Returns		: None
 Description	: a 1us wait
************************************************************************************/
void _WaitTime1us(void)
{
	/* +14cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle = 24cycle */
}


/************************************************************************************
 Name			: _WaitTime2us
 Parameters		: None
 Returns		: None
 Description	: a 2us wait
************************************************************************************/
void _WaitTime2us(void)
{
	/* +14cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle */
	asm("nop");		/* +1cycle = 48cycle */
}


//-------------------------------------------------------------------------------------------------------
//  以下为eeprom操作部分
//-------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------
//  Name:		
//  Parameters:	
//  Returns:    	
//  Description: 
//--------------------------------------------------------------------------------------
void init_eeprom(void)
{	
    initIicBus();
}
/************************************************************************************
 Name			: _WaitTime6ms
 Parameters		: None
 Returns		: None
 Description	: a 6ms wait
************************************************************************************/
void _WaitTime6ms(void)
{
	INT16 iee;
	for(iee=0;iee<6000;iee++)
    {
		asm("nop");		/* +1cycle */
		asm("nop");		/* +1cycle */
		asm("nop");		/* +1cycle */
		asm("nop");		/* +1cycle */
		asm("nop");		/* +1cycle */
		asm("nop");		/* +1cycle */
		asm("nop");		/* +1cycle */
		asm("nop");		/* +1cycle */
		asm("nop");		/* +1cycle */
		asm("nop");		/* +1cycle */
		asm("nop");		/* +1cycle */
		asm("nop");		/* +1cycle */
		asm("nop");		/* +1cycle */
		asm("nop");		/* +1cycle = 24cycle */	    
	}
}
UINT16 read_par(UINT16 par_num)
{
	UINT16 a1 = 0; 
	IicPack IicData_r; 	 
	IicData_r.iic_DeviceAddress = 0xA0;
	IicData_r.iic_MemoryAddress_h = (UINT8)(par_num>>7);
	IicData_r.iic_MemoryAddress_l = (UINT8)(par_num<<1);
	IicData_r.iic_Data = (UINT8*)(&a1);
	IicData_r.iic_NumberOfByte = 2;
	if(IicBusRead(&IicData_r) == NOACK) 
	{	
		sys.error = ERROR_20;
		sys.status = ERROR;
	}	
	return a1;	 
}
void write_par(UINT16 par_num,INT16 par)
{	
	IicPack IicData_w;				
	IicData_w.iic_DeviceAddress = 0xA0;
	IicData_w.iic_MemoryAddress_h = (UINT8)(par_num>>7);
	IicData_w.iic_MemoryAddress_l = (UINT8)(par_num<<1);
	IicData_w.iic_Data = (UINT8*)(&par);
	IicData_w.iic_NumberOfByte = 2;
	if(IicBusWrite(&IicData_w)== NOACK) 
	{	
		sys.error = ERROR_21;
		sys.status = ERROR;
	}
	_WaitTime6ms();
}

void read_para_group( UINT16 address,UINT8 *point,UINT16 len)
{
	UINT16 i,data;
	i = 0;
	while( i<len )
	{
		data = read_par(address + i);
		*point++ = (data >> 8)&0xff;
		*point++ = data&0xff;
		i+=2;
	}
}

void write_para_group( UINT16 address,UINT8 *point,UINT16 len)
{
	UINT16 i,data;
	i = 0;
	while( i<len )
	{
		data = *point++;
		data = (data <<8)+ *point++;
		write_par(address + i,data);
		_WaitTime6ms();
		i+=2;
	}
}

#endif
//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xindahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
