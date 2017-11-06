 //--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : main.c
//  Description: Core program to control the sewing machine
//  Version    Date     Author    Description
//  0.01     24/07/07   lm        created
//  ...
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
// 	Includes
//--------------------------------------------------------------------------------------
#include "..\..\include\sfr62p.h"       // M16C/62P special function register definitions
#include "..\..\include\typedef.h"      // Data type define
#include "..\..\include\common.h"       // Common constants definition
#include "..\..\include\variables.h"    // External variables declaration
#include "..\..\include\delay.h"        // delay time definition
#include "..\..\include\action.h"       // action function
//--------------------------------------------------------------------------------------
//  functions declaration
//--------------------------------------------------------------------------------------
void fw_solenoid(void);
void fa_solenoid(void);
void io_test(void);
//--------------------------------------------------------------------------------------
//  Name:		fw_solenoid 
//  Parameters:	None
//  Returns:	None
//  Description: thread wiper solenoid driver
//--------------------------------------------------------------------------------------
void fw_solenoid(void)
{		
	//--------------------------------------------------------------------------------------      
  	//  wiper move
  	//--------------------------------------------------------------------------------------
	if(u206 == 1)        // wiper enable
	{	
	  	SNT_H = 1;        // 24V to 33V
	  	FW = 1;
    	delay_ms(50);
	  	FW = 0;
	  	SNT_H = 0;        // 33V to 24V
	  	delay_ms(50);
	}
}
//--------------------------------------------------------------------------------------
//  Name:		aw_solenoid 
//  Parameters:	None
//  Returns:	None
//  Description: thread wiper solenoid driver
//--------------------------------------------------------------------------------------

void at_solenoid(void)
{
	//if(temp_tension_last != temp_tension)
	{
		da0 = 250;
		if( temp_tension > 250 )
		    temp_tension =250;
		temp_tension_last = temp_tension;
		tension_open_counter = 0;
		tension_open_switch = 1;
  	}	
}

//--------------------------------------------------------------------------------------
//  Name:		fa_solenoid 
//  Parameters:	None
//  Returns:	None
//  Description: thread trimmer solenoid driver
//--------------------------------------------------------------------------------------
void fa_solenoid(void)
{		
	
	//--------------------------------------------------------------------------------------      
  	//  trimmer move
  	//--------------------------------------------------------------------------------------	
	SNT_H = 1;        // 24V to 33V
  	delay_ms(50);
	SNT_H = 0;        // 33V to 24V
	delay_ms(200);
	
}

//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
