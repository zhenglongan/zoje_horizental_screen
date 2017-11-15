//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : communication.c
//  Description: Core program to control UART communication and protocol
//  Version    Date     Author    Description
///--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
// 	Includes
//--------------------------------------------------------------------------------------
#include "..\..\include\sfr62p.h"           // M16C/62P special function register definitions
#include "..\..\include\typedef.h"          // Data type define
#include "..\..\include\common.h"           // Common constants definition
#include "..\..\include\variables.h"        // External variables declaration
#include "..\..\include\stepmotor.h"        // constants definition
#include "..\..\include\action.h"           // action function
#include "..\..\include\delay.h"            // delay time definition
#include "..\..\include\motor.h"          // Motor function
#include "..\..\include\iic_bus_eeprom.h"
//--------------------------------------------------------------------------------------
//  Internal macro define
//--------------------------------------------------------------------------------------
#define BUF_MASK 255
#define BUF_MAX  900                   
#define BUF_MID  260                   
#define BUF_MIN  300
//--------------------------------------------------------------------------------------
//  Global variables define
//--------------------------------------------------------------------------------------
static UINT16 tra_ind_r;             // transmit buffer reading index
static UINT16 tra_ind_w;             // transmit buffer writing index
static UINT16 rec_ind_r;             // receive  buffer reading index
static UINT16 rec_ind_w;             // receive  buffer writing index
static UINT8 tra_buf[BUF_MID];       // transmit buffer  100 byte
UINT8 rec_buf[BUF_MID];       		 // receive  buffer  300 byte
UINT8 rec_status;                    // receive status

static UINT8 link_count;             // linked count
INT16 waiting_count;                 // linked count

UINT8 test_value1;                   // receive status
UINT8 test_value2;                   // receive status
UINT8 send_command[250];  			 // transmit buffer  100 byte

//--------------------------------------------------------------------------------------
// 	Internal constants definition
//--------------------------------------------------------------------------------------
#define DATA_START   			0x5A
#define DATA_END     			0xFF
#define SOFT_KEY_FOOTER         0x40
#define SOFT_KEY_FOOTER_RET     0xa0

#define SOFT_KEY_RUN          0x41
#define SOFT_KEY_RUN_RET         0xa1

#define SOFT_KEY_BOBBIN         0x42
#define SOFT_KEY_BOBBIN_RET         0xa2

#define SOFT_KEY_PAUSE         0x43
#define SOFT_KEY_PAUSE_RET         0xa3
#define RFID_WRITE 				0x44
#define RFID_WRITE_RET 			0xa4

#define RFID_READ 				0x45
#define RFID_READ_RET 			0xa5
#define NEW_PATTERN_DONE 		0x4e
#define NEW_PATTERN_DONE_RET 	0xae

#define FOOT_STATUS_FINISH      0x4d
#define FOOT_STATUS_FINISH_RET  0xad
#define READY_GO_SETOUT 		0x4f
#define READY_GO_SETOUT_RET 	0xaf
#define QUERY_IOOUTPUT    		0x60	
#define QUERY_IOOUTPUT_RET  	0xB0
#define CUT_COMMAND            	0x61
#define CUT_COMMAND_RET        	0xB1

#define FOOT_COMMAND            0x62
#define FOOT_COMMAND_RET        0xB2

#define STITCHUP				0x63	
#define STITCHUP_RET			0xB3

#define POINTSHIFT				0x64	
#define POINTSHIFT_RET			0xB4

#define LED_LIGHT_ADJUST		0x65       
#define LED_LIGHT_ADJUST_RET	0xB5

#define AUTO_SELECT_PATTERN     0x66          
#define AUTO_SELECT_PATTERN_RET 0xB6

#define CURRENTPOINTSHIFT		0x67	
#define CURRENTPOINTSHIFT_RET	0xB7

#define CHECK11_CHECK_SENSOR     0x68
#define CHECK11_CHECK_SENSOR_RET 0xb8

#define QUERY_FOOTER_HIGH		0x6A
#define QUERY_FOOTER_HIGH_RET	0xBA

#define PATTERNPOINTSHIFT		0x6B
#define PATTERNPOINTSHIFT_RET	0xBB

#define FOOTSTATEQUERY			0x6C
#define FOOTSTATEQUERY_RET		0xBC

#define IMFORMATIONQUERY		0x6d	
#define IMFORMATIONQUERY_RET	0xBd

#define ORIGIN					0x6E
#define ORIGIN_RET				0xBE

#define ADJUST_ORIGIN			0x6F
#define ADJUST_ORIGIN_RET		0xBF


#define XYSENSOR                0x70		
#define XYSENSOR_RET 			0xD0

#define SET_XY_ADJUST 			0x71  
#define SET_XY_ADJUST_RET 		0xD1

#define QUERY_IOINPUT    		0x72	
#define QUERY_IOINPUT_RET  		0xD2

#define ISENSOR      			0x73
#define ISENSOR_RET  			0xD3

#define MOTOCWW      			0x74
#define MOTOCWW_RET  			0xD4

#define STEPMOVE     			0x75
#define STEPMOVE_RET 			0xD5

#define MOTOSTA      			0x76
#define MOTOSTA_RET  			0xD6
#define COORCOM      			0x77	
#define COORCOM_RET  			0xD7

#define TEST_IOOUTPUT    		0x78	
#define TEST_IOOUTPUT_RET  		0xD8

#define CONFIG_INOUTPORT    	0x79	
#define CONFIG_INOUTPORT_RET  	0xD9

#define PREDIT_SHIFT 			0x7A
#define PREDIT_SHIFT_RET 		0xDA

#define DISPLAY_SERVOPARA 		0x7B    
#define DISPLAY_SERVOPARA_RET 	0xDB

#define MOTORMECHANICANGLECHECK 	0x7C
#define MOTORMECHANICANGLECHECK_RET 0xDC

#define MOTORMECHANICANGLEENTER 	0x7D
#define MOTORMECHANICANGLEENTER_RET 0xDD

#define CURRENTSTITCHQUERY     		0x7E
#define CURRENTSTITCHQUERY_RET  	0xDE

#define SEWSPEEDADJUST				0x7F
#define SEWSPEEDADJUST_RET			0XDF

#define CONNECT      				0x80	
#define CONNECT_RET  				0xE0

#define MISTAKE_RET  				0xA0

#define CHANGE       				0x81
#define CHANGE_RET   				0xE1

#define QUERY        				0x82
#define QUERY_RET    				0xE2

#define PARA         				0x83
#define PARA_RET     				0xE3

#define PATTERN      				0x84
#define PATTERN_RET  				0xE4

#define SHIFT        				0x85
#define SHIFT_RET    				0xE5

#define STEP         				0x86
#define STEP_RET     				0xE6

#define VERSION      				0x87
#define VERSION_RET  				0xE7


#define SERVOPARA      				0x88
#define SERVOPARA_RET  				0xE8

#define COOR         				0x89
#define COOR_RET     				0xE9

#define INMOVE       				0x8A
#define INMOVE_RET   				0xEA

#define HIGH         				0x8B
#define HIGH_RET     				0xEB

#define SMOTOR       				0x8C
#define SMOTOR_RET   				0xEC

#define SPEED        				0x8D
#define SPEED_RET    				0xED

#define INPUT        				0x8E
#define INPUT_RET    				0xEE

#define OUTPUT       				0x8F
#define OUTPUT_RET   				0xEF

#define STORE_OFFSET       			0x90
#define STORE_OFFSET_RET   			0xf0

#define QUERY_ID                    0x91
#define QUERY_ID_RET                0xF1

#define SET_MAIN_CONTROL_LOCK       0x92
#define SET_MAIN_CONTROL_LOCK_RET   0xF2

#define SWITCH_COMM_RATE            0x93
#define SWITCH_COMM_RATE_RET        0xF3

#define SET_BASELINE_ALARM          0x94
#define SET_BASELINE_ALARM_RET      0xF4


#define TEST         				0x22
#define TEST_RET     				0xB2

#define DRV_DATA            		0x95
#define DRV_DATA_RET        		0xF5
#define DRV_DATA_END            	0x96
#define DRV_DATA_END_RET        	0xF6

#define CONTROL_PARA				0x99
#define CONTROL_PARA_RET 			0xF9

#define SYS_PARAM_GROUP				0x9A
#define SYS_PARAM_GROUP_RET			0xFA

#define RESET_USERPARAM				0x9B
#define RESET_USERPARAM_RET			0xFB

#define CAL_USERPARAM_PACK			0x9C
#define CAL_USERPARAM_PACK_RET		0xFC

#define READ_USERPARAM				0x9D
#define READ_USERPARAM_RET			0xFD

#define SYS_CON_PARAM				0x9E  
#define SYS_CON_PARAM_RET			0xFE         
//--------------------------------------------------------------------------------------
//  Internal functions and subroutines declaration
//--------------------------------------------------------------------------------------
void init_uart0(void);

void init_comm(void);
#pragma INTERRUPT/E uart_tra_int 
void uart_tra_int(void);
#pragma INTERRUPT/E uart_rec_int 
void uart_rec_int(void);

UINT8 verify(UINT8* a);  
void tra_com(UINT8* command,UINT8 length);
void rec_com(void);
void protocol(UINT8* command);
void com_error(void);
void rfid_wr_ret(void);
UINT8 verify_code(UINT8 number);
//--------------------------------------------------------------------------------------
//  Name: init_uart0 routine
//  pars: None
//  Returns: None
//  Description: initial UART0 register
//--------------------------------------------------------------------------------------
void init_uart0(void)
{    
  u0mr = 0x00; // I have no idea why, but this line is required to make the UART
               // respond properly after waking up.  It shouldn't be, because
               // it is set again further down -- strange. 

  u0mr = 0x05; // 00000101 - UART_0 mode register
               // ||||||||
               // |||||+++-- 101 - UART mode, 8 bit transfer unit
               // ||||+----- 0 - Use internal clock
               // |||+------ 0 - One stop select
               // ||+------- Not used
               // |+-------- 0 - Parity disabled
               // +--------- Not used

  u0c0 = 0x10; // 00010000 - UART_0 control register 0
               // ||||||||
               // ||||||++-- 00 - use f1 count source
               // |||||+---- Not used
               // ||||+----- Read only bit
               // |||+------ 1 - CTS/RTS disabled
               // |++------- Nothing is assigned
               // +--------- 0 - transfer LSB first

  u0c1 = 0x04; // 00000100 - UART_0 control register 1
               // ||||||||
               // |||||||+-- 0 - Disabled transmission
               // ||||||+--- Transmit buffer empty flag (Read Only)
               // |||||+---- 1 - Enabled reception
               // ||||+----- Receive complete flag (Read Only)
               // ++++------ Nothing is assigned

  ucon &= ~0x00;// xxxxxxx0 - UART_0/UART_1 control register 2
                // ||||||||
                // |||||||+-- 0 - interrupt cause select: on transmit buffer empty
                // ||||||+--- Not used (applies to UART_1)
                // ||++++---- Not used 
                // |+-------- CTS/RTS shared pin
                // +--------- Nothing is assigned
  
  u0brg = BAUD_RATE ;
               // n = (fx / (16*Baud_rate) ) - 1 
               //   rearranging:
               //   Baud_rate = fx/((n+1)*16)
               //   For 57k6 Baud:
               //   let fx = f1 = 10MHz, now 
               //   n = (10e6 / (16 * 57600)) - 1 = 9.85
               //   so load 10 into u0brg to give an actual Baud 
               //   rate of 56.82k, which is close enough.

  // set ISR priorities
  s0tic = UART_TRANSMIT_IPL;  // UART0 TX: TXR transmit
  s0ric = UART_RECEIVE_IPL;   // UART0 RX: TXR receive
}

void urat0_rate_115200(void)
{ 
  INT16 i;

  rec_status = 0;//variables initial
  link_count = 0;
  waiting_count = 0; 
  
  tra_ind_r = 0; 
  tra_ind_w = 0; 
  rec_ind_r = 0; 
  rec_ind_w = 0;   
  
  for(i=0;i<BUF_MIN;i++)
  {
      tra_buf[i] = 0;  
	 
  } 
  for(i=0;i<BUF_MID;i++)
  {      
      rec_buf[i] = 0;	  
	  
  }    
  
  u0c1 = 0x00;
  u0mr = 0x00; // I have no idea why, but this line is required to make the UART
               // respond properly after waking up.  It shouldn't be, because
               // it is set again further down -- strange. 

  u0mr = 0x05; // 00000101 - UART_0 mode register
               // ||||||||
               // |||||+++-- 101 - UART mode, 8 bit transfer unit
               // ||||+----- 0 - Use internal clock
               // |||+------ 0 - One stop select
               // ||+------- Not used
               // |+-------- 0 - Parity disabled
               // +--------- Not used

  u0c0 = 0x10; // 00010000 - UART_0 control register 0
               // ||||||||
               // ||||||++-- 00 - use f1 count source
               // |||||+---- Not used
               // ||||+----- Read only bit
               // |||+------ 1 - CTS/RTS disabled
               // |++------- Nothing is assigned
               // +--------- 0 - transfer LSB first
			   
  u0brg =BAUD_RATE_115200;
 
  u0c1 = 0x04; // 00000100 - UART_0 control register 1
               // ||||||||
               // |||||||+-- 0 - Disabled transmission
               // ||||||+--- Transmit buffer empty flag (Read Only)
               // |||||+---- 1 - Enabled reception
               // ||||+----- Receive complete flag (Read Only)
               // ++++------ Nothing is assigned

  ucon &= ~0x00;// xxxxxxx0 - UART_0/UART_1 control register 2
                // ||||||||
                // |||||||+-- 0 - interrupt cause select: on transmit buffer empty
                // ||||||+--- Not used (applies to UART_1)
                // ||++++---- Not used 
                // |+-------- CTS/RTS shared pin
                // +--------- Nothing is assigned
  
  
               
  

  // set ISR priorities
  s0tic = UART_TRANSMIT_IPL;  // UART0 TX: TXR transmit
  s0ric = UART_RECEIVE_IPL;   // UART0 RX: TXR receive
  //asm("fset I"); 
}

//--------------------------------------------------------------------------------------
//  Name: init_comm routine
//  pars: None
//  Returns: None
//  Description: initial communication routine
//--------------------------------------------------------------------------------------
void init_comm(void)
{
  INT16 i;
  rec_status = 0;//variables initial
  link_count = 0;
  waiting_count = 0;
 
  tra_ind_r = 0; 
  tra_ind_w = 0; 
  rec_ind_r = 0; 
  rec_ind_w = 0;   
   
  for(i=0;i<BUF_MIN;i++)
  {
      tra_buf[i] = 0; 
  } 
  for(i=0;i<BUF_MID;i++)
  {      
      rec_buf[i] = 0;
  } 
  init_uart0(); //call initial uart0 function
  init_uart1();
  
  #if CHECK_BOARD
     OW_check();
  #endif
   
  
}
//--------------------------------------------------------------------------------------
//  Name: uart_tra_int routine
//  pars: None
//  Returns: None
//  Description: UART transmit interrupt routine
//--------------------------------------------------------------------------------------
void uart_tra_int(void)
{
  volatile UINT16 i= 300;
  if(tra_ind_r != tra_ind_w)
  {
    u0tb = tra_buf[tra_ind_r++];
  }
  else                             
  {
    te_u0c1 = 0;
    while(i--);										
    RDSET = 0;	
  }
}

//--------------------------------------------------------------------------------------
//  Name: uart_rec_int routine
//  pars: None
//  Returns: None
//  Description: UART receive interrupt routine
//--------------------------------------------------------------------------------------
void uart_rec_int(void)
{
//	rec_buf[rec_ind_w++] = (UINT8)u0rb;    
  if(rec_ind_w<299)
  	rec_buf[rec_ind_w++] = (UINT8)u0rb;
  else  
  {  
     rec_ind_w =299; 
     rec_buf[rec_ind_w] = (UINT8)u0rb;
  }
}

//-------------------------------------------------------------------------------------
//  Name:  verify
//  pars: *a
//  Returns: UINT8
//  Description: verify receive data
//--------------------------------------------------------------------------------------
UINT8 verify(UINT8* a)
{
	INT16 i;
	UINT8 result;
	
	result = 0xff;
	for(i=0;i<rec_ind_w;i++)
	{
		result ^= a[i];	
	}
	return result;
}
//--------------------------------------------------------------------------------------
//  Name: tra_com routine
//  pars: UINT8* command
//  Returns: None
//  Description: transmit command routine
//--------------------------------------------------------------------------------------
void tra_com(UINT8* command,UINT8 length)
{
	INT16 i;
	RDSET = 1;	
	if(!te_u0c1)
	{
		for(i=1;i<length;i++)
		{
			tra_buf[tra_ind_w++] = command[i];
		}
		u0tb = command[0];
		te_u0c1 = 1;
	}    
	else
	{    
		for(i=0;i<length;i++)
		{
			tra_buf[tra_ind_w++] = command[i];
		}
	}
	find_communication_start_flag = 0; 
}



//--------------------------------------------------------------------------------------
//  Name: rec_com routine
//  pars: None
//  Returns: None
//  Description: receive command routine
//--------------------------------------------------------------------------------------
void rec_com(void)
{
  UINT8 data_length;
  
	if(rec_ind_w>=2)                     // data transmitting
	{
		if( find_communication_start_flag ==1)
		{
			data_length=rec_buf[1];
			if(rec_ind_w < data_length+2)      // data has not been transmited
			{
				if(flag_1ms)
				{
					waiting_count++;
					flag_1ms = 0;
				}
				if(waiting_count >= 100)         // if wait time > 100ms 
				{
					#if COM_MONITOR_FUN
					da1 = 100;
					#endif
					waiting_count = 0;
					com_error();                   // return communication error to panel
				}
			}
			else
			{		
				if(rec_ind_w == data_length+2)   // data length is right
				{	
					if(verify(rec_buf) == 0)
					{	
						#if COM_MONITOR_FUN
						da1 = 50;
						#endif
						protocol(rec_buf);           // process penal protocol 
					}
					else
					{
						#if COM_MONITOR_FUN
						da1 = 200;
						#endif
						com_error();                 // return communication error to panel
					}
				}
				else                             // data length too much
				{
					#if COM_MONITOR_FUN
					da1 = 250;
					#endif
					com_error();                   // return communication error to panel
				}
				waiting_count = 0;	                 
			}	
		}
		else
		{
			if( rec_buf[0]!=0x5a )
			{
				tra_ind_r = 0; 
				tra_ind_w = 0; 
			    find_communication_start_flag = 0;
				rec_ind_r = 0; 
				rec_ind_w = 0; 
			}
			else
			{
				#if COM_MONITOR_FUN
				da1 = 0;
				#endif
				data_length=rec_buf[1];
				find_communication_start_flag = 1;
			}
			
		}	
	}	  
}

//--------------------------------------------------------------------------------------
//  Name: protocol routine
//  pars: UINT8* command
//  Returns: None
//  Description: process panel protocol routine
//--------------------------------------------------------------------------------------
void protocol(UINT8* command)
{	
	INT16 temp16;
	UINT32 temp32;
	UINT16 i;
	UINT16 temp,high,low,temp2;
	UINT8 temp8,index;
	UINT8 data_length;            //2012-02-08

	if(command[0] == 0x5A && command[rec_ind_w-1] == 0xFF)   // data package is correct
	{	
		switch(command[2])        // process the function code
		{    	
			//--------------------------------------------------------------------------------------      
			//  connect communication
			//--------------------------------------------------------------------------------------
			case CONNECT:       
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = CONNECT_RET;
				send_command[3] = verify_code(3);
				send_command[4] = DATA_END;                   	                         
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break; 
				
				
			case STORE_OFFSET:
				if( rec_buf[3] == 0)
				    pat_buff_write_offset = 0;
				else
				    pat_buff_write_offset = HALF_WRITE_OFFSET;
					
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = STORE_OFFSET_RET;
				send_command[3] = verify_code(3);
				send_command[4] = DATA_END;                   	                         
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
			break;
				
				
			//--------------------------------------------------------------------------------------      
			//  system status query
			//--------------------------------------------------------------------------------------              
			case QUERY:  
				tra_ind_r = 0; 
				tra_ind_w = 0; 
				send_command[0] = DATA_START;
				send_command[1] = 18;
				send_command[2] = QUERY_RET;
				send_command[3] = sys.status;
				send_command[4] = sys.error;
				temp8 = 0;
				if ( already_in_origin == 1)
				  temp8|=0x10;
				//针杆位置 0-110 250-360  1-上
				if( ((motor.angle_adjusted>=0)&&(motor.angle_adjusted <= 440))||((motor.angle_adjusted >= 1000)&&(motor.angle_adjusted <=CODE_SCALE)) )
				 temp8 |=0x08;
				if(foot_flag == 1)
				 temp8 |=0x04;
				if( inpress_type == AIR_INPRESSER )
				{
					if( inpress_flag == 1 )
					    temp8 |=0x02;
				}
				else if(inpress_position > inpress_high_base)
			        temp8 |=0x02;
				origin_flag = 0;
				if( (allx_step == 0)&&(ally_step == 0)&&(already_in_origin ==1) &&(not_in_origin_flag == 0) )
		     	  origin_flag =1;
				if(origin_flag ==1)
				  temp8 |=0x01; 
				  
				if((auto_function_flag ==1)&&(pattern_change_flag==1) )//2013-5-22
				  temp8 |=0x20;   
				  
				send_command[5] = temp8;                   
				send_command[6] = 0;       //iit报警值，请勿修改
				send_command[7] = 0;       //iit报警值，请勿修改
				send_command[8] = motor.spd_obj/100;    //主控大版本号4以上该位为面板查询主控转速，勿修改
				send_command[9] = 0;       //iit报警值，请勿修改
				send_command[10] = 0; 
				if( super_pattern_flag == 1) 
				    PatternShiftValue = pat_buff_total_counter;
				else
			    	PatternShiftValue = pat_point - (PATTERN_DATA *)(pat_buf);   
				send_command[11] = PatternShiftValue>>8;
				send_command[12] = PatternShiftValue;
							
				send_command[13] = allx_step >> 8;
				send_command[14] = allx_step;
				send_command[15] = ally_step >> 8;
				send_command[16] = ally_step;
				
				if(inpress_position >90 )
				   send_command[17] = 220; 
				else
				   send_command[17] = inpress_position;  

				send_command[18] = verify_code(18);
				send_command[19] = DATA_END;
				tra_com(send_command,20);
				rec_ind_r = 0; 
				rec_ind_w = 0;
				
				break;   
			//--------------------------------------------------------------------------------------      
			//  system status change
			//--------------------------------------------------------------------------------------              
			case CHANGE:  
				switch(command[3])
				{
					//--------------------------------------------------------------------------------------      
					//  system status change to ready
					//--------------------------------------------------------------------------------------  	
					case READY:    
						if(sys.status == ERROR)
						{
							switch(sys.error)
							{
								case 16:	     					                         
									sys.error = OK;	
									StatusChangeLatch = READY;
									predit_shift = 1;
									sys.status = ERROR; 

									origin_com = 1;
									break;
								case 22:
									aging_com = 0;
									origin_com = 0;	
									sys.error = OK;	
									StatusChangeLatch = READY;
									predit_shift = 1;
									sys.status = ERROR;
									break;
								case 81:
										if(bobbin_change_in_progress ==1 )
										   break;
								case 46:
								case 15:	     					                         
								case 45:
								case 90:
								case 91:
								case 92:
								case 17:
								case 2:
								case 19:
									sys.error = OK;
									StatusChangeLatch = READY;
									predit_shift = 1;
									sys.status = ERROR;
									break;
									
								default:
									break;
							}	     	            		                                   
						}
						else
						{	      
							switch(sys.status)
							{
								case SETOUT: 
									StatusChangeLatch = READY;
									predit_shift = 1;
									manual_com = 0;      		          			           
									origin_com = 0;
									sys.status =  SETOUT;
					
									break;
								case SLACK:
									StatusChangeLatch = READY;
									predit_shift = 1; 
									sys.status = SLACK;
						
									origin_com = 0;
									break;  
								case PREWIND:
								    StatusChangeLatch = READY;
									predit_shift = 1; 
									sys.status = PREWIND;
									break;   
								case PREEDIT:			
									StatusChangeLatch = READY;
									inpress_com = 1;
									sys.status = PREEDIT;
									predit_shift = 1;
									unaging_flag = 1;
									break;                           	            		            	                  	            		            		
								case CHECKI10:
									StatusChangeLatch = READY;
									inpress_flag =0;
									inpress_com = 1;
									inpress_first_flag = 1;
									sys.status = CHECKI10;
									predit_shift = 1;
								break;                       	            		            	                  	            		            		
								default: 	   
									break;        	
							}
						}
						break;      	            	
					//--------------------------------------------------------------------------------------      
					//  system status change to prewind
					//--------------------------------------------------------------------------------------	      	         	      	            		      	            		
					case PREWIND:  
						if(sys.status == ERROR)
						{
                           if(sys.error == ERROR_45)
						   {
						   		StatusChangeLatch = PREWIND;
								sys.error = OK;
								predit_shift = 1;
								motor.stop_angle = DEGREE_53;   // 53 degree
								motor.spd_obj = 0;                                          
								wind_com = 0;
								status_now = PREWIND;
								sys.status = ERROR; 
						   }
						}
						else
						{	
							if(sys.status == WIND)
							{
								StatusChangeLatch = PREWIND;
								predit_shift = 1;
								motor.stop_angle = DEGREE_53;   // 53 degree
								motor.spd_obj = 0;                                          
								sys.status = WIND; 
								wind_com = 0;
							}
							else
							{
								if(u205 == 1)
								{
									StatusChangeLatch = PREWIND;
									predit_shift = 1;
									sys.status = WIND; 
									wind_com = 1;
								}
								else if(u205 == 0)
								{
									
									sys.status = READY;
								} 
							}	
						}
						break;      	            	
					//--------------------------------------------------------------------------------------      
					//  system status change to setout
					//--------------------------------------------------------------------------------------
					case SETOUT: 
						if(sys.status == ERROR)
						{
                                   
						}
						else
						{ 
							if(sys.status == FINISH)
							{
								StatusChangeLatch = SETOUT;
								predit_shift = 1;
								sys.status = FINISH;
							
							}
						}                   
						break;      	            		           
					//--------------------------------------------------------------------------------------      
					//  system status change to preedit
					//--------------------------------------------------------------------------------------
					case PREEDIT: 
						if(sys.status == ERROR)
						{
							switch(sys.error)
							{
								case 15:						// out of sewing range
									StatusChangeLatch = PREEDIT;
									predit_shift = 1;
									OutOfRange_flag = 0;
									sys.error = OK;
									sys.status = ERROR;	   
									break;
								default:
									break;
							}                                
						}
						else
						{ 
							switch(sys.status)
							{
								case READY:
									if(origin_com == 0)
									{
										StatusChangeLatch = PREEDIT;
										predit_shift = 1;
										origin_com = 0; 
										sys.status = READY;
										inpress_com = 1;     
										foot_com = 0;
									}
									break;
								case SLACK:
									StatusChangeLatch = PREEDIT;
									predit_shift = 1;
									sys.status = SLACK;
									repeat_com = 1;
									foot_com = 0;
									break;
								default:
									break;
							}	
						}                 
						break;
					//--------------------------------------------------------------------------------------      
					//  system status change to finish
					//--------------------------------------------------------------------------------------
					case FINISH: 
						if(sys.status == ERROR)
						{
                                   
						}
						else
						{ 
							sys.status = FINISH;
						}                   
						break;	  
					//--------------------------------------------------------------------------------------      
					//  system status change to slack
					//--------------------------------------------------------------------------------------
					case SLACK: 
						if(sys.status == ERROR)
						{
                                   
						}
						else
						{    
							switch(sys.status)
							{  	            		           	 	
								case READY:
									predit_shift = 1;
									StatusChangeLatch = SLACK;
									sys.status = READY;
									break;
								case PREEDIT:
									predit_shift = 1;
									StatusChangeLatch = SLACK;
									sys.status = PREEDIT;
									break;
								case CHECKI10:
									inpress_first_flag = 1;	 
									inpress_action_flag =1;  
								case CHECKI11:	
								case CHECKI03:
								case CHECKI04:
								case CHECKI05:
								case CHECKI06:
								case CHECKI07:
								case PREWIND:
									predit_shift = 1;
									StatusChangeLatch = SLACK;
									FW = 0;
									L_AIR = 0;
									da0 = 0;
									FA = 0;
									
									break;
								case CHECKI08:
									back_from_checki08 = 1;
									predit_shift = 1;
									StatusChangeLatch = SLACK;
									FW = 0;
									L_AIR = 0;
									da0 = 0;
									FA = 0;
								break;
								default:
									break;
							}
						}                   
						break;		 
					//--------------------------------------------------------------------------------------      
					//  system status change to checki03
					//--------------------------------------------------------------------------------------
					case CHECKI03: 
						if(sys.status == ERROR)
						{
                                   
						}
						else
						{    
							switch(sys.status)
							{  
								case SLACK:
									predit_shift = 1;
									StatusChangeLatch = CHECKI03;
									sys.status = SLACK;
									break;
								default:
									break;
							}
						}                   
						break;		
					//--------------------------------------------------------------------------------------      
					//  system status change to checki04
					//--------------------------------------------------------------------------------------
					case CHECKI04: 
						if(sys.status == ERROR)
						{
                                   
						}
						else
						{    
							switch(sys.status)
							{  
								case SLACK:
									predit_shift = 1;
									StatusChangeLatch = CHECKI04;
									//origin_com = 1;            	                	            		           	  
									smotor_speed = 0; 
									sys.status = SLACK;
									break;
								default:
									break;
							}  
						}                   
						break;		              
					//--------------------------------------------------------------------------------------      
					//  system status change to checki05
					//--------------------------------------------------------------------------------------
					case CHECKI05: 
						if(sys.status == ERROR)
						{
                                   
						}
						else
						{       	
							switch(sys.status)
							{  
								case SLACK:
									predit_shift = 1;
									StatusChangeLatch = CHECKI05;
									output_com = 0; 
									sys.status = SLACK;
									break;
								default:
									break;
							}
						}                   
						break;		 
					//--------------------------------------------------------------------------------------      
					//  system status change to checki06
					//--------------------------------------------------------------------------------------
					case CHECKI06: 
						if(sys.status == ERROR)
						{
                                   
						}
						else
						{   
							switch(sys.status)
							{  
								case SLACK:
									predit_shift = 1;
									StatusChangeLatch = CHECKI06;
									origin_com = 0;            // feed move 	 
									foot_com = 0 ;      	            		           	 	            		             
									shift_flag = 0x00; 
									shift_reply = 0x00; 
									sys.status = SLACK;
									break;
								default:
									break;
							}
						}                   
						break;		              
					//--------------------------------------------------------------------------------------      
					//  system status change to checki07
					//--------------------------------------------------------------------------------------
					case CHECKI07: 
						if(sys.status == ERROR)
						{
                                   
						}
						else
						{ 
							switch(sys.status)
							{  
								case SLACK:
									predit_shift = 1;
									StatusChangeLatch = CHECKI07;
									origin_com = 0;            // feed move 
									stepmotor_comm = 0xff; 
									stepmotor_state = 0xff; 
									sys.status = SLACK;
									break;
								default:
									break;
							} 
						}                   
						break;
					//--------------------------------------------------------------------------------------      
					//  system status change to checki08
					//--------------------------------------------------------------------------------------
					case CHECKI08: 
						if(sys.status == ERROR)
						{
                                   
						}
						else
						{ 
							switch(sys.status)
							{  
								case SLACK:
									predit_shift = 1;
									StatusChangeLatch = CHECKI08;
									origin_com = 0;            // feed move 
									stepmotor_comm = 0xff; 
									stepmotor_state = 0xff; 
									sys.status = SLACK;
									break;
								default:
									break;
							} 
						}                   
						break;		           		
					//--------------------------------------------------------------------------------------      
					//  system status change to checki10
					//--------------------------------------------------------------------------------------
					case CHECKI10: 
						if(sys.status == ERROR)
						{
                                   
						}
						else
						{   
							switch(sys.status)
							{  
								case SLACK:
								case READY:
									predit_shift = 1;
									StatusChangeLatch = CHECKI10;
									origin_com = 0;             
									stepmotor_comm = 0xff; 
									stepmotor_state = 0xff; 
									stepmotor_single = 0xff;   
									sys.status = SLACK;
									inpress_high =inpress_high_base;
									inpress_action_flag =1;        
									break;
								default:
									break;
							}
						}                   
						break;	
					case CHECKI11: 
						if(sys.status == ERROR)
						{
                                   
						}
						else
						{   
							switch(sys.status)
							{  
								case SLACK:
								case READY:
									predit_shift = 1;
									StatusChangeLatch = CHECKI11;
									origin_com = 0;             
									stepmotor_comm = 0xff; 
									stepmotor_state = 0xff; 
									stepmotor_single = 0xff;   
									sys.status = SLACK;
									break;
								default:
									break;
							}
						}                   
						break;		
					//--------------------------------------------------------------------------------------      
					//  system status change to boardtest
					//--------------------------------------------------------------------------------------	            
		           	                                                     	            	                            	           
					case DOWNLOAD:
						sys.status = DOWNLOAD;  
						StatusChangeLatch = DOWNLOAD; 
					break;			           	                                                     	            	                            	           
					case DOWNLOAD_DRV1:
					    sys.status = DOWNLOAD_DRV1;
						StatusChangeLatch = DOWNLOAD_DRV1;
						break;
					case DOWNLOAD_DRV2:
					    sys.status = DOWNLOAD_DRV2;
						StatusChangeLatch = DOWNLOAD_DRV2;
					    break;	           	                                                     	            	                            	           
					case CONTINUE:
					    sys.status = CONTINUE;
						StatusChangeLatch = CONTINUE;
					break;		           	                                                     	            	                            	           
					//--------------------------------------------------------------------------------------      
					//  system status change to com_error
					//--------------------------------------------------------------------------------------			      	               	            		                					                      
					default:  
						com_error();       
						break;			
				}
				tra_ind_r = 0; 
				tra_ind_w = 0; 
				send_command[0] = DATA_START;
				send_command[1] = 0x05;                 
				send_command[2] = CHANGE_RET;
				send_command[3] = sys.status;
				send_command[4] = sys.error;
				send_command[5] = verify_code(5);
				send_command[6] = DATA_END;                   	                         
				tra_com(send_command,7);    
				rec_ind_r = 0; 
				rec_ind_w = 0;                                        
				break;  
			//--------------------------------------------------------------------------------------      
			//  receive parameter
			//--------------------------------------------------------------------------------------
			case PARA:    //  parameter                    
			    data_length = rec_buf[1];                           //                  
				MotorSpeedRigister = rec_buf[3];             		// sewing speed 0-9
				temp = (UINT16)rec_buf[4] << 8;                
				sew_stitch = temp | (UINT16)rec_buf[5];        		// sewing stitch counter   
				u10 = rec_buf[6];                               	// 1 speed no catch                        
				u11 = rec_buf[7];                               	// 2 speed no catch                        
				u12 = rec_buf[8];                               	// 3 speed no catch                        
				u13 = rec_buf[9];                               	// 4 speed no catch                        
				u14 = rec_buf[10];                              	// 5 speed no catch
				u201 = rec_buf[11];									// go original when enenergy
				u202 = rec_buf[12];									// sewing when pedal is up
				u203 = rec_buf[13];									// go original forbidden when pedal is up
				u204 = rec_buf[14];									// go original forbidden when taken-up is down
				u37 = rec_buf[15];                               	// state of the presser after end of sewing
				u38 = rec_buf[16];                               	// presser lifting motion at end of sewing
				u39 = rec_buf[17];									// check origin when sewing end(normal)
				u40 = rec_buf[17];									// check origin when sewing end(c pattern)
				u48 = rec_buf[18];									// setting the find origin style
				u49 = rec_buf[19];  		                      	// wind speed 
				u205 = rec_buf[20];									// wind switch   
				u206 = rec_buf[21];									// wiper switch  
				u207 = rec_buf[22];									// taken-up position when pause buttern is on
				u208 = rec_buf[23];									// pedal action when pause buttern is on
				u209 = rec_buf[24];									// puase switch style
				if( u209 == 0)//
				{
					PAUSE_OFF = 1;
					PAUSE_ON = 0;
				}
				else
				{
					PAUSE_OFF = 0;
					PAUSE_ON = 1;
				}
				u97 = rec_buf[25];                               	// emergency and thread trimming operation
				u210 = rec_buf[26];									// cut switch
				u211 = rec_buf[27];									// cut speed
				if( u211 <= 10)		
				    u211 = 10;
				u42 = rec_buf[28];                               	// up position or upper dead point
				u71 = rec_buf[29];  			                    // thread breakage detection select 
				u72 = rec_buf[30];                               	// number of invalid stitches at start of sewing of thread breakage detection              
				u73 = rec_buf[31];                               	// number of invalid stitches during sewing of thread breakage detection  
				u94 = rec_buf[32];								 	// check dead point when go origin or reset
				u104 = rec_buf[33];                              	// inpresser lowering timing
				u212 = rec_buf[34];									// cancel sewing sera potection 
//				u213 = rec_buf[35];									// x left limit 
//				u214 = rec_buf[36];									// x right limit 
//				u215 = rec_buf[37];									// y up limit
//				u216 = rec_buf[38];									// y down limit
				u217 = rec_buf[39];									// set the high speed
				u218 = rec_buf[40];									// set the low speed
				u219 = rec_buf[41];									// set the mid-high speed
				u220 = rec_buf[42];									// set the mid-low speed
			
				SpeedRange[0] = u218*100;
				SpeedRange[9] = u217*100;
				for ( i=1;i<9;i++)
				{
					SpeedRange[i] = SpeedRange[0]+(SpeedRange[9] - SpeedRange[0])/9*i;
					if(SpeedRange[i] > SpeedRange[9] )
					  SpeedRange[i] = SpeedRange[9];
					if(SpeedRange[i] < SpeedRange[0] )
					  SpeedRange[i] = SpeedRange[0];
					SpeedRange[i] = (SpeedRange[i]+ 50)/100 * 100;
				}
								
				u221 = rec_buf[43];									// single pedal
				   pedal_style =2;
				u222 = rec_buf[44];									// trim on delay
				
				  
				temp = (UINT16)rec_buf[45]<<8;            			// cut start angle
				cut_start_angle = temp | (UINT16)rec_buf[46];
				cut_start_angle = cut_start_angle<<2;
				cut_start_angle = cut_start_angle%CODE_SCALE;
				temp = (UINT16)rec_buf[47]<<8;						// cut end angle
				cut_end_angle = temp | (UINT16)rec_buf[48];
				cut_end_angle = cut_end_angle <<2;
				cut_end_angle = cut_end_angle%CODE_SCALE;
				u223 = rec_buf[49];									// tension start delay
				if( u223 == 0)
				  u223 = 50;
				else
				  u223 = u223 <<1;
				
				temp = (UINT16)rec_buf[50]<<8;						// tension start angle
				tension_start_angle = temp | (UINT16)rec_buf[51];   // 
				tension_start_angle = tension_start_angle*4;
				tension_start_angle = tension_start_angle%CODE_SCALE;
				
				temp = (UINT16)rec_buf[52]<<8;						// tension end angle
				tension_end_angle = temp | (UINT16)rec_buf[53];
				tension_end_angle = tension_end_angle%CODE_SCALE;
				tension_end_angle = tension_end_angle <<2;
				temp = (UINT16)rec_buf[54]<<8;						// wiper start time
				wiper_start_time = temp | (UINT16)rec_buf[55];
				temp = (UINT16)rec_buf[56]<<8;						// wiper hold time
				wiper_end_time = temp | (UINT16)rec_buf[57];
				//u224 = rec_buf[58];									// presser style									
				//u225 = rec_buf[59];									// presser wright
				tail_sewing_speed = u211;//rec_buf[59];	
				//if( tail_sewing_speed >=11)
				//    tail_sewing_speed = 11;
				//u226 = rec_buf[60];									// light presser
				//u227 = rec_buf[61];									// mid presser
				//u228 = rec_buf[62];									// heavy presser
				u229 = rec_buf[63];									// sewing material style
				u230 = rec_buf[64];									// thin material
				
				slowdown_stitchs = 5;				
				
				u231 = rec_buf[65];									// mid material
				u232 = rec_buf[66];									// thick material
				

				/*if(u229 >= 1)
				{
					if(u231<7)
				      u231 = 7;
					if(u231>14)
					  u231 =14;
				    if(u232<15)
				      u232 = 15;
					if(u232>20)
					  u232 = 20;
				}*/
				aging_flag = rec_buf[67];                        	// aging flag 
				aging_delay = rec_buf[68];                       	// aging delay time
				aging_com = 0;
				u233 = rec_buf[69];									// check origin style when aging
				u234 = rec_buf[70];									// pedal action times when aging
				temp16 = (INT16)rec_buf[71]<<8;
				x_bios = temp16 | (INT16)rec_buf[72];               //adjust for origin point
				temp16 = (INT16)rec_buf[73]<<8;
				y_bios = temp16 | (INT16)rec_buf[74];
				u235 = rec_buf[75];   								// inpresser current setting
				u236 = rec_buf[76]; 								// stop angle setting 

				temp16 = (INT16)rec_buf[77]<<8;
				AdjustAngleSet = temp16 | (INT16)rec_buf[78];  
				if(AdjustAngleSet > CODE_SCALE || AdjustAngleSet < 0)
				{
					AdjustAngleSet = 0;
				}
				else
				{
					AdjustAngle = AdjustAngleSet;  
				}     				
				temp16 = (INT16)rec_buf[80]<<8;						// rotate device control
				u240 = temp16|(INT16)rec_buf[81];
				temp16 = (INT16)rec_buf[82]<<8;	 								
				u241 = temp16|(INT16)rec_buf[83]; 					// Y axis minimum diatance,mm
				temp16 = (INT16)rec_buf[84]<<8;            					
				u238 = temp16|(INT16)rec_buf[85]; 					// origin check with rotate device
				special_footer_type = 0;
				if( u238 ==4 )
				{
					special_footer_type =2;
					stretch_foot_enable = 0;
					u238 = 0;
				}
				else if(u238 ==3)
				{
					special_footer_type =1;
					stretch_foot_enable = 0;
					u238 = 1;
				}
				else if(u238 ==2)
				   	stretch_foot_enable =1;
				else
				    stretch_foot_enable =0;
			
			
				
				auto_function_skip_flag = rec_buf[86]; //模板解锁标致：1是模板识别 0是手动切换  
				if( auto_function_skip_flag==0)
					last_pattern_number =0;
								
				u239 = 0;
				temp16 = (INT16)rec_buf[88]<<8;					
				u242 = temp16|(INT16)rec_buf[89];            
				u243 = rec_buf[90];	  
				wind_mode = rec_buf[91];
				
				footer_working_mode = rec_buf[92];
			    LRfooter_down_mode = rec_buf[93];
				LRfooter_up_mode = rec_buf[94];
				origin_footer_status = rec_buf[95];
				LTR_trim_option = rec_buf[96];//断线检测时是否剪线

				temp16 = (INT16)rec_buf[97]<<8;	
				nop_move_delay = temp16|(INT16)rec_buf[98];	
				
				x_step_current_level = rec_buf[103];
				if(x_step_current_level>31)
				   x_step_current_level = 31;     
				                     
				y_step_current_level = rec_buf[104];
				if(y_step_current_level>31)
				   y_step_current_level = 31;
				   
				for( i=0;i<10;i++)				
				     motor_para[i] = rec_buf[105+i] ;	
			     x_sensor_pos = rec_buf[115];
				 stop_foot_status = rec_buf[116]; //0-down 1-up
				
				cut_mode = rec_buf[117];			//0-电磁铁  1-气阀  2-步进
				delay_of_inpress_up = rec_buf[118];
				delay_of_wipper_down = rec_buf[119];
				delay_of_nop_move = rec_buf[120];
				delay_of_go_setout = rec_buf[121];
				dead_point_degree = rec_buf[122];
				dead_point_degree = dead_point_degree*4;
				dead_point_degree = dead_point_degree%CODE_SCALE;
				MoveMode = rec_buf[123];
								
				HighSpeedStitchLength = rec_buf[124];
				MAIN_MOTOR_TYPE = rec_buf[125];
			    MAIN_MOTOR_TYPE = 2;
				
				MotorPreInit();
				StitchSpeedCurve = rec_buf[126];				
				MoveStartAngle = rec_buf[127];				
				temp = (UINT16)rec_buf[128]<<8;            			
				iit_warning_value = temp | (UINT16)rec_buf[129];  
				
				inpress_origin = rec_buf[130];
				inpress_high_base = rec_buf[131];
				last_inpress_position = inpress_high_base;
				inpress_down_delay = rec_buf[132];
				go_origin_speed = rec_buf[133];
				if(go_origin_speed>9)
					go_origin_speed =9;
				single_move_speed = rec_buf[134];
				if(data_length > 135 )
				{
					one_step_delay = rec_buf[135];			
					if(data_length > 136 )
					{
						speed_display_mode = rec_buf[136];
				
						u224 = rec_buf[137];
						
						steper_footer_range = rec_buf[140];
						steper_footer_range = steper_footer_range*14/10;
						
						temp16 = (INT16)rec_buf[142]<<8;
						angle_test_x = temp16 | (INT16)rec_buf[143];
						
						temp16 = (INT16)rec_buf[144]<<8;//2013-8-17
						angle_test_y = temp16 | (INT16)rec_buf[145];
					}
					
				}
				//147-154
				temp16 = (INT16)rec_buf[147]<<8;
				u213 = temp16|(INT16)rec_buf[148];
				temp16 = (INT16)rec_buf[149]<<8;
				u214 = temp16|(INT16)rec_buf[150];
				temp16 = (INT16)rec_buf[151]<<8;
				u215 = temp16|(INT16)rec_buf[152];
				temp16 = (INT16)rec_buf[153]<<8;
				u216 = temp16|(INT16)rec_buf[154];
				u237 = 0; 
				
				if(data_length > 155 )
				{
				   stretch_out_delay = rec_buf[155];
				   stretch_up_delay = rec_buf[156];
				   stretch_down_delay = rec_buf[157];
				   inpress_type = rec_buf[158];		//0-气阀 1-步进 2-电磁铁 3-随动
				   if( inpress_type ==  AIR_INPRESSER)
				   {
					   inpress_high_base = 0;
					   last_inpress_position = 0;
				   }
				   led_light_adjust = rec_buf[170];   
				   temp = led_light_adjust;      
				   #if COM_MONITOR_FUN ==0
				   if(temp == 0)
					{  
					    da1 = 0;
						LED_POWER = 0;
					}
				   if( temp >0 && temp <= 100)//default :50  140~180
					{
					    da1 = 140 + temp*10/33;
						LED_POWER = 1;
					} 
				   #endif	
				}
				if(data_length > 171 )
				{
					x_motor_dir = rec_buf[171];
					y_motor_dir = rec_buf[172];
					if( y_motor_dir ==1)
					    y_motor_dir = 0;
					else
					    y_motor_dir = 1;
					z_motor_dir = rec_buf[173];
				    before_nopmove_cut_flag = rec_buf[174];
				    finish_cut_flag = rec_buf[175];
				    aging_mode = rec_buf[177];// 0-踏板 1-原点键
				}
				if(data_length > 177 )
				{
					tension_release_time = rec_buf[180];
					tension_release_value = rec_buf[181];
					if( k03 == ELECTRICAL)
					{
					  if( tension_release_time < 5 )
					      tension_release_time = 5;
					}
					//if( tension_release_value > 100)
					//    tension_release_value = 100;
				    if( tension_release_time > 60)
					     tension_release_time = 60; 
					tension_release_time = tension_release_time*1000;
				}
				if(data_length > 181 )
				{
					inpress_speed = rec_buf[182];
					inpress_speed = inpress_speed <<1 ;
					if(inpress_speed >31)
					 inpress_speed =31;
				}
				if(data_length > 182 )
				{
					inpress_follow_flag = rec_buf[183];
					special_machine_type = rec_buf[184];
				}
				if(data_length > 184 )
				{
					auto_select_flag = rec_buf[185];//				
				}
				if(data_length > 187 )
				{
					temp16 = (UINT16)rec_buf[186]<<8;
					temp16 = temp16|(UINT16)rec_buf[187];
					x_bios_offset = (INT16)temp16;//画笔X偏移
					x_bios_offset = x_bios_offset * RESOLUTION;
		            temp16 = (UINT16)rec_buf[188]<<8;
					temp16 = temp16|(UINT16)rec_buf[189];
					y_bios_offset = (INT16)temp16;
					y_bios_offset = y_bios_offset * RESOLUTION;
					marking_speed = rec_buf[190];
					if(marking_speed<3)
					marking_speed =3;
				}
				
				if(data_length >190)
				  {
					  temp16 = (UINT16)rec_buf[191]<<8;
					  cool_air_close_time = temp16|(UINT16)rec_buf[192];
					  temp16 = (UINT16)rec_buf[193]<<8;
					  cool_air_open_time = temp16|(UINT16)rec_buf[194];
				
					  mode0_time = (INT8)rec_buf[199];
					  temp16 = (UINT16)rec_buf[200]<<8;
					  temp16 = temp16|(UINT16)rec_buf[201];
					  mode0_angle = (INT16)temp16;
					  
					  mode1_x_time = (INT8)rec_buf[202];
					  temp16 = (UINT16)rec_buf[203]<<8;
					  temp16 = temp16|(UINT16)rec_buf[204];
					  mode1_x_angle = (INT16)temp16;
					  
					  mode1_y_time = (INT8)rec_buf[205];
					  temp16 = (UINT16)rec_buf[206]<<8;
					  temp16 = temp16|(UINT16)rec_buf[207];
					  mode1_y_angle = (INT16)temp16;
				 }
				
			   if(data_length > 217 )
				{
					needle_cool_flag = rec_buf[216];  //0-off  1-剪线后冷却，缝制中不冷却  2--剪线和缝制中都冷却
					temp16 = (UINT16)rec_buf[217]<<8;
					temp16 = temp16|(UINT16)rec_buf[218];
					x_origin_offset = -(INT16)temp16;          //phical origin point
		            temp16 = (UINT16)rec_buf[219]<<8;
					temp16 = temp16|(UINT16)rec_buf[220];
					y_origin_offset = -(INT16)temp16;
				}
				if( data_length >220)
				{
				   	formwork_identify_device =  rec_buf[221];
			    }
				  
				if(data_length>226)
				{
					after_trim_stop_angle_adjust = rec_buf[222];
					
					if(after_trim_stop_angle_adjust>u236)
					   after_trim_stop_angle_adjust = u236;
					   
					   
					temp = (UINT16)rec_buf[223]<<8;						// 
				    start_angle_speed_curve8 = temp | (UINT16)rec_buf[224];
				    start_angle_speed_curve8 = start_angle_speed_curve8%CODE_SCALE;
					
					temp = (UINT16)rec_buf[225]<<8;						// 
				    end_angle_speed_curve8 = temp | (UINT16)rec_buf[226];
				    end_angle_speed_curve8 = end_angle_speed_curve8%CODE_SCALE;
					
					delay_start_function = rec_buf[227];					
					delay_start_time     = rec_buf[228];
								
					thread_holding_switch = rec_buf[229];
					thread_holding_current = rec_buf[230];
					if( thread_holding_current == 16)
					    thread_holding_current = 15;
					thread_holding_current = thread_holding_current<<4;
				 }
				
				if( data_length >=232)
				    super_pattern_flag = rec_buf[231];
				if(data_length > 233)
				{
					sewingcontrol_flag = rec_buf[232];      //0-不加固 1--第一针加固 2-前几针加固
					sewingcontrol_stitchs = (INT8)rec_buf[233];
					sewingcontrol_tail_flag = rec_buf[234];  //0-不加固 1-最后1针加固 2-后几针加固
				}
					
				if( data_length >=235)
				{
				    x_step_curve = rec_buf[235];
					y_step_curve = rec_buf[236];
					
					temp = (UINT16)rec_buf[160]<<8;	//237,238					
				    thread_holding_start_angle = temp | (UINT16)rec_buf[161];
				    thread_holding_start_angle = thread_holding_start_angle <<2;
					
					temp = (UINT16)rec_buf[162]<<8;//239,240						
				    thread_holding_end_angle = temp | (UINT16)rec_buf[163];
				    thread_holding_end_angle = thread_holding_end_angle <<2;
					
					temp = (UINT16)rec_buf[164]<<8;//241,242						
				    thread_holding_cut_angle = temp | (UINT16)rec_buf[165];
				    thread_holding_cut_angle = thread_holding_cut_angle <<2;
					
				}
				if( data_length >=244)
				{
					k03 = rec_buf[245];
					base_tension = tension_release_value;
					sewing_tenion = base_tension;

				}
				
				if(auto_select_flag ==1)
				{
					auto_function_flag = 1;
				}
				else
				    auto_function_flag = 0;
	   
			    temp = (UINT16)rec_buf[137]<<8;						
				CoolingInterval = temp | (UINT16)rec_buf[138];
				CoolingDuration =  rec_buf[139];
		
				AgingLasting =  rec_buf[140];	//连续运转里，连续运行时间
				baseline_detect_switch = rec_buf[141];
				aging_selection =  rec_buf[166];//老化类型 0-整机 1-主轴
				sewingcontrol_tail_stitches = rec_buf[167];//结束加固针数
				
				bobbin_change_switch = rec_buf[176];//自动换梭功能开关
				
				
				yj_step_current_level = rec_buf[58];
				stepper_cutter_move_range = rec_buf[59];
				stepper_cutter_move_time = 25 + (INT8)rec_buf[60];
				if( stepper_cutter_move_time < 10)
					stepper_cutter_move_time = 10;
					
				temp = (UINT16)rec_buf[61]<<8;					
				holding_bobbin_start_angle = temp | (UINT16)rec_buf[62];
				holding_bobbin_start_angle = holding_bobbin_start_angle << 2;
				
				inpress_follow_range = rec_buf[99];
				if( inpress_follow_range < 10)
					inpress_follow_range = 10;
				inpress_follow_range =  inpress_follow_range*7/10;
				
				follow_up_inpresser_time_adj = (INT8)rec_buf[100];				
				temp = (UINT16)rec_buf[35]<<8;					
				fw_start_angle = temp | (UINT16)rec_buf[36];
				fw_start_angle = fw_start_angle << 2;	
				
				temp16 = (UINT16)rec_buf[37]<<8;
				temp16 = temp16|(UINT16)rec_buf[38];
				follow_up_inpresser_angle_adj = (INT16)temp16;
	
				 			
				inpress_lower_stitchs = rec_buf[101];//起针降低针数
				inpress_lower_steps = rec_buf[102];//起针降低步数
				if(inpress_lower_steps > inpress_high_base)
				{
					inpress_lower_steps = inpress_high_base;
				}
//				inpress_lower_steps =  inpress_lower_steps*7/10;				
				if(connect_flag == 0)  
				{
				 	connect_flag = 1;
				}
				
				
				//--------------------------------------------------------------------------------------      
				// compart symbol  
				//--------------------------------------------------------------------------------------	                 		                    
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = PARA_RET;
				send_command[3] = verify_code(3);
				send_command[4] = DATA_END;  	                
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				
				break;                
			//--------------------------------------------------------------------------------------      
			//  receive pattern data
			//--------------------------------------------------------------------------------------
			case PATTERN:
				recpat_point = (UINT8 *)&(rec_buf+4);   // data address
				if(rec_buf[3] > 100)   
				{
					com_error();
				
				}
				else
				{
				temp = 250*(rec_buf[3]-1);
				
				 if( super_pattern_flag != 1)
				{  
					
					for(i=0;i<rec_buf[1]-4;i++)
					{
						pat_buf[temp+i] = *recpat_point;
						recpat_point++;
					}			
					
				
				}
				else
				{      
					for(i=0;i<rec_buf[1]-4;i++)
					{
						pat_buf[pat_buff_write_offset+temp+i] = *recpat_point;
						recpat_point++; 						
					
					} 
				}
				}
				LastPatternHaveSOP = 0;
				sta_point = (PATTERN_DATA *)(pat_buf);				
			
				temp = pattern_number;
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x05;
				send_command[2] = PATTERN_RET;
				send_command[3] = (UINT8)(temp>>8);
				send_command[4] = (UINT8)temp; 
				
				send_command[5] = verify_code(5);
				send_command[6] = DATA_END;  	                
				tra_com(send_command,7);    
				rec_ind_r = 0; 
				rec_ind_w = 0;

				break; 
			//--------------------------------------------------------------------------------------      
			//  receive single step command
			//--------------------------------------------------------------------------------------
			case STEP:

				if( (sys.status != ERROR)&&(sys.status != RUN)&&(finish_nopmove_pause_flag != 1)&&(waitting_for_point_command==0) )
				{
					switch(rec_buf[3])                      
					{
						case 0x01: 
						    predit_shift = 1; 
							single_next();     //1
							break;
						case 0x81:
						    predit_shift = 1; 
							single_back();     //2
							break;    		            							
						case 0x31: 
							predit_shift = 1; 
							back_startpoint(); //3
							break;    		            								
						case 0x02: 
						    predit_shift = 1; 
							single_end();      //6
							break;    		            								
						case 0x82: 
						    predit_shift = 1;
							single_start();    //7
							break;    		            								
						case 0x52: 
							single_stop();     //8
							break;    		            											
						default:   
							com_error();      
							break;	
					}           
				}
				else
				{
					single_reply = 0x51;
					single_flag = 0;
					predit_shift = 0;
				}
				tra_ind_r = 0; 
				tra_ind_w = 0;                          		            
				send_command[0] = DATA_START;
				send_command[1] = 0x04;
				send_command[2] = STEP_RET;
				send_command[3] = single_reply;
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END;  	                
				tra_com(send_command,6);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;     
			//--------------------------------------------------------------------------------------      
			//  receive manual shift step command
			//--------------------------------------------------------------------------------------
			case SHIFT:          		             		                		            
				switch(rec_buf[3])                      // data package
				{
					case 0x0C: 
						shift_flag = 0x0C; 
						
						shift_reply = 0x0C; 
						break;
					case 0x01: 
						shift_flag = 0x01; 
						shift_reply = 0x01; 
						break;    		            							
					case 0x03: 
						shift_flag = 0x03; 
						shift_reply = 0x03; 
						break;  
					case 0x04: 
						shift_flag = 0x04; 
						shift_reply = 0x04; 
						break;
					case 0x06: 
						shift_flag = 0x06; 
						shift_reply = 0x06; 
						break;    		            							
					case 0x07: 
						shift_flag = 0x07; 
						shift_reply = 0x07; 
						break;
					case 0x09: 
						shift_flag = 0x09; 
						shift_reply = 0x09; 
						break;
					case 0x0A: 
						shift_flag = 0x0A; 
						shift_reply = 0x0A; 
						break;    		            									  		            								
					case 0xEC: 
						shift_flag = 0xEC; 
						shift_reply = 0xEC; 
						break;
					case 0xE1: 
						shift_flag = 0xE1; 
						shift_reply = 0xE1; 
						break;    		            							
					case 0xE3: 
						shift_flag = 0xE3; 
						shift_reply = 0xE3; 
						break;  
					case 0xE4: 
						shift_flag = 0xE4; 
						shift_reply = 0xE4; 
						break;
					case 0xE6: 
						shift_flag = 0xE6; 
						shift_reply = 0xE6; 
						break;    		            							
					case 0xE7: 
						shift_flag = 0xE7; 
						shift_reply = 0xE7; 
						break;
					case 0xE9: 
						shift_flag = 0xE9; 
						shift_reply = 0xE9; 
						break;
					case 0xEA: 
						shift_flag = 0xEA; 
						shift_reply = 0xEA; 
						break;  	
					case 0x88: 
						shift_flag_old = shift_flag; 
						shift_flag = 0x88; 
						shift_reply = 0x88; 
						break;     		            		 	
					case 0x5C: 
						shift_flag = 0x5C; 
						shift_reply = 0x5C; 
						break;
					case 0x51: 
						shift_flag = 0x51; 
						shift_reply = 0x51; 
						break;    		            							
					case 0x53: 
						shift_flag = 0x53; 
						shift_reply = 0x53; 
						break;  
					case 0x54: 
						shift_flag = 0x54; 
						shift_reply = 0x54; 
						break;
					case 0x56: 
						shift_flag = 0x56; 
						shift_reply = 0x56; 
						break;    		            							
					case 0x57: 
						shift_flag = 0x57; 
						shift_reply = 0x57; 
						break;
					case 0x59: 
						shift_flag = 0x59; 
						shift_reply = 0x59; 
						break;
					case 0x5A: 
						shift_flag = 0x5A; 
						shift_reply = 0x5A; 
						break;  	    		            		    		            		
					default:   
						break;
				}	                                     		                    
				tra_ind_r = 0; 
				tra_ind_w = 0;                          		            
				send_command[0] = DATA_START;
				send_command[1] = 0x04;
				send_command[2] = SHIFT_RET;
				send_command[3] = shift_reply;
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END;  	                
				tra_com(send_command,6);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				predit_shift = 1 ;//2012-3-1
				break;  
			//--------------------------------------------------------------------------------------      
			//  receive query version information command
			//--------------------------------------------------------------------------------------
			case VERSION:  
				tra_ind_r = 0; 
				tra_ind_w = 0;                          		            
				send_command[0] = DATA_START;
				send_command[1] = 23;
				send_command[2] = VERSION_RET;

				send_command[3] = Step1Version.MachineType;
				send_command[4] = Step1Version.FatherVersion;
				send_command[5] = Step1Version.ChildVersion;
				send_command[6] = stepversion1>>8;
				send_command[7] = stepversion1;

				send_command[8] = Step2Version.MachineType;
				send_command[9] = Step2Version.FatherVersion;
				send_command[10] = Step2Version.ChildVersion;
				send_command[11] = stepversion2>>8;
				send_command[12] = stepversion2;

				send_command[13] = MACHINE_TYPE;
				if( para.platform_type == FOURTH_GENERATION )//MainFatherVersion
					send_command[14] = MainFatherVersion;
				else
					send_command[14] = MainFatherVersion + 1;
					
				send_command[15] = MainChildVersion;
				send_command[16] = MainSVNVersion>>8;
				send_command[17] = MainSVNVersion;

				send_command[18] = MACHINE_TYPE;
				send_command[19] = MotorFatherVersion;
				send_command[20] = MotorChildVersion;
				send_command[21] = MotorSVNVersion>>8;
				send_command[22] = MotorSVNVersion;

				send_command[23] = verify_code(23);
				send_command[24] = DATA_END;  	                
				tra_com(send_command,25);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;     
			//--------------------------------------------------------------------------------------      
			//  receive query machine type information command
			//--------------------------------------------------------------------------------------
		/*	case MACHINE:          		             		                		                                               		                    
				tra_ind_r = 0; 
				tra_ind_w = 0;                          		            
				send_command[0] = DATA_START;
				send_command[1] = 0x07;
				send_command[2] = MACHINE_RET;
				temp = MACHINE_TYPE;
				send_command[3] = (UINT8)(temp >> 8);
				send_command[4] = (UINT8)temp;
				temp = DAHAO_TYPE;
				send_command[5] = (UINT8)(temp >> 8);
				send_command[6] = (UINT8)temp;  			  				          	          
				send_command[7] = verify_code(7);
				send_command[8] = DATA_END;  	                
				tra_com(send_command,9);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break; 
			*/
			//--------------------------------------------------------------------------------------      
			//  receive query x and y coordinate command
			//--------------------------------------------------------------------------------------
			case COOR:          		             		                		                                               		                    
				tra_ind_r = 0; 
				tra_ind_w = 0;                          		            
				send_command[0] = DATA_START;
				send_command[1] = 0x07;
				send_command[2] = COOR_RET;

				temp16 = allx_step;

				send_command[3] = (UINT8)(temp16 >> 8);
				send_command[4] = (UINT8)temp16;

				temp16 = ally_step;

				send_command[5] = (UINT8)(temp16 >> 8);
				send_command[6] = (UINT8)temp16;  			  				          	          
				send_command[7] = verify_code(7);
				send_command[8] = DATA_END;  	                
				tra_com(send_command,9);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;       
			//--------------------------------------------------------------------------------------      
			//  receive move inpresser command
			//--------------------------------------------------------------------------------------
			case INMOVE:            

				predit_shift = 1;
				temp16 = (INT16)rec_buf[3]<<8;
				inpress_high = temp16 | (INT16)rec_buf[4];
				tra_ind_r = 0; 
				tra_ind_w = 0;                          		            
				send_command[0] = DATA_START;
				send_command[1] = 0x04;
				send_command[2] = INMOVE_RET;  				           				          
				send_command[3] = inpress_high;  				           				          
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END; 
				tra_com(send_command,6);
				inpress_action_flag =1;     
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;     

			//--------------------------------------------------------------------------------------
			// receive current stitch query command
			//--------------------------------------------------------------------------------------
			case CURRENTSTITCHQUERY:
				tra_ind_r = 0; 
				tra_ind_w = 0;  			
				
				if(super_pattern_flag == 1) 
				    PatternShiftValue = pat_buff_total_counter;
				else
			    	PatternShiftValue = pat_point - (PATTERN_DATA *)(pat_buf);   
				    
				send_command[0] = DATA_START;  
				send_command[1] = 0x07;   
				send_command[2] = CURRENTSTITCHQUERY_RET;
				send_command[3] = PatternShiftValue>>8;
				send_command[4] = PatternShiftValue;
				send_command[5] = SewTestStitchCounter>>8;
				send_command[6] = SewTestStitchCounter;
				send_command[7] = verify_code(7);
				send_command[8] = DATA_END;
				tra_com(send_command,9);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;
			case FOOTSTATEQUERY:
				tra_ind_r = 0; 
				tra_ind_w = 0;                        		            
				send_command[0] = DATA_START;  
				send_command[1] = 0x04;   
				send_command[2] = FOOTSTATEQUERY_RET;
				send_command[3] = foot_flag;
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END;
				tra_com(send_command,7);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;
			//--------------------------------------------------------------------------------------
			// receive speed adjust command in sewing process
			//--------------------------------------------------------------------------------------
			case SEWSPEEDADJUST:
				MotorSpeedRigister = rec_buf[3];    
				tra_ind_r = 0; 
				tra_ind_w = 0;                          		            
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = SEWSPEEDADJUST_RET;  				           				          
 				send_command[3] = verify_code(3);
				send_command[4] = DATA_END;  	                
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break; 
			//--------------------------------------------------------------------------------------      
			//  receive move inpresser command
			//--------------------------------------------------------------------------------------
			case HIGH:       		                      		                                               		                    
				tra_ind_r = 0; 
				tra_ind_w = 0;                          		            
				send_command[0] = DATA_START;
				send_command[1] = 0x04;
				send_command[2] = HIGH_RET;  				           				          
				if(inpress_position >90 )
				  send_command[3] = 220; //2013-1-15
				else
				send_command[3] = inpress_position;  				           				          
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END;  	                
				tra_com(send_command,6);    
			
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;   
			//--------------------------------------------------------------------------------------      
			//  receive servo motor speed self test command
			//--------------------------------------------------------------------------------------
			case SMOTOR:       		            	                         		          
				smotor_speed = rec_buf[3];     		                        		                                               		                    
				smotor_direction = rec_buf[4];     		                        		                                               		                    
				if((smotor_direction !=0) && (smotor_direction !=1))      //2012-6-27
				    smotor_direction = 0;    		                        		                                               		                    
				tra_ind_r = 0; 
				tra_ind_w = 0;                          		            
				send_command[0] = DATA_START;
				send_command[1] = 0x04;
				send_command[2] = SMOTOR_RET;  				           				          
				send_command[3] = smotor_speed;  				           				          
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END;  	                
				tra_com(send_command,6);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;                 
			//--------------------------------------------------------------------------------------      
			//  receive servo motor speed query command
			//--------------------------------------------------------------------------------------
			case SPEED:       	
				temp8 = (motor.spd+50) / 100;	   //2010-5-26            		  
				tra_ind_r = 0; 
				tra_ind_w = 0;                          		            
				send_command[0] = DATA_START;
				send_command[1] = 0x04;
				send_command[2] = SPEED_RET;  				           				          
				send_command[3] = temp8;  				           				          
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END;  	                
				tra_com(send_command,6);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;                
			//--------------------------------------------------------------------------------------      
			//  receive input query command
			//--------------------------------------------------------------------------------------
			case INPUT:       	    		            	               		                        		                                               		                    
				tra_ind_r = 0; 
				tra_ind_w = 0;                          		            
				send_command[0] = DATA_START;
				send_command[1] = 0x14;
				send_command[2] = INPUT_RET;  	
				//--------------------------------------------------------------------------------------      
				//  XORG
				//--------------------------------------------------------------------------------------
				if(XORG == ON)
				{
					temp8 = OFF;//ON;
				}	
				else
				{
					temp8 = ON;//OFF;
				}				           				          
				send_command[3] = temp8;  				
				//--------------------------------------------------------------------------------------      
				//  YORG
				//--------------------------------------------------------------------------------------  
				if(YORG == ON)
				{
					temp8 = OFF;//ON;
				}	
				else
				{
					temp8 = ON;//OFF;
				}				           				          
				send_command[4] = temp8;     
				//--------------------------------------------------------------------------------------      
				//  PORG
				//--------------------------------------------------------------------------------------  
				if(PORG == ON)
				{
					temp8 = OFF;//ON;
				}	
				else
				{
					temp8 = ON;//OFF;
				}				           				          
				send_command[5] = temp8; 
				//--------------------------------------------------------------------------------------      
				//  PSENS
				//--------------------------------------------------------------------------------------  
				if(PSENS == ON)
				{
					temp8 = OFF;//ON;
				}	
				else
				{
					temp8 = ON;//OFF;
				}				           				          
				send_command[6] = temp8; 
				//--------------------------------------------------------------------------------------      
				//  CORG
				//--------------------------------------------------------------------------------------  
				if(CORG == ON)
				{
					temp8 = OFF;//ON;
				}	
				else
				{
					temp8 = ON;//OFF;
				}
#if(JITOUBAN == 0)
				send_command[7] = temp8; 
#else
				send_command[8] = temp8;
#endif
				//--------------------------------------------------------------------------------------      
				//  CSENS
				//--------------------------------------------------------------------------------------  
				if(CSENS == ON)
				{
					temp8 = OFF;//ON;
				}	
				else
				{
					temp8 = ON;//OFF;
				}				           				          
#if(JITOUBAN == 0)
				send_command[8] = temp8; 
#else
				send_command[7] = temp8;
#endif
				//--------------------------------------------------------------------------------------      
				//  IORG
				//--------------------------------------------------------------------------------------  
				if(IORG == ON)
				{
					temp8 = OFF;//ON;
				}	
				else
				{
					temp8 = ON;//OFF;
				}				           				          
				send_command[9] = temp8; 
				//--------------------------------------------------------------------------------------      
				//  SFSW
				//--------------------------------------------------------------------------------------  
				if(SFSW == ON)
				{
					temp8 = OFF;//ON;
				}	
				else
				{
					temp8 = ON;//OFF;
				}				           				          
				send_command[10] = temp8; 
				//--------------------------------------------------------------------------------------      
				//  TH_BRK
				//--------------------------------------------------------------------------------------  
				if(TH_BRK == ON)
				{
					temp8 = ON;
				}	
				else
				{
					temp8 = OFF;
				}				           				          
				send_command[11] = temp8; 
				//--------------------------------------------------------------------------------------      
				//  PAUSE
				//--------------------------------------------------------------------------------------  
				if(PAUSE == ON)
				{
					temp8 = OFF;//ON;
				}	
				else
				{
					temp8 = ON;//OFF;
				}				           				          
				send_command[12] = temp8;  
				//--------------------------------------------------------------------------------------      
				//  DVA
				//--------------------------------------------------------------------------------------  
				if(DVA == para.dvab_open_level)
				{
					temp8 = OFF;//ON;
				}	
				else
				{
					temp8 = ON;//OFF;
				}				           				          
				send_command[13] = temp8; 
				//--------------------------------------------------------------------------------------      
				//  DVB
				//--------------------------------------------------------------------------------------  
				if(DVB == para.dvab_open_level)
				{
					temp8 = OFF;//ON;3
				}	
				else
				{
					temp8 = ON;//OFF;
				}				           				          
				send_command[14] = temp8;
				//--------------------------------------------------------------------------------------      
				//  ADTCSM
				//--------------------------------------------------------------------------------------  
				//if(ADTCSM == ON)
				if(ADTCSM )
				{
					temp8 = OFF;//ON;3
				}	
				else
				{
					temp8 = ON;//OFF;
				}				           				          
				send_command[15] = temp8;
				//--------------------------------------------------------------------------------------      
				//  DVSM
				//-------------------------------------------------------------------------------------- 
				if(DVSM == ON)
				{
					temp8 = OFF;//ON;
				}	
				else
				{
					temp8 = ON;//OFF;
				}				           				          
				send_command[16] = temp8; 
				//--------------------------------------------------------------------------------------      
				//  no use
				//--------------------------------------------------------------------------------------  
				send_command[17] = temp8;
				send_command[18] = temp8;
				send_command[19] = temp8;  				                				          
				send_command[20] = verify_code(20);
				send_command[21] = DATA_END;  	                
				tra_com(send_command,22);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;   
			//--------------------------------------------------------------------------------------      
			//  receive output query command
			//--------------------------------------------------------------------------------------
			case OUTPUT:      
			    switch(rec_buf[3])                      // data package
				{
					case 0x01: 
					    switch(rec_buf[4])
					    {
							case 0:
							    fw_flag = 0;	
							break;
							case 1:
							    fw_flag = 1;	
							break;
					    }		
						output_com = 1;  
						break;
					case 0x02: 
					     switch(rec_buf[4])
					    {
							case 0:
							    L_AIR_flag = 0;	
							break;
							case 1:
							    L_AIR_flag = 1;	
							break;
					    }
						output_com = 2;  
						break;  
				
					case 0x05: 
					     switch(rec_buf[4])
					    {
							case 0:
							    da0_flag = 0;	
							break;
							case 1:
							    da0_flag = 1;	
							break;
					    }
						output_com = 5;  
						break; 			  
								            							    		            	 	
					default:   
		  				output_com = rec_buf[3];
						break;;	
				}	    
				temp8 = rec_buf[3];
				temp16 = rec_buf[4];
        	               		                        		                                               		                    
				tra_ind_r = 0; 
				tra_ind_w = 0;                          		            
				send_command[0] = DATA_START;
				send_command[1] = 0x05;
				send_command[2] = OUTPUT_RET;  				           				          
				send_command[3] = temp8;
				send_command[4] = temp16;  				           				          
				send_command[5] = verify_code(5);
				send_command[6] = DATA_END;  	                
				tra_com(send_command,7);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break; 
			//--------------------------------------------------------------------------------------
			// send motor mechanic angle for display
			//--------------------------------------------------------------------------------------
			case MOTORMECHANICANGLECHECK: 
				tra_ind_r = 0; 
				tra_ind_w = 0;                          		            
				send_command[0] = DATA_START;  
				send_command[1] = 0x05;   
				send_command[2] = MOTORMECHANICANGLECHECK_RET;
				
				send_command[3] = motor.angle>>8;
				send_command[4] = motor.angle;

				send_command[5] = verify_code(5);
				send_command[6] = DATA_END;
				tra_com(send_command,7);    
				rec_ind_r = 0; 
				rec_ind_w = 0;

				break;

			//--------------------------------------------------------------------------------------
			// send motor mechanic angle for adjust
			//--------------------------------------------------------------------------------------
			case MOTORMECHANICANGLEENTER: 
				temp16 = (INT16)rec_buf[3]<<8;
				AdjustAngle = temp16 | (INT16)rec_buf[4];
				tra_ind_r = 0; 
				tra_ind_w = 0;                          		            
				send_command[0] = DATA_START;  
				send_command[1] = 0x05;   
				send_command[2] = MOTORMECHANICANGLEENTER_RET;
				temp8 = rec_buf[3];
				send_command[3] = temp8;
				temp8 = rec_buf[4];
				send_command[4] = temp8;
				send_command[5] = verify_code(5);
				send_command[6] = DATA_END;
				tra_com(send_command,7);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;	
			//--------------------------------------------------------------------------------------      
			//  receive xysensor query command
			//--------------------------------------------------------------------------------------
			case XYSENSOR:        		                         
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x05;
				send_command[2] = XYSENSOR_RET;
				//--------------------------------------------------------------------------------------      
				//  XORG
				//--------------------------------------------------------------------------------------
				if(XORG == ON)
				{
					temp8 = OFF;
				}	
				else
				{
					temp8 = ON;
				}				           				          
				send_command[3] = temp8;  				
				//--------------------------------------------------------------------------------------      
				//  YORG
				//--------------------------------------------------------------------------------------  
				if(YORG == ON)
				{
					temp8 = OFF;
				}	
				else
				{
					temp8 = ON;
				}				           				          
				send_command[4] = temp8;       				          
				send_command[5] = verify_code(5);
				send_command[6] = DATA_END;                  
				tra_com(send_command,7);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;                                                      
			//--------------------------------------------------------------------------------------      
			//  receive stepping motor coordinate command
			//--------------------------------------------------------------------------------------
			case COORCOM:                                    		                         
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x07;
				send_command[2] = COORCOM_RET; 

				temp8 = rec_buf[3]; 		          				           				          
				send_command[3] = temp8;  
				temp16 = (INT16)temp8<<8;     				       
				temp8 = rec_buf[4];  				                 				          
				send_command[4] = temp8;
				temp16 = temp16|(INT16)temp8; 
				comx_step = temp16;
				
				temp8 = rec_buf[5];
				send_command[5] = temp8;  
				temp16 = (INT16)temp8<<8; 
				temp8 = rec_buf[6]; 
				send_command[6] = temp8; 
				temp16 = temp16|(INT16)temp8;
				comy_step = temp16;

				send_command[7] = verify_code(7); 
				send_command[8] = DATA_END;                 
				tra_com(send_command,9);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				coor_com = 1;
				predit_shift = 1;
				break; 
			//--------------------------------------------------------------------------------------      
			//  receive stepping motor state query command
			//--------------------------------------------------------------------------------------
			case PREDIT_SHIFT:        		                         
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x04;
				send_command[2] = PREDIT_SHIFT_RET;  
				if(origin_com ==1)//2012-11-24
				   predit_shift =1 ;
				send_command[3] = predit_shift;       				                 				          
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END;                  
				tra_com(send_command,6);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break; 
			case STITCHUP:
				tra_ind_r = 0; 
				tra_ind_w = 0;   
				StitchUpFlag = rec_buf[3];               
				send_command[0] = DATA_START;
				send_command[1] = 0x03; 
				send_command[2] = STITCHUP_RET;  
				send_command[3] = verify_code(3);
				send_command[4] = DATA_END; 
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				
				break;     
			case POINTSHIFT:
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = POINTSHIFT_RET;
				PointShiftDirection = rec_buf[3];
				send_command[3] = verify_code(3);
				send_command[4] = DATA_END;
				temp8 = rec_buf[4];
				temp16 = (INT16)temp8<<8;
				temp8 = rec_buf[5];
				temp16 = temp16|(INT16)temp8;
				PointShiftNumber = temp16; 
				
				if(super_pattern_flag == 1)
				{
				  if( rec_buf[6] == 0)
				    pat_buff_write_offset = 0;
				  else
				    pat_buff_write_offset = HALF_WRITE_OFFSET;
					
				 temp16 = (INT16)rec_buf[7]<<8;
				 inpress_high = temp16 | (INT16)rec_buf[8];
				 
				 last_inpress_position = inpress_high;
				 //inpress_high_flag = 1;
				}
				
					
				
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				PointShiftFlag = 1;
				break;    

 			case CURRENTPOINTSHIFT:
				tra_ind_r = 0; 
				tra_ind_w = 0; 
				CurrentPointShiftFlag = 1;
				temp8 = rec_buf[3];
				temp16 = (INT16)temp8<<8;
				temp8 = rec_buf[4];
				CurrentPointShiftPosition = temp16|(INT16)temp8;
				
				if( CurrentPointShiftPosition == 0 )
				{
					pat_point = (PATTERN_DATA *)(pat_buf);
					pat_buff_write_offset = 0;
					pat_buff_total_counter = 0;
				}
				  
				send_command[0] = DATA_START;
				send_command[1] = 0x03;   
				send_command[2] = CURRENTPOINTSHIFT_RET; 
				send_command[3] = verify_code(3);  
				send_command[4] = DATA_END; 
				tra_com(send_command,5);    
				rec_ind_r = 0;
				rec_ind_w = 0;          
				break;   
			case PATTERNPOINTSHIFT:
				tra_ind_r = 0; 
				tra_ind_w = 0;  

				temp8 = rec_buf[3];
				temp16 = (INT16)temp8<<8;
				temp8 = rec_buf[4];
				CurrentPointShiftPosition = temp16|(INT16)temp8;  
				temp8 = rec_buf[5];
				temp16 = (INT16)temp8<<8;
				temp8 = rec_buf[6];
				DestinationPointShiftPosition = temp16|(INT16)temp8;  

				MotionSet =  rec_buf[7];  
				MotiongSetFlag = 1;

				temp8 = rec_buf[8];
				temp16 = (INT16)temp8<<8;
				temp8 = rec_buf[9];
				comx_step = temp16|(INT16)temp8; 

				temp8 = rec_buf[10];
				temp16 = (INT16)temp8<<8;
				temp8 = rec_buf[11];
				comy_step = temp16|(INT16)temp8;  


				send_command[0] = DATA_START;
				send_command[1] = 0x03; 
				send_command[2] = PATTERNPOINTSHIFT_RET; 
				send_command[3] = verify_code(3);  
				send_command[4] = DATA_END; 
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0; 
				break; 
                                                                                                                                                                  
			//--------------------------------------------------------------------------------------      
			//  test communication
			//--------------------------------------------------------------------------------------
			case TEST:    
	
				break;                                       
			case ORIGIN:
				predit_shift =1;
				origin_com = 1;
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = ORIGIN_RET;
				send_command[3] = verify_code(3);
				send_command[4] = DATA_END;  	                
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;

			break;
			case ADJUST_ORIGIN:
			    predit_shift =1;
				origin_adjust_flag = rec_buf[3];
				predit_shift =0;
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = ADJUST_ORIGIN_RET;
				send_command[3] = verify_code(3);
				send_command[4] = DATA_END;  	                
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
			break;
			case FOOT_COMMAND:
				foot_com = rec_buf[3];
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = FOOT_COMMAND_RET;
				send_command[3] = verify_code(3);
				send_command[4] = DATA_END;  	                
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
			break;
			case SET_XY_ADJUST://2010-12-08
				temp16 = (INT16)rec_buf[3]<<8;
				x_bios = temp16 | (INT16)rec_buf[4];
				temp16 = (INT16)rec_buf[5]<<8;
				y_bios = temp16 | (INT16)rec_buf[6];
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = SET_XY_ADJUST_RET;
				send_command[3] = verify_code(3);
				send_command[4] = DATA_END;  	                
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
			break;
			case IMFORMATIONQUERY:   //2011-6-20
				tra_ind_r = 0; 
				tra_ind_w = 0;                          		            
				send_command[0] = DATA_START;  
				send_command[1] = 0x07;   
				send_command[2] = IMFORMATIONQUERY_RET;
				send_command[3] = OverCurrentOneCycle>>8;
				send_command[4] = OverCurrentOneCycle;
				send_command[5] = OutOfPositionTimes>>8;
				send_command[6] = OutOfPositionTimes;
				send_command[7] = verify_code(7);
				send_command[8] = DATA_END;
				tra_com(send_command,9);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				OverCurrentOneCycle = 0;
				OutOfPositionTimes = 0;
			break;
			//--------------------------------------------------------------------------------------      
			//  receive isensor query command
			//--------------------------------------------------------------------------------------
			case ISENSOR:        		                         
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x04;
				send_command[2] = ISENSOR_RET;
				//--------------------------------------------------------------------------------------      
				//  IORG
				//--------------------------------------------------------------------------------------  
				//if(IORG == ON)
				if( get_IORG_statu())
				{
					temp8 = ON;
				}	
				else
				{
					temp8 = OFF;
				}				           				          
				send_command[3] = temp8;       				                 				          
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END;                  
				tra_com(send_command,6);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
                    break; 
			//--------------------------------------------------------------------------------------      
      		case MOTOCWW: 
      			predit_shift = 1;//2011-8-18
				stepmotor_single = rec_buf[3];
      			tra_ind_r = 0; 
      			tra_ind_w = 0;                  
  	  			send_command[0] = DATA_START;
  	  			send_command[1] = 0x04;
  				send_command[2] = MOTOCWW_RET;  				          				           				          
  				send_command[3] = stepmotor_single;       				                 				          
  				send_command[4] = verify_code(4);
  				send_command[5] = DATA_END;                  
      			tra_com(send_command,6);    
      			rec_ind_r = 0; 
      			rec_ind_w = 0;
				
      			break; 	
			//--------------------------------------------------------------------------------------      
			//  receive stepping motor command
			//--------------------------------------------------------------------------------------
			case STEPMOVE: 
				predit_shift = 1;//2011-8-18
				stepmotor_comm = rec_buf[3];
					tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x04;
				send_command[2] = STEPMOVE_RET;  				          				           				          
				send_command[3] = stepmotor_comm;       				                 				          
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END;                  
				tra_com(send_command,6);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;                                                       
			//--------------------------------------------------------------------------------------      
			//  receive stepping motor state query command
			//--------------------------------------------------------------------------------------
			case MOTOSTA:        		                         
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x04;
				send_command[2] = MOTOSTA_RET;  				          				           				          
				send_command[3] = stepmotor_state;       				                 				          
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END;                  
				tra_com(send_command,6);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break; 
			//--------------------------------------------------------------------------------------      
			//  LED LIGHT ADJUST 
			//--------------------------------------------------------------------------------------	
			case LED_LIGHT_ADJUST:       	
					led_light_adjust = rec_buf[3];	     
					temp = led_light_adjust;    
					#if COM_MONITOR_FUN == 0  
				    if(temp == 0)
					{  
					    da1 = 0;
					}
				    if( temp >0 && temp <= 100)
					{
					    da1 = 140 + temp*10/33;
					} 
					#endif
					tra_ind_r = 0; 
					tra_ind_w = 0;                          		            
					send_command[0] = DATA_START;
					send_command[1] = 0x03;
					send_command[2] = LED_LIGHT_ADJUST_RET;  				           				           				           				          
					send_command[3] = verify_code(3);
					send_command[4] = DATA_END;  	                
					tra_com(send_command,5);    
							rec_ind_r = 0; 
							rec_ind_w = 0;
							break;                
			case FOOT_STATUS_FINISH:
				predit_shift =1;
				u38 = rec_buf[3];
				u210 = rec_buf[4];
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x04;
				send_command[2] = FOOT_STATUS_FINISH_RET;
				send_command[3] = u38;
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END;  	                
				tra_com(send_command,6);    
					rec_ind_r = 0; 
					rec_ind_w = 0;
					break;                
			case READY_GO_SETOUT:
				predit_shift =1;
				ready_go_setout_com = 1;
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = READY_GO_SETOUT_RET;
				send_command[3] = verify_code(3);
				send_command[4] = DATA_END;  	                
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;

			break;
			case CUT_COMMAND:
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = CUT_COMMAND_RET;  				          				           				          
				send_command[3] = verify_code(3);
				send_command[4] = DATA_END;                  
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				cut_test_flag = 1;
			break;
			
			case AUTO_SELECT_PATTERN:					
			    temp = pattern_number;
			    tra_ind_r = 0; 
				tra_ind_w = 0;   
			    send_command[0] = DATA_START;
				send_command[1] = 0x05;
				send_command[2] = AUTO_SELECT_PATTERN_RET;  
				send_command[3] = (UINT8)(temp>>8);
				send_command[4] = (UINT8)temp; 				           				           				           				          
				send_command[5] = verify_code(5);
				send_command[6] = DATA_END;  	                
				tra_com(send_command,7);    
				rec_ind_r = 0; 
				rec_ind_w = 0;				
			//	pattern_change_flag = 0;	//?			
			break;		
			
			case NEW_PATTERN_DONE:
	     		tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = NEW_PATTERN_DONE_RET;  				          				           				          
				send_command[3] = verify_code(3);
				send_command[4] = DATA_END;                  
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				new_pattern_done = 1;
				
			break;
			case RFID_WRITE:
					rc522_control_falg = 1;
					rc522_write_falg = 1;
					rc522_write_ret_falg = 1;
					temp16 = (UINT16)rec_buf[3]<<8;
			        Rfid_Nom = temp16 | (UINT16)rec_buf[4];//总针数
			  break;
			case  RFID_READ:		
					rc522_write_falg = 0;
					rc522_control_falg = 1; 
			  break;	
			case    QUERY_ID:
					tra_ind_r = 0; 
					tra_ind_w = 0;
					send_command[0] = DATA_START;
					send_command[1] = 0x0C;
					send_command[2] = QUERY_ID_RET;
					send_command[3] = OW_RomID[7];
					send_command[4] = OW_RomID[6];
					send_command[5] = OW_RomID[5];
					send_command[6] = OW_RomID[4];
					send_command[7] = OW_RomID[3];
					send_command[8] = OW_RomID[2];
					send_command[9] = OW_RomID[1];
					send_command[10] = OW_RomID[0];
					send_command[11] = main_control_lock_flag;
					send_command[12] = verify_code(12);
					send_command[13] = DATA_END;
					tra_com(send_command,14);
					rec_ind_r = 0; 
					rec_ind_w = 0; 
			  break;		
			 
			  case   SET_MAIN_CONTROL_LOCK:			 
			        main_control_lock_flag = rec_buf[3];					
			 		tra_ind_r = 0; 
					tra_ind_w = 0;
					send_command[0] = DATA_START;
					send_command[1] = 0x04;
					send_command[2] = SET_MAIN_CONTROL_LOCK_RET;
					send_command[3] = main_control_lock_flag;
					send_command[4] = verify_code(4);
					send_command[5] = DATA_END;                  
					tra_com(send_command,6);    
					rec_ind_r = 0; 
					rec_ind_w = 0;					
					main_control_lock_setup = 1;			
					
			  break;
			   
				   
			  case SET_BASELINE_ALARM:
					baseline_alarm_flag = rec_buf[3];
					temp16 = (UINT16)rec_buf[4]<<8;
			        baseline_alarm_stitchs = temp16 | (UINT16)rec_buf[5];//总针数
					
					tra_ind_r = 0; 
					tra_ind_w = 0;                  
					send_command[0] = DATA_START;
					send_command[1] = 0x03;
					send_command[2] = SET_BASELINE_ALARM_RET;  				          				           				          
					send_command[3] = verify_code(3);
					send_command[4] = DATA_END;                  
					tra_com(send_command,5);    
					rec_ind_r = 0; 
					rec_ind_w = 0;
			  break;
			  case SWITCH_COMM_RATE:
			        
				    tra_ind_r = 0; 
					tra_ind_w = 0;
					send_command[0] = DATA_START;
					send_command[1] = 0x03;
					send_command[2] = SWITCH_COMM_RATE_RET;
					send_command[3] = verify_code(3);
					send_command[4] = DATA_END; 					                 
					tra_com(send_command,5);  
					rec_ind_r = 0; 
					rec_ind_w = 0;	
					
					delay_us(10000);// 10ms									
					
					urat0_rate_115200();
			       break;

			//---------------------------------------------------------------------------------------
			//  send servo motor para to panel           
			//---------------------------------------------------------------------------------------
			case DISPLAY_SERVOPARA:
			    if( rec_buf[1] >= 4)
				    operate_dsp1ordsp2_flag = rec_buf[3];
				else
				    operate_dsp1ordsp2_flag = 0;
			    read_stepmotor_para();
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 97;
				send_command[2] = DISPLAY_SERVOPARA_RET;  				          				           				          
				for( i=0; i<93; i++)  
				{
					send_command[3+i] = svpara_disp_buf[i];
				}      
				send_command[96] = operate_dsp1ordsp2_flag; 				                 				          
				send_command[97] = verify_code(97);
				send_command[98] = DATA_END;                  
				tra_com(send_command,99);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;	
			//_____________________________________________________________________________________
			//receive servo para               
			//_____________________________________________________________________________________
			case SERVOPARA:
			    for(i=0;i<93;i++)
				{
					svpara_buf[i]=rec_buf[3+i];				
				} 
				if( rec_buf[1] >= 97)
				  	operate_dsp1ordsp2_flag = rec_buf[96];
				else
					operate_dsp1ordsp2_flag = 0;
				svpara_trans_flag = 1;
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = SERVOPARA_RET;
				send_command[3] = verify_code(3);
				send_command[4] = DATA_END;  	                
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				break;          
			//--------------------------------------------------------------------------------------			  
			//  DSP update  function code
			//--------------------------------------------------------------------------------------
						  
			case DRV_DATA:
			  
			    data_length_drv = rec_buf[1];

				if((0 == rec_buf[4]) && (0 == rec_buf[5])&& (0 == rec_buf[6]))
				   erase_falg = 1;
				else 
				   erase_falg = 0;
				
				
				recpat_point = (UINT8 *)(rec_buf);   // data address   
	            
				for(i=0;i<rec_buf[1]+2;i++)
				{
						pat_buf[i] = *recpat_point;
						recpat_point++;
				} 
				DRV_DATA_LEN++;    
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x04;//data_length;
				send_command[2] = DRV_DATA_RET;
				send_command[3] = DRV_DATA_LEN;//
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END;  	                
				tra_com(send_command,6);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				predit_shift = 5;
				break; 
			case DRV_DATA_END:
			    
			    data_length_drv = rec_buf[1]; 
				recpat_point = (UINT8 *)(rec_buf);   // data address   
	            
				for(i=0;i<rec_buf[1]+2;i++)
				{
						pat_buf[i] = *recpat_point;
						recpat_point++;
				} 
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x04;//data_length;
				send_command[2] = DRV_DATA_END_RET;
				send_command[3] = DRV_DATA_LEN;
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END;  	                
				tra_com(send_command,6);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
				predit_shift = 6;
				DRV_DATA_LEN =0;
				break; 
			//===================================================================	
			case CONTROL_PARA:			
				recpat_point = (UINT8 *)&(rec_buf+4);   // data address
				if(rec_buf[3] > 100)
				{
					com_error();					
				}
				else
				{
					temp = 250*(rec_buf[3]-1);	
					for(i=0;i<rec_buf[1]-4;i++)
					{
						pat_buf[temp+i] = *recpat_point;
						recpat_point++;
					}
					//sys_para_length = (UINT16)(temp + receive_ptr[1]-4);
					tra_ind_r = 0; 
					tra_ind_w = 0;
					send_command[0] = DATA_START;
					send_command[1] = 0x04;
					send_command[2] = CONTROL_PARA_RET;
					send_command[3] = rec_buf[3];	
					send_command[4] = verify_code(4);
					send_command[5] = DATA_END;
					tra_com(send_command,6);
					rec_ind_r = 0; 
					rec_ind_w = 0;
					if(rec_buf[3] > 7)
					{		
						//notice_write_sys_param(pattern.pattern.pattern_buff,sys_para_length);		
						SUM = 1;
					    delay_us(20000);
					    SUM = 0;
					}	
				}
					//predit_shift = 1;
			break;
			
			case RESET_USERPARAM:
					tra_ind_r = 0; 
					tra_ind_w = 0;
					send_command[0] = DATA_START;
					send_command[1] = 0x04;
					send_command[2] = RESET_USERPARAM_RET;
					send_command[3] = 0;	
					send_command[4] = verify_code(4);
					send_command[5] = DATA_END;
					tra_com(send_command,6);
					rec_ind_r = 0; 
					rec_ind_w = 0;
			break;
			case CAL_USERPARAM_PACK:
					tra_ind_r = 0; 
					tra_ind_w = 0;
					send_command[0] = DATA_START;
					send_command[1] = 0x06;
					send_command[2] = CAL_USERPARAM_PACK_RET;
					send_command[3] = 0;
					send_command[4] = 0;
					send_command[5] = 8;	
					send_command[6] = verify_code(6);
					send_command[7] = DATA_END;
					tra_com(send_command,8);
					rec_ind_r = 0; 
					rec_ind_w = 0;
			break;
			case READ_USERPARAM:
				    tra_ind_r = 0; 
					tra_ind_w = 0;
					send_command[0] = DATA_START;
					send_command[1] = 0x05;
					send_command[2] = READ_USERPARAM_RET;
					send_command[3] = rec_buf[3];	
					send_command[4] = rec_buf[4];
					send_command[5] = verify_code(5);
					send_command[6] = DATA_END;
					tra_com(send_command,7);
					rec_ind_r = 0; 
					rec_ind_w = 0;
			break;
			case SYS_PARAM_GROUP://读参数
					temp = ((UINT16)rec_buf[3] << 8) + rec_buf[4];
					tra_ind_r = 0; 
					tra_ind_w = 0;
		
					if( rec_buf[5] == 0 )//read
					{
						send_command[0] = DATA_START;
						send_command[1] = 210;
						send_command[2] = SYS_PARAM_GROUP_RET;						
						send_command[3] = rec_buf[5];
						send_command[4] = 0;//ok
						if( temp == 1)
							read_para_group(svpara_disp_buf,205);
						else if( temp == 3)
							read_stepmotor_config_para(1);
						else if( temp == 4)
							read_stepmotor_config_para(2);
						for( i=0; i< 205 ; i++)  
						{
							send_command[5+i] = svpara_disp_buf[i];
						}      
						send_command[210] = verify_code(210);
						send_command[211] = DATA_END;                  
						tra_com(send_command,212);    
						rec_ind_r = 0; 
						rec_ind_w = 0;
					}
					else  //write;
					{
						SUM = 1;
					    delay_us(20000);
					    SUM = 0;
						send_command[0] = DATA_START;
						send_command[1] = 5;
						send_command[2] = SYS_PARAM_GROUP_RET;						
						send_command[3] = rec_buf[5];
						send_command[4] = 0;//ok
						for( i=0; i< 205 ; i++)  
						{
							svpara_disp_buf[i] = rec_buf[6 + i];
						}  
						if( temp == 1)
							write_eeprom_para_flag = 1;
						else if( temp == 3)
						    wirte_stepermotor_para_flag = 1;
						else if( temp == 4)
							wirte_stepermotor_para_flag = 2;
				
						send_command[5] = verify_code(5);
						send_command[6] = DATA_END;                  
						tra_com(send_command,7);    
						rec_ind_r = 0; 
						rec_ind_w = 0;
					}
			break;
			case CHECK11_CHECK_SENSOR:
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x04;
				send_command[2] = CHECK11_CHECK_SENSOR_RET;
				//--------------------------------------------------------------------------------------      
				//  CORG
				//--------------------------------------------------------------------------------------  
				if( get_CORG_statu())
				{
					temp8 = ON;
				}	
				else
				{
					temp8 = OFF;
				}				           				          
				send_command[3] = temp8;       				                 				          
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END;                  
				tra_com(send_command,6);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
			break;
			case QUERY_FOOTER_HIGH:
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x04;
				send_command[2] = QUERY_FOOTER_HIGH_RET;
				send_command[3] = 0;       				                 				          
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END;                  
				tra_com(send_command,6);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
			break;
			case SOFT_KEY_FOOTER://         0x44
			    tra_ind_r = 0; 
				tra_ind_w = 0;                 
				software_key_footer = 1; 
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = SOFT_KEY_FOOTER_RET;
				send_command[3] = verify_code(3);       				                 				          
				send_command[4] = DATA_END;                  
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
			break;
			case SOFT_KEY_RUN://         0x44
			    tra_ind_r = 0; 
				tra_ind_w = 0;          
				software_key_run = 1;        
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = SOFT_KEY_RUN_RET;
				send_command[3] = verify_code(3);       				                 				          
				send_command[4] = DATA_END;                  
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
			break;
			case SOFT_KEY_BOBBIN://         0x44
			    tra_ind_r = 0; 
				tra_ind_w = 0;       
				software_key_bobbin = 1;           
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = SOFT_KEY_BOBBIN_RET;
				send_command[3] = verify_code(3);       				                 				          
				send_command[4] = DATA_END;                  
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
			break;
		    case SOFT_KEY_PAUSE://         0x44
			    tra_ind_r = 0; 
				tra_ind_w = 0;            
				software_key_pause = 1;      
				send_command[0] = DATA_START;
				send_command[1] = 0x03;
				send_command[2] = SOFT_KEY_PAUSE_RET;
				send_command[3] = verify_code(3);       				                 				          
				send_command[4] = DATA_END;                  
				tra_com(send_command,5);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
			break;
			
			case SYS_CON_PARAM:
			    predit_shift = 1;
			    data_length_drv = rec_buf[1];
				temp8 = rec_buf[4];
				index = rec_buf[5];
				recpat_point = (UINT8 *)(rec_buf);   // data address   
				
				for(i=0;i<rec_buf[1]-6;i++)
				{
					if((2 == rec_buf[3])||(4 == rec_buf[3]))//
						pat_buf[((UINT16)temp8)*240 + i] = *(recpat_point+6);
					else 
						svpara_disp_buf[((UINT16)temp8)*240 + i] = *(recpat_point+10);
					recpat_point++;
				} 
				if((temp8+1) == index )
				{
					switch(rec_buf[3])
					{
						case 0://主控配置参数升级    
							write_eeprom_para_flag = 1;
							break;	 					                         
						case 1://DSP1增益升级		  
							wirte_stepermotor_para_flag = 1;
							break;
						case 2://DSP1曲线升级
							write_stepmotor_curve_flag = 1;				                         
							break;
						case 3://DSP2增益升级
							wirte_stepermotor_para_flag = 2;
							break;
						case 4://DSP2曲线升级		
							write_stepmotor_curve_flag = 2;
							break;		
						default:
							break;
					}
						     	 
				}
				else 
					predit_shift = 0;
			
				//DRV_DATA_LEN++;    
				tra_ind_r = 0; 
				tra_ind_w = 0;                  
				send_command[0] = DATA_START;
				send_command[1] = 0x04;//data_length;
				send_command[2] = SYS_CON_PARAM_RET;
				send_command[3] = 0x01;//DRV_DATA_LEN;//
				send_command[4] = verify_code(4);
				send_command[5] = DATA_END;  	                
				tra_com(send_command,6);    
				rec_ind_r = 0; 
				rec_ind_w = 0;
			
				break; 
			
			
			
			//  no match function code
			//--------------------------------------------------------------------------------------
			default:   
				com_error();       // return communication error to panel
				break;            
		}
	}
	else
	{
		com_error();                  // return communication error to panel
	}	
}
void rfid_wr_ret(void)
{
	if(rc522_write_falg ==1)
	{	
		tra_ind_r = 0; 
		tra_ind_w = 0;           
		send_command[0] = DATA_START;
		send_command[1] = 0x04;
		send_command[2] = RFID_WRITE_RET; 
		send_command[3] = 0;//rc522_write_ret_falg;//rc522_write_falg_ret;				          				           				          
		send_command[4] = verify_code(4);
		send_command[5] = DATA_END;                  
		tra_com(send_command,6);    
		rec_ind_r = 0; 
		rec_ind_w = 0;
	}
	else
	{
		tra_ind_r = 0; 
		tra_ind_w = 0;                        		            
		send_command[0] = DATA_START;  
		send_command[1] = 0x05;   
		send_command[2] = RFID_READ_RET;		
		send_command[3] = serail_number>>8;
		send_command[4] = serail_number;
		send_command[5] = verify_code(5);
		send_command[6] = DATA_END;
		tra_com(send_command,7);    
		rec_ind_r = 0; 
		rec_ind_w = 0;
	}
	rc522_write_falg = 0;
	rc522_control_falg = 0; 
}
//--------------------------------------------------------------------------------------
//  Name:  com_error
//  pars: NONE
//  Returns: NONE
//  Description: return communication error to panel
//--------------------------------------------------------------------------------------
void com_error(void)
{

	tra_ind_r = 0; 
	tra_ind_w = 0; 
	find_communication_start_flag = 0; 
	rec_ind_r = 0; 
	rec_ind_w = 0; 
}
//--------------------------------------------------------------------------------------
//  Name:  verify_code
//  pars: UINT8 number
//  Returns: UINT8
//  Description: make verify code
//--------------------------------------------------------------------------------------
UINT8 verify_code(UINT8 number)
{
	UINT16 i; 
	UINT8 result; 
	
	result = send_command[0];
	for(i=1;i<number;i++)
	{
    	result = result ^ send_command[i];
	}	
	return result;
}
//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
