#include "..\..\include\sfr62p.h"           // M16C/62P special function register definitions
#include "..\..\include\typedef.h"          // data type define
#include "..\..\include\common.h"           // External variables declaration
#include "..\..\include\variables.h"        // External variables declaration
#include "..\..\include\initial.h"          // sine table 
#include "..\..\include\delay.h"            // delay time definition    
#include "..\..\include\stepmotor.h"        // stepmotor driver

#define  UART1_BUF_MIN  300
#define  UART1_BUF_MID  300


static UINT16 rec1_ind_r;             // receive  buffer reading index
static UINT16 rec1_ind_w;             // receive  buffer writing index

static UINT8 rec1_buf[UART1_BUF_MIN];       // receive  buffer
static UINT8 send1_command[60],comm1_status;
static UINT16 data_length;

#pragma INTERRUPT/E uart1_tra_int 
void uart1_tra_int(void);
#pragma INTERRUPT/E uart1_rec_int 
void uart1_rec_int(void);
void tra1_com(UINT8* command,UINT8 length);

#define START_FLAG                0x5A
#define END_FLAG                  0xFF

#define CRC_CHECK_INIT            0xFFFF

#define WRITESYSPARAM_RX          0x10
#define WRITESYSPARAM_TX          0x80
#define WRITESTEPPARAM_RX         0x11
#define WRITESTEPPARAM_TX         0x81
#define WRITESTEPCURVE_RX         0x12
#define WRITESTEPCURVE_TX         0x82



void initial_uart1_variable(void)
{
	  UINT16 i;
	  
	  tra1_ind_r = 0; 
	  tra1_ind_w = 0; 
	  rec1_ind_r = 0; 
	  rec1_ind_w = 0;  

	  for(i=0;i<255;i++)
	  {
		  tra1_buf[i] = 0;   
	  } 
	  for(i=0;i<UART1_BUF_MIN;i++)
	  {      
		  rec1_buf[i] = 0;  
	  } 
	  comm1_status = 0;
}


void init_uart1(void)
{   
  	u1mr = 0x00; 
  	u1mr = 0x05; 
  	u1c0 = 0x10;
  	u1c1 = 0x04;
  	ucon &= ~0x00;
  	u1brg = BAUD_RATE_9600 ;
  	re_u1c1 = 1;
  	s1tic = UART1_TRANSMIT_IPL;   
  	s1ric = UART1_RECEIVE_IPL_7; 
  	initial_uart1_variable();  
}
void init_uart1_RC522(void)
{   
  	u1mr = 0x00; 
  	u1mr = 0x05; 
  	u1c0 = 0x10;
  	u1c1 = 0x04;
  	ucon &= ~0x00;
  	//u1brg = BAUD_RATE ;//57600
	//u1brg = BAUD_RATE_38400;
	u1brg = BAUD_RATE_19200;
	//u1brg = BAUD_RATE_9600;
  	re_u1c1 = 1;
  	s1tic = UART1_TRANSMIT_IPL;   
  	s1ric = UART1_RECEIVE_IPL_7; 
  	initial_uart1_variable();  
}

void set_uart1_debug_mode(void)
{
  	u1mr = 0x00; 
  	u1mr = 0x05; 
  	u1c0 = 0x10;
  	u1c1 = 0x04;
  	ucon &= ~0x00;
  	u1brg = BAUD_RATE_115200 ;
  	re_u1c1 = 1;
  	s1tic = UART1_TRANSMIT_IPL;   
  	s1ric = UART1_RECEIVE_IPL_7; 
  	initial_uart1_variable();  

}


void uart1_tra_int(void)
{
  	volatile UINT16 i = 12;
	if( formwork_identify_device == 2)
	{
	}
	else
	{
	  	if(tra1_ind_r != tra1_ind_w)
	  	{
	    	u1tb = tra1_buf[tra1_ind_r++];
	  	}
	  	else                             
	  	{
	    	te_u1c1 = 0;
	    	while(i--);										
	  	}
	}
}

void uart1_rec_int(void)
{
	if( formwork_identify_device == 2)
	{
	}
	else
	{
		if(rec1_ind_w<299)
		   rec1_buf[rec1_ind_w++] = (UINT8)u1rb;
		else  
		{  
		   rec1_ind_w = 0; 
		   rec1_buf[rec1_ind_w++] = (UINT8)u1rb;
		}
	}
}

void tra1_com(UINT8* command,UINT8 length)
{
	INT16 i;
	
	if(!te_u1c1)
	{
		for(i=1;i<length;i++)
		{
			tra1_buf[tra1_ind_w++] = command[i];
		}
		u1tb = command[0];
		te_u1c1 = 1;
	}    
	else
	{    
		for(i=0;i<length;i++)
		{
			tra1_buf[tra1_ind_w++] = command[i];
		}
	}
}


#define CRC_POLY 0x1021

UINT16 cal_crc16(UINT8 *in, UINT16 len, UINT16 crc)
{
  UINT8 i;
  for (; len>0; --len)
  {
      crc = crc ^ ((UINT16)(*in++) << 8);
      for (i=0; i<8; ++i)
      {
	      if (crc & 0x8000)
	      {
	        crc = (crc << 1) ^ CRC_POLY;
	      }
	      else
	      {
	        crc = crc << 1;
	      }
      }
      crc = crc & 0xFFFF;
  }

  return crc;
}

static INT8 verify_rec(UINT8 * in, UINT16 len)
{
  UINT16 crc_read = ((UINT16)in[len-2] << 8) | (in[len-1] & 0xFF);
  UINT16 crc_cal = cal_crc16(in, len - 2, CRC_CHECK_INIT);

  if (crc_read == crc_cal)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void rec1_com(void)
{
  	UINT16 i;
		if( comm1_status != 0)
		{
			comm1_status = 0;
			init_uart1();
			return;
		}
	    if (serail_config_flag == 0)
		{
			serail_config_flag = 1;
			initial_uart1_variable();
	    }
  		if ( formwork_identify_device == 1 )
		{
		//	if( ms_scan_counter > 2 )	
			{
				if((rec1_buf[rec1_ind_w-1]==0xA)&&(rec1_buf[rec1_ind_w-2]==0xD) && (rec1_ind_w>=7))
				{			
					if (((rec1_buf[rec1_ind_w-7]=='D')&&(rec1_buf[rec1_ind_w-6]=='H'))||((rec1_buf[rec1_ind_w-7]==0x39)&&(rec1_buf[rec1_ind_w-6]==0x37)))
					{
						i= (rec1_buf[rec1_ind_w-5]-0x30)*100+(rec1_buf[rec1_ind_w-4]-0x30)*10+(rec1_buf[rec1_ind_w-3]-0x30);
						  while(i>999)
						  {
							  i-=999;
						  }
						  serail_number = i;				
					}
					rec1_ind_w = 0;	
					rec1_ind_r = 0;	
					rec1_buf[0] = 0;

					//if(serail_number!=0)
					//{
					//	SUM =1;
					//	pattern_alarm_flag =1;
	    			//	pattern_alarm_counter =0;
					//}
				}			
		    else
			    serail_number = 0;
				
	    	
				ms_scan_counter = 0;
			}
		}
		else if( formwork_identify_device == 2)
		{
			if( ms_scan_counter > 150 )
			{	
				RFID_SCAN();
				ms_scan_counter = 0;
			}
		}
		if( (formwork_identify_device == 0)||(auto_function_flag == 0))
		    serail_number = 0;
			
		if( (formwork_identify_device != 0)&&(auto_function_flag == 1) && (return_from_setout==1) )
		    pattern_process();

}


#define VAL2ASC(VAL) ((VAL)>=10?(VAL)+('A'-10):(VAL)+'0')
void printf_uart(const unsigned char* p,...)
{
  int data;
  int data_5,data_4,data_3,data_2,data_1,data_r;
  int index; 
  int *arg = ((int *)&p); // argument
  arg+=2;
  index  = 0;
  while(*p != 0)
  {
    switch(*p)
    {
      case '%':
      if(p[1]=='x')
      {
         	data = *arg++;
         	send1_command[index++] = VAL2ASC((data>>12)&0xf);
         	send1_command[index++] = VAL2ASC((data>>8)&0xf);
         	send1_command[index++] = VAL2ASC((data>>4)&0xf);
         	send1_command[index++] = VAL2ASC((data)&0xf);
         	p+=2;
      }
      else if(p[1]=='d')
      {
        	data = *arg++;
			if( data <0 )
			{
			    data = -data;
				send1_command[index++] = '-';
			}
	        data_5 = data/10000;
	        data_r = data%10000;
	        data_4 = data_r/1000;
	        data_r = data_r%1000;
	        data_3 = data_r/100;
	        data_r = data_r%100; 
	        data_2 = data_r/10;
	        data_1 = data_r%10;

	        if( data_5 > 0)
	          	send1_command[index++] = VAL2ASC(data_5);
	        if( data_4 >=0)
	          	send1_command[index++] = VAL2ASC(data_4);
	        if( data_3 >= 0)
	          	send1_command[index++] = VAL2ASC(data_3);
	        if( data_2 >=0) 
	        	send1_command[index++] = VAL2ASC(data_2);
				
	        send1_command[index++] = VAL2ASC(data_1);
	        p+=2;

      }
      else  if(p[1]=='l')
      {
        	data = arg[1];

        	send1_command[index++] = VAL2ASC((data>>12)&0xf);
        	send1_command[index++] = VAL2ASC((data>>8)&0xf);
        	send1_command[index++] = VAL2ASC((data>>4)&0xf);
        	send1_command[index++] = VAL2ASC((data)&0xf);
            
			data = arg[0];
			send1_command[index++] = VAL2ASC((data>>12)&0xf);
        	send1_command[index++] = VAL2ASC((data>>8)&0xf);
        	send1_command[index++] = VAL2ASC((data>>4)&0xf);
        	send1_command[index++] = VAL2ASC((data)&0xf);
            arg += 2;
        	p   += 2;
      }
	  break;
      default:
        	send1_command[index++] = *p;
             p++;
      break;
    }
  }
  tra1_com(send1_command,index); 
  delay_ms(100);
}


#if DEBUG_PARA_OUTPUT
void show_para_infoformation(void)
{
	printf_uart("==========(show parameter)==========\n");
	printf_uart("DSP1_para_1F=0x%x \n",para.DSP1_para_1F);
	printf_uart("DSP1_para_20=0x%x \n",para.DSP1_para_20);
	printf_uart("DSP1_para_21=0x%x \n",para.DSP1_para_21);
	printf_uart("DSP1_para_22=0x%x \n",para.DSP1_para_22);
	printf_uart("DSP1_para_23=0x%x \n",para.DSP1_para_23);
	printf_uart("DSP1_para_26=0x%x \n",para.DSP1_para_26);
	printf_uart("DSP1_para_27=0x%x \n",para.DSP1_para_27);
	printf_uart("DSP1_para_28H=0x%x \n",para.DSP1_para_28H);
	printf_uart("DSP1_para_28M1=0x%x \n",para.DSP1_para_28M1);
	printf_uart("DSP1_para_28M2=0x%x \n",para.DSP1_para_28M2);	
	printf_uart("DSP1_para_28L=0x%x \n",para.DSP1_para_28L);

	printf_uart("DSP2_para_1F=0x%x \n",para.DSP2_para_1F);
	printf_uart("DSP2_para_20=0x%x \n",para.DSP2_para_20);
	printf_uart("DSP2_para_21=0x%x \n",para.DSP2_para_21);
	printf_uart("DSP2_para_22=0x%x \n",para.DSP2_para_22);
	printf_uart("DSP2_para_23=0x%x \n",para.DSP2_para_23);
	printf_uart("DSP2_para_27=0x%x \n",para.DSP2_para_27);
	printf_uart("DSP2_para_28H=0x%x \n",para.DSP2_para_28H);
	printf_uart("DSP2_para_28M1=0x%x \n",para.DSP2_para_28M1);
	printf_uart("DSP2_para_28M2=0x%x \n",para.DSP2_para_28M2);	
	printf_uart("DSP2_para_28L=0x%x \n",para.DSP2_para_28L);
	
	printf_uart("dsp1A_half_current=0x%x \n",para.dsp1A_half_current);
	printf_uart("dsp1B_half_current=0x%x \n",para.dsp1B_half_current);
	printf_uart("dsp2A_half_current=0x%x \n",para.dsp2A_half_current);
	printf_uart("dsp2B_half_current=0x%x \n",para.dsp2B_half_current);

	printf_uart("platform_type=0x%x \n",para.platform_type);
	printf_uart("mainmotor_type=0x%x \n",para.mainmotor_type);
	printf_uart("x_origin_mode=0x%x \n",para.x_origin_mode);
	printf_uart("yj_org_direction=0x%x \n",para.yj_org_direction);
	
	printf_uart("Corner_deceleration_speed=0x%x \n",para.Corner_deceleration_speed);
	
}

#endif


void uart1_send_char(UINT8 ch)
{
	ti_u1c1 = 0;
	u1tb = ch;
	te_u1c1 = 1;
	while( !ti_u1c1)
		rec_com();
	ti_u1c1 = 0;
}

UINT8 uart1_get_char(void)
{
	counter_1ms = 0;
	while (ri_u1c1 == 0)
	{
		if( counter_1ms >1000)
		{
			sys.status = ERROR;
			StatusChangeLatch = ERROR;
			if( sys.error == 0)
	      	    sys.error = ERROR_97;
			break;
		}
	  	rec_com();
	}
	ri_u1c1 = 0;  
	return (UINT8)u1rb;
}
/////////////////////////////////////////////////////////////////////
//功  能：读RC632寄存器
//参数说明：Address[IN]:寄存器地址
//返  回：读出的值
//94 14 28 94  --0x14
/////////////////////////////////////////////////////////////////////
UINT8 ReadRawRC(UINT8 Address)
{
 	 UINT8 ucAddr,i=0;
	 ucAddr =  Address | 0x80;//最高位＝1 表示读
	 uart1_send_char(ucAddr);
	 return uart1_get_char();

} 

/////////////////////////////////////////////////////////////////////
//功  能：写RC632寄存器
//参数说明：Address[IN]:寄存器地址
//      value[IN]:写入的值
/////////////////////////////////////////////////////////////////////
void WriteRawRC(UINT8 Address, UINT8 value)
{
   UINT8 ucAddr,i ;   
   ucAddr = Address&0x7F;
   uart1_send_char(ucAddr);
   uart1_send_char(value);
   i = uart1_get_char();
}


//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xindahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
