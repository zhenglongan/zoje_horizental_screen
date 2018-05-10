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

	  for(i=0;i<UART1_BUF_MIN;i++)
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
  	volatile UINT16 i= 300;
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

void uart1_rec_int(void)
{
//rec1_buf[rec1_ind_w++] = (UINT8)u1rb; 
  if(rec1_ind_w<299)
  	rec1_buf[rec1_ind_w++] = (UINT8)u1rb;
  else  
  {  
     rec1_ind_w = 0; 
     rec1_buf[rec1_ind_w] = (UINT8)u1rb;
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
			if((rec1_buf[rec1_ind_w-1]==0xA)&&(rec1_buf[rec1_ind_w-2]==0xD) && rec1_ind_w>2)
			{			
				if (((rec1_buf[0]=='D')&&(rec1_buf[1]=='H')))
				{
					  i= (rec1_buf[rec1_ind_r +2]-0x30)*100+(rec1_buf[rec1_ind_r +3]-0x30)*10+(rec1_buf[rec1_ind_r +4]-0x30);
					  while(i>999)
					  {
						  i-=999;
					  }
					  serail_number = i;				
				}
				else if(rec1_buf[0] == 'A' && rec1_buf[rec1_ind_w-3] == 'A')
				{
					 i= (rec1_buf[1]-0x30)*100+(rec1_buf[2]-0x30)*10+(rec1_buf[3]-0x30);
					 while(i>999)
					 {
						i-=999;
					  }
					 serail_number = i;	
				}		
				rec1_ind_w = 0;	
				rec1_ind_r = 0;	
				rec1_buf[0] = 0;
			}
		    else
			    serail_number = 0;
		}
		else if( formwork_identify_device == 2)
		{
			if( ms_scan_counter > 500 )
			{	
				RFID_SCAN();
				ms_scan_counter = 0;
			}
		}
		if( (formwork_identify_device == 0)||(auto_function_flag == 0))
		    serail_number = 0;
			//
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

void process_uart1_download(void)
{
  	UINT16 temp;
  	UINT16 i,j,crc_temp;
	UINT8 port,write_read,package_no;
  
	if( comm1_status == 0)
	{
			set_uart1_debug_mode();
			comm1_status =1;
			rec1_ind_w = 0;	
			rec1_ind_r = 0;	
			return;
	}
	
    switch( rec1_status_machine)
    {
		 case 0:
			 if( (rec1_ind_w >= 3)&&(rec1_buf[0] == START_FLAG) )
			 {
				rec1_package_length = rec1_buf[1];
				rec1_package_length = (rec1_package_length<<8) + rec1_buf[2];
		 		//da1 = 100;
				rec1_status_machine = 1;
			 }	
			 else if( rec1_buf[0] != START_FLAG )
			 {

				rec1_ind_w = 0;
				rec1_ind_r = 0;
				//da1 = 50;
			}

		 break;
		 
		 case 1:
		 	if ( rec1_ind_w >= rec1_package_length -1 )
			{
				//da1 = 150;
				rec1_status_machine = 0;
				switch(rec1_buf[3])
				{
					case WRITESYSPARAM_RX:
						 //WRITESYSPARAM_TX
					break;
					case WRITESTEPPARAM_RX:
						  write_read = rec1_buf[4];
						  port       = rec1_buf[5];
						  package_no = rec1_buf[6];									
						  recpat_point = (UINT8 *)&(rec1_buf + 7);
						  temp = 240*(UINT16)(rec1_buf[6]);
						  
						  for(i=0;i< rec1_package_length - 8;i++)
						  {
							  pat_buf[ temp + i] = *recpat_point;
							  recpat_point++;
						  }
						  
						  if( rec1_package_length - 8 < 240 )
						  {
							  SUM = 1;
						      delay_ms(100);
						      SUM = 0;
							  if( verify_rec(pat_buf,rec1_package_length-8) == 0)
							  {
								  write_stepmotor_config_para(port + 1, pat_buf);
								  tra1_ind_r = 0; 
								  tra1_ind_w = 0;
								  send1_command[0] = START_FLAG;
								  send1_command[1] = 0;
								  send1_command[2] = 8;
								  send1_command[3] = WRITESTEPPARAM_TX;
								  send1_command[4] = write_read;
								  send1_command[5] = port;
								  send1_command[6] = package_no;
								  crc_temp         = cal_crc16(send1_command, 7, CRC_CHECK_INIT);
								  send1_command[7] = (UINT8)(crc_temp >> 8);
								  send1_command[8] = (UINT8)(crc_temp);
								  send1_command[9] = END_FLAG;
								  tra1_com(send1_command ,10);
								  SUM = 1;
								  delay_ms(100);
								  SUM = 0;
						      }
							  rec1_ind_w = 0;	
							  rec1_ind_r = 0;	
							  pat_buf[0] = 0;
						  }
					break;
					
					case WRITESTEPCURVE_RX:
						 write_read = rec1_buf[4];
						 port       = rec1_buf[5];
						 package_no = rec1_buf[6];	
						 								
						 recpat_point = (UINT8 *)&(rec1_buf + 7);  //数据
						 temp = 240*(UINT16)(rec1_buf[6]);		   //包号＝》存储地址
						 for( i=0;i< rec1_package_length - 8;i++)
						 {
							 pat_buf[ temp + i] = *recpat_point;
							 recpat_point++;
							 rec1_datalength++;
						 }
						 rec1_ind_w = 0;
				
						 //da1 = 200;
						 delay_ms(500);						 
						 
						 tra1_ind_r = 0; 
						 tra1_ind_w = 0;
						 send1_command[0] = START_FLAG;
						 send1_command[1] = 0;
						 send1_command[2] = 8;
						 send1_command[3] = WRITESTEPCURVE_TX;
						 send1_command[4] = write_read;
						 send1_command[5] = port;
						 send1_command[6] = package_no;
						 crc_temp         = cal_crc16(send1_command, 7, CRC_CHECK_INIT);
						 send1_command[7] = (UINT8)(crc_temp >> 8);
						 send1_command[8] = (UINT8)(crc_temp);
						 send1_command[9] = END_FLAG;
						 tra1_com(send1_command ,10);						 
						 rec1_ind_r = 0;	 
					     
						 
						 if( package_no >= 29 )
						 {
							SUM = 1;
						    delay_ms(100);
						    SUM = 0;
							if( verify_rec(pat_buf,rec1_datalength) == 0)
							{
								delay_ms(100);
								SUM = 1;
								delay_ms(200);
								SUM = 0;
								write_stepmotor_curve_flag = port + 1;	
								rec1_datalength = 0;						  						  
							}
						 }
					break;
					
				}
			}
		 break;	  
	  }			
	  //if( rec1_ind_w == 0 )
	  //  da1 = 0;

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
	printf_uart("DSP2_para_26=0x%x \n",para.DSP2_para_26);
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

/////////////////////////////////////////////////////////////////////
//功  能：读RC632寄存器
//参数说明：Address[IN]:寄存器地址
//返  回：读出的值
//94 14 28 94  --0x14
/////////////////////////////////////////////////////////////////////
UINT8 ReadRawRC(UINT8 Address)
{
 	 UINT8 ucAddr,i=0;
	 ucAddr =  Address | 0x80;
	 tra1_ind_r = 0; 
	 tra1_ind_w = 0;
	 send1_command[0] = ucAddr;
	 send1_command[1] = 0;
	 tra1_com(send1_command ,1);						 
	 rec1_ind_r = 0;
	 rec1_ind_w = 0;
	 counter_1ms = 0;
	 while( rec1_ind_w == 0)
	 { 	
		 rec_com();	 
		 if(counter_1ms > 100)
		 {
			 i++;
			 counter_1ms = 0;
			 tra1_ind_r = 0; 
			 tra1_ind_w = 0;
			 send1_command[0] = ucAddr;
			 send1_command[1] = 0;
			 tra1_com(send1_command ,1);						 
			 rec1_ind_r = 0;
			 rec1_ind_w = 0;
		}
		if( i > 5 )
		{
			sys.status = ERROR;
			StatusChangeLatch = ERROR;
			if( sys.error == 0)
	      	    sys.error = ERROR_97;
			break; 	
		}
	 }	
	 //T_HALF = 0; //OUT4
     return rec1_buf[0];
}

/////////////////////////////////////////////////////////////////////
//功  能：写RC632寄存器
//参数说明：Address[IN]:寄存器地址
//      value[IN]:写入的值
/////////////////////////////////////////////////////////////////////
void WriteRawRC(UINT8 Address, UINT8 value)
{
   UINT8 ucAddr,i=0;
   ucAddr = Address&0x7F;
   tra1_ind_r = 0; 
   tra1_ind_w = 0;
   rec1_ind_w = 0;
   send1_command[0] = ucAddr;
   tra1_com(send1_command ,1);						 
   rec1_ind_r = 0;
   counter_1ms =0;
   while( rec1_ind_w == 0)
   {
		rec_com();
		if(counter_1ms > 100)
		{
			i++;
			counter_1ms = 0;
			tra1_ind_r = 0; 
   			tra1_ind_w = 0;
			send1_command[0] = ucAddr;
			tra1_com(send1_command ,1);	
			rec1_ind_r = 0;	
		}
		if( i > 5 )
		{
			sys.status = ERROR;
			StatusChangeLatch = ERROR;
			if( sys.error == 0)
	      	    sys.error = ERROR_97;
			break; 	
		}		
   }
   //FK_OFF =0; //OUT3	 	
   tra1_ind_r = 0; 
   tra1_ind_w = 0;
   send1_command[0] = value;
   tra1_com(send1_command ,1);						 
   rec1_ind_r = 0;
}


//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xindahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
