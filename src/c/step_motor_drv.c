/*
--------------------------------------------------------------------------------------
      COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
                     ALL RIGHTS RESERVED 
  Project Number: sewing_machine_controller 
  File Name : stepmotor.c
  Description: stepmotor control 
  Version     Date     Author    Description
--------------------------------------------------------------------------------------
protocol :
	0x0001 -- Write control parameter to  Moter driver
	0x0002 -- Read control parameter from Moter driver
	0x0006 -- Read moter driver version
	0x0011 -- Config motor current
*/

#include "..\..\include\sfr62p.h"       //M16C/62P special function register definitions
#include "..\..\include\typedef.h"      //Data type define
#include "..\..\include\variables.h"    //External variables declaration
#include "..\..\include\common.h"       //Common constants definition
#include "..\..\include\delay.h"        //delay time definition
#include "..\..\include\iic_bus_eeprom.h"

static UINT16 x_data_nf;
static UINT16 y_data_nf;
static UINT16 yj_data_nf;
static UINT16 zx_data_nf;
static UINT16 step_cfg_data;
static UINT16 trans_dsp1;
static UINT16 trans_dsp2;
static UINT8 dsp1;
static UINT8 dsp2;
//--------------------------------------------------------------------------------------
//  global variables declaration
//--------------------------------------------------------------------------------------
union TRANS trans_x;
union TRANS trans_y;
union TRANS trans_z;
union RECV recieve_x;
union RECV recieve_y;
union RECV recieve_z;
//--------------------------------------------------------------------------------------
//  Internal functions and subroutines declaration
//--------------------------------------------------------------------------------------
void init_stepmotor_drv(void);
void set_openorclose_loop(UINT8 set);
void send_dsp_command(UINT8 port,UINT16 command);
void send_dsp1_command(UINT16 command,UINT16 data);
void send_dsp2_command(UINT16 command,UINT16 data);
UINT16 read_stepmotor_curve_crc(UINT8 port);

#pragma	INTERRUPT/E spiint
void spiint(void);

/*
��5λ��ʾ��������������ֵ����i+1)*0.3A; ��3λK��ʾ���ֵ��������ֵ���ֵ����������*��k+1)/8
����������忪�����16��������������λ���32�����������һ����Ӧ����������0.6AΪ�ɵ��ڵ�λ
������ʱΪ����������һ��

*/


//--------------------------------------------------------------------------------------
//  Name:		 SPI_init
//  pars:	     None
//  Returns:	 None
//  Description: initial SPI
//--------------------------------------------------------------------------------------
void SPI_init(void)
{
	SPISTE1 = 1;     // DSP1 SPI disable 
	SPISTE2 = 1;     // DSP2 SPI disable 	
	prc2 = 1;        // protect disable
	pd9_5 = 1;       // set SPI CLK  pin to output
	pd9_7 = 0;       // set SPI SIN  pin to input
	pd9_6 = 1;       // set SPI SOUT pin to output
	prc2 = 0;        // protect enable	
	p9_5 = 0;	     // leave CLK low	
	prc2 = 1;        // protect disable
	s4c = 0x68;      // internal clock, MSB, CLK, f1SIO 
  	s4brg = 0x0B;    // Divide clock by 12(0x0B)---1M    40(0x27)---300K
	prc2 = 0;        // protect enable	
	ir_s4ic = 0;	 // clear the ir bit
	s4ic = SPI_IPL;  // 7 level
	ifsr = 0;        // interrupt source select
}
//--------------------------------------------------------------------------------------
//  Name:		spiint
//  pars:	    None
//  Returns:	None
//  Description: SPI interrupt
//--------------------------------------------------------------------------------------
void check_dsp_error(UINT16 error_code)
{
	switch(error_code)
	{
		case OVC_DSP1:
			if( dsp1 == 1)
				sys.error = ERROR_50;//X�������	
			else
				sys.error = ERROR_94;//���ߵ������
		break;

		case OVD_DSP1:
		    if( dsp1 == 1)
				sys.error = ERROR_54;//X�������
			else	
				sys.error = ERROR_96;//���ߵ������	
		break;					
		case OVC_DSP2:
		    if( dsp1 == 1)
				sys.error = ERROR_51;//Y�������	
			else
				sys.error = ERROR_93;//��ѹ�ŵ������
		break;
		case OVD_DSP2:
		    if( dsp1 == 1)
				sys.error = ERROR_55;//Y�������	
			else
				sys.error = ERROR_95;//��ѹ�ŵ������	
		break;
		case SPI_RX_ERR_CHK:
			if( dsp1 == 1)
				sys.error = ERROR_59;//�ŷ�ͨѶ����1
			else
				sys.error = ERROR_61;//�ŷ�ͨѶ����3
		break;
		case SPI_RX_ERR_ILLG:
			if( dsp1 == 1)
				sys.error = ERROR_60;//�ŷ�ͨѶ����2
			else
				sys.error = ERROR_62;//�ŷ�ͨѶ����3
		break;
		/*
		default:
		    if( dsp1 == 1)
			{
				err_num_dsp1++;
				dsp1_message = error_code;
			}
			else
			{
				err_num_dsp2++;
				dsp2_message = error_code;
			}
		break;
		*/
	}
}


void spiint(void)
{
	if(sys.status == POWEROFF)  
		return;
	
	switch(spi_flag)
	{
 	  	case 6:         
	  	{
		  	ir_int5ic=0;
		  	recieve_z.byte.byte2= s4trr;
			SPISTE1=1;
		  	SPISTE2=1;
			
		  	if((dsp1==1)&&(recieve_z.word != trans_dsp1))
	  		{
	  			check_dsp_error(recieve_z.word);		
	  		}
			
		  	if((dsp2==1)&&(recieve_z.word != trans_dsp2))
	  		{
				check_dsp_error(recieve_z.word);
	  		}
			
			if(err_num_dsp1 >=1 )
			{
				if( sys.error == 0) 
					sys.error = ERROR_79;		
			}
			if(err_num_dsp2 >=1 )
			{
				if( sys.error == 0) 
					sys.error = ERROR_30;		
			}	
			spi_flag=0;
			dsp1=0;
		  	dsp2=0;	
			err_num_dsp1 = 0;
			err_num_dsp2 = 0;	
			if( sys.error != 0)
			{
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
			}
	  		break;
	  	}
    	//--------------------------------------------------------------------------------------
    	//  spi_flag==5
    	//--------------------------------------------------------------------------------------		
    	case 5:
    	{
	    	ir_int5ic = 0;
	    	recieve_z.byte.byte1=s4trr;
	    	s4trr=trans_z.byte.byte2;
	    	spi_flag = 6;
	    	break;
    	}
    	//--------------------------------------------------------------------------------------
    	//  spi_flag==4
    	//--------------------------------------------------------------------------------------		
    	case 4:         
    	{
	    	ir_int5ic=0;
	    	recieve_y.byte.byte2= s4trr; 
			            	
	    	if((dsp1==1)&&(recieve_y.word!=trans_dsp1))
    		{
				check_dsp_error(recieve_y.word);						    				
    		}
	    	if((dsp2==1)&&(recieve_y.word!=trans_dsp2))
    		{
				check_dsp_error(recieve_y.word);			
    		}
			
	    	if(dsp1==1)
	    	{
    			trans_dsp1=trans_y.word;
	    	}
	    	if(dsp2==1)
	    	{
    			trans_dsp2=trans_y.word;    		
    		}    	
	    	s4trr=trans_z.byte.byte1;									
	    	spi_flag=5;
	    	break;
    	}
    	//--------------------------------------------------------------------------------------
    	//  spi_flag==3
    	//--------------------------------------------------------------------------------------		
    	case 3:
    	{
	    	ir_int5ic=0;
	    	recieve_y.byte.byte1=s4trr;
	    	s4trr=trans_y.byte.byte2;
	    	spi_flag=4;
	    	break;
    	}
    	//--------------------------------------------------------------------------------------
    	//  spi_flag==2
    	//--------------------------------------------------------------------------------------		
    	case 2:        
    	{
	    	ir_int5ic=0;
	    	recieve_x.byte.byte2= s4trr;
			if( (dsp1 == 1) && (recieve_x.word == 0xFFFF) )
			{
				err_num_dsp1 ++;
			}
			else if( (dsp2 == 1) && (recieve_x.word == 0xFFFF) )
			{
				err_num_dsp2 ++;
			}
   	
	    	if(dsp1==1)
	    	{
    			trans_dsp1=trans_x.word;    		
    		}
	    	if(dsp2==1)
	    	{
    			trans_dsp2=trans_x.word;    		
    		}
    		s4trr=trans_y.byte.byte1;	    										
    		spi_flag=3;
    		break;
    	}
    	//--------------------------------------------------------------------------------------
    	//  spi_flag==1
    	//--------------------------------------------------------------------------------------		
    	case 1:
    	{
	    	ir_int5ic=0;
	    	recieve_x.byte.byte1=s4trr;
	    	s4trr=trans_x.byte.byte2;		    	
	    	spi_flag=2;
	    	break;
    	}
  	}
	
}
//--------------------------------------------------------------------------------------
//  Name:		init_stepmotor_drv
//  pars:	    None
//  Returns:	None
//  Description: initial step motor drv
//--------------------------------------------------------------------------------------
void init_stepmotor_drv(void)
{	
	x_data_nf = 0;
	y_data_nf = 0;	
	yj_data_nf = 0; 
	zx_data_nf = 0; 
	
	SPI_init();
	spi_flag = 0;
	trans_dsp1 = 0x0d155;
	trans_dsp2 = 0x0d255;
	dsp1 = 0;
	dsp2 = 0;
	
}


						 
//--------------------------------------------------------------------------------------
//  Name:	     version_check                
//  Parameters:	 None
//  Returns:	 None
//  Description: check the step soft version 
//--------------------------------------------------------------------------------------
void version_check(void)
{
	send_dsp2_command(0x0001,0x5555);  
	if(recieve_x.word == 0x5555)
	{
		send_dsp2_command(0x0006,0x5555);
	}  
	stepversion2 = recieve_x.word;
	delay_ms(100);		
	send_dsp1_command(0x0001,0x5555);
	if(recieve_x.word == 0x5555)
	{
		send_dsp1_command(0x0006,0x5555);
	}  
	stepversion1 = recieve_x.word;	
				
}
//--------------------------------------------------------------------------------------
//  Name:	     movestep_x
//  Parameters:	 None
//  Returns:	 None
//  Description: move x step motor 
//--------------------------------------------------------------------------------------

void movestep_x(int x_data)
{ 

	while( spi_flag > 0 );  			//��ֹ����

	if( (sys.status == ERROR) || (x_data == 0) )//������0���ݲ���������
	 	return;
		
	if( movestepx_time > fabsm(x_data)*20 )
	 	movestepx_time =fabsm(x_data)*20;
	if( movestepx_time > 63 )	    //Э��ʱ�����޶�
		movestepx_time = 63 ;
		
	if(x_data>0)
	{
			if( x_motor_dir == 0)		
			   trans_x.word=(UINT16)0x0000+((UINT16)x_data<<6)+(UINT16)movestepx_time;
			else
			   trans_x.word=(UINT16)0x4000+((UINT16)x_data<<6)+(UINT16)movestepx_time;
	}	
	else 
	{
		x_data_nf=-x_data;		
		if( x_motor_dir == 0)	
		     trans_x.word=(UINT16)0x4000+((UINT16)x_data_nf<<6)+(UINT16)movestepx_time;		
		else
		     trans_x.word=(UINT16)0x0000+((UINT16)x_data_nf<<6)+(UINT16)movestepx_time;	
	}
	//
	spi_flag=1;	 
	if( trans_x.word == 0x5555 )
	    trans_x.word = trans_x.word + 1;
	trans_y.word=(~trans_x.word)&0x7fff;
	//trans_z.word = 0x5500 | (motor.spd_obj/100);
	trans_z.word = 0x5555;
	dsp1 = 1;
	SPISTE1=0;                                                            
	s4trr=trans_x.byte.byte1;  	

}
//--------------------------------------------------------------------------------------
//  Name:	     movestep_y
//  Parameters:	 None
//  Returns:	 None
//  Description: move y step motor 
//--------------------------------------------------------------------------------------

void movestep_y(int y_data)
{ 
	while( spi_flag > 0 ); 	
	 
	if( (sys.status == ERROR) || (y_data == 0) )
	 	return;
	if( movestepy_time > fabsm(y_data)*20)
		movestepy_time = fabsm(y_data)*20;
		
	if( movestepy_time > 63 )
		movestepy_time = 63 ;
			 
	if(y_data>0)
	{
		if( y_motor_dir == 0)		
		     trans_x.word=(UINT16)0x8000+((UINT16)y_data<<6)+(UINT16)movestepy_time;
		else
		     trans_x.word=(UINT16)0xC000+((UINT16)y_data<<6)+(UINT16)movestepy_time;
	}	
	else 
	{
		y_data_nf=-y_data;
			
		if( y_motor_dir == 0)	
		     trans_x.word=(UINT16)0xC000+((UINT16)y_data_nf<<6)+(UINT16)movestepy_time;	
        else			 
		     trans_x.word=(UINT16)0x8000+((UINT16)y_data_nf<<6)+(UINT16)movestepy_time;
	}
	spi_flag=1;
	if(trans_x.word == 0x5555 )
	   trans_x.word = trans_x.word + 1;
	trans_y.word=(~trans_x.word)&0x7fff;
	//trans_z.word = 0x5500 | (motor.spd_obj/100);
	trans_z.word = 0x5555;
	dsp1 = 1;
	SPISTE1=0; 
	s4trr=trans_x.byte.byte1; 

}
//--------------------------------------------------------------------------------------
//  Name:	     movestep_zx
//  Parameters:	 None
//  Returns:	 None
//  Description: move zx step motor 
//--------------------------------------------------------------------------------------
void movestep_zx(int zx_data,UINT16 time)
{
	 if(spi_flag > 0)
		delay_us(400);
	if(spi_flag > 0)
	{
		if( sys.error == 0)
	   	    sys.error = ERROR_30;	
		sys.status = ERROR;
	    StatusChangeLatch = ERROR;
		spi_flag=0;
		dsp1=0;
		dsp2=0;
	}
	if( time > fabsm(zx_data)*20 )
	 	time =fabsm(zx_data)*20;
	if( time >63)
	    time = 63;
		
	if(zx_data>0)
	{
		spi_flag=1;
		if( zx_data > 255)
		 	zx_data = 255;
		if( z_motor_dir == 0)
			trans_x.word=(UINT16)0xc000+((UINT16)zx_data<<6)+(UINT16)time;         
		else
			trans_x.word=(UINT16)0x8000+((UINT16)zx_data<<6)+(UINT16)time;	   
		if( trans_x.word  == 0x5555 )
		{
			trans_x.word += 1;
		}
		trans_y.word=(~trans_x.word)&0x7fff;
		trans_z.word=0x5555;
		dsp2 = 1;
		SPISTE2=0;                                                               
		s4trr=trans_x.byte.byte1;                                              
	}	                                                                      
	else if(zx_data<0)
	{                                                                       
		zx_data_nf = -zx_data;                                                  
		spi_flag=1;	
		if( zx_data_nf > 255)
		 	zx_data_nf = 255;
		if( z_motor_dir == 0)
			trans_x.word=(UINT16)0x8000+((UINT16)zx_data_nf<<6)+(UINT16)time;         
		else
			trans_x.word=(UINT16)0xc000+((UINT16)zx_data_nf<<6)+(UINT16)time;
		if( trans_x.word  == 0x5555)
		{
			trans_x.word += 1;
		}
		trans_y.word=(~trans_x.word)&0x7fff;
		trans_z.word=0x5555;
		dsp2 = 1;
		SPISTE2=0;		                         
		s4trr=trans_x.byte.byte1;  
	}	
}


//--------------------------------------------------------------------------------------
//  Name:	     movestep_yj
//  Parameters:	 None
//  Returns:	 None
//  Description: move yj step motor 
//--------------------------------------------------------------------------------------
void movestep_yj(int zx_data,UINT16 time)
{
	while(spi_flag > 0) ;

	if( time >63)
	    time = 63;	
	if(zx_data>0)
	{
		spi_flag=1;
		
		if( para.yj_org_direction == 0)
			trans_x.word=(UINT16)0x4000+((UINT16)zx_data <<6 )+(UINT16)time;         
		else
			trans_x.word=(UINT16)0x0000+((UINT16)zx_data <<6 )+(UINT16)time;	   
		if( trans_x.word  == 0x5555) 
		{
			trans_x.word += 1;
		}
		trans_y.word=(~trans_x.word)&0x7fff;
		trans_z.word=0x5555;
		dsp2 = 1;
		SPISTE2=0;                                                               
		s4trr=trans_x.byte.byte1;                                              
	}	                                                                      
	else if(zx_data < 0)
	{                                                                       
		zx_data_nf = -zx_data;                                                  
		spi_flag = 1;	
		
		if( para.yj_org_direction == 0)
			trans_x.word=(UINT16)0x0000+((UINT16)zx_data_nf<<6)+(UINT16)time;         
		else
			trans_x.word=(UINT16)0x4000+((UINT16)zx_data_nf<<6)+(UINT16)time;
		if( trans_x.word  == 0x5555 ) 
		{
			trans_x.word += 1;
		}
		trans_y.word=(~trans_x.word)&0x7fff;
		trans_z.word=0x5555;
		dsp2 = 1;
		SPISTE2=0;		                         
		s4trr=trans_x.byte.byte1;  
	}	
}


//======================================================
UINT16 get_x_distance(void)
{
	UINT16 tmp;
	send_dsp1_command(0x0007,0x5555);  
	tmp = recieve_x.word;
	return tmp;	  
}

UINT16 get_y_distance(void)
{
	UINT16 tmp;
	send_dsp1_command(0x0008,0x5555);  
	tmp = recieve_x.word;
	return tmp;
}

void nop_move_emergency(UINT16 x, UINT16 y)
{
	while(spi_flag > 0);

	if( x > QUICKMOVE_JUDGEMENT )//
	{
		send_dsp_command(1,0x0005);
		delay_ms(2);
	}
	if(  y > QUICKMOVE_JUDGEMENT )//
	{
		send_dsp_command(1,0x0009);
		delay_ms(2);
	}
}
void ready_dsp1_time(void)
{ 	
	spi_flag=1;				
	trans_x.word=(UINT16)0x0003;	      
	trans_y.word=(~trans_x.word)&0x7fff;
	trans_z.word=0x5555;
	dsp1 = 1;
	SPISTE1=0;                                                       
	s4trr=trans_x.byte.byte1;                                            
}

void stepmotor_para(void)    //0-dsp1 1-dsp2
{
	UINT8 k;
	send_dsp_command(operate_dsp1ordsp2_flag+1,0x000A);
 
	for(k=0;k<93;k++)
	{
	    send_dsp_command(operate_dsp1ordsp2_flag + 1, ( (UINT16)k<<8 ) + (UINT16)svpara_buf[k]);
	}

} 
void read_stepmotor_para(void)
{
    UINT8 m;	
	for(m=0;m<93;m++)
	{
		if( operate_dsp1ordsp2_flag == 0)//dsp1
			send_dsp1_command(0x0002,0x5555);
		else
		    send_dsp2_command(0x0002,0x5555);
		//svpara_disp_buf[m] = recieve_x.word;
	}
}

//return 0x1234 ---ok 
UINT8 check_motion_done(void)
{
		UINT8 ret;
		send_dsp1_command(0x0012,0x5555);
		if( recieve_x.word == 0x1234 )
		    ret =1;
		else
		    ret = 0;
		return ret;								
}

void send_stepmotor_up_drv(void)    
{
	 UINT8 k;
	 while(spi_flag > 0);
	 
	 send_dsp_command(download_drv_flag,0x00F0);	    
     delay_ms(1);
	 for(k=0;k<data_length_drv+2;k++)
	 {
		send_dsp_command(download_drv_flag,((UINT16)k<<8) + pat_buf[k]);
		rec_com();
	}
} 


void send_stepmotor_end_drv(void)    
{
	 UINT8 k;
	 while(spi_flag > 0);
	 send_dsp_command(download_drv_flag,0x00FF);	 
	 delay_ms(2);
} 
/*
�ض�����״̬
*/
UINT16 read_stepmotor_up_drv(void)
{
	    UINT8 m;
		if( download_drv_flag==1)
	 	{
			send_dsp1_command(0x0010,0x5555);
		}
		else
		{
			send_dsp2_command(0x0010,0x5555);
		}
		return (UINT16)recieve_x.word;
}
/*
����������״̬�£��ɲ�������л�״̬������
֪ͨ����׼����������״̬
*/
void jump_to_begin(void)
{
    if((1==download_drv_flag)&&(stepversion1<60000))
	{
		while(spi_flag > 0);
		send_dsp_command(1,0x000F);
		delay_ms(1000);
	}
	if((2==download_drv_flag)&&(stepversion2<60000))
	{
	    while(spi_flag > 0);
		send_dsp_command(2,0x000F);
		delay_ms(1000);
	}
}

void send_dsp_command(UINT8 port,UINT16 command)
{
	 while(spi_flag > 0);	     
	 spi_flag=1;				
	 trans_x.word= command;      
	 trans_y.word= (~trans_x.word)&0x7fff;
	 trans_z.word=0x5555;
	 if( port == 1)
	 {
		 dsp1 = 1;
	 	 SPISTE1 = 0;
	 }
	 else
	 {
		 dsp2 = 1;
	 	 SPISTE2 = 0;
	 }
	 s4trr=trans_x.byte.byte1; 
	 delay_us(500);
}
void send_dsp1_command(UINT16 command,UINT16 data)    
{
	 send_dsp_command(1,command);
	 send_dsp_command(1,data);
	 while(spi_flag > 0);	      
} 
void send_dsp2_command(UINT16 command,UINT16 data)    
{
	 send_dsp_command(2,command);
	 send_dsp_command(2,data);
	 while(spi_flag > 0);
} 
/*
#define DSP_CMD_QUICK_MOVE                  0x0000    // ������� 
#define DSP_CMD_QUERY_VERSION               0x0001    // ��ѯ�����汾 
#define DSP_CMD_READ_SERVO_PARAM            0x0002    // ��ȡ�ŷ����� 
#define DSP_CMD_WRITE_STEP_ALL_CONF_PARAM   0x0003    // д�벽��ȫ�����ò��� 
#define DSP_CMD_READ_STEP_ALL_CONF_PARAM    0x0004    // ��ȡ����ȫ�����ò��� 
#define DSP_CMD_EMERGENCY_STOP1             0x0005    // ��һ·��X�ᣩ��ͣ 
#define DSP_CMD_READ_EMERG_STOP_LEN1        0x0007    // ��ȡ��һ·��X�ᣩ�����ͼ�ͣ���߹��ľ��루����1���� 
#define DSP_CMD_READ_EMERG_STOP_LEN2        0x0008    // ��ȡ�ڶ�·��Y�ᣩ�����ͼ�ͣ���߹��ľ��루����1����
#define DSP_CMD_EMERGENCY_STOP2             0x0009    // �ڶ�·��Y�ᣩ��ͣ 
#define DSP_CMD_WRITE_SERVO_PARAM           0x000A    // дȡ�ŷ����� 
#define DSP_CMD_WRITE_CURVE                 0x000B    // д������������ 
#define DSP_CMD_READ_CURVE_CRC              0x000C    // ����������CRC 
#define DSP_CMD_CONFIG_PLATFORM             0x0011    // ���õ��ƽ̨ 
#define DSP_CMD_QUERY_ACTION_COMPLETED      0x0012    // ��ѯ���������Ƿ����(��ԭ�㡢���ͼ�ͣ�Ϳ��ߵĲ�ѯ) 
#define DSP_CMD_ORIGIN_AND_CURRENT          0x0013    // ������ԭ�㡢ȡ����ָ����� 
#define DSP_CMD_CONFIG_OPEN_CLOSED_MODE     0x001F    // �����ջ��л� 
#define DSP_CMD_CONFIG_ENCODER_LINE1        0x0020    // �趨��һ·��X�ᣩ���������� 
#define DSP_CMD_CONFIG_ENCODER_LINE2        0x0021    // �趨�ڶ�·��Y�ᣩ���������� 
#define DSP_CMD_CONFIG_STEP_ANGLE1          0x0022    // �趨��һ·��X�ᣩ�����ϵ�� 
#define DSP_CMD_CONFIG_STEP_ANGLE2          0x0023    // �趨�ڶ�·��Y�ᣩ�����ϵ�� 
#define DSP_CMD_CONFIG_MOTOR_TYPE1          0x0024    // �趨��һ·��X�ᣩ������ 
#define DSP_CMD_CONFIG_MOTOR_TYPE2          0x0025    // �趨�ڶ�·��Y�ᣩ������ 
#define DSP_CMD_CONFIG_OPENLOOP_CURRENT     0x0026    // �趨��·��������Ĺ��������ͱ��ֵ��� 
#define DSP_CMD_CONFIG_HALF_CURRENT_TIME    0x0027    // �趨�ջ����쵽λ���п���������ʱʱ�� 
#define DSP_CMD_CONFIG_STEP_PARAM           0x0028    // ���ø�·���� 
#define DSP_CMD_QUERY_AND_CLEAR_ENCODER     0x0029    // ��ȡ�����������״̬ 

5����20m��ʱ��((��λ+1)/16)*10.9
4����30m��ʱ��((��λ+1)/16)*7.4		
��λ��0.025mm
1��=0.025*(n+1)mm
�����������0.4mm
*/

void setup_stepper_moter(void)
{
 	UINT16 current1,current2;
	if( para.platform_type == FIFTH_GENERATION )//ƽ̨����	
		send_dsp1_command(0x0011,0x0005);
	else
		send_dsp1_command(0x0011,0x0004);
	
	send_dsp1_command(0x001F,para.DSP1_para_1F);  //�����ջ��л� 1��ʾ�ջ���2��ʾ������3��ʾת��ģʽ��4��ʾ�涯ģʽ��5��ʾ˫��ͬ��
	send_dsp1_command(0x0020,para.DSP1_para_20);  //����������
	send_dsp1_command(0x0021,para.DSP1_para_21);		
	send_dsp1_command(0x0022,para.DSP1_para_22);  //�����
	send_dsp1_command(0x0023,para.DSP1_para_23);
	current1 = x_step_current_level;
	current2 = y_step_current_level;
	current1 = ((current1<<3) + (UINT16)para.dsp1A_half_current)<<8;
	current2 = ((current2<<3) + (UINT16)para.dsp1B_half_current) + current1;
	
	send_dsp1_command(0x0026,current2 );
	send_dsp1_command(0x0027,para.DSP1_para_27);  
	send_dsp1_command(0x0028,para.DSP1_para_28H); //��һ·����	
	send_dsp1_command(para.DSP1_para_28M1,para.DSP1_para_28M2);  //�ڶ�·����+��һ·����ϵ��  X��ȷY�ŷ�
    send_dsp_command(1,para.DSP1_para_28L);
	
	//if( PAUSE == PAUSE_OFF )
	if(!((PAUSE == PAUSE_ON)&&((DVA == para.dvab_open_level)||(DVB == para.dvab_open_level))))
	{
	 	if( para.platform_type == FIFTH_GENERATION )
			send_dsp2_command(0x0011,0X0005);
		else
			send_dsp2_command(0x0011,0X0004);//ƽ̨����		

	 	send_dsp2_command(0x001F,para.DSP2_para_1F);  //�����ջ��л� 1��ʾ�ջ���2��ʾ������3��ʾת��ģʽ��4��ʾ�涯ģʽ��5��ʾ˫��ͬ��
		send_dsp2_command(0x0020,para.DSP2_para_20);  //����������
		send_dsp2_command(0x0021,para.DSP2_para_21);		
		send_dsp2_command(0x0022,para.DSP2_para_22);  //�����
		send_dsp2_command(0x0023,para.DSP2_para_23);  
		current1 = yj_step_current_level;
		current2 = u235;
		current1 = ((current1<<3) + (((UINT16)(para.dsp2A_half_current))&0x0007))<<8;
		current2 = ((current2<<3) + (((UINT16)(para.dsp2B_half_current))&0x0007)) + current1;
		
		send_dsp2_command(0x0026,current2 );
		send_dsp2_command(0x0027,para.DSP2_para_27);  
		send_dsp2_command(0x0028,para.DSP2_para_28H); //��һ·����	
		send_dsp2_command(para.DSP2_para_28M1,para.DSP2_para_28M2);  //�ڶ�·����+��һ·����ϵ��  X��ȷY�ŷ�
	    send_dsp_command(2,para.DSP2_para_28L);
	}
	
}


void write_stepmotor_config_para(UINT8 port,UINT8 *pdata)    
{
	 UINT8 k;
	 send_dsp_command(port,0x0003);//д�벽�����ò���
	 for( k = 0;k < 205; k++ )
	 {
		send_dsp_command(port,((UINT16)k<<8) + (UINT16)pdata[k] );
		//printf_uart("%x,",((UINT16)k<<8) + (UINT16)pdata[k]);
		delay_ms(1);
	 }
} 

void read_stepmotor_config_para(UINT8 port)
{
    UINT8 m;	
	for(m=0; m<205; m++)
	{
		if( port == 1 )//dsp1
		    send_dsp1_command(0x0004,0x5555);
		else
			send_dsp2_command(0x0004,0x5555);
		svpara_disp_buf[m] = recieve_x.word;
		delay_us(1000);
		
	}
}

/*
ֱ�Ӱ�16λд�뵽DSP��
һ��4008�����ݣ�ǰ��1+���ݳ���2+����1+DSP��1+data....+У��2+����1
���ݹ�4000���ֽڣ���2000���֣�ǰ1999���������ݣ�������ֽ���ǰ1999У���
*/

UINT16 crc_calcu(UINT16 far *crc_in, UINT16 length, UINT16 init)
{
  UINT16 crc_i,crc_j,crc_out;
  crc_out = init;
  for(crc_j = 0; crc_j < length; ++crc_j)
  {
      crc_out ^= crc_in[crc_j];
      for (crc_i = 0; crc_i < 16; ++crc_i)
      {
    	  if (crc_out & 1)
              crc_out = (crc_out >> 1) ^ 0x1021;
    	  else
              crc_out = (crc_out >> 1);
      }
  }
  return crc_out;
}

UINT16 read_stepmotor_curve_crc(UINT8 port)
{
	if( port == 1)
	    send_dsp1_command(0x000C,0x5555);//��ȡ�������ߵ�CRCֵ
	else
		send_dsp2_command(0x000C,0x5555);
	return recieve_x.word;
}

UINT8 write_stepmotor_curve(UINT8 port,UINT8 *pdata)    
{
	 UINT16 k,point,val,crc;	 
	 UINT8 ret;
	 crc = crc_calcu((UINT16*)pdata,3499,0XFFFF);
	 
	 send_dsp_command(port,0x000B);//д�벽�����߲���
	 point = 0;
	 for( k = 0;k < 3499; k++ )
	 {
		val = pdata[point++]+((UINT16)pdata[point++]<<8 )  ;
		send_dsp_command(port,val );
		//printf_uart("%d ",val);
		delay_us(200);
	 }
	 //У��
	 send_dsp_command(port,crc );
	 printf_uart("crc[%d]=%x",port,crc);
	 delay_ms(10000);
	 val = read_stepmotor_curve_crc(port);
	 
	 printf_uart("read=%x",val);
	 
	 if(  val== crc)
	 {
		if(1 == write_stepmotor_curve_flag)
			para.dsp1_step_crc = crc;
		if(2 == write_stepmotor_curve_flag)
			para.dsp2_step_crc = crc;
		cpy_para_buff();
		write_para_group(svpara_disp_buf,205);
	    return 1;
		 
	 }
	 else
	 {
		if(1 == write_stepmotor_curve_flag)
			para.dsp1_step_crc = 0;
		if(2 == write_stepmotor_curve_flag)
			para.dsp2_step_crc = 0;
		cpy_para_buff();
		write_para_group(svpara_disp_buf,205);
	 	return 0;
	 }
	 
} 

void x_quickmove(UINT16 quick_time,INT32 tempx_step)
{
	UINT16 low16,high16;
	UINT32 tmp32;
	send_dsp_command(1,0x0000);
	
	delay_us(500);	
	if( tempx_step > 0)
	{
		tmp32 = tempx_step;
		low16  = tmp32 & 0xffff;
		if( x_motor_dir == 0)	
		    high16 = (UINT16)0x0000 + ((tmp32>>16)&0xff);
		else
			high16 = (UINT16)0x4000 + ((tmp32>>16)&0xff);
	}
	else
	{
		tmp32 = - tempx_step;
		low16  = tmp32 & 0xffff;		
		if( x_motor_dir == 0)		
		    high16 = (UINT16)0x4000 + ((tmp32>>16)&0xff);
		else
			high16 = (UINT16)0x0000 + ((tmp32>>16)&0xff);
	}
	if(fabsm(tempx_step)<(quick_time>>3))
	{
		high16 |= (1<<13);
		//quick_time =((UINT16)u230)*10;
	}
	send_dsp_command(1,high16);	
	delay_ms(1);
	if( low16 == 0x5555)
	    low16 = low16 +1;
	send_dsp_command(1,low16);	
	delay_ms(1);
	if( quick_time == 0x5555)
	    quick_time = quick_time +1;
	send_dsp_command(1,quick_time);
	delay_ms(1);
	
	#if 0
	printf_uart("high16=%d",high16);
	printf_uart("low16=%d",low16);
	printf_uart("tempx_step=%d",tempx_step);
	printf_uart("quick_time=%d",quick_time);
	#endif
}

void y_quickmove(UINT16 quick_time,INT32 tempy_step)
{
	UINT16 low16,high16;
	UINT32 tmp32;
	send_dsp_command(1,0x0000);	
	delay_us(500);	
	if( tempy_step > 0)
	{
		tmp32 = tempy_step;
		low16  = tmp32 & 0xffff;
		if( y_motor_dir == 0)
		{	
		    high16 = 0x8000;
			high16 += (tmp32>>16)&0xff;
		}
		else
		{
			high16 = 0xc000; 
			high16 += (tmp32>>16)&0xff;
		}
	}
	else
	{
		tmp32 = - tempy_step;
		low16  = tmp32 & 0xffff;		
		if( y_motor_dir == 0)
		{		
		    high16 = 0xc000; 
			high16 += (tmp32>>16)&0xff;
		}
		else
		{
			high16 = 0x8000;
			high16 += (tmp32>>16)&0xff;
		}
	}
	if(fabsm(tempy_step)<(quick_time>>3))
	{
		high16 |= (1<<13);
	//quick_time  = ((UINT16)u230)*10;
	}
	send_dsp_command(1,high16);	
	delay_ms(1);
	if( low16 == 0x5555)
	    low16 = low16 +1;
	send_dsp_command(1,low16);	
	delay_ms(1);
	if( quick_time == 0x5555)
	    quick_time = quick_time +1;
	send_dsp_command(1,quick_time);
	delay_ms(1);
	
	#if 0
	printf_uart("yhigh16=%d",high16);
	printf_uart("ylow16=%d",low16);
	printf_uart("tempy_step=%d",tempy_step);
	printf_uart("yquick_time=%d",quick_time);
	#endif
}

void zx_quickmove(UINT16 quick_time,INT32 tempz_step)
{
	UINT16 low16,high16;
	UINT32 tmp32;
	send_dsp_command(2,0x0000);	
	delay_us(500);	
	if( tempz_step > 0)
	{
		tmp32 = tempz_step;
		low16  = tmp32 & 0xffff;
		if( z_motor_dir == 0)
		{	
		    high16 = 0xc000;
			high16 += (tmp32>>16)&0xff;
		}
		else
		{
			high16 = 0x8000; 
			high16 += (tmp32>>16)&0xff;
		}
	}
	else
	{
		tmp32 = - tempz_step;
		low16  = tmp32 & 0xffff;		
		if( y_motor_dir == 0)
		{		
		    high16 = 0x8000; 
			high16 += (tmp32>>16)&0xff;
		}
		else
		{
			high16 = 0xc000;
			high16 += (tmp32>>16)&0xff;
		}
	}

	high16 |= (1<<13);
	send_dsp_command(2,high16);	
	delay_ms(1);
	if( low16 == 0x5555)
	    low16 = low16 +1;
	send_dsp_command(2,low16);	
	delay_ms(1);
	if( quick_time == 0x5555)
	    quick_time = quick_time +1;
	send_dsp_command(2,quick_time);
	delay_ms(1);	
}
UINT16 get_IORG_statu(void)
{
	send_dsp2_command(0x0029,0xa000);
	send_dsp_command(2,0x5555);
	return recieve_x.word;
}

UINT16 get_CORG_statu(void)
{
	send_dsp2_command(0x0029,0x2000);
	send_dsp_command(2,0x5555);
	return recieve_x.word;
}
//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xindahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------