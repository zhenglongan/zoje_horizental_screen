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
static UINT16 trans_dsp3;
static UINT16 trans_dsp4;
static UINT8 dsp1;
static UINT8 dsp2;
static UINT8 dsp3;
static UINT8 dsp4;
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
void send_dsp3_command(UINT16 command,UINT16 data);
void send_dsp4_command(UINT16 command,UINT16 data);
UINT16 read_stepmotor_curve_crc(UINT8 port);

#pragma	INTERRUPT/E spiint
void spiint(void);

/*
高5位表示工作电流，电流值＝（i+1)*0.3A; 低3位K表示保持电流，保持电流值＝工作电流*（k+1)/8
工作电流面板开放最高16档，步进电流档位最多32档，所以面板一档对应步进两档，0.6A为可调节单位
半流暂时为工作电流的一半

*/


//--------------------------------------------------------------------------------------
//  Name:		 SPI_init
//  pars:	     None
//  Returns:	 None
//  Description: initial SPI
//--------------------------------------------------------------------------------------
void SPI_init(void)
{
//#define SPISTE1     p10_0
//#define SPISTE2     p10_1
//#define SPISTE3     p0_4
//#define SPISTE4     p3_6
	pd10_0 = 1;
	pd10_1 = 1;
	pd0_4 = 1;
	pd3_6 = 1;


	
	SPISTE1 = 1;     // DSP1 SPI disable 
	SPISTE2 = 1;     // DSP2 SPI disable
	SPISTE3 = 1;     // DSP3 SPI disable 
	SPISTE4 = 1;     // DSP4 SPI disable
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
	
	#if ROTATE_CUTTER_ENABLE
	case OVC_DSP1:
			if( dsp1 == 1)
				sys.error = ERROR_50;//X电机过流	
			else if( dsp2 == 1)
				sys.error = ERROR_94;//剪线电机过流
			else if( dsp3 == 1)
			    sys.error = ERROR_105;//旋转切刀电机过流
		break;

		case OVD_DSP1:
		    if( dsp1 == 1)
				sys.error = ERROR_54;//X电机超差
			else if( dsp2 == 1)	
				sys.error = ERROR_96;//剪线电机超差	
			else if( dsp3 == 1)
				sys.error = ERROR_100;//旋转切刀电机超差
		break;					
		case OVC_DSP2:
		    if( dsp1 == 1)
				sys.error = ERROR_51;//Y电机过流	
			else if( dsp2 == 1)
				sys.error = ERROR_93;//中压脚电机过流
			else if( dsp3 == 1) 
				sys.error = ERROR_101;
		break;
		case OVD_DSP2:
		    if( dsp1 == 1)
				sys.error = ERROR_55;//Y电机超差	
			else if( dsp2 == 1)
				sys.error = ERROR_95;//中压脚电机超差	
			else if( dsp3 == 1)
				sys.error = ERROR_102;
		break;
		case SPI_RX_ERR_CHK:
			if( dsp1 == 1)
				sys.error = ERROR_59;//伺服通讯错误1
			else if( dsp2 == 1)
				sys.error = ERROR_61;//伺服通讯错误3
			else if( dsp3 == 1) 
				sys.error = ERROR_103;//伺服通讯错误4
		break;
		case SPI_RX_ERR_ILLG:
			if( dsp1 == 1)
				sys.error = ERROR_60;//伺服通讯错误2
			else  if( dsp2 == 1)
				sys.error = ERROR_62;//伺服通讯错误3
			else  if( dsp3 == 1)
				sys.error = ERROR_104;//伺服通讯错误5
		break;
	#else
	//dsp1: x + y   dsp2: cutter + inpresser
	switch(error_code)
	{
		case OVC_DSP1:
			if( dsp1 == 1)
				sys.error = ERROR_50;//X电机过流	
			else
				sys.error = ERROR_94;//剪线电机过流
		break;

		case OVD_DSP1:
		    if( dsp1 == 1)
				sys.error = ERROR_54;//X电机超差
			else	
				sys.error = ERROR_96;//剪线电机超差	
		break;					
		case OVC_DSP2:
		    if( dsp1 == 1)
				sys.error = ERROR_51;//Y电机过流	
			else
				sys.error = ERROR_93;//中压脚电机过流
		break;
		case OVD_DSP2:
		    if( dsp1 == 1)
				sys.error = ERROR_55;//Y电机超差	
			else
				sys.error = ERROR_95;//中压脚电机超差	
		break;
		case SPI_RX_ERR_CHK:
			if( dsp1 == 1)
				sys.error = ERROR_59;//伺服通讯错误1
			else
				sys.error = ERROR_61;//伺服通讯错误3
		break;
		case SPI_RX_ERR_ILLG:
			if( dsp1 == 1)
				sys.error = ERROR_60;//伺服通讯错误2
			else
				sys.error = ERROR_62;//伺服通讯错误3
		break;

	}
	#endif
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
			SPISTE3=1;
		  	SPISTE4=1;
		  	if((dsp1==1)&&(recieve_z.word != trans_dsp1))
	  		{
	  			check_dsp_error(recieve_z.word);		
	  		}
			
		  	if((dsp2==1)&&(recieve_z.word != trans_dsp2))
	  		{
				check_dsp_error(recieve_z.word);
	  		}

			if((dsp3==1)&&(recieve_z.word != trans_dsp3))
			{
				check_dsp_error(recieve_z.word);		
			}
		  	if((dsp4==1)&&(recieve_z.word != trans_dsp4))
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
			
			if(err_num_dsp3 >=1 )
			{
				if( sys.error == 0) 
					sys.error = ERROR_61;		
			}

			if(err_num_dsp4 >=1 )
			{
				if( sys.error == 0) 
					sys.error = ERROR_30;		
			}

			spi_flag=0;
			dsp1=0;
		  	dsp2=0;	
			dsp3 =0;
			dsp4 =0;
			err_num_dsp1 = 0;
			err_num_dsp2 = 0;	
			err_num_dsp3 = 0;
			err_num_dsp4 = 0;	
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
	    	if((dsp3==1)&&(recieve_y.word!=trans_dsp3))
    		{
				check_dsp_error(recieve_y.word);						    				
    		}
	    	if((dsp4==1)&&(recieve_y.word!=trans_dsp4))
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

			if(dsp3 == 1)
			{
				trans_dsp3 = trans_y.word;   		
			}
			if(dsp4 == 1)
			{
				trans_dsp4 = trans_y.word;   		
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
			if( (dsp2 == 1) && (recieve_x.word == 0xFFFF) )
			{
				err_num_dsp2 ++;
			}
			if( (dsp3 == 1) && (recieve_x.word == 0xFFFF) )
			{
				err_num_dsp3 ++;
			}
			if( (dsp4 == 1) && (recieve_x.word == 0xFFFF) )
			{
				err_num_dsp4 ++;
			}
   	
	    	if(dsp1==1)
	    	{
    			trans_dsp1=trans_x.word;    		
    		}
	    	if(dsp2==1)
	    	{
    			trans_dsp2=trans_x.word;    		
    		}

			if(dsp3 == 1)
			{
				trans_dsp3 = trans_x.word;   		
			}
			if(dsp4 == 1)
			{
				trans_dsp4 = trans_x.word;   		
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
	trans_dsp3 = 0x0d155;
	trans_dsp4 = 0x0d155;
	dsp1 = 0;
	dsp2 = 0;
	dsp3 = 0;
	dsp4 = 0;
}


						 
//--------------------------------------------------------------------------------------
//  Name:	     version_check                
//  Parameters:	 None
//  Returns:	 None
//  Description: check the step soft version 
//--------------------------------------------------------------------------------------
void version_check(void)
{
	int i;
	for(i=0;i<5;i++)
	{
		send_dsp2_command(0x0001,0x5555);  
		if(recieve_x.word == 0x5555)
		{
			send_dsp2_command(0x0006,0x5555);
		}  
		stepversion2 = recieve_x.word;
	}
	delay_ms(100);		
	for(i=0;i<5;i++)
	{
		send_dsp1_command(0x0001,0x5555);
		if(recieve_x.word == 0x5555)
		{
			send_dsp1_command(0x0006,0x5555);
		}  
		stepversion1 = recieve_x.word;	
	}
	delay_ms(100);	
	#if MULTIPULE_IO_ENABLE
	for(i=0;i<5;i++)
	{
		send_dsp3_command(0x0001,0x5555);  
		if(recieve_x.word == 0x5555)
		{
			send_dsp3_command(0x0006,0x5555);
		} 
		stepversion3 = recieve_x.word;
	}
	delay_ms(100);	
	for(i=0;i<5;i++)
	{
		send_dsp4_command(0x0001,0x5555);  
		if(recieve_x.word == 0x5555)
		{
			send_dsp4_command(0x0006,0x5555);
		} 
		stepversion4 = recieve_x.word;
	}
	delay_ms(100);	
	#endif			
}
//--------------------------------------------------------------------------------------
//  Name:	     movestep_x
//  Parameters:	 None
//  Returns:	 None
//  Description: move x step motor 
//--------------------------------------------------------------------------------------

void movestep_x(int x_data)
{ 

	while( spi_flag > 0 );  		

	if( (sys.status == ERROR) || (x_data == 0) )//出错或0数据不发给驱动
	 	return;
		
	if( movestepx_time > fabsm(x_data)*20 )
	 	movestepx_time =fabsm(x_data)*20;
	if( movestepx_time > 63 )	    //协议时间做限定
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
	while(spi_flag > 0);
	
	if( (sys.status == ERROR) || (zx_data == 0) )
	 	return;
		
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
	}	                                                                      
	else 
	{                                                                       
		zx_data_nf = -zx_data;                                                  
		spi_flag=1;	
		if( zx_data_nf > 255)
		 	zx_data_nf = 255;
		if( z_motor_dir == 0)
			trans_x.word=(UINT16)0x8000+((UINT16)zx_data_nf<<6)+(UINT16)time;         
		else
			trans_x.word=(UINT16)0xc000+((UINT16)zx_data_nf<<6)+(UINT16)time;
	}
	
#if	USE_SC0716_BOARD
	trans_x.word &= ~(1<<15);//bit15=0，表示使用第1路，即DSP2B
#endif

	if( trans_x.word  == 0x5555 )
	{
		trans_x.word += 1;
	}
	trans_y.word=(~trans_x.word)&0x7fff;
	trans_z.word=0x5555;
	dsp2    = 1;
	SPISTE2 = 0;    
	s4trr=trans_x.byte.byte1;  	
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
	if( (sys.status == ERROR) || (zx_data == 0) )
	 	return;	
	if(zx_data>0)
	{
		spi_flag=1;		
		if( para.yj_org_direction == 0)
			trans_x.word=(UINT16)0x4000+((UINT16)zx_data <<6 )+(UINT16)time;         
		else
			trans_x.word=(UINT16)0x0000+((UINT16)zx_data <<6 )+(UINT16)time;
	}	                                                                      
	else 
	{                                                                       
		zx_data_nf = -zx_data;                                                  
		spi_flag = 1;			
		if( para.yj_org_direction == 0)
			trans_x.word=(UINT16)0x0000+((UINT16)zx_data_nf<<6)+(UINT16)time;         
		else
			trans_x.word=(UINT16)0x4000+((UINT16)zx_data_nf<<6)+(UINT16)time;
	}
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

#if ROTATE_CUTTER_ENABLE
//--------------------------------------------------------------------------------------
//  Name:	     movestep_qd
//  Parameters:	 None
//  Returns:	 None
//  Description: move qd step motor 
//--------------------------------------------------------------------------------------
void movestep_qd(int zx_data,UINT16 time)
{
	while(spi_flag > 0) ;

	if( time >63)
	    time = 63;	
	if(zx_data>0)
	{
		spi_flag=1;
		
		if( para.qd_org_direction == 0)
			trans_x.word=(UINT16)0x4000+((UINT16)zx_data <<6 )+(UINT16)time;         
		else
			trans_x.word=(UINT16)0x0000+((UINT16)zx_data <<6 )+(UINT16)time;	   
		if( trans_x.word  == 0x5555) 
		{
			trans_x.word += 1;
		}
		trans_y.word=(~trans_x.word)&0x7fff;
		trans_z.word=0x5555;
		dsp3 = 1;
		SPISTE3=0;                                                               
		s4trr=trans_x.byte.byte1;                                              
	}	                                                                      
	else if(zx_data < 0)
	{                                                                       
		zx_data_nf = -zx_data;                                                  
		spi_flag = 1;	
		
		if( para.qd_org_direction == 0)
			trans_x.word=(UINT16)0x0000+((UINT16)zx_data_nf<<6)+(UINT16)time;         
		else
			trans_x.word=(UINT16)0x4000+((UINT16)zx_data_nf<<6)+(UINT16)time;
		if( trans_x.word  == 0x5555 ) 
		{
			trans_x.word += 1;
		}
		trans_y.word=(~trans_x.word)&0x7fff;
		trans_z.word=0x5555;
		dsp3 = 1;
		SPISTE3 =0;		                         
		s4trr=trans_x.byte.byte1;  
	}	
}

void movestep_qiedao(INT16 StepNum,UINT16 time)//0上/1下轴、步数，时间
{
    UINT16 setdata=0;
	UINT32 tmp32;
	while(spi_flag > 0) ;
//	qd_quickmove(StepNum,time);
		
}
#endif
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
	send_dsp1_command(0x0012,0x5555);		
	if( recieve_x.word == 0x1234 )
	    return(1);
	else
	    return(0);							
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
回读驱动状态
*/
UINT16 read_stepmotor_up_drv(void)
{
	    UINT8 m;
		switch(download_drv_flag)
		{
			case 1:
			send_dsp1_command(0x0010,0x5555);
			break;
			case 2:
			send_dsp2_command(0x0010,0x5555);
			break;
			case 3:
			send_dsp3_command(0x0010,0x5555);
			break;
			case 4:
			send_dsp4_command(0x0010,0x5555);
			break;
		}
		return (UINT16)recieve_x.word;
}
/*
在正常启动状态下，由操作面板切换状态引发；
通知步进准备进入升级状态
*/
void jump_to_begin(void)
{
    //确保是在应用程序状态升级，底下是bootloader就不用发这条指令了
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
	if((3==download_drv_flag)&&(stepversion3<60000))
	{
		while(spi_flag > 0);
		send_dsp_command(3,0x000F);
		delay_ms(1000);	
	}
	if((4==download_drv_flag)&&(stepversion4<60000))
	{
		while(spi_flag > 0);
		send_dsp_command(4,0x000F);
		delay_ms(1000);	
	}
}

//==============================================================
void select_dsp( UINT8 port)
{
	dsp1 = 0;
	dsp2 = 0;
	dsp3 = 0;
	dsp4 = 0;
	SPISTE1 = 1;
	SPISTE2 = 1;
	SPISTE3 = 1;
	SPISTE4 = 1;
	if( port == DSP1)      //dsp1
	{
	    SPISTE1 = 0;
		dsp1 = 1;		
	}
	else if( port == DSP2) //dsp2
	{
	 	SPISTE2 = 0;
		dsp2 = 1;
	}
	else if( port == DSP3)  //dsp3
	{
		SPISTE3 = 0;
		dsp3 = 1;
	}
	else
	{
		SPISTE4 = 0;
		dsp4 = 1;
	}
}

void send_dsp1_command2(UINT16 command)
{
	while(spi_flag > 0);
	spi_flag=1;
	select_dsp(1);	
	trans_x.word = command;      
	trans_y.word = (~trans_x.word)&0x7fff;
	trans_z.word = 0x5555;
	s4trr=trans_x.byte.byte1;   
    delay_us(500);//800
	while(spi_flag > 0);
}

void send_dsp_command(UINT8 port,UINT16 command)
{
	 while(spi_flag > 0);
	 spi_flag = 1;
	 select_dsp(port);	 
	 trans_x.word = command;	  
	 trans_y.word = (~trans_x.word)&0x7fff;
	 trans_z.word = 0x5555;
	 s4trr=trans_x.byte.byte1;	 
 //  if( sys.status != RUN)
	 delay_us(800);//800
	 while(spi_flag > 0);

}
void send_dsp_command_data(UINT8 port,UINT16 command,UINT16 data)
{
	send_dsp_command(port,command);
	send_dsp_command(port,data);
}

void send_dsp1_command(UINT16 command,UINT16 data)    
{
	 send_dsp_command_data(DSP1,command,data);	      
} 
void send_dsp2_command(UINT16 command,UINT16 data)    
{
	send_dsp_command_data(DSP2,command,data);
} 
void send_dsp3_command(UINT16 command,UINT16 data)    
{
	 send_dsp_command_data(DSP3,command,data);
} 
void send_dsp4_command(UINT16 command,UINT16 data)    
{
	 send_dsp_command_data(DSP4,command,data);
} 
/*
#define DSP_CMD_QUICK_MOVE                  0x0000    // 申请快走 
#define DSP_CMD_QUERY_VERSION               0x0001    // 查询软件版本 
#define DSP_CMD_READ_SERVO_PARAM            0x0002    // 读取伺服参数 
#define DSP_CMD_WRITE_STEP_ALL_CONF_PARAM   0x0003    // 写入步进全部配置参数 
#define DSP_CMD_READ_STEP_ALL_CONF_PARAM    0x0004    // 读取步进全部配置参数 
#define DSP_CMD_EMERGENCY_STOP1             0x0005    // 第一路（X轴）急停 
#define DSP_CMD_READ_EMERG_STOP_LEN1        0x0007    // 读取第一路（X轴）（空送急停）走过的距离（精度1步） 
#define DSP_CMD_READ_EMERG_STOP_LEN2        0x0008    // 读取第二路（Y轴）（空送急停）走过的距离（精度1步）
#define DSP_CMD_EMERGENCY_STOP2             0x0009    // 第二路（Y轴）急停 
#define DSP_CMD_WRITE_SERVO_PARAM           0x000A    // 写取伺服参数 
#define DSP_CMD_WRITE_CURVE                 0x000B    // 写步进曲线数据 
#define DSP_CMD_READ_CURVE_CRC              0x000C    // 读步进曲线CRC 
#define DSP_CMD_CONFIG_PLATFORM             0x0011    // 配置电控平台 
#define DSP_CMD_QUERY_ACTION_COMPLETED      0x0012    // 查询步进动作是否完成(找原点、空送急停和快走的查询) 
#define DSP_CMD_ORIGIN_AND_CURRENT          0x0013    // 步进找原点、取消或恢复电流 
#define DSP_CMD_CONFIG_OPEN_CLOSED_MODE     0x001F    // 开、闭环切换 
#define DSP_CMD_CONFIG_ENCODER_LINE1        0x0020    // 设定第一路（X轴）编码器线数 
#define DSP_CMD_CONFIG_ENCODER_LINE2        0x0021    // 设定第二路（Y轴）编码器线数 
#define DSP_CMD_CONFIG_STEP_ANGLE1          0x0022    // 设定第一路（X轴）步距角系数 
#define DSP_CMD_CONFIG_STEP_ANGLE2          0x0023    // 设定第二路（Y轴）步距角系数 
#define DSP_CMD_CONFIG_MOTOR_TYPE1          0x0024    // 设定第一路（X轴）电机编号 
#define DSP_CMD_CONFIG_MOTOR_TYPE2          0x0025    // 设定第二路（Y轴）电机编号 
#define DSP_CMD_CONFIG_OPENLOOP_CURRENT     0x0026    // 设定二路开环电机的工作电流和保持电流 
#define DSP_CMD_CONFIG_HALF_CURRENT_TIME    0x0027    // 设定闭环车缝到位后切开换锁轴延时时间 
#define DSP_CMD_CONFIG_STEP_PARAM           0x0028    // 配置各路参数 
#define DSP_CMD_QUERY_AND_CLEAR_ENCODER     0x0029    // 读取和清零编码器状态 

5代：20mΩ时：((档位+1)/16)*10.9
4代：30mΩ时：((档位+1)/16)*7.4		
单位：0.025mm
1步=0.025*(n+1)mm
即可配置最大0.4mm
*/

void setup_stepper_moter(void)
{
 	UINT16 current1,current2;
	if( para.platform_type == FIFTH_GENERATION )//平台类型	
		send_dsp1_command(0x0011,0x0005);
	else
		send_dsp1_command(0x0011,0x0004);
	
	send_dsp1_command(0x001F,para.DSP1_para_1F);  //开环闭环切换 1表示闭环，2表示开环，3表示转速模式，4表示随动模式，5表示双轴同步
	send_dsp1_command(0x0020,para.DSP1_para_20);  //编码器线数
	send_dsp1_command(0x0021,para.DSP1_para_21);		
	send_dsp1_command(0x0022,para.DSP1_para_22);  //步距角
	send_dsp1_command(0x0023,para.DSP1_para_23);
	current1 = x_step_current_level;
	current2 = y_step_current_level;
	current1 = ((current1<<3) + (UINT16)para.dsp1A_half_current)<<8;
	current2 = ((current2<<3) + (UINT16)para.dsp1B_half_current) + current1;
	
	send_dsp1_command(0x0026,current2 );
	send_dsp1_command(0x0027,para.DSP1_para_27);  
	send_dsp1_command(0x0028,para.DSP1_para_28H); //第一路超差	
	send_dsp1_command(para.DSP1_para_28M1,para.DSP1_para_28M2);  //第二路超差+第一路精度系数  X精确Y伺服
    send_dsp_command(1,para.DSP1_para_28L);
	

	if(!((PAUSE == PAUSE_ON)&&((DVA == para.dvab_open_level)||(DVB == para.dvab_open_level))))
	{
	 	if( para.platform_type == FIFTH_GENERATION )
			send_dsp2_command(0x0011,0X0005);
		else
			send_dsp2_command(0x0011,0X0004);//平台类型		

	 	send_dsp2_command(0x001F,para.DSP2_para_1F);  //开环闭环切换 1表示闭环，2表示开环，3表示转速模式，4表示随动模式，5表示双轴同步
		send_dsp2_command(0x0020,para.DSP2_para_20);  //编码器线数
		send_dsp2_command(0x0021,para.DSP2_para_21);		
		send_dsp2_command(0x0022,para.DSP2_para_22);  //步距角
		send_dsp2_command(0x0023,para.DSP2_para_23);  
		current1 = yj_step_current_level;
		current2 = u235;
		current1 = ((current1<<3) + (((UINT16)(para.dsp2A_half_current))&0x0007))<<8;
		current2 = ((current2<<3) + (((UINT16)(para.dsp2B_half_current))&0x0007)) + current1;
		
		send_dsp2_command(0x0026,current2 );
		send_dsp2_command(0x0027,para.DSP2_para_27);  
		send_dsp2_command(0x0028,para.DSP2_para_28H); //第一路超差	
		send_dsp2_command(para.DSP2_para_28M1,para.DSP2_para_28M2);  //第二路超差+第一路精度系数  X精确Y伺服
	    send_dsp_command(2,para.DSP2_para_28L);
	}
	
	#if ROTATE_CUTTER_ENABLE
	if( PAUSE == PAUSE_OFF)
	{
		send_dsp3_command(0x0011,0X0004);
	 	send_dsp3_command(0x001F,para.DSP3_para_1F);  //开环闭环切换 1表示闭环，2表示开环，3表示转速模式，4表示随动模式，5表示双轴同步
		send_dsp3_command(0x0020,para.DSP3_para_20);  //编码器线数
		send_dsp3_command(0x0021,para.DSP3_para_21);		
		send_dsp3_command(0x0022,para.DSP3_para_22);  //步距角
		send_dsp3_command(0x0023,para.DSP3_para_23);  
		current1 = yj_step_current_level;
		current2 = u235;
		current1 = ((current1<<3) + (((UINT16)(para.dsp3A_half_current))&0x0007))<<8;
		current2 = ((current2<<3) + (((UINT16)(para.dsp3B_half_current))&0x0007)) + current1;
		
		send_dsp3_command(0x0026,current2 );
		send_dsp3_command(0x0027,para.DSP3_para_27);  
		send_dsp3_command(0x0028,para.DSP3_para_28H); //第一路超差	
		send_dsp3_command(para.DSP3_para_28M1,para.DSP3_para_28M2);  //第二路超差+第一路精度系数  X精确Y伺服
	    send_dsp_command(3,para.DSP3_para_28L);
	}
	#endif
	
#if MULTIPULE_IO_ENABLE
	if(!((PAUSE == PAUSE_ON)&&((DVA == para.dvab_open_level)||(DVB == para.dvab_open_level))))
	{
//		SUM=1;
//		delay_ms(200);
//		SUM=0;
		
		send_dsp3_command(0x0011,0X0004);//平台类型		

	 	send_dsp3_command(0x001F,para.DSP3_para_1F);  //开环闭环切换 1表示闭环，2表示开环，3表示转速模式，4表示随动模式，5表示双轴同步
		send_dsp3_command(0x0020,para.DSP3_para_20);  //编码器线数
		send_dsp3_command(0x0021,para.DSP3_para_21);		
		send_dsp3_command(0x0022,para.DSP3_para_22);  //步距角
		send_dsp3_command(0x0023,para.DSP3_para_23);  
		current1 = (para.dsp3A_current_level)%16;;
		current2 = (para.dsp3B_current_level)%16;
		current1 = ((current1<<3) + (((UINT16)(para.dsp3A_half_current))&0x0007))<<8;
		current2 = ((current2<<3) + (((UINT16)(para.dsp3B_half_current))&0x0007)) + current1;
		
		send_dsp3_command(0x0026,current2 );
		send_dsp3_command(0x0027,para.DSP3_para_27);  
		send_dsp3_command(0x0028,para.DSP3_para_28H); //第一路超差	
		send_dsp3_command(para.DSP3_para_28M1,para.DSP3_para_28M2);  //第二路超差+第一路精度系数  X精确Y伺服
	    send_dsp_command(3,para.DSP3_para_28L);
	}
#endif
	
}


void write_stepmotor_config_para(UINT8 port,UINT8 *pdata)    
{
	 UINT8 k;
	 send_dsp_command(port,0x0003);//写入步进配置参数
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
		switch (port)
		{
			case 1:
				send_dsp1_command(0x0004,0x5555);
			break;
			case 2:
				send_dsp2_command(0x0004,0x5555);
			break;
			case 3:
				send_dsp3_command(0x0004,0x5555);
			break;
			case 4:
				send_dsp4_command(0x0004,0x5555);
			break;
		}
		svpara_disp_buf[m] = recieve_x.word;
		//delay_us(1000);,因为读写已经延时了800us了，所以不需要再延时了，否则主控面板重发数次后报错
		
	}
}



/*
对于气阀输出的控制字：
mmm xdddddd  pppppp
mmm---控制字 
dddddd-----端点号，气阀编号
pppppp-----输出电平,打开为1,关闭为0
对于电机的控制字
mmm x ddddddd ttttt
mmm---命令字  x--方向  ddddddd--7位位移 ttttt-5位时间
001---第一路90V驱动 	CZ7411  X34
010---第二路90V驱动 	CZ7412  X33
011---第二路7078     	CZ749   X30
100---第一路7078输出 	CZ7410  X29
101---6064驱动电机输出  CZ7413  X28
110---气阀输出          CZ748   X31
						1-4  AIR1 ---PORTF4  --BIT0   1100 0000 01 00 000x
						2-5  AIR2 ---PORTF3  --BIT1   1100 0000 10 00 00x0
						3-6  AIR3 ---PORTF2  --BIT2   1100 0001 00 00 0x00
110---气阀输出          CZ749   X32
						1-4  AIR4 ---PORTB6  --BIT3   1100 0010 00 00 x000
						2-5  AIR5 ---PORTC0  --BIT4   1100 0100 00 0x 0000
						3-6  AIR6 ---PORTB5  --BIT5   1100 1000 00 x0 0000
mmm xdddddd  pppppp          


X34    0X2000    换梭臂电机  0.36d
X33    0X4000    抓剪线电机/旋转切刀角度电机/ 0.9d
X28    0XA000    梭盘电机 0.9d

*/
void movestep_cs3(UINT16 command,INT16 x_data,UINT8 timer_need)
{ 
	#if MULTIPULE_IO_ENABLE == 1
	UINT8 dir;
	while(spi_flag > 0);
    
	if( timer_need >= 63 )
	    timer_need = 63;
		
	if( x_data !=0 )
	{
		SPISTE1 = 1;
	   	SPISTE2 = 1;
	   	SPISTE4 = 1;
	   	SPISTE3 = 0;

	}
	else 
		return;
	if( (command == 0x2000)||(command == 0x4000) ) //换梭臂电机/旋转切刀电机
	{
		command = 0x0000;//第一路电机
		dir = para.dsp3a_motor_dir;
	}
	else if( command == 0xA000) //梭盘电机
	{
		command = 0x8000;//第二路电机
		dir = para.dsp3b_motor_dir;
	}
	if(x_data > 0)
	{   
		spi_flag = 1;	
		trans_x.word = command + ((UINT16)x_data<<6)+(UINT16)timer_need;
		if( dir == 0)
	       	trans_x.word += 0x4000; 
	}	
	else if(x_data<0)
	{
		x_data_nf = -x_data;
		spi_flag = 1;
		trans_x.word= command + ((UINT16)x_data_nf<<6)+(UINT16)timer_need;	
		if( dir != 0)
	    	trans_x.word += 0x4000; 
	}
	if( trans_x.word == 0x5555 )
	{
		trans_x.word = trans_x.word + 1;
	}
	trans_y.word=(~trans_x.word)&0x7fff;
	trans_z.word=0x5555;
	dsp3 = 1;
	SPISTE3=0;   
	s4trr=trans_x.byte.byte1;  
	
	#else
 	while(spi_flag > 0);
    if( timer_need >= 32 )
	    timer_need = 31;
	if( x_data !=0 )
	{
	   SPISTE3 = 0;
	   SPISTE1 = 1;
	   SPISTE2 = 1;
	}
	if(x_data>0)
	{   
		spi_flag=1;	
		if( dsp3_moto1_direction == 0)
	        trans_x.word=command+((UINT16)x_data<<5)+(UINT16)timer_need;
		else
			trans_x.word=(command|0x1000) + ((UINT16)x_data<<5)+(UINT16)timer_need; 
		if(trans_x.word == 0x2AAA || trans_x.word == 0x5555 || trans_x.word == 0xAAAA || trans_x.word == 0x55AA || trans_x.word == 0xAA55 || trans_x.word == 0x2A55)
		{
			trans_x.word = trans_x.word + 1;
		}
		trans_y.word=(~trans_x.word)&0x7fff;
	    trans_z.word=0x5555;
		dsp3 = 1;
		SPISTE3=0;   
		s4trr=trans_x.byte.byte1;  
	}	
	else if(x_data<0)
	{
		x_data_nf=-x_data;
		spi_flag=1;	
		if( dsp3_moto1_direction == 0)
	    	trans_x.word=(command|0x1000) + ((UINT16)x_data_nf<<5)+(UINT16)timer_need; 
		else
			trans_x.word= command + ((UINT16)x_data_nf<<5)+(UINT16)timer_need; 
	    if(trans_x.word == 0x2AAA || trans_x.word == 0x5555 || trans_x.word == 0xAAAA || trans_x.word == 0x55AA || trans_x.word == 0xAA55 || trans_x.word == 0x2A55)
		{
			trans_x.word = trans_x.word + 1;
		}		   
		trans_y.word=(~trans_x.word)&0x7fff;
		trans_z.word=0x5555;
		dsp3 = 1;
		SPISTE3=0;		             
		s4trr=trans_x.byte.byte1;    
	}
	#endif
}
//===============================================
UINT16 check_DSP3_input(void)
{

	while(spi_flag > 0);
	spi_flag=1;
	select_dsp(DSP3);
	delay_us(100);	
	trans_x.word = 0x0018;      
	trans_y.word = (~trans_x.word)&0x7fff;
	trans_z.word = 0x5555;
	s4trr=trans_x.byte.byte1;   
	while(spi_flag > 0);
	spi_flag=1;
	select_dsp(DSP3);	
	delay_us(100);
	trans_x.word = 0x0018;      
	trans_y.word = (~trans_x.word)&0x7fff;
	trans_z.word = 0x5555;
	s4trr=trans_x.byte.byte1;
	while(spi_flag > 0);
	delay_us(100);			
	return recieve_x.word;					
}





/*
直接按16位写入到DSP里
一共4008个数据，前导1+数据长度2+命令1+DSP号1+data....+校验2+结束1
数据共4000个字节，即2000个字，前1999个字是数据，最后两字节是前1999校验和
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
	switch (port)
	{
		case 1:
			send_dsp1_command(0x000C,0x5555);//读取步进曲线的CRC值
		break;
		case 2:
			send_dsp2_command(0x000C,0x5555);//读取步进曲线的CRC值
		break;
		case 3:
			send_dsp3_command(0x000C,0x5555);//读取步进曲线的CRC值
		break;
		case 4:
			send_dsp4_command(0x000C,0x5555);//读取步进曲线的CRC值
		break;
	}
	return recieve_x.word;
}

UINT8 write_stepmotor_curve(UINT8 port,UINT8 *pdata)    
{
	 UINT16 k,point,val,crc;	 
	 UINT8 ret;
	 crc = crc_calcu((UINT16*)pdata,3499,0XFFFF);
	 
	 send_dsp_command(port,0x000B);//写入步进曲线参数
	 point = 0;
	 for( k = 0;k < 3499; k++ )
	 {
		val = pdata[point++]+((UINT16)pdata[point++]<<8 )  ;
		send_dsp_command(port,val );
		//printf_uart("%d ",val);
		delay_us(200);
	 }
	 //校验
	 send_dsp_command(port,crc );
	 printf_uart("crc[%d]=%x",port,crc);
	 delay_ms(10000);
	 val = read_stepmotor_curve_crc(port);
	 
	 printf_uart("read=%x",val);
	 
	 if(  val == crc)
	 {
		if(1 == write_stepmotor_curve_flag)
			para.dsp1_step_crc = crc;
		if(2 == write_stepmotor_curve_flag)
			para.dsp2_step_crc = crc;
		if(3 == write_stepmotor_curve_flag)
			para.dsp3_step_crc = crc;
		if(4 == write_stepmotor_curve_flag)
			para.dsp4_step_crc = crc;
		cpy_para_buff();
		write_para_group(100,svpara_disp_buf,205);
	    return 1;
		 
	 }
	 else
	 {
		if(1 == write_stepmotor_curve_flag)
			para.dsp1_step_crc = 0;
		if(2 == write_stepmotor_curve_flag)
			para.dsp2_step_crc = 0;
		if(3 == write_stepmotor_curve_flag)
			para.dsp3_step_crc = 0;
		if(4 == write_stepmotor_curve_flag)
			para.dsp4_step_crc = 0;
		cpy_para_buff();
		write_para_group(100,svpara_disp_buf,205);
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
void qd_quickmove(UINT16 quick_time,INT32 tempx_step)
{
	UINT16 low16,high16;
	UINT32 tmp32;
	send_dsp_command(3,0x0000);	
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
	}
	send_dsp_command(3,high16);	

	if( low16 == 0x5555)
	    low16 = low16 +1;
	send_dsp_command(3,low16);	
	if( quick_time == 0x5555)
	    quick_time = quick_time +1;
	send_dsp_command(3,quick_time);
}

UINT16 get_IORG_statu(void)
{
	if( para.inpresser_org_select == 1)
	{
		send_dsp2_command(0x0029,0xa000);
		send_dsp_command(2,0x5555);
	return recieve_x.word;
	}
	else
	{
		if( IORG ==1 )
			return 1;
		else
			return 0;
	}
}

UINT16 get_CORG_statu(void)
{
	send_dsp2_command(0x0029,0x2000);
	send_dsp_command(2,0x5555);
	return recieve_x.word;
}
//多功能IO升级程序
void multipule_program_beginning(UINT8 port)
{
	while(spi_flag > 0);
	send_dsp_command(port,0x1100);//切换程序
	delay_ms(1000);
}

void send_multipule_program_data(UINT8 port)    
{
	 UINT8 k;
	 while(spi_flag > 0);	 
	 send_dsp_command(port,0x1101);//写入程序	    
     delay_ms(1);
	 for(k=0;k<data_length_drv+2;k++)
	 {
		send_dsp_command(port,((UINT16)k<<8) + pat_buf[k]);
		rec_com();
	}
} 

UINT16 read_multipule_program_status(UINT8 port)
{
	send_dsp_command(port,0x1102);//状态查询
	send_dsp_command(port,0x5555);
	return (UINT16)recieve_x.word;
}
void multipule_program_end(UINT8 port)    
{
	 while(spi_flag > 0);
	 send_dsp_command(port,0x1104);//升级完成	 
	 delay_ms(2);
} 
/*
  2017-9-9
  角度由小变大是逆时针旋转
  rotated_position  ----当前主轴角度位置
  rotated_abs_angle ----目标主轴角度位置		 
  约定的是第一象限是0度区间，逆方向旋转是角度逐步变大的过程。
*/

#if ROTATE_CUTTER_ENABLE
void rotated_by_data(INT16 rotated_now_angle)
{
	INT16 temp16;
	INT16 delta1,delta2,rotated_change_angle;
	UINT16 time;
	INT8 rotated_dir;
	
	if(rotated_position != rotated_now_angle)
	{
		//确认最佳的旋转方向和位移
		rotated_dir = 0;		
		if(rotated_position > rotated_now_angle)//设计是从高到低，正常是顺时转
		{
			delta1 = rotated_position  - rotated_now_angle;		//顺时旋转的度数范围
			delta2 = rotated_now_angle + 360 - rotated_position;  //逆时旋转的度数范围
			if( delta1 < delta2 ) 
			{
				rotated_dir = 0;  //顺时针
				temp16 = delta1;
			}
			else
			{
				rotated_dir = 1;  //逆时针
				temp16 = delta2;
			}
		}
		else if(rotated_position < rotated_now_angle)
		{
			delta1 = rotated_now_angle - rotated_position;
			delta2 = rotated_position  + 360 - rotated_now_angle;
			if( delta1 > delta2 ) //逆时针转的范围 大于 顺时针范围 ，采用顺时旋转
			{
				rotated_dir = 0;  //顺时针
				temp16 = delta2;
			}
			else
			{
				rotated_dir = 1;  //逆时针
				temp16 = delta1;
			}
		}
	
		//每步是1度		
		if(rotated_dir ==0) rotated_change_angle =-temp16*Rot_Trans_Ratio;
		else rotated_change_angle =temp16*Rot_Trans_Ratio;//16;		
		
		time = fabsm(temp16)<<1;
		if(time<u230)time = u230;
		
		qd_quickmove(time,rotated_change_angle);
	    rotated_position = rotated_now_angle;
	}
}
#endif
//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xindahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
