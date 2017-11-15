 //--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : watch.c
//  Description: common constants definition 
//  Version    Date     Author    Description
//  0.01     03/07/07   pbb        created
//  0.02     07/11/07   lm         commented
//  ...
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
// Includes
//--------------------------------------------------------------------------------------
#include "..\..\include\sfr62p.h"       // M16C/62P special function register definitions
#include "..\..\include\typedef.h"      // Data type define
#include "..\..\include\common.h"       // Common constants definition
#include "..\..\include\variables.h"    // External variables declaration
#include "..\..\include\action.h"       // action function
#include "..\..\include\motor_def.h"
//--------------------------------------------------------------------------------------
// Global variables define
//--------------------------------------------------------------------------------------
static INT16 SNT_count;
static INT16 SNT_count_1;
static INT16 SNT_count_2;
static INT16 SNT_count_3;
static INT16 stuck_timer;
static INT16 big_current_counter_8000;
static INT16 big_current_counter_7000;
static INT16 big_current_counter_6000;
static INT16 big_current_counter_5000;
static INT16 counter_flag_2;
static INT16 counter_flag_3;
static INT16 u24_add;
static UINT8 ipm_ovl;
static UINT8 ipm_ovl_flag;
static UINT8 ipm_under_flag;
static INT16 Y_k0;
static UINT8 ac_ovdt_count;
static UINT8 snt_ovl_count;
static INT16 Y_k_u;
static UINT8 ipm_cro;
static INT16 spider;
static UINT32 sum_vol;      //
static UINT8  sample_20ms;
static UINT8  sample_64times;
static UINT8  sample_overvalue;
static UINT16 ipm_counter_i;       
static UINT16 ipm_counter_current;
static UINT16 ipm_current[100];
static UINT16 ipm_cro1;
static UINT32 iit_sum;
static UINT32 iit_counter;
static UINT8 iit_enterflag;
static UINT8 sample_undervalue;
//--------------------------------------------------------------------------------------
// Internal constants definition
//--------------------------------------------------------------------------------------
#define A_2  0x38b8  //(25Hz,0x71b5),(50Hz,0x664d),(100Hz,0x5532),(120Hz,0x4fdb),(150Hz,0x48fa),(200Hz,0x3fd8),(220Hz,0x3cc8),(250Hz,0x38b8)                        
#define B0_2 0x4747  //(25Hz,0x0e4a),(50Hz,0x19B2),(100Hz,0x2acd),(120Hz,0x3024),(150Hz,0x3705),(200Hz,0x4027),(220Hz,0x4337),(250Hz,0x4747)
//--------------------------------------------------------------------------------------
// Internal functions and subroutines declaration
//--------------------------------------------------------------------------------------
#pragma	INTERRUPT/B int1_int
void int1_int(void);
#pragma	INTERRUPT/B int2_int
void int2_int(void);
//--------------------------------------------------------------------------------------
//  Name:		init_sys_watch 
//  Parameters:	void
//  Returns:	void
//  Description: intial system watch module function
//--------------------------------------------------------------------------------------
void init_watch(void)        
{
  	INT16 i;
  	SNT_count = 0;
	SNT_count_1=0;
	SNT_count_2=0;
	SNT_count_3=0;
	stuck_timer=0;
	big_current_counter_8000=0;
	big_current_counter_7000=0;
	big_current_counter_6000=0;
	big_current_counter_5000=0;
	counter_flag_2 = 0;
	counter_flag_3=0;
  	u24_add = 0;
  	ipm_ovl = 0;
	ipm_ovl_flag = 0;
	ipm_under_flag = 0;
  	int1ic = 0;
  	int2ic = 0;
	Y_k0 = 0;
	Y_k_u = 0;
	ac_ovdt_count = 0;
	snt_ovl_count = 0;
	Y_k_u = 300;
	ipm_cro = 0;
	spider = 0;
	spider_man = 0;
	spider_lim = 0;
	sum_vol = 0;      //
    sample_20ms = 0;
	sample_64times = 0;
	sample_overvalue = 0;
	ipm_counter_i = 0;     
    ipm_counter_current = 0;	
	ipm_cro1 = 0;
    for(i=0;i<100;i++)
     ipm_current[i] = 0;	
	iit_sum= 0;
	iit_counter=0;
	iit_enterflag = 0;
    sample_undervalue = 0;
}
//--------------------------------------------------------------------------------------
//  Name:		sys_watch routine
//  Parameters:	void
//  Returns:	INT16
//  Description: system watch function
//--------------------------------------------------------------------------------------
INT16 sys_watch(void)        
{
	INT16 temp16;
	UINT16 temp1,temp2; 
	INT16 delta_s;
	INT16 spider_time;
	UINT16 ipm_counter_flag = 0;
	UINT32 ipm_ii = 0;
	UINT32  temp_iit=0;
	static UINT16 TOC_count = 0;
  	sys.uzk_val = ((INT32)Y_k_u * A_2+(INT32)(ad0) * B0_2)>>15;
  	Y_k_u = sys.uzk_val;
	
	//--------------------------------------------------------------------------------------
  	// power-off status
  	//--------------------------------------------------------------------------------------
  	if(sys.status == POWEROFF)   
    	return OK;
	//--------------------------------------------------------------------------------------
  	// watch 300V overvoltage
  	//--------------------------------------------------------------------------------------	
#if USE_SC013K_PALTFORM
#else
	if(AC_OVDT)            
	{
		ac_ovdt_count++;
		if(ac_ovdt_count)
		{
			U=1;U_=1;V=1;V_=1;W=1;W_=1;
			prcr = 0x02;
			inv03 = 0;
			prcr = 0x00;
			OUTPUT_ON = 1;      // output_on disable
			return ERROR_05;    // 300V overvoltage
		}
	}
	else
		ac_ovdt_count = 0;	
#endif
//=====================================	
#if USE_SC013K_PALTFORM
	/* ��Դ�����ѹ=(((ad0/1023)*5)*100)/1.414 */
#define UZKIN_SAMPLE_UNDER  453
#define UZKIN_SAMPLE_OVER   765
  sample_20ms++;
  if (sample_20ms >= 1)
  {
    sample_20ms = 0;
    sum_vol = sum_vol + ad0;
    sample_64times++;
    if (sample_64times >= 64)
    {
      sample_64times = 0;
      sum_vol = sum_vol >> 6;
   
      if(sum_vol < UZKIN_SAMPLE_UNDER)
      {
        sample_undervalue++;
      }
      else
      {
        sample_undervalue = 0;
      }
      sum_vol = 0;

      if (sample_undervalue > 1)
      {
        sample_undervalue = 0;
        
        if (POWEROFF != sys.status)/* �������״̬ */
        {
			U=1;V=1;W=1;
		  	prcr = 0x02;
			inv03 = 0;
			U_=0;V_=0;W_=0;
		  	prcr = 0x00;
			sys.status = POWEROFF;
			m_status = 3;
			OUTPUT_ON = 1;
			motor.iq = 0;
			Run_SpdIq.Sum_Iq = 0;	//scx
			motor.iq_last = 0;		//scx
			motor.spd_obj = 0;
			motor.spd_ref = 0;
			motor.stop_flag = 1;
		    return OK;
        }

      }
    }
  }
#undef UZKIN_SAMPLE_UND
#undef UZKIN_SAMPLE_OVER
#else	
	sample_20ms++;
	if(sample_20ms>=10)
	{
		sample_20ms = 0;
		sum_vol += ad0;
		sample_64times++;
		if(sample_64times >= 64)
		{
			sample_64times = 0;
			sum_vol = sum_vol>>6;
			/*
			160V * 1.414 = 226.24V /100 =2.26v
			463=>164v
			451=>161v
			 2^10   463
			 -----=-----
			  5v    2.26v
			 
			*/
			if(sum_vol < 445)
    		{
				sum_vol = 0;
				sample_overvalue++;
				if(sample_overvalue>4)
				{
				   sample_overvalue =0;
				disable_24V_output();           // 24V disable
 		 		emergency();          // emergency measure
    			return ERROR_04;      // 300V undervoltage

				}   
    		 }
    		 else
			 {
     		    if( para.platform_type == FOURTH_GENERATION )
     		    	PWR_LED = 0;
				sample_overvalue = 0;
			 }
			 sum_vol = 0;
		}
	}
#endif	
	//================================
  	//--------------------------------------------------------------------------------------
  	// watch 24V overvoltage and undervoltage
  	//--------------------------------------------------------------------------------------
	if( para.platform_type == FIFTH_GENERATION )
	{
	  	if(SNT5_ON == 0)
	  	{
			/*
			p0_2��4.67382V��Ӧ33V��ad2��10λ������5V��Ӧ1023��daֵ����da2=1ʱ��Ӧ33V��Դ��ѹֵΪ(((33/4.67)*5)/1023)=0.0345375���ۺϵ�Q15��ֵΪ0.0345375*2^15=1131.7248��1132//
			*/
			sys.u24_val = ((UINT32)ad2 * 1132) >> 15;
	        if(sys.u24_val < 20)    //����+33V�����ѹ����20V//
		    {
	  		     disable_24V_output();         //�ض�+33V���//
	  		     return ERROR_09;    //����24V��ѹ����//
	    	}		
	  	}
	}
	else
	{
  		sys.u24_val = ((long)ad2*965)>>15;
  		if(SNT4_ON == 0)
	  	{
	    	if(sys.u24_val < 10)    // 24V is undervoltage
	    	{
	  	  		disable_24V_output();           // disable 24V
	      		return ERROR_09;      // 24V is undervoltage
	    	}
			if(SNT_OVL)             // 24V is overvoltage
			{
				snt_ovl_count++;
				if(snt_ovl_count)
				{
					snt_ovl_count = 0;
					if(CutActionFlag==0 && DAActionFlag==0)
					{
			  	  		disable_24V_output();         // disable 24V
						da0 = 0;
						L_AIR = 0;			
						FA =0;			
			      		return ERROR_08;    // 24V is overvoltage
					}
				}
			}
			else
				snt_ovl_count = 0;
	  	}
	}  	
  	//--------------------------------------------------------------------------------------
  	// watch motor encoder connect
  	//-------------------------------------------------------------------------------------- 
  	if(adtc_flag == 0)
  	{
    	if(ADTC == 1)            
    	{	
    		adtc_count++;
    		if(adtc_count >= 5)
    		{
    	  		adtc_flag = 1;
    	  		disable_24V_output();             // 24V disable
    	  		emergency();            // emergency measure 
        		return ERROR_13;        // no motor encoder connect	
    		}
    	}
    	else
    	{
    		if(adtc_count != 0)
    		{  		
    	  		adtc_count = 0;
    		}
    	}
  	}
   	//--------------------------------------------------------------------------------------
  	// watch thread break when machine run
  	//--------------------------------------------------------------------------------------  
  	if((sys.status == RUN)&&( making_pen_actoin_flag == 0))           
  	{
  		if(u71 == 1)
    	{    	
    		if(thbrk_flag == 0)
    		{    		
    			if(stitch_counter > u72)
    	  		{    	  
    	  			if(thbrk_count > u73)
    	  			{    	  	
    	  				thbrk_flag = 1;     //thread breakage
    	  			}	
    	  		}
    	  		else
    	  		{
    	  			thbrk_count = 0;
    	  		}
    		}
    		else
    		{
    			thbrk_count = 0;
    		}	    		
    	}
    	else
    	{
    		thbrk_count = 0;
    		thbrk_flag = 0;
    	}		
  	}
  	//--------------------------------------------------------------------------------------
  	// watch IPM overvoltage and undervoltage
  	//--------------------------------------------------------------------------------------
	if(ipm_ovl)               
	{
		if(pwm_forbid_flag)
		{
		
			U=1;U_=1;V=1;V_=1;W=1;W_=1;
    		prcr = 0x02;
			inv03 = 0;
    		prcr = 0x00;
			ipm_ovl = 0;
			ipm_ovl_flag = 0;
			ipm_under_flag = 0;
			emergency();
			return ERROR_40;
			
		}
		
		else
		{
		if(BLDC_ON)
			ipm_ovl_flag ++;
		else
			ipm_under_flag++;
		    	
		if(ipm_ovl_flag > 4)
		{	
			disable_24V_output();           // 24V disable
			emergency();          // emergency measure
	      	return ERROR_34;      // BLDC_ON alawys ON,abnormal current // modified by zz on 20100311
    	} 
		else if(ipm_under_flag > 1)
		{
			U=1;U_=1;V=1;V_=1;W=1;W_=1;
      		prcr = 0x02;
			inv03 = 1;
    		prcr = 0x00;
			ipm_ovl = 0;
			ipm_ovl_flag = 0;
			ipm_under_flag = 0;
			}
		}
	}
//*************************************************
//IPM PROTECT
//*************************************************	
	spider ++;
	if(spider > 6000)
	{
		spider = 6000;
	}
	if(ipm_cro)
	{
		ipm_cro = 0;
		spider_time = spider;
		spider_lim ++;
		spider = 0;
		
		if(spider_time <= 2)
			delta_s = 100;
		else if(spider_time <= 5)
			delta_s = 95;
		else if(spider_time <= 8)
			delta_s = 90;
		else if(spider_time <= 13)
			delta_s = 85;
		else if(spider_time <= 20)
			delta_s = 80;
		else if(spider_time <= 30)
			delta_s = 75;
		else if(spider_time <= 40)
			delta_s = 70;
		else if(spider_time <= 50)
			delta_s = 65;
		else if(spider_time <= 60)
			delta_s = 60;
		else if(spider_time <= 70)
			delta_s = 50;
		else if(spider_time <= 100)
			delta_s = 40;
		else if(spider_time <= 200)
			delta_s = 30;
		else if(spider_time <= 1000)
			delta_s = 20;
		else if(spider_time <= 4000)
			delta_s = 10;
		else
		{
			delta_s = 0;
			//spider_man = 0;
		}
		
		spider_man += delta_s;
		
	}
	if(spider_man > 600 || spider_lim >= 10)
	{
		emergency();
		return ERROR_35; // IPM over cuurrent frequently// modified by zz on 20100311
	}
//--------------------------------------------------------
	if(ipm_cro1)               
	{
		ipm_counter_flag = 1;
		ipm_cro1 = 0;
	}
	else
	   ipm_counter_flag = 0; 
	   
	ipm_counter_current = ipm_counter_current + ipm_counter_flag - ipm_current[ipm_counter_i];
	ipm_current[ipm_counter_i] =  ipm_counter_flag;
	ipm_counter_i ++;
	if(ipm_counter_i >= 100)
	   ipm_counter_i = 0;    
	if(ipm_counter_current > 10)
    {
	    emergency();
		return ERROR_36; // over cuurrent more than 10 times in 100ms // modified by zz on 20100311
    }
	if((motor_stuck_flag == 1)||(EncoderZ_flag == 0) )//2011-6-13
	{
		emergency();
		return ERROR_14;				// motor is not normal
	}
	
//************************************
//motor stuck protection
//************************************	

		if(abs(motor.iq) > 8000 && m_spd_n <= 200)
		{
			if(abs(motor.iq_last) > 8000)
			{
				big_current_counter_8000++;
			}
			else
			{
				big_current_counter_8000 = 0;
			}
			
		}
		else
		{
			big_current_counter_8000 = 0;
		}
		
		if(big_current_counter_8000 > 1000)
		{
			big_current_counter_8000 = 0;
			
			#if ERROR_OUTPUT_DEBUG
			OUTPUT_ON = 0;
			FR_ON =1;
			#endif
			emergency();
			return ERROR_37;
		}
		
	 
	//**************************
	//dither_tactic use time
	//**************************	
	SNT_count_1 ++;
	
	if(SNT_count_1 >= 1)
	{
		SNT_count_1 = 0;
		
		if(tactic_flag == 2)
		{
			if(tactic_flag_last == 2)
			{
				stuck_timer ++;
			}
			else
			{
				stuck_timer = 0;
			}
		}
		else
		{
			stuck_timer = 0;
		}

	}

//---------------------------------
//---------------------------------
	

//********************************
//AC tactic use least time
//********************************
	if((tactic_flag == 3) && (tactic_flag_last == 1))
	{
		counter_flag_2 = BEGIN;
	}
	if((tactic_flag == 1) && (tactic_flag_last == 0))
	{
		counter_flag_2 = END;
	}	
	if(counter_flag_2 == BEGIN)	
	{
		SNT_count_2++;
		if(SNT_count_2 <= LIMIT_TIME_AC)
		{
			door_ac = 0;
		}
		else
		{
			door_ac = 1;
			SNT_count_2 = LIMIT_TIME_AC + 50;
		}
	}
	else
	{
		SNT_count_2 = 0;
	}

//****************************************************************************

	if((tactic_flag == 3) && (tactic_flag_last == 2))
	{
		counter_flag_3 = BEGIN;
	}
	if(((tactic_flag == 1) && (tactic_flag_last == 0)) || ((tactic_flag == 2) && (tactic_flag_last == 0)))
	{
		counter_flag_3 = END;
	}
	if(counter_flag_3 == BEGIN)
	{
		SNT_count_3 ++;
		if(SNT_count_3 <= LIMIT_TIME_DT)
		{
			door_dt = 0;
		}
		else
		{
			door_dt = 1;
			SNT_count_3 = LIMIT_TIME_DT + 10;
		}
	}
	else
	{
		SNT_count_3 = 0;
	}
//****************************************************************************
//iq_max_tester
//****************************************************************************
	if(tactic_flag != DT)	
	{
		if(motor.iq == PID_SPD_MAX_P)			
		{
			if(motor.iq_last == PID_SPD_MAX_P)
			{
				iq_max_tester++;
				if(iq_max_tester > 2000)
				{
					iq_max_tester = 2000;
				}
			}
			else
			{
				iq_max_tester = 0;
			}
		}
		else
		{
			iq_max_tester = 0;
		}
	}
	else
	{
		iq_max_tester = 0;
	}

//****************************************************************************
//low_speed_last_tester
//****************************************************************************
	if(tactic_flag != DT)
	{
		if(abs(m_spd_n) < 60)
		{
			if(abs(m_spd_n_last) < 60)
			{
				speed_min_tester++;
				if(speed_min_tester >= 100)
				{
					speed_min_tester = 100;
				}
			}
			else
			{
				speed_min_tester = 0;
			}
		}
		else
		{
			speed_min_tester = 0;
		}
	}
	else
	{
		speed_min_tester = 0;
	}
	//2011-6-28
	if(T_OC == 1)
	{
		disable_24V_output();
		emergency();
		return ERROR_10;
	}
  	//--------------------------------------------------------------------------------------
  	// system no error
  	//--------------------------------------------------------------------------------------
  	return OK;
}
//--------------------------------------------------------------------------------------
//  Name:		motor_watch routine
//  Parameters:	void
//  Returns:	INT16
//  Description: system watch of initialize function
//--------------------------------------------------------------------------------------
INT16 watch_initialization(void)
{
	INT16 count;
	INT16 temp16;
	//--------------------------------------------------------------------------------------
  // system is power-off status
  //--------------------------------------------------------------------------------------
  #if USE_SC013K_PALTFORM
  #else
	if(PWR_ON == 0)
	{
		sys.status = POWEROFF;  
        return OK;
	}
  #endif
  	//--------------------------------------------------------------------------------------
  	// watch IPM overvoltage and overcurrent
  	//--------------------------------------------------------------------------------------
	if(BLDC_ON)               
		return ERROR_07;    // IPM is overvoltage or overcurrent
  	//--------------------------------------------------------------------------------------
  	// watch motor encoder connect
  	//--------------------------------------------------------------------------------------
  	if(ADTC == 1)             
    	return ERROR_13;    // no motor encoder connect
	//--------------------------------------------------------------------------------------
  	// watch IPM overvoltage and overcurrent
  	//--------------------------------------------------------------------------------------
	U=1;U_=1;V=1;V_=1;W=1;W_=1;
	U_=0;V_=0;W_=0;
	count = 0;
  	while(1)
  	{
	    if(ir_ta0ic)        //test 1ms timer
	    {
		  	count++;
			if(count >= 20)   //delay 20ms 
			{
				break;
			}	
		}
		if(ipm_ovl)         
		{
			U_=1;V_=1;W_=1;
			return ERROR_07;  // IPM is overvoltage or overcurrent
		}
	}
	//--------------------------------------------------------------------------------------
  	// system no error
  	//-------------------------------------------------------------------------------------- 
	return OK; 
}
//--------------------------------------------------------------------------------------
//  Name:		int1_int 
//  Parameters:	void
//  Returns:	void
//  Description: INT1 interrupt routine
//  BLDC_ON
//--------------------------------------------------------------------------------------
void int1_int(void)      // IPM is overvoltage or overcurrent
{
	U=1;V=1;W=1;
  	prcr = 0x02;
	inv03 = 0;
	U_=0;V_=0;W_=0;
  	prcr = 0x00;
	ipm_ovl = 1;
	ipm_cro = 1;
	ipm_cro1 = 1;
	OverCurrentOneCycle++;
}
//--------------------------------------------------------------------------------------
//  Name:		int2_int 
//  Parameters:	void
//  Returns:	void
//  Description: INT2 interrupt routine
//--------------------------------------------------------------------------------------
// PWR_ON
void int2_int(void)      // power-off
{
	U=1;V=1;W=1;
  	prcr = 0x02;
	inv03 = 0;
	U_=0;V_=0;W_=0;
  	prcr = 0x00;
	sys.status = POWEROFF;
	//sys.error = OK;
	//SUM = 1; 
	m_status = 3;
	OUTPUT_ON = 1;
	motor.iq = 0;
	Run_SpdIq.Sum_Iq = 0;	//scx
	motor.iq_last = 0;		//scx
	motor.spd_obj = 0;
	motor.spd_ref = 0;
	motor.stop_flag = 1;
}
//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------