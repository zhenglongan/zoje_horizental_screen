/*
--------------------------------------------------------------------------------------
         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
                     ALL RIGHTS RESERVED 
  Project Number: sewing_machine_controller 
  File Name : main.c
  Description: Core program to control the sewing machine
  Version    Date     Author    Description
--------------------------------------------------------------------------------------
*/
//--------------------------------------------------------------------------------------
// 	Includes
//--------------------------------------------------------------------------------------
#include "..\..\include\sfr62p.h"         // M16C/62P special function register definitions
#include "..\..\include\typedef.h"        // Data type define
#include "..\..\include\common.h"         // Common constants definition
#include "..\..\include\variables.h"      // External variables declaration
#include "..\..\include\initial.h"        // External variables declaration
#include "..\..\include\system.h"         // External variables declaration
#include "..\..\include\motor.h"          // Motor function
#include "..\..\include\delay.h"          // delay time definition
#include "..\..\include\watch.h"          // System watch function
#include "..\..\include\communication.h"  // Communication function
#include "..\..\include\stepmotor.h"      // stepper motor function
#include "..\..\include\action.h"         // action function
#include "..\..\include\solenoid.h"       // solenoid driver definition
#include "..\..\include\iic_bus_eeprom.h" 
#include "..\..\include\MFRC522.h" 
//--------------------------------------------------------------------------------------
//  Internal functions and subroutines declaration
//--------------------------------------------------------------------------------------
#pragma	INTERRUPT/E ta0_int
void ta0_int(void);
#pragma interrupt tb3_int
void tb3_int(void);
#pragma interrupt tb4_int
void tb4_int(void);
//--------------------------------------------------------------------------------------
//  Name:	main
//  pars:	None
//  Returns:	None
//  Description:	call initial function and receive command function
//--------------------------------------------------------------------------------------
void main(void)
{
	INT16  count;	
  	asm("fclr I");
	//--------------------------------------------------------------------------------------
  	// call initial function
  	//--------------------------------------------------------------------------------------
	initial();
	reset_panel();
	
#if INSTALLMENT	
	main_control_lock_flag = read_par(0);
#endif	
    version_check();	   
	while(connect_flag == 0)  
  	{
  		delay_ms(100);
		if( (sys.status == DOWNLOAD)||(sys.status == DOWNLOAD_DRV1)||(sys.status == DOWNLOAD_DRV2) )
		     break;
  	}  
	
    enable_24V_output();
	if( para.platform_type == FIFTH_GENERATION )	
	{
	    SNT_H = 1;    				//��27V��������33V
	    delay_ms(5);
	    sys.u24_val = ((UINT32)ad2 * 1132) >> 15;
	    if(sys.u24_val < 8)        	//����+33V�����ѹ����8V//
		{
	  		disable_24V_output();            	//�ض�+33V���//
	  		sys.error = ERROR_09;  	//����24V��ѹ����//
		}
		SNT_H = 0; 
	}
	delay_ms(300);     
	count = 0;
	while(1)
	{
	    if(ir_ta0ic)       
	    {
			ir_ta0ic = 0;
			count++;
			if(count >= 83)
			break;
		}
	}
	ta0ic = TA0_IPL;		// set ta0 interrupt priority to 3
	
#if CHECKBOARD
    OW_check();
	OW_status();
#endif
    if(!((PAUSE == PAUSE_ON)&&((DVA == para.dvab_open_level)||(DVB == para.dvab_open_level))))
		initial_mainmotor();
	delay_ms(80);   
	
	if( stepversion1 >= 60000)
		sys.status = DOWNLOAD_DRV1;   
	else if ( stepversion2 >= 60000 )
		sys.status = DOWNLOAD_DRV2;   
	else 
	{
		setup_stepper_moter();	
		if(( para.dsp1_step_crc != read_stepmotor_curve_crc(1))||(para.dsp2_step_crc != read_stepmotor_curve_crc(2)))
		{
			sys.status = ERROR;
			StatusChangeLatch = ERROR;
	      	sys.error = ERROR_92; 	
		}
		if(PAUSE == PAUSE_OFF)
		{
			if( u201 == 1 )         
		   	    go_origin_allmotor();		
			if((auto_function_flag == 1)&&(formwork_identify_device ==2))
			{
			   RFID_initial();
			   init_uart1_RC522(); 	
			   rfid_config_flag = 1;
			}
		}
	}
	#if AUTO_CHANGE_FRAMEWORK   //�Զ���ģ��
		AUTO_LEFT_HOLDING_POSITION = 0;
		AUTO_LEFT_FRAME_STANDBY = 0;
		AUTO_RIGHT_HOLDING_POSITION = 0;
		AUTO_RIGHT_FRAME_STANDBY = 0;
	#endif	
  	//--------------------------------------------------------------------------------------
  	// system status switch
  	//--------------------------------------------------------------------------------------
	while(1)
  	{
		rec_com();    
	  	switch(sys.status)
    	{
	      	case READY:     ready_status();     break;	
	      	case RUN:       run_status();       break;
	      	case ERROR:     error_status();     break;       	      
	      	case PREWIND:   prewind_status();   break;	
	      	case WIND:      wind_status();      break;		
	      	case POWEROFF:  poweroff_status();  break;                               
	      	case SETOUT:    setout_status();    break;   	
	      	case PREEDIT:   preedit_status();   break;   	
	      	case FINISH:    finish_status();    break;		
	      	case SLACK:     slack_status();     break;		
	      	case CHECKI03:  checki03_status();  break;		
	      	case CHECKI04:  checki04_status();  break;		
	      	case CHECKI05:  checki05_status();  break;		
	      	case CHECKI06:  checki06_status();  break;		
	      	case CHECKI07:  checki07_status();  break;		
	      	case CHECKI08:  checki08_status();  break;		
			case CHECKI10:  checki10_status();  break;
			case CHECKI11:  checki11_status();  break;
			case DOWNLOAD:  download_status();  break;
    		case CONTINUE:  continue_status();  break;
			case DOWNLOAD_DRV1:
			case DOWNLOAD_DRV2:download_drv_status(); break;
	      	default:  
			       sys.status = READY; 
				   StatusChangeLatch=READY;    
				   break;        	   	
    	} 
		
  		if((sys.status == READY)&&(formwork_identify_device !=0)&&(single_flag ==0)&&(auto_function_flag == 1)
		    && (shift_flag == 0)&&(predit_shift == 0)&&(already_in_origin ==1))
		{
			rec1_com();
		}
		
	}
}
//--------------------------------------------------------------------------------------
//  Name:	ta0_int
//  pars:	None
//  Returns:	None
//  Description: TA0 timer interrupt function(1ms)
//--------------------------------------------------------------------------------------
void ta0_int(void)
{
	UINT8 j ,flag ;
	UINT32 cnt;
	flag_1ms = 1;	
	counter_1ms++;
	ms_scan_counter++;
	aging_mode_counter_1++;
	motor_control();
	if( barcoder_time_between_same_code == 1)
	{
		  pattern_change_counter++;
	      if(pattern_change_counter >= 5000)
	      {	     
			  barcoder_time_between_same_code = 0;
			  last_pattern_number =0;
		  }  
	}
	
	if(CutActionFlag==1)
	{
		CutActionCounter++;
		if(CutActionCounter >= CUTACTIONTIME)
		{
			CutActionFlag = 0;
			trim_io_control(OFF);
			CutActionCounter = 0;
			
		}
	}
	
	if( cutter_delay_flag == 1)
	{
		if( cutter_delay_counter > 0)
			cutter_delay_counter--;
		else
			cutter_delay_flag = 0;
	}

	if(tension_open_switch == 1)
	{
		tension_open_counter++;
		if( k03 == MECHANICAL )
		{
			if(tension_open_counter >= tension_release_time)
			{	
					da0 =0;
					tension_open_switch =0;
					tension_open_counter =0;
			}
			else if(tension_open_counter >= u223)          
			   		da0 = tension_release_value;
		}    
		else
		{
			if(tension_open_counter >= 15)
			{
			   da0 = temp_tension;
			   tension_open_switch = 0;
			}
		}
	}
	
	if( blow_air_action_flag == 1)
	{
		if( blow_air_counter > 0)
		    blow_air_counter--;
		else
		{
			blow_air_counter = 0;
			blow_air_action_flag = 0;
			BLOW_AIR = 0;
		}
	}

	if( needle_cool_flag ==2 )//
	{
		if( CoolingActionFlag == 1)
		{
			Cooling1sCounter++;
			if( Cooling1sCounter >=1000)
			{
				Cooling1sCounter = 0;
				CoolingDurationCounter++;
				if( CoolingDurationCounter >= CoolingDuration)
				{
					COOL_AIR = 0;
				    CoolingActionFlag = 0;
					CoolingIntervalCounter = 0;
				}
			}
		}
		else
		{
			if( (sys.status == RUN)&&(motor.spd_obj >0) )
			{
				if( CoolingIntervalCounter >= CoolingInterval)
				{
				   COOL_AIR = 1;
				   CoolingDurationCounter = 0;
				   CoolingActionFlag = 1;
				}
			}
		}
	}
		
	if(movex_delay_flag ==1 )
	{
		if(movex_delay_counter>0)
		   movex_delay_counter--;
	}
	
	if(movey_delay_flag ==1 )
	{
		if(movey_delay_counter>0)
		   movey_delay_counter--;
	}
	
	if( movezx_delay_flag ==1)
	{
		if(movezx_delay_counter>0)
		   movezx_delay_counter--;
		   
	}
	
	if( sys.status != RUN  )
    {
		if( software_key_bobbin == 1)
		{
			software_key_bobbin = 0;
			if( cover_position_flag == 0)
			{
				FR_ON =1;
				cover_position_flag = 1;
			}
			else
			{
			    if( sys.status != CHECKI05)
					FR_ON =0;
				cover_position_flag = 0;
			}
		}
		if( (SFSW == ON)&&(pause_inpress_flag ==0) )         
	    {	
	      	sfsw_count++;
	      	if(sfsw_count >= 5)  
			{	
	      		  pause_inpress_flag = 1;
				  sfsw_count = 0;
				  if( cover_position_flag == 0)
				  {
					 FR_ON =1;
					 cover_position_flag = 1;
				  }
				  else
				  {
					  if( sys.status != CHECKI05)
						FR_ON =0;
					   cover_position_flag = 0;
				  }
			}
	    }  
	    else if(SFSW == OFF)
		{
		      	sfsw_count++;
		      	if(sfsw_count >= 10)  
				{
					sfsw_count = 0;
		      		pause_inpress_flag = 0;
				}
		 }
    }
	
	if(test_action_flag == 1 )
	{
		test_action_counter++;
		if(test_action_counter >= 15)
		{
			   da0 = temp_tension;
		}
		if(test_action_counter >= 5000)
		{
			FW = 0;
			trim_io_control(OFF);
			da0 = 0;
			FA = 0;
			test_action_flag = 0;
		}
	}
    if( fw_action_flag == 1)
	{
		fw_action_counter++;
		if( fw_action_counter >= 1000)
		{
			FW = 0;
			fw_action_flag = 0;
			fw_action_counter = 0;
		}
	}
	if( pause_flag == 0)
    {		  	
  	  	if( PAUSE == PAUSE_ON)
  	  	{
  	  		pause_count++;
  	  		if( pause_count >= 50)
  	  		{
  	  			pause_count = 0;
  	  			pause_flag = 1;       
  	  		}	
  	  	}
	}
		
	  	  	
	//--------------------------------------------------------------------------------------
  	//  call system watch function
  	//--------------------------------------------------------------------------------------
	if( sys.error == 0)
		sys.error = sys_watch();
  	if( sys.error != 0)
	{
    	sys.status = ERROR;
		StatusChangeLatch = ERROR;
		if( motor.spd != 0)
		    sewing_stop();
	}

 	if(flag_start_waitcom == 1)
 	{
 		if(++counter_wait_com > 100)
 		{
			flag_wait_com = 1;
 		}
 	}
 	else
 	{
		counter_wait_com = 0;
		flag_wait_com = 0;
 	}
	
	if( fk_action_flag == 1)
	{
		if( fk_action_counter < 80 )
		{
			fk_action_counter++;
		}
		else
		{
			fk_action_counter = 0;
			fk_action_flag = 0;
			AIR_OUT = 0;    			//��������3  �رմ�������
		}
	}
	#if AUTO_CHANGE_FRAMEWORK   //�Զ���ģ��
	
	//if( power_on_allow_keypress == 1 )	
	{	
		/*
		���Ҷ�λ��������׼��ȷ���������й����У�ֻ��û���е�һ����Բ���������
		*/
		if( AUTO_LEFT_FOOTER_SWITCH == para.dvab_open_level )//���ļпۿ���
		{
			if( ((sys.status == READY)&&(left_footer_lock_flag == 0) )|| ( (left_footer_lock_flag == 0)&&( waitting_for_pattern_done != 1))  )
			{
				
				left_footer_counter++;
				if( left_footer_counter >= 100 ) //100ms
				{
					left_footer_lock_flag = 1;
				    left_footer_counter = 0;		
					if( left_footer_status == 0)
						{
							AUTO_LEFT_FRAME_HOLDER = 1;//�н�
							left_footer_status = 1;
							foot_flag = 0;
						}	
						else
						{
							AUTO_LEFT_FRAME_HOLDER = 0;//�ɿ�
							left_footer_status = 0;
							foot_flag = 1;
						}					
						   
				}
			}
		} 
		else 
		{
			left_footer_lock_flag = 0;
			left_footer_counter = 0;
		}
	
		if( AUTO_LEFT_RUNNING_SWITCH == para.dvab_open_level )//��������
		{
			
			if( (left_start_lock_flag == 0)&&( waitting_for_pattern_done != 1) )
			{
				//SUM =1;
				left_start_counter++;
				if( left_start_counter>=100 ) //100ms
				{
					left_start_lock_flag = 1;
				    left_start_counter = 0;		
					if( left_second_footer_status == 0)
					{
						AUTO_LEFT_FRAME_STANDBY = 1;					
						left_second_footer_status = 1;
						left_footer_delay_flag = 1;//
						left_footer_delay_counter = 0;
					}	
					else
					{
						AUTO_LEFT_FRAME_STANDBY = 0;				
						left_second_footer_status = 0;
						left_quest_running = 0;
						left_footer_delay_flag = 1;//
						left_footer_delay_counter = 0;
					}	   
				}
			}
			else if (waitting_for_pattern_done == 1)//������
			{
				left_start_counter++;
				if( left_start_counter>=100 ) //100ms
				{
					left_start_counter = 0;
					pedal_state = 1;
				}
			}
		} 
		else 
		{
			left_start_lock_flag = 0;
			left_start_counter = 0;
		}
	
		if( left_footer_delay_flag == 1)
		{
			left_footer_delay_counter++;
			if( left_footer_delay_counter >=200 )//200ms
			{
				left_footer_delay_flag = 0;
				if( left_second_footer_status == 1)
				{
					AUTO_LEFT_HOLDING_POSITION = 0; //��λ����ȥ��׼������
				    left_quest_running = 1;
					//SUM = 1;
				}
				else
				{
					AUTO_LEFT_HOLDING_POSITION = 1; //��λ������������λ��
					left_quest_running = 0;	
				}
			}
		}
		//===================================================================
		if( AUTO_RIGHT_FOOTER_SWITCH == 1 )
		{
			if( ((sys.status == READY)&&(right_footer_lock_flag == 0) )|| ((right_footer_lock_flag == 0)&&( waitting_for_pattern_done != 2)) )
			{
				right_footer_counter++;
				if( right_footer_counter >= 100 ) //100ms
				{
				
					right_footer_lock_flag = 1;
				    right_footer_counter = 0;
			
						if( right_footer_status == 0)
						{
							AUTO_RIGHT_FRAME_HOLDER = 1;
							right_footer_status = 1;
							foot_flag = 0;
						}	
						else
						{
							AUTO_RIGHT_FRAME_HOLDER = 0;
							right_footer_status = 0;
							foot_flag = 1;
						}	
					   
				}
			}
		} 
		else 
		{
			right_footer_lock_flag = 0;
			right_footer_counter = 0;
		}
	
		if( AUTO_RIGHT_RUNNING_SWITCH == 1 )
		{
			if( (right_start_lock_flag == 0)&&( waitting_for_pattern_done != 2) )
			{
				right_start_counter++;
				if( right_start_counter>=100 ) //100ms
				{
					//SUM = 1;
					right_start_lock_flag = 1;
					right_start_counter = 0;
					if( right_second_footer_status == 0)
					{
						AUTO_RIGHT_FRAME_STANDBY = 1;
						right_second_footer_status = 1;
						right_footer_delay_flag = 1;
						right_footer_delay_counter = 0;
					}	
					else
					{
						AUTO_RIGHT_FRAME_STANDBY = 0;
						right_second_footer_status = 0;
						right_footer_delay_flag = 1;
						right_footer_delay_counter = 0;
					}				   
				}
			}
			else if (waitting_for_pattern_done == 2)//������
			{
				right_start_counter++;
				if( right_start_counter>=100 ) //100ms
				{
					right_start_counter = 0;
					pedal_state = 1;
				}
			}
		} 
		else 
		{
			right_start_lock_flag = 0;
			right_start_counter = 0;
		}
	
		if( right_footer_delay_flag == 1)
		{
			right_footer_delay_counter++;
			if( right_footer_delay_counter >=200 )//200ms
			{
				right_footer_delay_flag = 0;
				if( right_second_footer_status == 1)
				{
					AUTO_RIGHT_HOLDING_POSITION = 0; //��λ����ȥ��׼������
				    right_quest_running = 1;
					//SUM = 1;
				}
				else
				{
					AUTO_RIGHT_HOLDING_POSITION = 1; //��λ������������λ��
					right_quest_running = 0;	
				}
			}
		}
		
	
	}
	
	#endif
	
	
	
	#if AUTO_CHANGE_FRAMEWORK ==0
	if(sys.status == READY)
	{	
		if(DVA == para.dvab_open_level)           					// foot sensor is pushed,
		{
			if(foot_ready_counter_dva++ >10)
			{
				if((DVA == para.dvab_open_level) && (DVALastState != para.dvab_open_level))
				{
					pedal_state = 1;
					DVALastState = DVA;								
				}
				foot_ready_counter_dva =10;
			}	
		}
		else
		{
		    foot_ready_counter_dva =0;	
			DVALastState = DVA;
		}
		
			
		if(DVB == para.dvab_open_level)           					// foot sensor is pushed,
		{
			if(foot_ready_counter_dvb++ >10)
			{
				if((DVB == para.dvab_open_level) && (DVBLastState != para.dvab_open_level))
				{
					pedal_state = 2;
					DVBLastState = DVB;								
				}
				foot_ready_counter_dvb =10;
			}	
		}
		else
		{
		    foot_ready_counter_dvb =0;	
			DVBLastState = DVB;
		}
		
	}
	else
	{
		pedal_state = 0;
	}
	#endif
}
//--------------------------------------------------------------------------------------
//  Name:	tb3_int
//  pars:	None
//  Returns:	None
//  Description: TB3 timer interrupt function(1ms)
//--------------------------------------------------------------------------------------
void tb3_int(void)
{	
	ms_counter++;	
	if(wipe_start_flag == 1)
	{
		wipe_time++;
	}
	else if(wipe_start_flag == 0)
	{
		wipe_time = 0;
	}
}

//--------------------------------------------------------------------------------------
//  Name:	tb4_int
//  pars:	None
//  Returns:	None
//  Description: TB4 timer interrupt function(100us)
//--------------------------------------------------------------------------------------
void tb4_int(void)
{
	UINT8 coder;
	us_counter++;
	#if INSERPOINT_ENABLE
	if( laser_cutter_aciton_flag == 1 )
	{
		if( rec1_total_counter > 0 )//����������
		{
			coder = tra1_buf[tra1_ind_r];
			tra1_ind_r = (tra1_ind_r+1 )%250;
			rec1_total_counter --;
			switch(coder)
			{
				case 1:
					  movestepx_time = 0;
					  movestep_x(1);
				break;
				case 2:
					  movestepx_time = 0;
					  movestep_x(-1);
				break;
				case 3:
					  movestepy_time = 0;
					  movestep_y(1); 
				break;
				case 4:	 
					  movestepy_time = 0;
					  movestep_y(-1);	  
				break;
			}
		}
	}	
	#endif			    	  		
}

//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------