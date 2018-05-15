//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : main.c
//  Description: Core program to control the sewing machine
//  Version    Date     Author    Description
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
// 	Includes
//--------------------------------------------------------------------------------------
#include "..\..\include\sfr62p.h"         // M16C/62P special function register definitions
#include "..\..\include\typedef.h"        // Data type define
#include "..\..\include\common.h"         // Common constants definition
#include "..\..\include\variables.h"      // External variables declaration
#include "..\..\include\motor.h"          // constants definition
#include "..\..\include\delay.h"          // delay time definition
#include "..\..\include\solenoid.h"       // solenoid driver definition
#include "..\..\include\communication.h"  // Communication function
#include "..\..\include\stepmotor.h"      // stepper motor function
#include "..\..\include\action.h"         // action function
#include "..\..\include\math.h"           // math library definition
#include "..\..\include\iic_bus_eeprom.h"


#define double_pedal 2
#define single_pedal 1
void process_nop_move_pause(UINT8 direction);

void trim_action(void);
void shift_func(UINT8 shift_num);
void single_move_func(void);
//==================================================================
UINT16 identify_pattern(void)
{
	UINT16 tmp_pattern;
	tmp_pattern =0;
	if(formwork_identify_device == 0) // 模版识别设备为传感器 
	{
		delay_ms(250);
					
		if(PORG == 1)
			  tmp_pattern |= 0x01;
		if(PSENS == 1)
			  tmp_pattern |= 0x02;
#if (JITOUBAN == 0)
		if(CORG == 1)
			  tmp_pattern |= 0x04;
		if(CSENS == 1)
			  tmp_pattern |= 0x08;
#else
		if(CORG == 1)
			  tmp_pattern |= 0x08;
		if(CSENS == 1)
			  tmp_pattern |= 0x04;
#endif
		if(ADTCSM )
			  tmp_pattern |= 0x10;
	}	 
	else 	
	    tmp_pattern = serail_number;	
		  
	return tmp_pattern;
}
//==================================================================

void pattern_process(void)
{
	UINT16 tmp_pattern,tmp_pattern2;
  
    tmp_pattern = identify_pattern();
	
	if( (formwork_identify_device == 0)&&(tmp_pattern!=0) )
	{
		if( pattern_change_flag == 0)
		{
			delay_ms(200);			
			tmp_pattern2 = identify_pattern();  
			if(  (tmp_pattern2 !=0)&&(tmp_pattern == tmp_pattern2) )
			{
				if( last_pattern_number != tmp_pattern2)
				{
				  if( already_in_origin == 0)                            
				  {
						sys.status = ERROR;
						StatusChangeLatch = ERROR;
						sys.error = ERROR_46; 
						predit_shift = 0;  

						pattern_number = 0;
						last_pattern_number = 0;
						serail_number = 0;
						new_pattern_done = 0;
						pattern_change_flag = 0;
						return;
				   }
				   pattern_change_flag = 1;
				   pattern_number = tmp_pattern2;
				   last_pattern_number = tmp_pattern2;
				   return_from_setout = 0;
				}
			}
		}
	}
	if ((formwork_identify_device != 0)&&(tmp_pattern!=0))
	{
			tmp_pattern2 = identify_pattern();  
			if(  (tmp_pattern2 !=0)&&(tmp_pattern == tmp_pattern2) )
			{
				if( last_pattern_number != tmp_pattern2)
				{
				     if( already_in_origin == 0)                            
					 {
						sys.status = ERROR;
						StatusChangeLatch = ERROR;
						sys.error = ERROR_46; 
						predit_shift = 0;  
		
						pattern_number = 0;
						serail_number = 0;
						new_pattern_done = 0;
						pattern_change_flag = 0;
						return;
					 }
				   pattern_change_flag = 1;//find new pattern
				   pattern_number = tmp_pattern2;
				   last_pattern_number = tmp_pattern2;
				   barcoder_time_between_same_code = 1;
				   pattern_change_counter = 0;
				   //return_from_setout = 0;
				   SUM =1;
				   rfid_alarm_counter = 800;
				   rfid_alarm_flag = 1;
				}
			}
		}	
}



void pre_running(void)
{
	UINT8 temp8;
						
	if(manual_operation_flag == 1 || pattern_change_flag == 1 || coor_com == 1 || PointShiftFlag == 1 || waitting_for_point_command == 1 )
	   return;
	pedal_state = 0;   
	if(((foot_flag==1) || (( (footer_working_mode != 0)||(u238 >= 1) ) && foot_half_flag ==1)) && (u202 ==0))
	{
		sys.status = ERROR;
		StatusChangeLatch = ERROR;
		status_now = READY;
    	sys.error = ERROR_45; 
		return;
	}

	if(already_in_origin ==0)
	{
		sys.status = ERROR;
		StatusChangeLatch = ERROR;
		sys.error = ERROR_46; 
		return;
	}

	if((footer_working_mode == 0 && foot_flag==0) || (footer_working_mode != 0 && foot_flag==0 && foot_half_flag == 0)||(u202 ==1) )
	{
		if(StopStatusFlag == 0 || StopStatusFlag == 1 && stop_number != 2) //不是从暂停启动，或者不是下暂停
		{
			temp8 = detect_position();
			if(temp8 == OUT)
			{
				find_dead_center();
			}
			if(inpress_flag == 0)
			{
				inpress_up();
				delay_ms(150);
			}
		}
		
		if(stay_end_flag == 1 && auto_function_flag == 0)
		{
			if(para.x_origin_mode == AUTO_FIND_START_POINT)
			{
				if (already_auto_find_start_point == 0)
				{
					find_start_point_x();
					already_auto_find_start_point = 1;
				}
			}
			if( sys.error == 0)
			go_setoutpoint();
			return;
		}
		
		if( (sewingcontrol_flag == 2)&&(sewingcontrol_stitchs != 0) )
		     need_backward_sewing = 1;  
	
							
		if( pat_point == sta_point)
		{	
			if(pat_point == (PATTERN_DATA *)(pat_buf))
			{
	      		last_pattern_point = pat_point;
				last_allx_step = allx_step;
				last_ally_step = ally_step;
				bakeup_total_counter = pat_buff_total_counter;
				
				if(para.x_origin_mode == AUTO_FIND_START_POINT)
				{
					if (already_auto_find_start_point == 0)
					{
						find_start_point_x();
						already_auto_find_start_point = 1;
					}
				}
				if( sys.error == 0)
				go_startpoint();	
				if( nop_move_pause_flag == 1)
					return;			
				if( ((origin2_lastmove_flag == 1)&&(pat_point!=(PATTERN_DATA *)(pat_buf)))|| ( (StopStatusFlag == 1)&&(pat_point!=(PATTERN_DATA *)(pat_buf) )) )
				{
					predit_shift = 0;
					delay_ms(10);
					DVALastState = DVA;
					AIR_OUT = 0;
					return; 
				}
			}
			else
			{
				process_data();//refresh flags
                do_pat_point_sub_one();
			}
		}

		if(origin2_lastmove_flag == 1)//start from the second origin point
		{							
				process_data();
				if(origin2_lastmove_flag == 1)						
				{
            		process_data();
					origin2_lastmove_flag = 0;
				}
				if(nopmove_flag == 1)
				{	  
					while(nopmove_flag == 1 )
					{
						//pat_point--;
						do_pat_point_sub_one();
						go_beginpoint(0);
						delay_ms(30);
						process_data();
						if(nopmove_flag == 0 )
						{
							//pat_point--;
							do_pat_point_sub_one();
							if(stop_flag == 1)
							{
								StopStatusFlag = 1;
							}
							break;
						}
					}
				}
				else
					//pat_point--;
					do_pat_point_sub_one();
											
		}
		else if(StopStatusFlag == 1)	//start from stop condicton,stay in READY
		{
			process_data();			//jump over stop code
			process_data();
			if(nopmove_flag == 1)
			{	  
				while( nopmove_flag == 1 )
				{
					//pat_point--;
					do_pat_point_sub_one();
					go_beginpoint(0);
					delay_ms(30);
					process_data();
					if( nopmove_flag == 0 )
					{
						//pat_point--;
						do_pat_point_sub_one();
						break;
					}
				}
			}
			else 
				//pat_point--;
				do_pat_point_sub_one();
		}
		  
		if(stop_flag == 1 || origin2_lastmove_flag ==1 )
		{
		   sys.status = READY;
		   StatusChangeLatch = READY;
			if(stop_flag == 1)
			{
				StopStatusFlag = 1;
				if(u208 == 1)
				{
					footer_both_up();
				}
				if(stop_number == 2)			//dstp
				{
					if(inpress_flag ==1)
					{
						inpress_down(inpress_high_base);
						delay_ms(150);
					}
				  	needle_down();
				}
				if(pat_point->para == 0)     //2012-5-14
					footer_both_down();
				else if(pat_point->para == 1)
					footer_both_up();
			}
		}
		else
		{
			if(nop_move_pause_flag ==1)
		       process_nop_move_pause(1);	
			if( nop_move_pause_flag ==0 )
			{
				
				sys.status = RUN;
				StatusChangeLatch = RUN;
				StopStatusFlag = 0;	 
			
				inpress_high = inpress_high_base;
			
				if(aging_flag==1)
				{
					aging_com=1;
				}	  
			} 
		}  		    			  	
	}		    	    	      	    
					  	
}

void process_nop_move_pause(UINT8 direction)
{
	UINT16 i,temp16_x,temp16_y,temp16_max,quick_time,limit_value;
	INT16 tempx_step,tempy_step;	
	UINT16 j,tmpx,tmpy;	
	nop_move_pause_flag = 0;
	
		inpress_up();        
		delay_ms(200);         
	if( direction == 2)//需要从空送停止位置返回到空送的原起点
	{
		//1.0 计算要回退多少距离
		temp16_y = fabsm(read_step_y);
		temp16_x = fabsm(read_step_x);
		tempx_step = -read_step_x;
		tempy_step = -read_step_y;
	}
	else
	{
		//1.0 计算还有多少距离没走
		temp16_y = fabsm(nop_move_remainy);
		temp16_x = fabsm(nop_move_remainx);
		tempx_step = nop_move_remainx;
		tempy_step = nop_move_remainy;
	}
	//3.0 看要走距离是采用快走 还是 普通车缝协议
	if(temp16_x > temp16_y)
  	{
  		temp16_max = temp16_x;
  	}
  	else
  	{
  		temp16_max = temp16_y;
  	}

   	limit_value = QUICKMOVE_JUDGEMENT;
	if(temp16_max>0)
	{
		quick_time = Calculate_QuickMove_Time(temp16_x,temp16_y);
	}
	//4.0 执行真正的X、Y轴动作
	if(temp16_y>0)
	{
		y_quickmove(quick_time,tempy_step);
	}
	delay_ms(1);
    if(temp16_x>0)
	{
		x_quickmove(quick_time,tempx_step);
	}
	//5.0  延时等待框架的动作完成 
	
	if( quick_time < 98)
	    quick_time = 98;	 
	for(i=0;i<quick_time - 98;i++)//58
	{
	     delay_ms(1);
		 #if NOPMOVE_STOP_ENABLE
		 if(PAUSE == PAUSE_ON)   
		 {
		     delay_ms(2);
		     if(PAUSE == PAUSE_ON)
		     {				
					 nop_move_pause_flag = 1; 					 
				     nop_move_emergency(temp16_x,temp16_y); //STOP MOVING
					 for(j=0;j<2000;j++)
					 {
						delay_ms(1);

						if( check_motion_done() )
						   break;

				  	 }
				     if( j == 2000)
					 {
					 	   sys.error = ERROR_70;//ERROR_15;
						   special_go_allmotor_flag = 1;
						   sys.status = ERROR;
						   return;
					 }
					 tmpx = 0;
					 tmpy = 0;
					 if( temp16_x > QUICKMOVE_JUDGEMENT )
					     tmpx = get_x_distance();					 
					 else if( temp16_x != 0)
					     tmpx = temp16_x;
					 else
					 	 tempx_step = 0;
						 
					 delay_ms(10);
					 
					 if( temp16_y > QUICKMOVE_JUDGEMENT  )
					     tmpy = get_y_distance();
					 else if( temp16_y != 0 )
					     tmpy = temp16_y;					 
					 else
						 tempy_step = 0;
										 
					 if( tmpx == STEPPER_IGNORE_STOP )
					     tmpx = temp16_x;					 
					 if( tmpy == STEPPER_IGNORE_STOP )
					     tmpy = temp16_y;
					 
					 //从上次的空送急停点到这次更新点，形成新的更新点坐标。
					 if( direction == 2)
					 {
						 if( read_step_x >= 0)//之前向左移动，接下来向右后退
						 {
							 read_step_x -= tmpx;
							 nop_move_remainx += tmpx;
						 }
						 else
						 {
							 read_step_x += tmpx;
							 nop_move_remainx -= tmpx;
						 }	  
						 if( read_step_y >= 0)
						 {
					 	 	 read_step_y -= tmpy;					 
						 	 nop_move_remainy += tmpy;
						 }
						 else
						 {							 
							 read_step_y += tmpy;					 
						 	 nop_move_remainy -= tmpy;
						 }
					 }
					 else
					 {
						 if( read_step_x >= 0) //之前的坐标是向右移动，接着继续向右移动
						 {
						     read_step_x += tmpx;
							 nop_move_remainx -= tmpx;
						 }
						 else 
						 {
						     read_step_x -= tmpx;
							 nop_move_remainx += tmpx;
						 }
						 if( read_step_y >= 0)
						 {
					 	 	 read_step_y += tmpy;					 
						 	 nop_move_remainy -= tmpy;
						 }
						 else
						 {
							 read_step_y -= tmpy;					 
						 	 nop_move_remainy += tmpy;
						 }
					 }
					 
					 break;
		  	     }
			 }
		 
		#endif
	  }


	 if( nop_move_pause_flag == 0)
	 {
		for(i=0;i<quick_time;i++)
		{
				delay_ms(1);
				if( check_motion_done() )
				{
				   delay_ms(50);
				   break;
				}
		}
		if( direction == 2)//需要从空送停止位置返回到空送的原起点
		{
			//2.0 恢复起点的花样指针、坐标、处理总针数
			pat_point = last_pattern_point;
			allx_step = last_allx_step;
			ally_step = last_ally_step;
			pat_buff_total_counter = bakeup_total_counter;
		}
		else //从空送停止位置向前移动
		{
			//2.0 如果采用的是头部正反加固，恢复真正结束点的指针、坐标、总针数
			//if( (sewingcontrol_flag == 2)&&(need_backward_sewing == 1) )
			{
				pat_point = target_pat_point;
	    		allx_step = target_allx_step;
				ally_step = target_ally_step;
				pat_buff_total_counter = target_total_counter;
			}
		}
	}

	
    
}
//--------------------------------------------------------------------------------------
//  Name:		ready_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of ready stauts
//--------------------------------------------------------------------------------------

void ready_status(void)
{   
	UINT8 temp8,tmp_pattern;
	UINT16 temp16,i;	
	INT16 tempx_step,tempy_step; 
	
#if CHECKBOARD
	OW_random();
#endif      	
 
    if(motor.stop_flag == 0 || motor.spd_obj != 0||motor.spd != 0)
	{
		sewing_stop();      	        	  
	    while(motor.stop_flag == 0)
		{
			rec_com();
		}
	}
	
	//if( auto_function_skip_flag == 1)
	//	SUM =1;	
	//else
	//	SUM = 0;				 
	
	#if DA0_OUTPUT_IMMEDIATELY
			if( temp_tension != tension_release_value)
			{
				da0 = 250;
				temp_tension = tension_release_value;
				test_action_counter = 0;
				test_action_flag = 1;
				at_solenoid();
			}
	#endif
	
  	if(PAUSE == PAUSE_ON )   
	{
		delay_ms(100);
		if(PAUSE == PAUSE_ON)
		{				
			if(sys.status != ERROR)
			   status_now = sys.status;
			else
			   status_now = READY;
				   
			sys.status = ERROR;
			StatusChangeLatch = ERROR;
			if( sys.error == 0 )
	    		sys.error = ERROR_19;	  			  	
			single_flag=0; 			  	
			predit_shift = 0;
			pause_flag = 0;
			manual_cut_flag = 0;
	    	return;
			/*
			if(manual_cut_flag == 0) 
			{
				if(sys.status != ERROR)
				   status_now = sys.status;
				else
				   status_now = READY;
				   
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
				if( sys.error == 0 )
	      			sys.error = ERROR_19;	  			  	
				single_flag=0; 			  	
				predit_shift = 0;
	      		return;
			}
			else
			{
				manual_cut_flag = 0;
				rec_com();
				trim();				
				pause_flag = 0;
			}
			*/	
			aging_com = 0;
			aging_flag = 0;
	  	}
  	}
		
 
    if(cut_test_flag == 1)
	{
 		cut_test_flag = 0;
		if( (nop_move_pause_flag ==0 )&&(finish_nopmove_pause_flag ==0) )
		{
			if( (foot_half_flag == 1) || (foot_flag == 1))
		         footer_both_down();
			delay_ms(100);
		    inpress_down(inpress_high_base);
			delay_ms(50);
			motor.dir = 0;
			motor.spd_obj = 10*u211; 
			while(1)
		    {
		    	rec_com(); 
			   	if(motor.spd_ref == motor.spd_obj)
			   	{
			    	break;
			   	}
		    }
		 	trim_action();
			delay_ms(100);
			inpress_up();
			delay_ms(100);
		}
	}
	if((shift_flag == 0x88)&&(shift_flag_old&0xf0))
	{
		shift_flag = shift_flag_old;
		shift_low_flag = 1;		
	}	
	if(shift_flag != 0)
	{
		if(foot_flag == 1)
		{		 
		  	footer_both_down();       	
			delay_ms(200);
		}

		shift_func(shift_flag);
	}
	
	#if AUTO_CHANGE_FRAMEWORK
	
	
	if( waitting_for_pattern_done == 0)//扫描到新条码，并且已经下载了
	{
		//if( power_on_allow_keypress == 1 )
		{
			if( left_quest_running == 1)//先左后右
			{			
				waitting_for_pattern_done = 1;
				delay_ms(100);
				take_frame_from_one_side(1);//从左边拿模板过来				
			}
			else if(right_quest_running == 1)
			{				
				waitting_for_pattern_done = 2;
				delay_ms(100);
				take_frame_from_one_side(2);//从右边拿模板过来					
			}	
		}
	}
	else
	{
	
		if( new_pattern_done == 1 )
		{
			new_pattern_done = 0;
			pattern_change_flag = 0;			
			pattern_change_counter = 0;				
			pattern_number = 0;			
			return_from_setout = 1;
			ready_go_setout_com = 0;
			foot_flag = 0;							
			//temp16 = (UINT16)delay_start_time * 100;	
			//delay_ms(temp16);
			if( nop_move_pause_flag ==1)
		    	process_nop_move_pause(1);
			if(nop_move_pause_flag ==0)
				pre_running();
			return;	
			
		}	
	}
	#else
	if( new_pattern_done == 1 )
	{
		if( request_rfid_number == last_pattern_number)
		{
			pattern_change_flag = 0;					
			pattern_number = 0;				
		}
		new_pattern_done = 0;
		pattern_change_counter = 0;	
		return_from_setout = 1;
		ready_go_setout_com = 0;
	}	
	#endif
	/*
	Last sew cycle the footer keep down,press pedal release footer.
	*/
	if(FootUpCom == 1)
	{
		if(u38 == 0)//No need to taken up the footer when sewing done!
		{
				//--------------------------------------------------------------------------------------
			  	//  foot sensor
			  	//-------------------------------------------------------------------------------------- 
			  	if(DVB == para.dvab_open_level)           				
				{
					delay_ms(10);
					if(DVB == para.dvab_open_level && DVBLastState != para.dvab_open_level)
					{				
						if ( footer_working_mode == 0)//the pedal only for footer
						{
							foot_up();
							FootUpCom = 0;
						}
						else if(LRfooter_down_mode == 1 )
						{
							foot_half_up();
							delay_ms(10);
							foot_up();
							FootUpCom = 0;
						}
						else
						{
							foot_up();
							delay_ms(10);
							foot_half_up();
							FootUpCom = 0;
						}
				  	}
			  	}
			 	DVBLastState = DVB;
				rec_com();               				
	   
   		}
  		else
		{
			FootUpCom = 0;
		}
 		//--------------------------------------------------------------------------------------
	  	//  switch system status 
	  	//-------------------------------------------------------------------------------------- 
		if(StatusChangeLatch != READY)
		{
			predit_shift = 0;
			sys.status = READY;
			StatusChangeLatch = READY;
			return;
		}
	}
	else
	{
		//--------------------------------------------------------------------------------------
	  	//  switch system status 
	  	//--------------------------------------------------------------------------------------  	
	 
		if((aging_com == 1)&&(aging_selection ==0 ) )
	  	{
	  		temp16 = (UINT16)aging_delay * 100;
	  		if(temp16 > 9900)
	  		{
	  			temp16 = 9900;
	  		}
	  		delay_ms(temp16);  
			foot_com = 0;

			if ( already_in_origin == 0)
				{
					sys.status = ERROR;
					StatusChangeLatch = ERROR;
	      			sys.error = ERROR_46; 
					predit_shift = 0;  
					inpress_com=0;
					aging_com = 0 ;
					return;
				}
	  	}
		if(inpress_com ==1 )
		{
			if(stop_number == 2)
			{ }
			else
			{   	
				temp8 = detect_position();	
		    	if(temp8 == OUT)     
		      	{
					find_dead_center();
		      	}
			}
			if(inpress_first_flag == 1)
			{
			   go_origin_zx();
			   inpress_first_flag = 0;
			}
			else
			{
				  if(stop_number == 2)
			        { }
				  else
			        inpress_up();
			}				
            delay_ms(100); 
			inpress_com=0;
		}
	 	//--------------------------------------------------------------------------------------
	  	// inpress down or up
	  	//-------------------------------------------------------------------------------------- 
		if(inpress_action_flag == 1)
		{
			if( ( already_in_origin == 0)&&(u224 ==2) )
			{
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
      			sys.error = ERROR_46;
				predit_shift = 0;
				inpress_action_flag =0;
				return;
			}
			if(inpress_first_flag == 1)
			{
				go_origin_zx();
				inpress_first_flag = 0;
                delay_ms(100);
			}
			if(inpress_high < 0)
			{
				if(( steper_footer_position !=0 )&&(u224 ==2) )
				{
					foot_down();
					delay_ms(100);
				}
				foot_down();       
				delay_ms(100);
				inpress_down(inpress_high_base);      
				if( k03 == MECHANICAL )
				{
					if(tension_release_time >0 )
					{
						tension_open_counter = 0;
						thread_switch=1;
				        tension_open_switch = 1;
			         	DAActionFlag=0;
				        da0 = tension_release_value + 50;
					}
				}
				else
				     da0 = 0 ;
					 
				manual_operation_flag = 1;
				delay_ms(200);
			}
			else if(inpress_high == 81)
			{
		 		temp8 = detect_position();
	    		if(temp8 == OUT)
	      		{
					find_dead_center();
	      		}
				inpress_up();
                delay_ms(200);

				tension_open_counter = 0;
			    tension_open_switch = 0;
				thread_switch=0;
		        DAActionFlag=0;
			    da0 = 0;
				manual_operation_flag = 0;
			}
			inpress_action_flag = 0;
			predit_shift = 0;//?????
		}
		//------------------------------------------------------------------------------------
		//stitch up command
		//------------------------------------------------------------------------------------	
		
	 	if(StitchUpFlag ==1)
		{
			if ( already_in_origin == 0)
			{
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
      			sys.error = ERROR_46; 
				predit_shift = 0;  
				StitchUpFlag = 71;
				return;
			}
			delay_ms(150);
			temp8 = detect_position();	
	    	if(temp8 == OUT)     
	      	{
				keep_running_for_dead_center();
	      	}
			
			inpress_up();
			delay_ms(300);
			StitchUpFlag = 71;
			
		}
		else if(StitchUpFlag ==0)
		{
			if ( already_in_origin == 0)
			{
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
      			sys.error = ERROR_46; 
				predit_shift = 0;  
				StitchUpFlag = 71;
				return;
			}
			  if(inpress_first_flag == 1)
			  {
				go_origin_zx();
				inpress_first_flag = 0;
                delay_ms(100);
			  }
			  else
			  {
			     foot_down();
				 delay_ms(100);
			  }
			  inpress_down(inpress_high_base);
		      delay_ms(150);			  
			  needle_down();			  
			  delay_ms(500);
			  StitchUpFlag = 71;
		}
		//--------------------------------------------------------------------------------------
	  	// foot auto down or up,action same time!
	  	//-------------------------------------------------------------------------------------- 
	  	switch(foot_com)
	  	{
	  		case 0:                         	// command down  
	  			if(foot_flag == 1)     			
	  	        {	
					    footer_both_down();
		  	     }
	  	        break;	         
	    	case 1:                            // command up
	    	  	if(foot_flag == 0)     		   
	  	        {	
					   footer_both_up();
				}	
	  	        break;		
  		    default: 
			   break;                   	         	
	  	}  
		if( ready_go_setout_com  ==1)
		{
			if(u213 <250 && stretch_foot_enable ==1 && ready_go_setout_com  !=1)
			{
				if(((foot_flag==1) || (( (footer_working_mode != 0)||(u238 >= 1) ) && foot_half_flag ==1)) && (u202 ==0))
				{
					sys.status = ERROR;
					StatusChangeLatch = ERROR;
					status_now = READY;
			    	sys.error = ERROR_45; 
					ready_go_setout_com = 0;
					return;
				}
		   	
			}
			go_origin_allmotor();
			
			if( already_in_origin ==0 )
		    {
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
		   		sys.error = ERROR_46; 
				predit_shift = 0;  
				ready_go_setout_com = 0;
				return;
		    }
			if(foot_flag == 1)
			{
			  	footer_both_down();       
				delay_ms(200);
			}
			temp8 = detect_position();	           
			if(temp8 == OUT)    
	  		{
			    find_dead_center();
	  		}
			if(inpress_flag == 0)
			{
				inpress_up();        	
				delay_ms(100);
			}
			if( nop_move_pause_flag ==1)
		    	process_nop_move_pause(1);	
			if( nop_move_pause_flag ==0 )
			{
				if(para.x_origin_mode == AUTO_FIND_START_POINT)
				{
					if (already_auto_find_start_point == 0)
					{
						find_start_point_x();
						already_auto_find_start_point = 1;
						delay_ms(50);
					}
				}
				if( sys.error == 0)
				go_setoutpoint();
			}
			/*
			if( sys.status != PREEDIT)
			{
				if( origin_footer_status == 0)
					footer_both_down();
				else
					footer_both_up();
			}
			*/	
			ready_go_setout_com = 0;
			predit_shift = 0;
			fun_default_flag = 0;
			return_from_setout = 1;
		}
		//------------------------------------------------------------------------------------
		//xy steper find origin point command
		//------------------------------------------------------------------------------------	
		if(origin_com == 1 )		
		{		 		  
			if( nop_move_pause_flag ==1)
			{
				process_nop_move_pause(1);
			} 	  
			go_origin_allmotor();
			#if AUTO_CHANGE_FRAMEWORK   //定位柱升起来
			//if( power_on_allow_keypress == 0)
			{
			  AUTO_LEFT_HOLDING_POSITION = 1;
			  AUTO_RIGHT_HOLDING_POSITION = 1;
			  AUTO_LEFT_FRAME_HOLDER = 0;
			  left_footer_status = 0;
			  AUTO_RIGHT_FRAME_HOLDER = 0;
			  right_footer_status = 0;
			  right_quest_running = 0;
			  left_quest_running = 0;
			  waitting_for_pattern_done = 0;
			}
			power_on_allow_keypress = 1;
			#endif
			
			
			if(sys.status == ERROR)
				return;

			last_pattern_number = 0;
			origin_com = 0;
			not_in_origin_flag = 0;

			if ( already_in_origin == 0)
			{
				return;
			}			
			fun_default_flag = 0;

			if((aging_flag==1)&&(aging_mode == 1)&&(unaging_flag ==0))
			{
		        if( u224 == 2) 
				    delay_ms(300);
		        footer_both_down();
			    aging_com=1;
			}
			unaging_flag = 0;
		}
	#if AUTO_CHANGE_FRAMEWORK
		if((aging_com == 1)&&(aging_selection ==0 ) )
		{
			if( (foot_flag==0)||((foot_flag==1)&&(u202 ==1)) )
			{
				delay_ms(500);
				temp8 = detect_position();	
	    		if(temp8 == OUT)     
	      		{
					find_dead_center();
	      		}	
				tension_open_switch =0;
				tension_open_counter =0;
				da0 =0;
				if ( finish_nopmove_pause_flag == 1 )
				{
					   origin_com =1;
					   //special_go_allmotor_flag = 1;
				}
				else
				{
					  if( nop_move_pause_flag ==1)
			    			  process_nop_move_pause(1);
					  if( nop_move_pause_flag ==0 )
	                      	 pre_running();//
				}
	    	}
		}
	#else	
	  	if((aging_com == 1)&&(aging_selection ==0 ) )
		{
			if( (foot_flag==0)||((foot_flag==1)&&(u202 ==1)) )
			{
				delay_ms(500);
				temp8 = detect_position();	
	    		if(temp8 == OUT)     
	      		{
					find_dead_center();
	      		}	
				tension_open_switch =0;
				tension_open_counter =0;
				da0 =0;
				if ( finish_nopmove_pause_flag == 1 )
				{
					   origin_com =1;
					   //special_go_allmotor_flag = 1;
				}
				else
				{
					  if( aging_selection == 1)//单主轴老化
					  {
						  
					  }
					  else
					  {
						  if( nop_move_pause_flag ==1)
			    			  process_nop_move_pause(1);
						  if( nop_move_pause_flag ==0 )
	                      	 pre_running();//
					  }
				}
	    	}
		}
	#endif	
		else
		{	
			//--------------------------------------------------------------------------------------
		  	//  switch system status 
		  	//-------------------------------------------------------------------------------------- 
			if(StatusChangeLatch != READY)
			{
				predit_shift = 0;
				sys.status = StatusChangeLatch;
				return;
			} 	
			else
			{
				//--------------------------------------------------------------------------------------
				//  foot sensor
				//-------------------------------------------------------------------------------------- 
				if(pedal_state == 2)           					// foot sensor is pushed,
				{
					if(manual_operation_flag == 0)
					footer_procedure();	
					single_flag=0;
					pedal_state=0;			
				}
						
				if( software_key_footer == 1)
				{
					software_key_footer = 0;
					footer_procedure();	
					single_flag=0;			
				}
				if( special_machine_type == 0)
				{
					if(DVSM == 0)
					{
						delay_ms(10);
						if(DVSM == 0 && DVSMLastState == 1)
						{		
							if(foot_half_flag == 1)
							{
							  	if(foot_flag == 0)//2012-4-1 only footer is down
								{
								   foot_half_down();
								}
							}
							else
						    		foot_half_up();
							
							DVSMLastState = 0;		    	    
						}
					}
					else 
					    DVSMLastState = 1;		    	    
				}
				//--------------------------------------------------------------------------------------
				//  start sensor
				//--------------------------------------------------------------------------------------  
				//if( (DVA == para.dvab_open_level)&&(DVB != para.dvab_open_level)&&(single_flag ==0)) 
				if((pedal_state ==1)&&(single_flag ==0))
				{				
					pedal_state = 0;
					//SUM = 1;
					if(already_in_origin ==0)
					{
						sys.status = ERROR;
						StatusChangeLatch = ERROR;
      					sys.error = ERROR_46; 
						return;
					}
					if((footer_working_mode == 0 && foot_flag==0) || (footer_working_mode != 0 && foot_flag==0 && foot_half_flag == 0)||(u202 ==1) )//2010-7-14
					{		
						if ( finish_nopmove_pause_flag == 1 )
						{
							origin_com =1;
							//special_go_allmotor_flag = 1; 							
						}
						else
						{
							if( nop_move_pause_flag ==1)
		    					process_nop_move_pause(1);
							if( nop_move_pause_flag ==0)
							{	
								if((formwork_identify_device ==2)&&(auto_function_flag == 1)&&( return_from_setout ==1)&&( auto_function_skip_flag == 1 ))
								{
									if( serail_number != 0 )
										pre_running();
									else 
									{
										sys.status = ERROR;
										StatusChangeLatch = ERROR;
      									sys.error = ERROR_91; 	
										return;
									}
								}
								else
									pre_running();
							}
							return;
                        }
					}
						  		
					else if( (delay_start_function ==1)&&(single_flag ==0)) 
					{
						if( (foot_flag == 1))										
							footer_both_down();																		
						temp16 = (UINT16)delay_start_time * 100;	
						delay_ms(temp16);
						if( nop_move_pause_flag ==1)
		    				process_nop_move_pause(1);
						if(nop_move_pause_flag ==0)
							pre_running();//
						return;								
						      
					}
					if(((foot_flag==1) || (footer_working_mode != 0 && foot_half_flag ==1)) && (u202 ==0))//warning!//2010-7-21
					{
						sys.status = ERROR;
						StatusChangeLatch = ERROR;
						status_now = READY;
      					sys.error = ERROR_45; 
						return;
					}
						
			}

			if( software_key_run ==  1)
			{
				software_key_run = 0;
				if( already_in_origin ==0)
				{
					sys.status = ERROR;
					StatusChangeLatch = ERROR;
      				sys.error = ERROR_46; 
					return;
				}
				if(((foot_flag==1) || (footer_working_mode != 0 && foot_half_flag ==1)) && (u202 ==0))
				{
					sys.status = ERROR;
					StatusChangeLatch = ERROR;
					status_now = READY;
      				sys.error = ERROR_45; 
					return;
				}
				if( (footer_working_mode == 0 && foot_flag==0) || (footer_working_mode != 0 && foot_flag==0 && foot_half_flag == 0)||(u202 ==1) )//2010-7-14
				{
					if ( finish_nopmove_pause_flag == 1 )
					{
						 origin_com =1;										
					}
					else
					{
					  	if( nop_move_pause_flag ==1)
		    				process_nop_move_pause(1);
						if( nop_move_pause_flag ==0)
						{	
							#if ( AUTO_CHANGE_FRAMEWORK ==0 )
							if( (formwork_identify_device ==2)&&(auto_function_flag == 1)&&( pat_point == sta_point) )
							{
								if( serail_number != 0 )
								{
								    pre_running();
									return;
								}
							}
							#else
							pre_running();
							return;
							#endif
						}
						
                    }
				}				
			}				
			rec_com();
			
			//--------------------------------------------------------------------------------------
		  	//  coordinate command 
		  	//--------------------------------------------------------------------------------------
			if( coor_com  == 1)
		  	{
				temp8 = detect_position();	      
		        if(temp8 == OUT)    
		        {
				    find_dead_center();
		        }
				footer_both_down();
				inpress_up();
				delay_ms(150);
				predit_shift = 1;
				if( nop_move_pause_flag ==1)
				{
					process_nop_move_pause(1);
				} 
				if(para.x_origin_mode == AUTO_FIND_START_POINT)
				{
					if (already_auto_find_start_point == 0)
					{
						find_start_point_x();
						already_auto_find_start_point = 1;
					}
				}
				if( sys.error == 0)
		  		go_commandpoint(comx_step,comy_step);
				predit_shift = 0;
				//delay_ms(500);	
				
				need_action_once = 0;//跳转以后不能直接加固，要判定位置
				return_from_setout = 0;
				coor_com = 0;
				waitting_for_point_command = 1;
		  	}
			
			if (PointShiftFlag == 1)
			{
				if( already_in_origin ==0 )
				{
					sys.status = ERROR;
					StatusChangeLatch = ERROR;
	      			sys.error = ERROR_46; 
					predit_shift = 0;  
					single_flag = 0;
					return;
				}

				if(super_pattern_flag != 1)
				{
					temp8 = detect_position();	            
			        if(temp8 == OUT)    
			        {
					    find_dead_center();
			        }
					footer_both_down();
					inpress_up();
					delay_ms(120);
					tempx_step = 0;
					tempy_step = 0;
					pat_point = (PATTERN_DATA *)(pat_buf);
					for(i=0;i<PointShiftNumber;i++)
					{
						process_data();
						if( end_flag ==1 )
						{
							//pat_point -- ;
							do_pat_point_sub_one();
							break;
						}
						tempx_step += xstep_cou;
						tempy_step += ystep_cou;
					}
					go_commandpoint(tempx_step,tempy_step);
				}
				else
				{	
					pat_point = (PATTERN_DATA *)(pat_buf) + PointShiftNumber%6000;
					pat_buff_total_counter = PointShiftNumber;
					PointShiftNumber = 0;
				  
				}
				PointShiftFlag = 0;
				waitting_for_point_command = 0;
			}
			//--------------------------------------------------------------------------------------
  			//  single step move 
  			//--------------------------------------------------------------------------------------
			single_move_func();
				
			}//sys.status already in READY
		}//working mode not aging mode
	}//not footup commadnd
    
}
//剪线模式选择
void trim_io_control(UINT8 a)
{
	switch(cut_mode)
	{
		case AIR_CUTTER:
			 L_AIR = a;	
		break;
		case SOLENOID_CUTTER:
			 FA = a;	
		break;
		case STEPPER_MOTER_CUTTER:
			 L_AIR = a;
		break;
	}
}


void trim_action(void)
{
	UINT8 temp8,flag;
	INT16 s8tmp;
	INT16 temp16;	
	UINT8 action0_flag , action1_flag , action2_flag,action3_flag,action4_flag,action5_flag;
	
	action0_flag = 0;
	action1_flag = 0;
	action2_flag = 0;
	action3_flag = 0;
	action4_flag = 0;
	action5_flag = 0;
	motor.dir = 0;
	motor.spd_obj = 10*u211; 
	
#if FOLLOW_INPRESS_FUN_ENABLE
	if( inpress_follow_high_flag == FOLLOW_INPRESS_HIGH )
	{
		movestep_zx(-inpress_follow_range,inpress_follow_down_speed);
		inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
	}

#endif
	temp16 = motor.angle_adjusted;
	if( (temp16 > cut_start_angle)||(temp16 > 1200) )//
	{
		while(temp16 >16 )
		{
			  rec_com();
			  temp16 = motor.angle_adjusted;
		}
	}
		
	temp16 = motor.angle_adjusted;
	while( temp16 < 1400 )//350
	{
		 flag_start_waitcom = 1;
	     if(flag_wait_com == 1) 
	     {
		    rec_com();
			counter_wait_com = 0;
			flag_wait_com = 0;
		 }
		 //1 剪线
		 if( (temp16 >= cut_start_angle) &&( action0_flag == 0) )
		 {
			 action0_flag = 1;
			 if( u210 == 1)
			 {
				 if( cut_mode != STEPPER_MOTER_CUTTER )
				 {
					 CutActionCounter = 0;
					 cut_pwm_counter = 0;
					 trim_io_control(ON);
					 CutActionFlag=1;
				 }
				 else
				 {
					 #if MACHINE_14090_MASC_PLUS
					 movestep_yj(stepper_cutter_move_range ,stepper_cutter_move_time);//平刀的分线过程
					 cutter_delay_counter = u222;//stepper_cutter_move_time<<1;
					 cutter_delay_flag = 1;
					 #else
					 movestep_yj(-90 ,stepper_cutter_move_time);
					 cutter_delay_counter = u222;//stepper_cutter_move_time<<1;
					 cutter_delay_flag = 1;
					 #endif
					 
				 }
			 }
		 }
		 //2 松线
		 temp16 = motor.angle_adjusted;
		 if( (temp16 >= tension_start_angle) &&( action1_flag == 0) )
		 {
			 action1_flag = 1;
			 if( u210 == 1)
			 {
				 if( k03 == MECHANICAL )
				 {
				   	 da0 = 255;
					 tension_open_counter = 0;	
				   	 tension_open_switch = 1;
				     DAActionFlag = 1; 
				     DAActionCounter =0;
				 }
				 else
				 {
					 temp_tension = cut_tension;    
					 temp_tension_last = cut_tension;   
					 da0 = temp_tension;
				 }
			 }			 
		 }
		 //3 扣线
		 #if 0
		 if( holding_bobbin_start_angle > 0)
		 { 
			 temp16 = motor.angle_adjusted;
			 if( (temp16 >= holding_bobbin_start_angle) &&( action2_flag == 0) )
			 {
				 action2_flag = 1;
				 HOLDING_BOBBIN_SOLENOID = 1;
				 CutActionFlag = 1;
				 CutActionCounter = 0;			
			 }
		 }
		#endif
		 //4 夹线
		 temp16 = motor.angle_adjusted;
		 if( (thread_holding_switch ==1 )&&(temp16 >= fw_start_angle) &&( action4_flag == 0) )
		 {
			 action4_flag = 1;
			 FW = 1; 
			 fw_action_flag = 1;
			 fw_action_counter = 0;				
		 }
		 
		 //退出
		 temp16 = motor.angle_adjusted;
		 if( (temp16 >= 1200) &&( action3_flag == 0) )
		 {
			 action3_flag = 1;
			 //sewing_stop();
		 }
	}
	
	sewing_stop();
	
	if( cut_mode == STEPPER_MOTER_CUTTER )
	{
		#if MACHINE_14090_MASC_PLUS
			flag = 1;
			if( (u206 == 1)&&(u210 ==1) )
			{
				flag = 0;
				if( para.wipper_type == AIR_WIPPER)
					AIR_FW = 1;
				else
				{
					SNT_H = 1; 
					FW = 1;
				}
			}	
			
			while( cutter_delay_flag == 1)
			{
				rec_com();
			}
			action5_flag = 1;
			movestep_yj(-stepper_cutter_move_range ,stepper_cutter_move_time);//开始断线
			cutter_delay_counter = stepper_cutter_move_time;
			cutter_delay_flag = 1;
				 
			HOLDING_BOBBIN_SOLENOID = 1;
			CutActionFlag = 1;
			CutActionCounter = 0;
			/*
			while( motor.stop_flag == 0)//等待停车到位
			{
				if( cutter_delay_flag == 0)	
				{
					 action5_flag = 1;
					 movestep_yj(-stepper_cutter_move_range ,stepper_cutter_move_time);//开始断线
					 cutter_delay_counter = stepper_cutter_move_time;
					 cutter_delay_flag = 1;
					 
					 HOLDING_BOBBIN_SOLENOID = 1;
				 	 CutActionFlag = 1;
				 	 CutActionCounter = 0;
				}	
			}
			*/
		#else
		while( cutter_delay_flag == 1)
		{
			rec_com();
		}

			temp16 = motor.angle_adjusted;
			while( temp16 > 1000 )//等待新的一圈
			{
				temp16 = motor.angle_adjusted;
				if( motor.stop_flag == 1)
					    break;
			}	
			flag = 1;
			while( motor.stop_flag == 0)//等待停车到位
			{	
				temp16 = motor.angle_adjusted;
				
				if( (temp16 >= 40 )&&(flag == 1) )//160=>40
				{
					if( (u206 == 1)&&(u210 ==1) )
					{
						flag = 0;
						if( para.wipper_type == AIR_WIPPER)
							AIR_FW = 1;
						else
						{
							SNT_H = 1; 
							FW = 1;
						}
					}					
				}
				
				if( (cutter_delay_flag == 0) &&(action5_flag ==0 ) )
				{
					action5_flag = 1;
					 #if MACHINE_14090_MASC_PLUS
					 movestep_yj(-stepper_cutter_move_range ,stepper_cutter_move_time);//开始断线
					 cutter_delay_counter = stepper_cutter_move_time;
					 cutter_delay_flag = 1;
					 #else
				 	 movestep_yj(90-stepper_cutter_move_range ,20);//断线位置
					 cutter_delay_counter = 40;
					 cutter_delay_flag = 1;
					 #endif
				}
				
			}
		#endif	
	    if(action5_flag ==0 )
		{
		   action5_flag = 1;
		   #if MACHINE_14090_MASC_PLUS
		   movestep_yj(-stepper_cutter_move_range ,stepper_cutter_move_time);//开始断线
		   cutter_delay_counter = stepper_cutter_move_time;
		   cutter_delay_flag = 1;
		   #else
		   movestep_yj(90-stepper_cutter_move_range ,20);//断线位置
		   cutter_delay_counter = 40;
		   cutter_delay_flag = 1;
		   #endif
		}
	}
	if( (u206 == 0 )&&(cut_mode == STEPPER_MOTER_CUTTER ) )
	{
		inpress_up();
		already_up_flag = 1;	
		
	}
	//while( motor.stop_flag == 0)
	// 	   rec_com();
	
	if( cut_mode == STEPPER_MOTER_CUTTER )
	{
		if( u210 == 1)
		{
			#if MACHINE_14090_MASC_PLUS
			#else
			while( cutter_delay_flag == 1)
			{
				 rec_com();
			}
			movestep_yj(stepper_cutter_move_range ,stepper_cutter_move_time);
			cutter_delay_counter = stepper_cutter_move_time<<1;
			cutter_delay_flag = 1;
			#endif
		}
	}
	else
	{
		if(u210 == 1)
		{
			CutActionCounter = 0;
			CutActionFlag = 0;
			trim_io_control(OFF);			
			delay_ms(5);
			fw_action_flag = 0;
			FW = 0;
			if( para.wipper_type == AIR_WIPPER)
			{
				AIR_FW = 0;
			}
		}
	}
	if(u210 == 1)
	{
		da0 = 0;
	    tension_open_switch =0;
	    tension_open_counter =0;
	}
		
	if( u42 == 1)
	{
		find_dead_point();
	}
	
	if( u42 == 0 && after_trim_stop_angle_adjust != 0)
	{
	    delay_ms(500);
		temp8 = dead_point_degree;
		dead_point_degree = u236 - after_trim_stop_angle_adjust;
		find_dead_point();
		delay_ms(100);
		dead_point_degree = temp8;			
	}
	
	if( (u206 == 1)&&(u210 ==1) )		
	{
		if( flag == 1)
		{
			SNT_H = 1; 
		    delay_ms(wiper_start_time);
		    if( para.wipper_type == AIR_WIPPER)
		    	AIR_FW = 1;
			else
				FW = 1;	
		}	
	}	
	delay_ms(wiper_end_time);
	HOLDING_BOBBIN_SOLENOID = 0;
	if( para.wipper_type == AIR_WIPPER)
		AIR_FW = 0;
	else
		FW = 0;
	SNT_H = 0; 
	delay_ms( 20 + delay_of_wipper_down );
		
	SNT_H = 0; 
				
	if(u42 == 0 && after_trim_stop_angle_adjust != 0)
	{
		temp8 = detect_position();	
		if(temp8 == OUT)     
		{
			find_dead_center();
		}
		delay_ms(100);
	}
	trim_io_control(OFF);
	HOLDING_BOBBIN_SOLENOID = 0;
	if( cut_mode == STEPPER_MOTER_CUTTER )
	{
		while( cutter_delay_flag == 1)
		{
			 rec_com();
		}
		delay_ms(25);
		go_origin_yj();
	}
		
	#if BOBBIN_THREAD_DETECT
	if( baseline_detect_switch == 1)
	{
		if( detect_bottom_thread_status() )
		{
			sys.status = ERROR;
			StatusChangeLatch = ERROR;
		   	sys.error = ERROR_81;        
			status_now = READY;	 
		}				
	}
	#endif
	if( baseline_alarm_flag == 1)
	{
		if( pat_buff_total_counter >= baseline_alarm_stitchs)
		 {
			sewing_stop();
        	sys.status = ERROR;
			StatusChangeLatch = ERROR;
        	sys.error =  ERROR_81;     		     
			status_now = READY;	   				 
		 }
	}
	while( motor.stop_flag == 0)
	 	   rec_com();
	brkdt_flag = 0;
	thbrk_count = 0;
	thbrk_flag = 0;
}
//--------------------------------------------------------------------------------------
//  Name:		run_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of run stauts
//--------------------------------------------------------------------------------------
void run_status(void)
{			
	UINT8 temp8,last_stitch_down,sewingcontrol_stitchs_abs,flag1,inpress_high_action_flag;
	INT8 tmp_stitchs;
	INT16 temp16,next_xstep_cou,next_ystep_cou;
	PATTERN_DATA *tmp_point,*TempStart_pointTemp;
	INT16 i;		
	INT16 inpress_up_angle,inpress_down_angle;
	UINT16 temp_speed,dly;
	INT16 x_temp,y_temp;
#if FOLLOW_INPRESS_FUN_ENABLE
	INT16 max_angle,inpress_position_tmp;
	UINT8 action_flag0,action_flag1,action_flag2,action_flag3,action_flag4;
#endif	
	//--------------------------------------------------------------------------------------
  	//  flag initial
  	//--------------------------------------------------------------------------------------
	inpress_high_action_flag = 0; 
	SUM = 0;			
  	zpl_pass = 1;
	stop_flag = 0;
  	cut_flag = 0;
  	end_flag = 0;	
	move_flag = 0;
  	process_flag = 1;	
  	motor.dir = 0;
  	stitch_counter = 1;  
  	laststitch_flag = 0;
  	pause_count = 0;               // clear pause counter
    pause_flag = 0;                // pause flag clear
    stay_flag = 0;                 // emergency break flag clear
  	thbrk_count = 0;               // clear thread breakage counter
  	thbrk_flag = 0;                // clear thread breakage flag
  	brkdt_flag = 0;                // clear thread breakage detection flag  
	FootRotateFlag = 0;
	OutOfRange_flag = 0; 		  
	inpress_high_flag = 0;         
	inpress_action_flag = 0;       
	cut_move_flag = 0;            
	manual_cut_flag = 0;
	
	StitchStartFlag = 1;	  
	pause_inpress_flag = 0;
  	//--------------------------------------------------------------------------------------
  	//parameter confirm
  	//--------------------------------------------------------------------------------------
  	para_confirm();	 	
  	//--------------------------------------------------------------------------------------
	return_from_setout = 0; 
	bobbin_change_done_flag = 0;
  	//-------------------------------------------------------------------------------------- 
  	while(1)
  	{	    
		rec_com();
   		origin_com = 0;
		if( scan_pause_func(&pause_flag,READY))
			return;
		
    	if( end_flag == 1 || stay_flag == 1 || stop_flag == 1) 
    	{   	
			break;
    	}
    	else
    	{
			process_data();
    		if(nopmove_flag == 1)
    		{	   
				while(nopmove_flag == 1 )
				{
					do_pat_point_sub_one();
					if( k03 == MECHANICAL )
					{
						/*if(tension_release_time >0 )
					    {
							if(tension_open_switch == 0)
							{
								tension_open_counter = 0;	
						        tension_open_switch = 1;
								thread_switch =1;
					         	DAActionFlag=1;
						        da0 = 255; 
							}
					    }*/
					}
					else
					{
						if( tension_release_time >0 )
						{
							/*if(tension_open_switch == 0)
							{
								tension_open_counter = 0;
								tension_open_switch = 1;
								DAActionFlag=0;
								temp_tension = 70;
								da0 = 0;
								//delay_ms(50);//2017-1-13
							}*/
							da0 = 0;
						}
					}
					go_beginpoint(0); 
				
					if( nop_move_pause_flag == 1)//空送过程中发生急停，直接切换到错误状态
					{
							status_now = READY;
			      			sys.status = ERROR;
			      			StatusChangeLatch = ERROR;
							if( sys.error == 0)
      		      				sys.error = ERROR_02;
							return;
					}
					if( sys.status == ERROR)
					    return;
					
					if(OutOfRange_flag == 1)
					{
						break;
					}
					process_data();
					
					if(origin2_lastmove_flag == 1)
					{
					    do_pat_point_sub_one();
					    sys.status = READY;
						StatusChangeLatch = READY;
					    return;
					}
					else if( nopmove_flag ==0)
					    break;
				}
			}
			if(OutOfRange_flag == 1)
			{
				sys.error = ERROR_15;
				StatusChangeLatch = ERROR;
				break;
			}
			if(move_flag == 1 || RotateFlag == 1 || SewingStopFlag == 1 )
			{
				move_flag = 0;
				do_pat_point_sub_one();
				zpl_pass = 1;
	  	    	end_flag = 0;	
	  	    	process_flag = 1;	
		  	    motor.dir = 0;
				cut_flag = 0;
				stop_flag = 0;
	  	    	stitch_counter = 1;	
			}
			else if(stop_flag == 1)
			{
				do_pat_point_sub_one();
				
				if(u208 == 1)
				{
					footer_both_up();
				}
				if(stop_number == 2)			//dstp
				{
					if( (motor.stop_angle > (DEGREE_180+20))||(motor.stop_angle < (DEGREE_180-20)))
			  		  {	
						 
						 TempStart_pointTemp = pat_point;
		                 TempStart_pointTemp --;
	 	                 while(TempStart_pointTemp > (PATTERN_DATA *)(pat_buf))
		                 {
			                if( (TempStart_pointTemp->func == 0x1d) && (TempStart_pointTemp->para == 2) )
			                  {
				                   inpress_delta = ((INT16)((TempStart_pointTemp)->xstep));
				                   inpress_high = inpress_high_base + inpress_delta;
				                   if(inpress_high > 80)
					                  inpress_high = 80;
				                   if(inpress_high < 0)
					                  inpress_high = 0;
				                   break;
			                   }
			                  TempStart_pointTemp--;
		                 }
		               if(TempStart_pointTemp == (PATTERN_DATA *)(pat_buf))
		                 {
			                inpress_down(inpress_high_base);		
		                    inpress_high = inpress_high_base;
							
		                 }
		              else
		                {
			                inpress_to(inpress_high);
		                }	
		               delay_ms(150);
					   needle_down();
					 }
				}
				last_inpress_position = inpress_high;   
				StopStatusFlag = 1;
				if(pat_point->para == 0)
			       footer_both_down();
			    else if(pat_point->para == 1)
			       footer_both_up();
				if( k03 == MECHANICAL )
				{   
					if(tension_release_time >0 )
				    {
						if(tension_open_switch == 0)
						{
							tension_open_counter = 0;	
					        tension_open_switch = 1;
							thread_switch =1;
				         	DAActionFlag=1;
					        da0 = tension_release_value + 50;   
    
						}
				    }
				}
				break;
			}
			else if(end_flag == 1)
			{
				if(inpress_flag == 0)           
					inpress_up();
				do_pat_point_sub_one();
				process_flag = 0;
				break;
			}
			else if( FootRotateFlag == 1 )
			{
				process_making_pen_signal(0);
				FootRotateFlag = 0;
				if( sys.error == 0)
					continue;
				else
					return;
			}
			else if (cut_flag ==1 )
			{
				cut_flag = 0;
				continue;
			}
			else if(inpress_high_flag == 1)
			{
				if(inpress_flag == 1 )      
				{
				   R_AIR = 1;
				   inpress_flag = 0;
				}
				delay_ms(200);  
				inpress_to(inpress_high);  
				inpress_action_flag = 1;
				inpress_high_flag = 0;
				continue;	
			}       
			else
			     continue;
				
    	}
    	
    	if(scan_pause_func(&pause_flag,READY))
    		return;
		 
		
		if( baseline_alarm_flag == 1)
		{
			 if( baseline_alarm_stitchs == 0)
			 {
			 	sewing_stop();
        		sys.status = ERROR;
				StatusChangeLatch = ERROR;
        		sys.error = ERROR_81;     		     
				status_now = READY;	   				 
				break;
			 }
		}
		
		
		if( (second_start_switch == 1)&&(aging_com == 0) )//二次启动开关
		{
		   if( second_start_counter > 0)
		   {
			 second_start_counter --;
			 status_now = READY;
			 sys.status = READY;
			 StatusChangeLatch = READY;
			 inpress_down(inpress_high_base);		
			 delay_ms(20);
			 return;
		   }
		}
		
		if( k03 == MECHANICAL )
		{
			da0 = 0;   
			tension_open_counter = 0;
			tension_open_switch = 0;
			DAActionFlag=0;
		}
		else
		{
			SNT_H = 1;
			temp_tension = sewing_tenion;       
      		at_solenoid(); 
		}
     if( making_pen_actoin_flag == 0)
	 {
    	if( thread_holding_switch == 0 )
		{
				AIR_OUT = 1;  //空移后吹气
				//delay_ms(80 );   
				//AIR_OUT = 0;
		}

		blow_air_counter = para.blow_air_counter;
		if( blow_air_counter > 0)
		{
			blow_air_action_flag = 1;
			BLOW_AIR = 1;
		}

		//--------------------------------------------------------------------------------------
    	//  motor run
    	//--------------------------------------------------------------------------------------			
	  	if(inpress_action_flag == 1 || cut_move_flag == 1)
		{
			inpress_action_flag = 0;
			cut_move_flag = 0;
		}
		else
		{	
			flag1 = 0;				
			if(inpress_flag == 1 )      
			 {
				R_AIR = 1;
				inpress_flag = 0;
				delay_ms(50);//100
		  	 }
			if(pat_point == TempStart_point&&pat_buff_total_counter<TOTAL_STITCH_COUNTER)
             {
				#if FIRST_STITCH_NOT_ACTION == 0
				if( inpress_lower_stitchs != 0)
				{
					inpress_down( inpress_high_base - inpress_lower_steps);
					inpress_high = inpress_high_base - inpress_lower_steps;
					flag1 = 1;	
				}
				else 
				#endif
				{
	                inpress_down(inpress_high_base);		
	                inpress_high = inpress_high_base;
				}
             }
			 else
			 {	
				#if FIRST_STITCH_NOT_ACTION == 0
				if( inpress_lower_stitchs != 0)
				{
			
					inpress_down( inpress_high - inpress_lower_steps);
					inpress_high = inpress_high_base - inpress_lower_steps;
					flag1 = 1;	
				}
				else
				#endif
					inpress_to(inpress_high);
			 }			 	  			
		}
		delay_ms(60);
		AIR_OUT = 0;
		delay_ms(30);   //200	
		temp8 = 0;
		if(( sewingcontrol_flag == 2 )&&( making_pen_actoin_flag == 0))
		{
			if( need_action_once == 1)
			   temp8 = 1;
			else 
			{
				tmp_point = pat_point;				
				for( i=0; i<2; i++)
				{
					if( tmp_point > (PATTERN_DATA *)(pat_buf) )
					    tmp_point--;
					if( (tmp_point->func == 0x1b)||(tmp_point->func == 0x03) )
					{
						temp8 = 3;
						break;
					}
				}
				
			}
		}
		else if(( sewingcontrol_flag ==1 )&&( making_pen_actoin_flag == 0))
		{
			if( need_action_two ==1)
			  temp8 = 2;
			else
			{
				tmp_point = pat_point;			
				for( i=0; i<1; i++)
				{
					if( tmp_point > (PATTERN_DATA *)(pat_buf) )
					    tmp_point--;
					if( (tmp_point->func == 0x1b)||(tmp_point->func == 0x03) )
					{
						temp8 = 2;
						break;
					}
				}
			}
		}
		
		if( (temp8 == 1 )||(temp8 == 3) )
		{
			   if( temp8 ==3)
			   {
				   tmp_stitchs = sewingcontrol_stitchs;
				   if ( sewingcontrol_stitchs >0 )
				     sewingcontrol_stitchs = -sewingcontrol_stitchs;
			   }
			   SewingReverse();
			   if( temp8 ==3)
			     sewingcontrol_stitchs = tmp_stitchs;
				 	  
			   need_backward_sewing = 0;
			   need_action_once =0;
			   StitchStartFlag = 0;
		}
		else if(temp8 == 2 )
		{
			   motor.spd_obj  = u10 * 100;  
	    	   process_data();
			   if(move_flag == 1)
			   {
				   //check_data_speed = motor.spd_obj;
				   calculate_angle();
			   	   zoom_in_one_stitch();
				   
		       	   need_action_two =0;
				   StitchStartFlag = 0;
			   }
			   else
			      do_pat_point_sub_one();
		}
		else
		{
		      	motor.spd_obj = u10 * 100; 
				if(motor.spd_obj > 800)
				{
					motor.spd_obj = 800;
				}
		    	if(RotateFlag == 1)
				{
					motor.spd_obj = 600;
				}
		}
		
		if( (thread_holding_switch == 1)&&(sewingcontrol_flag ==0) )
		{
			temp16 = motor.angle_adjusted;
	      	while(temp16 <= thread_holding_start_angle)//270d
	        { 
	        	rec_com();
				temp16 = motor.angle_adjusted;
			}
			FW = 1;
		}
		if( needle_cool_flag >=1 )
		    COOL_AIR = 0;
	 }
		StitchStartFlag = 1;   
		movex_delay_flag = 0;
		movey_delay_flag = 0;	    
		movex_delay_counter = 0;
		movey_delay_counter = 0;
    	//--------------------------------------------------------------------------------------
    	//  sewing cycle 
    	//--------------------------------------------------------------------------------------                
  		while(1)
    	{	
			#if ENABLE_LED_ALARM_FUNCTION 
			if( sys.error == 0)
			{
				GREEN_LED = 1;
				YELLOW_LED = 0;
				RED_LED = 0;
				led_stay_green_counter = 0;
				led_turn_green_flag = 1;
			}
			else
			{
				RED_LED = 1;
				GREEN_LED = 0;
				YELLOW_LED = 0;
				led_turn_green_flag = 0;
			}
			#endif
			
			if(StitchStartFlag == 1)
			{
				StitchStartFlag = 0;
			}
			else if(StitchStartFlag == 0)
			{
				if( (thread_holding_switch == 1)&&(stitch_counter ==1) )
				{
	                while(motor.angle_adjusted <= thread_holding_end_angle) 
					      rec_com();
					FW = 0;
				}
				
				//while(motor.angle_adjusted > 1200) //1440+50-1318 = 172 = 43度，确保3000转以下时时间大于2.4毫秒能够完整的接收一包花样数据。
				//   rec_com();    				
			
				while((motor.angle_adjusted >= 400)  && (RotateFlag == 0) )
	    		{
				
					flag_start_waitcom = 1; 
					if(flag_wait_com == 1)  
					{
						rec_com();          
						counter_wait_com = 0;
						flag_wait_com = 0;
					}

					if( (sys.status == ERROR)&&(stay_flag==0) )
					{
					   sys.status = ERROR;
					   StatusChangeLatch = ERROR;
						return;
					}											                
	    		}
				flag_start_waitcom = 0;	
				stitch_counter++;  	 
			}
			test_flag = 0;			
			test_flag2 = 0;
			CoolingIntervalCounter++;
			/*
		    if( (thread_holding_switch == 1)&&(stitch_counter ==2) )
			{
                while(motor.angle_adjusted <= tension_end_angle) 
				      rec_com();
				FW = 0;
			}
			*/
			test_flag = 0;
			cool_air_counter++;
			//-----------------------------------------------------------------------------------
			//adjust speed
			//---------------------------------------------------------------------------------
			if(speed_display_mode != 0)
			{
				if( MotorSpeedRigister>=2 && MotorSpeedRigister <= MAXSPEED0 )
				{
					sew_speed = ((UINT16)MotorSpeedRigister) * 100;
				}
			}
			else
			{
				if( MotorSpeedRigister<10 )
				{
					sew_speed = SpeedRange[MotorSpeedRigister];
				}
			}
		
			if( (stay_flag == 0)&&(brkdt_flag == 0) && (sfsw_flag == 0) )
			{
				zpl_process();  				
				//--------------------------------------------------------------------------------------
		    	//  area  limit adjust
		     	//--------------------------------------------------------------------------------------   	              	                   	    
		     	if(move_flag == 1)
		     	{
					allx_step = allx_step + xstep_cou;                        
	  				ally_step = ally_step + ystep_cou;
				
					if(allx_step < -RESOLUTION*u213 || allx_step > RESOLUTION*u214 || ally_step < -RESOLUTION*u216 || ally_step > RESOLUTION*u215)
					{
						allx_step = allx_step - xstep_cou;
			    		ally_step = ally_step - ystep_cou;						
						do_pat_point_sub_one();
						move_flag = 0;
						stay_flag = 1;						
						OutOfRange_flag = 1;					
					}
					else
					{
						allx_step = allx_step - xstep_cou;                        
	  					ally_step = ally_step - ystep_cou;
						if(( sewingcontrol_tail_flag ==1 )&&(making_pen_actoin_flag == 0) )
						{
							if( (pat_point->func ==0x02)||(pat_point->func ==0x1f))//||(pat_point->func ==0x03)||(pat_point->func ==0x1b) 
							{
								{
									zoom_in_one_stitch();
									move_flag = 0;
									continue;
								}
							}
						}
					}
				}
  	    	}
			
			 if(FootRotateFlag == 1)
			 {
				if( motor.spd_obj > 0)
				{
					sewing_stop();
				    while(motor.stop_flag == 0)
					{
						rec_com();
					}
				}
				process_making_pen_signal(0);	
				FootRotateFlag = 0;
				if( sys.error != 0)
					return;
				if( making_pen_actoin_flag == 0)
				{				
					break;
				}				
				else
				{
					StitchStartFlag	=1;
				}	
			 }
			 if( making_pen_actoin_flag == 1)
			 {
				 if ( making_pen_status == 1 )
				 	PEN_SIGNAL = 1;
				 else if ( making_pen_status == 4 )
				 	LASER_SIGNAL =1;
			 }
      		//--------------------------------------------------------------------------------------
      		//  pause
      		//-------------------------------------------------------------------------------------- 
	      	if(stay_flag == 1)
	      	{
	      		#if FOLLOW_INPRESS_FUN_ENABLE
				if( inpress_follow_high_flag == FOLLOW_INPRESS_HIGH )
			    {
					movestep_zx(-inpress_follow_range,inpress_follow_down_speed);
					inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
				}
				#endif
				if(( motor.spd_obj > 0)||(making_pen_actoin_flag==1))
				    pause_stop();
				return;
	      	}	
			if(RotateFlag == 1)
			{
				if(pat_point->func == 2)
				{
					motor.spd_obj = u211*10;
					trim_action();
					SNT_H = 0;
					delay_ms(140+delay_of_wipper_down);
					inpress_up();  
					delay_ms(50);
	  				cut_flag = 0;
					RotateFlag = 0;
					do_pat_point_add_one();
		        	break;
				}
				else
				{	
					motor.spd_obj = u211*10;		
					temp16 = motor.angle_adjusted;
		      		while( temp16 <= 600 )
					{
						flag_start_waitcom = 1; 
						if(flag_wait_com == 1)  
						{ 
		        			rec_com();
							counter_wait_com = 0;
							flag_wait_com = 0;
						}    						                                                  
						temp16 = motor.angle_adjusted;      
						if( (sys.status == ERROR)&&(stay_flag==0) )
						{
						   sys.status = ERROR;
						   StatusChangeLatch = ERROR;
							return;
						}     
					}
					flag_start_waitcom = 0;
					sewing_stop();
					
					RotateFlag = 0;
					process_data();
					do_pat_point_sub_one();
					if(nopmove_flag == 1)
					{
						//need_speed_up = 1;
						nopmove_flag = 0;
					}
					temp16 = motor.angle_adjusted;
		      		while( temp16 <= 1000 ) 
		        	{
						flag_start_waitcom = 1;  
						if(flag_wait_com = 1)   
						{ 
		        			rec_com();    						
							counter_wait_com = 0;
							flag_wait_com = 0;
						}                                                  
						temp16 = motor.angle_adjusted;      
						if( (sys.status == ERROR)&&(stay_flag==0) )
						{
						   sys.status = ERROR;
						   StatusChangeLatch = ERROR;
							return;
						}     
					}
					flag_start_waitcom = 0;  
					inpress_up();
					delay_ms(40); 
					break;
				}
			}
	      	//--------------------------------------------------------------------------------------
	      	//  thread breakage 
	      	//-------------------------------------------------------------------------------------- 
	      	if( brkdt_flag == 1 )
	      	{
				if( motor.spd_obj > 0)
					pause_stop();

				delay_ms(50);
		      	if( inpress_flag == 0)  
				{
					inpress_up();
					delay_ms(100);
				}
	        	sys.status = ERROR;
				StatusChangeLatch = ERROR;
	        	sys.error = ERROR_17;        
				status_now = READY;	      
	      		return;
	      	}        
			//--------------------------------------------------------------------------------------
			// sewing stop
			//--------------------------------------------------------------------------------------
			if(SewingStopFlag == 1)
			{
				if( motor.spd_obj > 0)
				{
					if (u42 ==  1)
						motor.stop_angle = 	dead_point_degree ;
					else
					    motor.stop_angle = 	u236 ;
				  	motor.spd_obj = 0;
					while(motor.stop_flag == 0)
					{
						rec_com();
					}
				}
				inpress_up();    
				delay_ms((UINT16)SewingStopValue*1000);
				SewingStopFlag = 0;
				break;
			}
	    	//--------------------------------------------------------------------------------------
	      	//  cut and stop
	      	//-------------------------------------------------------------------------------------- 
	    	if( (cut_flag == 1)&&(making_pen_actoin_flag == 0) )
	      	{  
				tail_sewing();
				trim_action();					   
				if( (sys.status == ERROR)&&(stay_flag==0) )
				{
				     sys.status = ERROR;
				     StatusChangeLatch = ERROR;
				     return;
				}  				
				process_data();
				if( move_flag == 1)
				{
					move_flag = 0;
					cut_move_flag = 1;
				}
				else
				{
					  	if( (end_flag == 1)&&(u224 ==2) )
						{
						   //go_origin_zx();
						   inpress_up();
						   delay_ms(10);
						}
						else if(RotateFlag == 1)
						{
						   RotateFlag = 0;	
						}
						else
						{
					        inpress_up();
							delay_ms(10); 
							if( needle_cool_flag >=1 )
		   					    COOL_AIR = 1;
						}
						end_flag = 0;
						stop_flag = 0;
				}

				blow_air_counter = para.cut_air_counter;
				if( blow_air_counter != 0)
				{
					blow_air_action_flag = 1;
					BLOW_AIR = 1;
				}
	
				do_pat_point_sub_one();
				cut_flag = 0;
	        	break;
	        }
	      	//--------------------------------------------------------------------------------------
	      	//  stop
	      	//-------------------------------------------------------------------------------------- 
	    	if(stop_flag == 1) 
	      	{           
		    	if( motor.spd_obj > 0)
				{
					if( stop_number == 1)			//ustp
						motor.stop_angle = 	u236 ;
					else
					  	motor.stop_angle = 	DEGREE_180 ;
					motor.spd_obj = 0; 
					while(motor.stop_flag == 0)
					{
						rec_com();
					}
				}  	     	
	      		delay_ms(20);
				if(stop_number ==1) 
				{
					inpress_up(); 
					delay_ms(100);
					if( k03 == MECHANICAL )
					{
						if(tension_release_time >0 )
					    {
							if(tension_open_switch == 0)
							{
								tension_open_counter = 0;	
						        tension_open_switch = 1;
					         	DAActionFlag=1;
						        da0 = tension_release_value + 50;            
							}
					    }
					}
				}
				//if(stop_foot_status ==1)
				//{
				//	footer_both_up();    
				//}
				do_pat_point_sub_one();	
				if( pat_point->para == 0)
				    footer_both_down();
				else if(pat_point->para == 1)
				    footer_both_up(); 					
							 
	        	break;
	      	} 
			if(atum_flag == 1 || hevi_flag ==1 )    
	      	{           
				if( motor.spd_obj > 0)
				{
					motor.stop_angle = 	u236 ;
					motor.spd_obj = 0; 
					while(motor.stop_flag == 0)
					{
						rec_com();
					}
				}
				inpress_up(); 
				delay_ms(120);
				if(u208 == 1)
				{
					footer_both_up();
				} 
				do_pat_point_sub_one();
	        	break;
	      	} 
			// ------------------------------------------------------------------------------------------
			// check the last stitch in normal line/circle sewing without cut code or stop code
			// ------------------------------------------------------------------------------------------ 
			if(end_flag == 1)
			{
				if( motor.spd_obj > 0)
				{
					if(finish_cut_flag ==1)
					{
						tail_sewing();
						trim_action();
					}
					else
					{
						process_flag = 0;
						temp16 = motor.angle_adjusted;

			      		while(temp16 <= tension_start_angle)
			        	{ 
							flag_start_waitcom = 1;    
							if(flag_wait_com == 1)     
							{
			        			rec_com();
								counter_wait_com = 0;
								flag_wait_com = 0;
							}    					                                               
							temp16 = motor.angle_adjusted;      
			        	}
						flag_start_waitcom = 0;         //2015-11-04
			      		sewing_stop();  
						while(motor.stop_flag == 0)
						{
							rec_com();
						}
			      		delay_ms(10);
						if(u42 == 1)
						{
							find_dead_point();
						}
					}
				}
				laststitch_flag = 0;
				do_pat_point_sub_one();
				inpress_up(); 
		        delay_ms(50);             
	        	break;
			}
			// ------------------------------------------------------------------------------------------
			// check the last stitch in normal line/circle sewing without cut code or stop code
			// ------------------------------------------------------------------------------------------ 
			if(nopmove_flag == 1)
			{	 
				if( motor.spd_obj > 0)
				{      
					if( before_nopmove_cut_flag ==1 )
					{
						tail_sewing();
						trim_action();
					} 
					else
					{		
							if(finish_cut_flag ==1)
							{
								 TempStart_pointTemp = pat_point;
								 while((TempStart_pointTemp ->func ==0x1b)||(TempStart_pointTemp->func ==0x03) )
				                 {
					                //pat_point ++;
									TempStart_pointTemp++;
								    if( TempStart_pointTemp->func == 0x1f) 
					                {
						                end_flag = 1;   
										 break;
					                }
								    if(TempStart_pointTemp->func == 0x1D)
								    {
								    	switch(TempStart_pointTemp->para)
											{
												case 0://FUN5

												break;
												case 1://FUN6

												break;
												case 2://FUN7

												break;
												case 3://1d 03 00 00
													FootRotateFlag = 1;
											
												break;
												case 6:

												break;
											}
								    }
				                 }

								 if(end_flag ==1 ||FootRotateFlag == 1)
								 {
									 end_flag =0;
									 FootRotateFlag = 0;
									 tail_sewing();
									 trim_action();
									 laststitch_flag = 0;
									 nopmove_flag = 0;
									 do_pat_point_sub_one();
									 inpress_up();       
									 delay_ms(100); //?
									 break;
								 }

							}
								temp16 = motor.angle_adjusted;

					      		while((temp16 <= tension_start_angle) || (temp16 >= MAGIN_AREA))
					        	{
									flag_start_waitcom = 1;         
									if(flag_wait_com == 1)          
									{
					        			rec_com();    				
										counter_wait_com = 0;
										flag_wait_com = 0;
									}                                                 
									temp16 = motor.angle_adjusted;      
					        	}
								flag_start_waitcom = 0;          

					      		sewing_stop();  
								while(motor.stop_flag == 0)
								{
									rec_com();
								}
					      		delay_ms(20);
		
								if(u42 == 1)
									find_dead_point();
								if( k03 == MECHANICAL )
								{
									if(tension_release_time >0 )
								    {
										if(tension_open_switch == 0)
										{
											tension_open_counter = 0;	
									        tension_open_switch = 1;
											thread_switch =1;
								         	DAActionFlag=0;
									        da0 = tension_release_value + 50;
										}
								    }
								}
						}
				}
				if( making_pen_actoin_flag == 1)
				{
					while( rec1_total_counter > 0 )
						rec_com();
				
					tb4s = 0;
					laser_cutter_aciton_flag = 0; 
				}
				laststitch_flag = 0;
				nopmove_flag = 0;
				do_pat_point_sub_one();
				inpress_up();       
				delay_ms(100); 
	        	break;
			}
			#if FOLLOW_INPRESS_FUN_ENABLE	
			
            if(inpress_high_flag == 1)
			{				
				inpress_high_flag = 0;
				inpress_to(inpress_high);
			}
				#if FIRST_STITCH_NOT_ACTION == 0
				if( inpress_lower_stitchs > 0)
				{
					if( (stitch_counter > inpress_lower_stitchs )&&(flag1 == 1) )
					{
						flag1 = 0;
						inpress_high_action_flag  = 1;
						inpress_high = inpress_position + inpress_lower_steps;
						inpress_follow_delta = inpress_lower_steps;
						inpress_position = inpress_high;	
					}
				}
				#endif
	
			#endif
			
			#if ROTATE_CUTTER_ENABLE
			if( rotated_function_flag == 1)
			{	
				while( rec1_total_counter > 0 )
				{
					rec_com();
					SUM =1;
				}
				//tb4s = 0;
				SUM = 0;
				delay_ms(500);				
				rotated_by_data(rotated_abs_angle);
				delay_ms(500);
				rotated_function_flag = 0;				
			}
			#endif
	      	//--------------------------------------------------------------------------------------
	      	//  move stepper motor
	      	//--------------------------------------------------------------------------------------   
			if(move_flag == 1)
	      	{  			   	
				if( making_pen_actoin_flag == 1)//车缝中的记号笔功能
				{				
					#if INSERPOINT_ENABLE
					if(inpress_flag == 0)
						inpress_up();
					PBP_Line(0);
					if (laser_cutter_aciton_flag == 0)
					{
						dly = marking_speed;
						dly = dly*100 + 250;
						if( dly > 2730 )
							dly = 2730;
						tb4 = dly *24;
						laser_cutter_aciton_flag = 1;						
						tb4s = 1;
					}
				   allx_step = allx_step + xstep_cou;                        
  				   ally_step = ally_step + ystep_cou;

					#else
					StitchStartFlag	=1;
					x_temp = fabsm(xstep_cou);
					y_temp = fabsm(ystep_cou);
					if( x_temp > y_temp)
						temp_speed = spdlimit_10080_345_tab[x_temp-1] ;
					else
						temp_speed = spdlimit_10080_345_tab[y_temp-1] ;
					temp_speed = temp_speed/100;
					movestepx_time = MoveTime_Speed_10080_x[temp_speed];
					movestepy_time = MoveTime_Speed_10080_y[temp_speed];
					if( movestepy_time > movestepx_time)
						movestepx_time = movestepy_time;
					else
						movestepy_time = movestepx_time;
						
					if( x_temp > 0)
					    movestep_x(xstep_cou);
					delay_us(400);	
					if( y_temp > 0)
				        movestep_y(ystep_cou);  
				   delay_ms(movestepy_time);
				   
				   allx_step = allx_step + xstep_cou;                        
  				   ally_step = ally_step + ystep_cou;
				   #endif
				}			
				else
				{
					
				#if FOLLOW_INPRESS_FUN_ENABLE		
					if(inpress_type == FOLLOW_UP_INPRESSER) 
					{		
						inpress_up_angle   = inpress_follow_up_angle   << 2;
						inpress_down_angle = inpress_follow_down_angle << 2;
				
						action_flag0 = 1;
						action_flag1 = 1;
						action_flag2 = 1;
						action_flag3 = 1;
						action_flag4 = 1;				
						/*			
		                if( (inpress_down_angle > inpress_up_angle )&&(last_stitch_down ==1) )
						     action_flag0 = 0;	
						else if( (inpress_down_angle > inpress_up_angle )&&(last_stitch_down ==0))
						{
							 inpress_down_angle = 5;
						}
					    */
					
						temp16 = motor.angle_adjusted;
						
						while( temp16 < 1400 )//350d
						{
							temp16 = motor.angle_adjusted;
							if( motor.spd_obj < 400)
							    rec_com();
							
							if( (sys.status == ERROR )||(motor.spd_obj == 0) )
							    break;
					
							if( (temp16 > inpress_down_angle ) &&( action_flag0 == 1) )//中压脚下降角度
							{
								action_flag0 = 0;
								if( inpress_follow_high_flag == FOLLOW_INPRESS_HIGH )
								{
									
									if( movezx_delay_flag == 1)
										while( movezx_delay_counter >0 );
										
									//if( (inpress_delta >0 ) &&(inpress_high_flag == 1) )
									if( (inpress_high_action_flag == 1) &&(inpress_follow_delta>0))
									{		
										inpress_high_action_flag = 0;
										inpress_follow_delta =  inpress_follow_delta*7/10;
										movestep_zx(-inpress_follow_range + inpress_follow_delta,inpress_follow_down_speed);
									}
									else
									{
									
										movestep_zx(-inpress_follow_range,inpress_follow_down_speed);
									}								
									test_flag = 1;
									inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
									movezx_delay_counter = inpress_follow_down_speed ;
									movezx_delay_flag =1;								
								}						
							}
					
							if( (temp16 > inpress_up_angle ) &&( action_flag1 == 1) )
							{
								action_flag1 = 0;
								#if FIRST_STITCH_NOT_ACTION 
								  if( (inpress_follow_high_flag == FOLLOW_INPRESS_LOW )&&(stitch_counter > inpress_lower_stitchs) )
								#else
								  if( inpress_follow_high_flag == FOLLOW_INPRESS_LOW )
								#endif
								{
									if( movezx_delay_flag == 1)
										while( movezx_delay_counter >0 );

									//if( (inpress_delta < 0 ) &&(inpress_high_flag == 1) )
									if( (inpress_high_action_flag == 1) &&(inpress_follow_delta<0))
									{									
										inpress_high_action_flag = 0;
										inpress_follow_delta =  inpress_follow_delta*7/10;
										movestep_zx(inpress_follow_range + inpress_follow_delta,inpress_follow_up_speed);
									}
									else
									{
										movestep_zx(inpress_follow_range,inpress_follow_up_speed);	
										
									}								
									
									test_flag = 21;
									inpress_follow_high_flag = FOLLOW_INPRESS_HIGH;
									movezx_delay_counter = inpress_follow_up_speed ;
									movezx_delay_flag =1;
							
									//========================================
									/*
									stitch_counter++;
									pat_point++;
									check_data(0);//看看下针转速，但不改变实际转速
									inpress_down_angle = inpress_follow_down_angle<<2;
									pat_point--;
									stitch_counter--;
									if( inpress_down_angle > inpress_up_angle)//默认情况下是下降小于起始的
									{
										action_flag0 = 1;//允许再次下降
										last_stitch_down = 1;
									}
									else
										last_stitch_down = 0;
									*/
									//========================================
								}
							}
					
						    temp16 = motor.angle_adjusted;
							if( (xstep_cou != 0) &&(temp16 > movestepx_angle ) &&( action_flag2 == 1) )
							{
								if( movex_delay_flag == 1)
								    while(movex_delay_counter >0 );
								movex_delay_flag =0;
								movestep_x(xstep_cou);					
								movex_delay_counter = movestepx_time + 1;
								movex_delay_flag = 1;
								action_flag2 = 0;
								allx_step = allx_step + xstep_cou;  							
							}
							temp16 = motor.angle_adjusted;							
							if( (ystep_cou != 0) &&(temp16 > movestepy_angle ) &&( action_flag3 == 1) )
							{
								if( movey_delay_flag == 1)
								    while( movey_delay_counter >0 );
								movey_delay_flag =0;
								movestep_y(ystep_cou);					
								//test_flag = 21;					
								movey_delay_counter = movestepy_time + 1;
								movey_delay_flag = 1;
								action_flag3 = 0;
								ally_step = ally_step + ystep_cou;
															
							}
						}//1400
					}
					else
						move_xy(); 			
					#else
				
					move_xy();     
					if(inpress_type == STEPPER_INPRESSER)     
					{    					
			        	temp16 = motor.angle_adjusted;
						if(movestep_angle <1360 )//340d=>1360
						{
						    while(temp16 <= movestep_angle + 80)
		        	     	{
								flag_start_waitcom = 1;
								if( flag_wait_com == 1 ) 
								{
		        	     			rec_com();   
									counter_wait_com = 0;
									flag_wait_com = 0;
								}
		        	     		temp16 = motor.angle_adjusted;
		        	     	}
						}
						flag_start_waitcom = 0;  
		    	     	inpress_to_forsingle(inpress_high);
					}	
					
					#endif
				}//marking_pen
				move_flag = 0;
	      	}//move_flag
	      	rec_com();                                     
			if(sys.status == ERROR)
			break;   
		                       
    	}//while(1)
		if( stop_flag == 1) 
		{
			StopStatusFlag = 1;
			break;
		}
		if( sys.status == ERROR)
		{
			StatusChangeLatch = ERROR;
			break;
		}
  	}
	//--------------------------------------------------------------------------------------
	//  switch system status 
	//--------------------------------------------------------------------------------------  	    			        
	
	if(stop_flag == 1)
	{
		sys.status = READY;
		StatusChangeLatch = READY;
		StopStatusFlag = 1;
	} 
	else
	{
		if(StatusChangeLatch == ERROR)
		{
			sys.status = ERROR;
			StatusChangeLatch = ERROR;
		}
		else
		{
			sys.status = FINISH;
			StatusChangeLatch = FINISH;			
			allx_step_last = allx_step;
	        ally_step_last = ally_step;	
		   
		}
	}
}
//--------------------------------------------------------------------------------------
//  Name:		error_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of error stauts
//--------------------------------------------------------------------------------------
void error_status(void)
{		
	//-------------------------------------------------------------------------------------- 
  	// sound count                                                                  
  	//-------------------------------------------------------------------------------------- 
	if(motor.stop_flag == 0 || motor.spd_obj != 0 || motor.spd !=0)
	{
		sewing_stop();      	        	  
	    while(motor.stop_flag == 0)
		{
			rec_com();
		}
	}
	pedal_state = 0;
	if( k03 == ELECTRICAL )
	{
		tension_open_switch = 0;
		test_action_flag = 0;
		da0 = 0;
		temp_tension = 0;
		temp_tension_last = 0;
	}
	
	FW = 0;
	COOL_AIR = 0;
	predit_shift = 0;
	sound_count++;
  	if(sound_count >= 50000)
  	{
  		sound_count = 0;
  	}   
	origin_com = 0; 
  	//-------------------------------------------------------------------------------------- 
  	// switch error number                                                                  
  	//--------------------------------------------------------------------------------------     	
  	switch(sys.error)
  	{
  		case 21:			// during the find origin process,next is free
			//--------------------------------------------------------------------------------------
            // turn on alarm led
            //-------------------------------------------------------------------------------------- 	    	           	       	  
    	  	flash_led();   		// flash alarm led                                                   
            flash_buz();	  	// flash alarm buzzer 
			pause_flag = 0;
            break;
		case 22:				// during time delay in aging process,next is ready
			pause_flag = 0;
            break;
		case 15: 				// out of sewing range
		case 83:
		case 84:
			break;	
		case 97: 
			break;  
    	//--------------------------------------------------------------------------------------
    	// thread breakage
    	//--------------------------------------------------------------------------------------	 	         
    	case 17: // thread breakage
			#if 0
	           		delay_ms(2000);
		       		sys.status = status_now;
					StatusChangeLatch = status_now;
                	sys.error = OK; 
					thbrk_flag = 0;    	       
    	            brkdt_flag = 0; 
					predit_shift = 0;
					smotor_speed = 0;
					da0 = 0;  //错误状态关闭松线器
					return;
			#endif
			flash_led(); 
            flash_buz();
		    break;          
		//--------------------------------------------------------------------------------------
    	// dangerous error
    	//--------------------------------------------------------------------------------------	      
    	case 4:  // 300V undervoltage
     	case 7:  // IPM overcurrent or overvoltage
    	case 8:  // 24V overvoltage 
    	case 9:  // 24V undervoltage
     	case 13: // no motor encoder connect
    	case 14: // motor is not normal  
    		    
    	case 18: // cut knife is not in normal position	    	
    	case 20: // step motor software version problem 	 	     
    		//--------------------------------------------------------------------------------------
            // turn on alarm led
            //-------------------------------------------------------------------------------------- 
					    	           	       	  
    	  	flash_led();   // flash alarm led                                                   
            flash_buz();	  // flash alarm buzzer  	      
     	    break;  
			
		case 35: // IPM over cuurrent frequently	
		case 37: // motor is stucked(condition 1) 			 
		case 34: // BLDC_ON alawys ON,abnormal current		
		case 36: // over cuurrent more than 10 times in 100ms		
		case 38: // motor is stucked(condition 2) 
		case 39: // motor over speed
		case 40: // over current in stop status
		case 41: // motor over load
		case 47: // sm_overload 1
		case 49: // sm_overload 2
		 	      
				  disable_24V_output(); 
				  da0 = 0;
				  trim_io_control(OFF); 
				  emergency();     // emergency measure
	    	      turnon_led();    // turn on alarm led	     	                                                      
	              flash_buz();	    // flash alarm buzzer    		          		 
	        break;
	   	case 23: // catcher is not in normal position  	 	           
    	case 24: // panel is not matching   	 	           
    	case 25: // X origin sensor is not normal  	 	           
    	case 26: // Y origin sensor is not normal  	 	           
    	case 27: // press origin sensor is not normal  	 	             	 	           	
    	case 28: // catch thread origin sensor is not normal  	 	             	 	  
    	case 29: // inpress origin sensor is not normal  
    	case 30: // stepping motor driver communication is not normal 
    	case 31: // stepping motor overcurrent                        
    	case 32: // stepping motor driver power is not normal         	             	 	           	        	
    	case 33: // length is more than 12.7mm  
		case 42: // DC line abnormal
	    	emergency();     // emergency measure
	    	turnon_led();    // turn on alarm led	     	                                                      
	        flash_buz();	    // flash alarm buzzer    		          		 
	        break;
    	//--------------------------------------------------------------------------------------
    	// normal error
    	//--------------------------------------------------------------------------------------                                                        
    	case 5:  // 300V overvoltage
		#if USE_SC013K_PALTFORM
		
		#else
			if(AC_OVDT)
    		{    					 	          	  
    	    	flash_led();   // flash alarm led                                                   
               	flash_buz();	  // flash alarm buzzer
    		}
    		else
    		{
    			turnoff_ledbuz();	 // turn off alarm led and buzzer
    		}
		#endif
			break;   
		case 10:					// fan error,valve error
			disable_24V_output();      // 24V disable
			//--------------------------------------------------------------------------------------
            // turn on alarm led
            //-------------------------------------------------------------------------------------- 	    	           	       	  
    	  	flash_led();   // flash alarm led                                                   
            flash_buz();	  // flash alarm buzzer 
			//if(T_OC == 0)
			//{
			//	SNT_H == 0;
			//	sys.error = OK;
			//	turnoff_ledbuz();					// turn off alarm led and buzzer
			//}
			break;	 
    	case 19: 
		case 2:										// pause button is not in normal position	
			aging_com = 0;
			aging_flag = 0;
			pause_flag = 0;	
			stay_flag = 0;
			predit_shift = 0;
			smotor_speed = 0;
			aging_com = 0;
			aging_flag = 0;
			inpress_com = 0;
			software_key_pause = 0;
			last_inpress_position = inpress_high;  
    	 	#if 1
			if(PAUSE == PAUSE_OFF)              
	        {			
	           	delay_ms(200);
				if(PAUSE == PAUSE_OFF)
				{
					turnoff_ledbuz();					// turn off alarm led and buzzer 
	           		sys.status = status_now;
					StatusChangeLatch = status_now;
                	sys.error = OK;					
					pause_flag = 0;	
					stay_flag = 0;
					predit_shift = 0;
					smotor_speed = 0;
					aging_com = 0;
					aging_flag = 0;
					inpress_com = 0;
					last_inpress_position = inpress_high;  
					delay_ms(300);
                	return;  
				}	    	    	      	    
             }
		 	#endif
			 flash_buz();
             flash_led();   							// flash alarm led                                                   
    		 pause_flag = 0;
    		 AIR_OUT = 0;
			 da0 = 0;  
     	     break;    
        case 3:
			if( sfsw_flag == 0 )
			{
		 	    delay_ms(1000);
		 		turnoff_ledbuz();	
           		sys.status = status_now;
				StatusChangeLatch = status_now;
            	sys.error = OK; 
				predit_shift = 0;
			}
			else
			 {
				 da0 = 0;
		 	     trim_io_control(OFF);
				 flash_buz();
			 }
		break;			   	          
    
		case 0:
			enable_24V_output();
			turnoff_ledbuz();					
			break;	
		case 45: 
			if(DVB == para.dvab_open_level)           					// foot sensor is pushed,
			{
				delay_ms(10);
				if(DVB == para.dvab_open_level && DVBLastState != para.dvab_open_level)
				{
	           		sys.status = status_now;
					StatusChangeLatch = status_now;
                	sys.error = OK; 
                	return;  
				}	    	    	      	    
             }
   			 
			 if( (foot_half_flag == 1)&&(u238>=1) )
			 {
				 if(DVSM == 0)
				 {
					delay_ms(10);
					if(DVSM == 0 && DVSMLastState == 1)
					{
						sys.status = status_now;
						StatusChangeLatch = status_now;
	                	sys.error = OK; 
	                	return;
					}
				 }
			 }				
      	     break;  
		case 46:
		break;
		case 88:
		case 89:
				turnoff_ledbuz();
				sys.error = OK;
				sys.status = READY;
				StatusChangeLatch = READY;
		break;
		case 90:
		case 91:
		case 92:
		break;
		case 81:
				    #if BOBBIN_CHANGER_ENABLE
					if( ( bobbin_change_switch == 1)&&(bobbin_change_done_flag == 0) )
					{
						bobbin_change_process();
						bobbin_change_done_flag = 1;
						#if BOBBIN_THREAD_DETECT
						if( baseline_detect_switch == 1)
						{
							delay_ms(100);
							if( detect_bottom_thread_status() )
							{
								
							}				
							else
							{
							    //sys.status = status_now;
								//StatusChangeLatch = status_now;
                				//sys.error = OK;
								sys.status = ERROR;
			    				sys.error = ERROR_90; 	
							}
						}
						else
						{
							sys.status = ERROR;
			    			sys.error = ERROR_90; 
						}
						#endif
					}
					#endif
					//delay_ms(2000);				
	           		//sys.status = status_now;
					//StatusChangeLatch = status_now;
                	//sys.error = OK; 
					//baseline_alarm_flag = 0;
					return;
					    	    
		break;
    	default: // no number error
    	   	disable_24V_output();      // 24V disable                                  	           
           	emergency();     // emergency measure
     	    turnon_led();    // turn on alarm led	     	                                                      
            flash_buz();	 // flash alarm buzzer    		          		 
            break;    	    	   	            	         
  	}
	
	if(StatusChangeLatch != ERROR) 
	{
 		predit_shift = 0;
		turnoff_ledbuz();					// turn off alarm led and buzzer
		sys.error = OK;
		sys.status = StatusChangeLatch;
	}
}
//--------------------------------------------------------------------------------------
//  Name:		prewind_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of prewind stauts
//--------------------------------------------------------------------------------------
void prewind_status(void)
{	
	UINT16 temp16;
	UINT16 spd_tmp,i;
	UINT8 temp8;
	while(motor.stop_flag == 0)    
  	{
    	rec_com();    // communication with panel                  	
  	}
  	FW = 0;
 	if(inpress_flag == 0)
	{
		temp8 = detect_position();
	    if(temp8 == OUT)     
	      {
			find_dead_center();
	      }
		inpress_up();
		delay_ms(120);
	}
 		
	if(StatusChangeLatch != PREWIND) 
	{
			predit_shift = 0;
			sys.status = StatusChangeLatch;
	}
	else
	{
		 
		  	if(DVB == para.dvab_open_level)           				
			{
				delay_ms(10);
				if(DVB == para.dvab_open_level && DVBLastState != para.dvab_open_level)
				{				
					footer_procedure();
				}
			 }
			DVBLastState = DVB;
			
			if(DVA == para.dvab_open_level) 
			{
				delay_ms(10);
				if(DVA == para.dvab_open_level && DVALastState != para.dvab_open_level)
				{				
					if( already_in_origin == 0)
					{
						sys.status = ERROR;
						StatusChangeLatch = ERROR;
		      			sys.error = ERROR_46; 
						predit_shift = 0;  
						return;
					}

			    	{		
			    		motor.dir = 0;
						temp16 = u49 * 100;
						if(u237 == 1)
						{
				    		if(temp16 > MAXSPEED1*100 || temp16 < 200)
				    		{
				    			temp16 = 1600;
				    		}	
						}
						else if(u237 == 0)
						{
							if(temp16 > MAXSPEED0*100 || temp16 < 200)
				    		{
				    			temp16 = 1600;
				    		}
						}
						
						inpress_down(inpress_high_base);
						delay_ms(50);
						for(i=0;i<5;i++)
				    	{	  	  	  		     
							if(i==0)
							  spd_tmp = u10;	
							else if(i==1)  
							  spd_tmp = u11;	
							else if(i==2)
							  spd_tmp = u12 ;	
							else if(i==3)
							  spd_tmp = u13;	
							else if(i==4)
							  spd_tmp = u14;
							if(spd_tmp >= u49)
							   motor.spd_obj = u49*100;
							else
							   motor.spd_obj = spd_tmp*100;
							delay_ms(10);  
							while(motor.angle_adjusted >= 16)
					    		{
					    			rec_com();    						                
					    		}  	 
							while(motor.angle_adjusted < 16)
					    		{
					    			rec_com();    						                
					    		} 
						}
						motor.spd_obj = temp16;
						sys.status = WIND;
						StatusChangeLatch = WIND;
	        			predit_shift = 0;  
						DVALastState = DVA;   				    	
			    	}    	    	      	    
			  	}
		  	}
		 	else
			{
				delay_ms(10);
				if(DVA != para.dvab_open_level)
				{
					DVALastState = DVA;
				}
			}

	}//other status
	
}
//--------------------------------------------------------------------------------------
//  Name:		wind_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of wind stauts
//--------------------------------------------------------------------------------------
void wind_status(void)
{	

	//--------------------------------------------------------------------------------------
	//  start sensor
	//--------------------------------------------------------------------------------------  
	if(DVA == para.dvab_open_level)           // foot sensor is pushed
	{
		delay_ms(10);
		if(DVA == para.dvab_open_level && DVALastState != para.dvab_open_level)
		{				

	    	{	
				sewing_stop();
			
				while(motor.stop_flag == 0)    
			  	{
			    	rec_com();    // communication with panel                  	
			  	} 	  
				inpress_up();
				delay_ms(120);					
	    		wind_com = 0;
				sys.status = PREWIND;
				StatusChangeLatch = PREWIND;
				DVALastState = DVA;
	    	}		    	    	      	    
	     }
		DVALastState = DVA;
	  }
	 else
	  {
			delay_ms(10);
			if(DVA != para.dvab_open_level)
			{
				DVALastState = DVA;
				if( wind_mode == 0)
				{
					sewing_stop();
				
					while(motor.stop_flag == 0)    
				  	{
				    	rec_com();    // communication with panel                  	
				  	} 	  
					inpress_up();
					delay_ms(150);
		    		wind_com = 0;
					sys.status = PREWIND;
					StatusChangeLatch = PREWIND;
					DVALastState = DVA;
				}
			}
	}

	if(scan_pause_func(&pause_flag,PREWIND))
		return;

	if(StatusChangeLatch != WIND) 
	{
		if(motor.spd_obj > 0)
		{
			sewing_stop();
			while(motor.stop_flag == 0)    
		  	{
		    	rec_com();    // communication with panel                  	
		  	} 	  
		 	inpress_up();
			delay_ms(50);
    		wind_com = 0;
		}
		sys.status = StatusChangeLatch;
		predit_shift = 0;
	}        
}

//--------------------------------------------------------------------------------------
//  Name:		poweroff_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of power off stauts
//--------------------------------------------------------------------------------------
void poweroff_status(void)
{
  	disable_24V_output();       // u24 disable
	if( para.platform_type == FOURTH_GENERATION )
	{
	  	PWR_LED = 1;      // power_on led disable
	  	ALARM_LED = 0;    // alarm led disable
	}

	SUM = 0;          // alarm buzzer disable
	//--------------------------------------------------------------------------------------
  	// turn off all output
  	//-------------------------------------------------------------------------------------- 
	if(motor.spd > 10)
	{
		U=1;U_=1;V=1;V_=1;W=1;W_=1;
		prcr = 0x02;
		inv03 = 0;
		prcr = 0x00;
		U_=0;V_=0;W_=0;
	}
	else
	{
		U=1;U_=1;V=1;V_=1;W=1;W_=1;
		prcr = 0x02;
		inv03 = 0;
		prcr = 0x00;
		OUTPUT_ON = 1;
	}
	#if USE_SC013K_PALTFORM
	
	#else
	//--------------------------------------------------------------------------------------
  	// turn off all output
  	//-------------------------------------------------------------------------------------- 
	if(power_off_count < 10)
		power_off_count++;
	else
	{	
		if(AC_OVDT)
		{
			OUTPUT_ON = 1;
			sys.error = ERROR_05;
			sys.status = ERROR;
			StatusChangeLatch = ERROR;
			return;
		}
	}
	//--------------------------------------------------------------------------------------
  	// turn off all output
  	//-------------------------------------------------------------------------------------- 		
	if(PWR_ON)
  	{    
	    RST_PN = 1;								// reset control panel
	    prcr = 0x07;              				// Protect mode reg    
	    pm0 = 0x08;               				// Processor mode reg0 
	    prcr = 0x00;              				// Protect mode reg    
  	}  
	#endif
}

//--------------------------------------------------------------------------------------
//  Name:		setout_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of setout stauts
//--------------------------------------------------------------------------------------
void setout_status(void)
{
	UINT8 temp8,i;		
 
	CoolingIntervalCounter = 0;
	making_pen_actoin_flag = 0;
	COOL_AIR = 0;
	
	
 	#if AUTO_CHANGE_FRAMEWORK
	if ( PointShiftFlag == 1 )
		PointShiftFlag = 0;
	 if(coor_com  == 1)
  	  	coor_com = 0;
	if( aging_com == 1)
	{
		  if( waitting_for_pattern_done == 1)//左边运行着
		   {
			   return_frame_back(1);//左边送回模板
			   left_quest_running = 0;
			   //启动另一边
			   AUTO_RIGHT_FRAME_STANDBY = 1;
			   right_second_footer_status = 1;
			   right_footer_delay_flag = 1;
			   right_footer_delay_counter = 0;
		   }
		   else if( waitting_for_pattern_done == 2)
		   {
			   return_frame_back(2);//右边送回模板
			   right_quest_running = 0;
			   //启动另一边
			   AUTO_LEFT_FRAME_STANDBY = 1;					
			   left_second_footer_status = 1;
			   left_footer_delay_flag = 1;//
			   left_footer_delay_counter = 0;
		   }
	}	
	else
	{
		   if( waitting_for_pattern_done == 1)//左边运行着
		   {
			   return_frame_back(1);//左边送回模板
			   left_quest_running = 0;
		   }
		   else if( waitting_for_pattern_done == 2)
		   {
			   return_frame_back(2);//右边送回模板
			   right_quest_running = 0;
		   }
	}
   waitting_for_pattern_done = 0;
	//6 回原点
	delay_ms(50);
	go_origin_allmotor();
	#else
	

	if( super_pattern_flag != 1 )
	{
		inpress_high = inpress_origin;        
		if( (fun_default_flag==1)&&(special_machine_type==0))
		{
			fun_default_flag = 0;
			T_HALF=0;                
			T_DIR =0;
			T_CLK = 0;
			FR_ON =0;
			FK_OFF=0;
		}
		/*
		if( k03 == MECHANICAL )
		{
			if( (tension_release_time >0)&&(u210 == 0))
			{
				if(tension_open_switch == 0)
				{
					tension_open_counter = 0;
					tension_open_switch = 1;
					DAActionFlag=0;
					da0 = tension_release_value + 100;
				}
			}
		}
		else 
		     da0 = 0;
		*/
		 switch(u37) //foot status after sewing finish 									
		 {
		  		case 0: 
						if( u39	!= 0 )
						{
							if( (u39 == 1)&&(LastPatternHaveSOP == 1) )//need find origin point	
							 	  go_setoutpoint();
							else if(u39 == 2)
							{
								switch( u48)
								{
									case 0:
									  	 go_setoutpoint();
									break;
									case 1:
										 	if(start_flag == 0)
											{
												while(pat_point != sta_point)
												{	
													course_back();
												}
											}
									break;
									case 2:
										go_origin_allmotor(); 
										delay_ms(50);
										go_startpoint();
									break;
								}
							}
							else 
							  go_origin_allmotor();
						}
						else
						{
						//	pat_point = (PATTERN_DATA *)(pat_buf);
						//	allx_step = 0;
						//	ally_step = 0;
						stay_end_flag = 1;	
						}
						PEN_SIGNAL = 0;
						LASER_SIGNAL = 0;
						if(u38 == 0) //allow footer taken up
						{
	                		if( LRfooter_up_mode == 0 )
							{
								footer_both_up(); 
								FootNumber=0;
							}
							else if (LRfooter_up_mode == 1)//keep left down
							{
								foot_up(); //right up
							}		
							else
							{
							    foot_half_up();       					
							}
						}
				break;
	              
		  		case 1: 
					    PEN_SIGNAL = 0;
						LASER_SIGNAL = 0;					
						if(u38 == 0)
						{
	                		if( LRfooter_up_mode == 0 )
							{
								footer_both_up(); 
								FootNumber=0;
							}
							else if (LRfooter_up_mode == 1)//keep left down
							{
								foot_up(); //right up
							}		
							else
							{
							    foot_half_up();       					
							}
						}
						if( u39	!= 0 )
						{
							if( (u39 == 1)&&(LastPatternHaveSOP == 1) )//need find origin point	
							  	  go_setoutpoint();
							else if(u39 == 2)
							{
								switch( u48)//
								{
									case 0:
									  	 go_setoutpoint();
									break;
									case 1:
										 	if(start_flag == 0)
											{
												while(pat_point != sta_point)
												{	
													course_back();
												}
											}
									break;
									case 2:
										go_origin_allmotor(); 
										delay_ms(50);
										go_startpoint();
									break;
								}
							}
							else
							  go_origin_allmotor();//2011-7-12
						 
						}
						else
						{
						//	pat_point = (PATTERN_DATA *)(pat_buf);
						//	allx_step = 0;
						//	ally_step = 0;	
						stay_end_flag = 1;
						}
						predit_shift = 0;	
		        break;
		  		case 2:
						if( u39	!= 0 )
						{
							if( (u39 == 1)&&(LastPatternHaveSOP == 1) )//need find origin point	
							  	  go_setoutpoint();
							else if(u39 == 2)
							{
								switch( u48)//
								{
									case 0:
									  	 go_setoutpoint();
									break;
									case 1:
										 	if(start_flag == 0)
											{
												while(pat_point != sta_point)
												{	
													course_back();
												}
											}
									break;
									case 2:
										go_origin_allmotor(); 
										delay_ms(50);
										go_startpoint();
									break;
								}
							}
							else
							  go_origin_allmotor();//2011-7-12
						  
						}
						else
						{
						//	pat_point = (PATTERN_DATA *)(pat_buf);
						//	allx_step = 0;
						//	ally_step = 0;	
						stay_end_flag = 1;
						}
						PEN_SIGNAL = 0;
						LASER_SIGNAL = 0;
						predit_shift = 0;
						FootUpCom = 1;
	           	break;
	  		
		  	default:
				break;		
		}
		
		SewTestStitchCounter = 0; //2010-8-19  
	}
	
	if(super_pattern_flag ==1)
	{
		if ( PointShiftFlag == 1 )
		{
				if(origin_check_flag == 1)
				  {
					  PointShiftFlag = 0;
					  origin_check_flag = 0;
					  return;

				  }
				  if(u39 == 0)
				  {
					PointShiftFlag = 0;
					return;
				  }
					if( already_in_origin ==0 )
					{
						sys.status = ERROR;
						StatusChangeLatch = ERROR;
						sys.error = ERROR_46;
						predit_shift = 0;
						single_flag = 0;
						return;
					}
					pat_point = (PATTERN_DATA *)(pat_buf) + PointShiftNumber%6000;
					pat_buff_total_counter = PointShiftNumber;
					//SewTestStitchCounter = pat_buff_total_counter;
					PointShiftFlag = 0;
				}
	  }
	#endif
	nop_move_k = 1; 
	move_flag = 0;
	lastmove_flag = 0;
	nopmove_flag = 0;
	origin2_lastmove_flag = 0;
	end_flag = 0;
	PORG_action_flag = 0 ;	
	course_next_flag = 0;
	DVALastState = DVA;
	
	speed_down_stitchs = 0;
	start_to_speed_down= 0;
	speed_down_counter = 0;
	
	manual_cut_flag = 0;
	return_from_setout = 1; 	
	TempStart_point = pat_point;
	base_tension = tension_release_value;
	sewing_tenion = base_tension;
	temp_tension_last = 255;
	if( (stretch_foot_enable==1)&&(u38 == 0)&&(finish_nopmove_pause_flag==0) )
    {
		FOOTHALF_UP;
	}
	if( second_start_switch == 1)
	{
		second_start_counter = 1;
	}
	already_auto_find_start_point = 0;
	//--------------------------------------------------------------------------------------
  	//  switch system status 
  	//--------------------------------------------------------------------------------------
	//if(scan_pause_func(&pause_flag,READY))
	//	return;
	
	if(sys.status == ERROR)
	{
		if(sys.error == ERROR_02)
		{
		   footer_both_down();      
		   status_now = READY;
		}
	}
	else
	{
		StatusChangeLatch = READY;
	  	sys.status = READY;   
		PatternDelayFlag = 0;
		if( k03 == MECHANICAL )
		{
				if( (tension_release_time >0) && (u210 ==1) )
				{
					if(tension_open_switch == 0)
					{
						tension_open_counter = 0;
						tension_open_switch = 1;
						thread_switch =1;
						DAActionFlag=0;
						da0 = tension_release_value + 100;
					}
				}
		}
		else 
			da0 = 0;
		//if( u224 != 2 )
		//   go_origin_zx();   

	}

}

/*
30rpm =>600mm/min => 0.01mm/ms => 0.1mm 10ms

*/
void delay_process(void)
{
	UINT16 tmp;
	if(shift_low_flag == 1)//停止减速处理
	{
		if(one_step_delay ==1 )
		{
			if(DelayCounter > 14)
			    DelayCounter=14;
			DelayCounter --;
			delay_us (9000-DelayCounter*500);
		}
		else if(one_step_delay ==2 )
		{
			if(DelayCounter > 16)
			    DelayCounter=16;
			DelayCounter --;
			delay_us (9000-DelayCounter*500);
		}
		else 
		{
			if(DelayCounter > 17)
			    DelayCounter=17;
			DelayCounter --;
			delay_us (9000-DelayCounter*500);
		}
		if(DelayCounter ==0)
		{
			DelayCounter = 0;
			shift_flag = 0; 
			shift_reply = 0;
			shift_low_flag = 0;
		}
	}
	else if( one_step_delay == 1)
	{
		if( DelayCounter >= 14 )
			delay_ms(2);
		else
		{
			DelayCounter ++;
			delay_us (9000-DelayCounter*500);
		}
	}
	else if ( one_step_delay == 2)
	{
		if( DelayCounter >= 16 )
			delay_ms(1);
		else
		{
			DelayCounter ++;
			delay_us (9000-DelayCounter*500);
		}
	}
	else
	{
		if( DelayCounter >= 17 )
			delay_us(500);
		else
		{
			DelayCounter ++;
			delay_us (9000-DelayCounter*500);
		}	
	}	
}
//--------------------------------------------------------------------------------------
//  Name:		preedit_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of preedit stauts
//--------------------------------------------------------------------------------------
void preedit_status(void)
{
	INT16 temp16;
	INT16 i;
	UINT8 temp8;
	INT16 tempx_step,tempy_step;
	PATTERN_DATA *TempStart_pointTemp;
#if AUTO_CHANGE_FRAMEWORK
	AUTO_LEFT_HOLDING_POSITION = 0;
	AUTO_RIGHT_HOLDING_POSITION = 0;
	left_footer_status = 0;
	right_footer_status = 0;
#endif
	if(para.x_origin_mode == AUTO_FIND_START_POINT)
	{
		if (already_auto_find_start_point == 0)
		{
			find_start_point_x();
			already_auto_find_start_point = 1;
		}
	}
	if(origin_com == 1)
	{		
		temp8 = detect_position();	
		if( temp8 == OUT)   
  		{
			find_dead_center();
  		}
		go_origin_allmotor();	  
		predit_shift = 0;
		origin_com = 0;
	  	if(sys.status == ERROR)
    	{
      		return;
    	}
  	}
 	
 	switch(foot_com)
	{
	  		case 0:                  
	  			if(foot_flag == 1)     			
	  	        {
					footer_both_down();
	  	        }
	  	        break;
  	         
	    	case 1:       
	    	  	if(foot_flag == 0)     		   
	  	        {
					footer_both_up();
	  	        }
	  	        break;		
  	}  

	//--------------------------------------------------------------------------------------
	// inpress down or up
	//-------------------------------------------------------------------------------------- 
	if(inpress_action_flag ==1)
	{
			if(inpress_high == -1 )     
			{
			    if(inpress_flag == 1)  		
		  	    {
		  	       	inpress_down(inpress_high_base);      			
					delay_ms(200);
		  	    }			
				
			}

			else if(inpress_high == 81)
			{
				if( (motor.angle_adjusted>=440) &&(motor.angle_adjusted <=1000) )
					 find_dead_center();
				//if(inpress_flag ==0)
				{
		  	     		inpress_up();        
						delay_ms(200);         
				}
			}
			else
			{
			  R_AIR = 1;
			  delay_ms(200);
			  last_inpress_position =inpress_high;
			  inpress_to(inpress_high);
			  delay_ms(200); 
			}
			inpress_action_flag = 0;
			predit_shift = 0;
	}
	
	
 	if(MotiongSetFlag == 1)
	{
		switch(MotionSet)
		{
			case 0:
				predit_shift = 1;
				pat_point = (PATTERN_DATA *)(pat_buf);
				pat_point = pat_point + DestinationPointShiftPosition%6000;
				pat_buff_total_counter = DestinationPointShiftPosition;
				predit_shift = 0;
				break;
			case 1:
				pat_point = (PATTERN_DATA *)(pat_buf);
				pat_point = pat_point + DestinationPointShiftPosition%6000;
				pat_buff_total_counter = DestinationPointShiftPosition;
				if( inpress_flag ==0)
				{
					inpress_up();
					delay_ms(200);
				}
				predit_shift = 1;
				go_commandpoint(comx_step,comy_step);
				predit_shift = 0;
				
				MotionSet = 0;
				break;
			case 2:
				predit_shift = 1;
				if(comx_step < -RESOLUTION*u213 || comx_step > RESOLUTION*u214 || comy_step < -RESOLUTION*u216 || comy_step > RESOLUTION*u215)
				{

					allx_step_temp = 0;
					ally_step_temp = 0;
					
					for(TempStart_pointTemp = (PATTERN_DATA *)(pat_buf);TempStart_pointTemp <= pat_point;TempStart_pointTemp++)
					{
						switch(pat_point->func & 0xf0)
						{
							case 0x03:
							case 0x1b:
							case 0x01:
							case 0x21:
							case 0x41:
							case 0x61:
								allx_step_temp = allx_step_temp + ChangeX(TempStart_pointTemp);
								ally_step_temp = ally_step_temp + ChangeY(TempStart_pointTemp);
								break;
							default:
								break;
						}
					}
					
					if( inpress_flag ==0)
					{
						inpress_up();
						delay_ms(200);
					}
					go_commandpoint(allx_step_temp,ally_step_temp);
					
					sys.error = ERROR_15;
					StatusChangeLatch = ERROR;
				}
				else
				{
					pat_point = (PATTERN_DATA *)(pat_buf);
					pat_point = pat_point + DestinationPointShiftPosition%6000;
					pat_buff_total_counter = DestinationPointShiftPosition;
					if(inpress_flag ==0)
					{
				  	    inpress_up();        
						delay_ms(200);         
					}
					go_commandpoint(comx_step,comy_step);
				}
				predit_shift = 0;
				MotionSet = 0;
				break;
		}
		MotiongSetFlag = 0;
	}
	
	//------------------------------------------------------------------------------------
	//stitch up command
	//------------------------------------------------------------------------------------	
	if( StitchUpFlag ==1)
	{
		temp8 = detect_position();	          
        if( temp8 == OUT)    
        {
		    find_dead_center();
        }
		inpress_up();
		delay_ms(150);
		StitchUpFlag = 71;
	}
	else if(StitchUpFlag ==0)
	{
		if( inpress_flag ==1)
		{
		  	inpress_down(0);
		  	delay_ms(150);
		}
		needle_down();
		StitchUpFlag = 71;
	}
	
	
	if ( PointShiftFlag == 1)
	{
			if( already_in_origin ==0 )
			{
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
      			sys.error = ERROR_46; 
				predit_shift = 0;  
				single_flag = 0;
				return;
			}
			
			if(super_pattern_flag != 1)
			{
				if((foot_half_flag == 1) || (foot_flag == 1))
				{
				   footer_both_down();
				   delay_ms(100);
				}
				inpress_up();
				delay_ms(120);
				tempx_step = 0;
				tempy_step = 0;
				pat_point = (PATTERN_DATA *)(pat_buf);
				for(i=0;i<PointShiftNumber;i++)
				{
					process_data();
					if( end_flag ==1 )
					{
						do_pat_point_sub_one();
						break;
					}
					tempx_step += xstep_cou;
					tempy_step += ystep_cou;
				}
				go_commandpoint(tempx_step,tempy_step);	
			}
			else
			{			
				pat_point = (PATTERN_DATA *)(pat_buf) + PointShiftNumber%6000;
				pat_buff_total_counter = PointShiftNumber;
			}
			PointShiftNumber = 0;
			PointShiftFlag = 0;
	}

	if( CurrentPointShiftFlag == 1)
	{
		CurrentPointShiftFlag = 0;
		pat_point = (PATTERN_DATA *)(pat_buf);
		pat_point = pat_point + CurrentPointShiftPosition%6000;
		pat_buff_total_counter = CurrentPointShiftPosition;
	}
	//--------------------------------------------------------------------------------------
	//  foot sensor
	//-------------------------------------------------------------------------------------- 
	if(DVB == para.dvab_open_level)           		
	{
		delay_ms(10);
		if(DVB == para.dvab_open_level && DVBLastState != para.dvab_open_level)
		{				
			footer_procedure();
   	    	DVBLastState = DVB;		    	    
	  	}
	}
	else
	{
		DVBLastState = DVB;
	} 	
	
	single_move_func();
	if((shift_flag == 0x88)&&(shift_flag_old&0xf0))
	{
		shift_flag = shift_flag_old;
		shift_low_flag = 1;		
	}
	if(shift_flag != 0)
	{
		if(foot_flag == 1)
		{		 
		  	footer_both_down();       	
			delay_ms(200);
		}
		shift_func(shift_flag);
	}
  	
	//--------------------------------------------------------------------------------------
  	//  coordinate command 
  	//--------------------------------------------------------------------------------------
  	if(coor_com  == 1)
  	{
		temp8 = detect_position();	           
        if(temp8 == OUT)    
        {
		    find_dead_center();
        }
		inpress_up();
		delay_ms(150);
		predit_shift = 1;
		
  		go_commandpoint(comx_step,comy_step);
		predit_shift = 0;
		coor_com = 0;
  	}
	predit_shift = 0;		
	if(StatusChangeLatch != PREEDIT)
	{
		sys.status = StatusChangeLatch;
		if( StatusChangeLatch == READY)
		{
		  temp8 = detect_position();	
		  if( temp8 == OUT)    
		 	  find_dead_center(); 
		}	   
	}
	
}


//--------------------------------------------------------------------------------------
//  Name:		finish_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of finish stauts
//--------------------------------------------------------------------------------------
void finish_status(void)
{	

#if 0	
	if( (motor.angle_adjusted > u236+10)||(motor.angle_adjusted < u236-10))
		OutOfPositionTimes=1;
#endif	

#if INSTALLMENT	
	if( main_control_lock_setup == 1)
	{
		write_par(0,main_control_lock_flag);  		
		main_control_lock_setup = 0;
	}
#endif	

	if(u39 != 0)
	  pat_buff_total_counter = 0; 
   
   #if AUTO_CHANGE_FRAMEWORK
 
   #else	
	
   if(super_pattern_flag == 1)
   {
  	 	if(coor_com  == 1)
  	 	{
			last_allx_step = allx_step;
			last_ally_step = ally_step;
			bakeup_total_counter = pat_buff_total_counter;
			
			predit_shift = 1;
			if( k03 == MECHANICAL )
			{
				if( (tension_release_time >0) && (u210 ==1) )
				{
					if(tension_open_switch == 0)
					{
						tension_open_counter = 0;
						tension_open_switch = 1;
						DAActionFlag=0;
						da0 = tension_release_value + 20;
					}
				}
			}
			else
			    da0 = 0;
	 
		 	
			switch(u37) //foot status after sewing finish 									
		 	{
	  			case 0: 
					if( u39	!= 0 )
					{
						if( (u39 == 1)&&(LastPatternHaveSOP == 1) )//need find origin point	
					   	  	coor_com_fun();
						else if(u39 == 2)
						{
							switch( u48)//
							{
								case 0:
								case 1:
									 coor_com_fun();
								break;
								case 2:
									go_origin_allmotor(); 
									delay_ms(50);
									coor_com_fun();
								break;
							}
						}
						else
						{
						  	origin_check_flag = 1;
							
						  	go_origin_allmotor();						  
						}
					}
					PEN_SIGNAL = 0;					
					LASER_SIGNAL = 0;
					if( (u38 == 0)&&(finish_nopmove_pause_flag == 0) ) //allow footer taken up
					{
                		if( LRfooter_up_mode == 0 )
						{
							footer_both_up(); 
							FootNumber=0;
						}
						else if (LRfooter_up_mode == 1)//keep left down
						{
							foot_up(); 
						}		
						else
						{
						    foot_half_up();       					
						}
					}
					predit_shift = 0;
			break;
	              
	  		case 1: 
				    PEN_SIGNAL = 0;
					LASER_SIGNAL = 0;
					if(u38 == 0)
					{
                		if( LRfooter_up_mode == 0 )
						{
							footer_both_up(); 
							FootNumber=0;
						}
						else if (LRfooter_up_mode == 1)//keep left down
						{
							foot_up(); 
						}		
						else
						{
						    foot_half_up();       					
						}
					}
				
					if( u39	!= 0 )
					{						
						if( (u39 == 1)&&(LastPatternHaveSOP == 1) )//need find origin point	
							 coor_com_fun();
						else if(u39 == 2)
						{
							switch( u48)
							{
								case 0:
								case 1:
								  	 coor_com_fun();
								break;
								case 2:
									go_origin_allmotor(); 
									delay_ms(50);
									coor_com_fun();
								break;
							}
						}
						else
						{
							origin_check_flag = 1;
						    go_origin_allmotor();							
						}						 
					}	
					predit_shift = 0;	
	        break;
	  		case 2:
					if( u39	!= 0 )
					{
						if( (u39 == 1)&&(LastPatternHaveSOP == 1) )//need find origin point	
							  coor_com_fun();
						else if(u39 == 2)
						{
							switch( u48)//
							{
								case 0:								  	 
								case 1:
										coor_com_fun();
								break;
								case 2:
										go_origin_allmotor(); 
										delay_ms(50);
										coor_com_fun();
								break;
							}
						}
						else
						{
						  origin_check_flag = 1;
						  go_origin_allmotor();
						}						  
					}
					
					PEN_SIGNAL = 0;
					LASER_SIGNAL = 0;
					predit_shift = 0;
					FootUpCom = 1;
           	   break;
		}
		//if( u224 != 2 )
		//	go_origin_zx();
		coor_com = 0;		
	  }
			
    }
	#endif
	
	
	predit_shift = 0;
	origin_com = 0;
	if(StatusChangeLatch != FINISH) 
	{
 		predit_shift = 0;
		sys.status = StatusChangeLatch;
	}
	
}


//--------------------------------------------------------------------------------------
//  Name:		slack_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of slack stauts
//--------------------------------------------------------------------------------------	
extern void show_para_infoformation(void);
void slack_status(void)
{	
	UINT8 temp8,i;	
	INT16 tempx_step,tempy_step; 
	if(origin_com == 1)
	{
		go_origin_allmotor();
		origin_com = 0;
	}
	if( (inpress_flag == 0)&&(back_from_checki08==1) )
	{
		go_origin_zx();
		back_from_checki08 = 0;
	}
	if(svpara_trans_flag == 1)
	{
		stepmotor_para();
		svpara_trans_flag = 0;
	}
	if( wirte_stepermotor_para_flag != 0)
	{		
		write_stepmotor_config_para(wirte_stepermotor_para_flag,svpara_disp_buf);
		wirte_stepermotor_para_flag = 0;
		SUM = 1;
		delay_ms(500);
		SUM = 0;
		predit_shift =0;		
	}
	if( write_stepmotor_curve_flag != 0)
	{
	    if( write_stepmotor_curve(write_stepmotor_curve_flag, pat_buf) )
		{
			SUM = 1;
			delay_ms(100);
			SUM = 0;
	
			
			write_stepmotor_curve_flag = 0;
			predit_shift =0;
		}	
	}
	if(auto_function_flag == 1)
	{
		if((formwork_identify_device == 2)&&(rc522_control_falg==1))
		{	
			if(rfid_config_flag == 0)
			{
				init_uart1();
				RFID_initial();
		   		init_uart1_RC522(); 	
		   		rfid_config_flag = 1;
			}	
			RFID_SCAN();
	        rfid_wr_ret();
		}
	}
	else
        process_uart1_download();
#if INSTALLMENT	
	if( main_control_lock_setup == 1)
	{
		write_par(0,main_control_lock_flag);  		
		main_control_lock_setup = 0;
	}
#endif	
  if( write_eeprom_para_flag >= 1)
  {
	  if( write_eeprom_para_flag == 1)
	  {
	    	write_para_group(100,svpara_disp_buf,205);
			delay_ms(300);
	    	restore_para_from_eeprom();
	  }
	  else 
	  {
			write_para_group(400,svpara_disp_buf,205);
			delay_ms(300);
			//app_GetProgramFromEeprom();
	  }
	  write_eeprom_para_flag = 0;	
	  SUM = 1;
	  delay_ms(100);
	  SUM = 0;
	  predit_shift =0;
  }
	//--------------------------------------------------------------------------------------
	//  coordinate command 
	//--------------------------------------------------------------------------------------
	if( coor_com  == 1)
	{
		temp8 = detect_position();	      
		if(temp8 == OUT)    
		{
				    find_dead_center();
		 }
		footer_both_down();
		inpress_up();
		delay_ms(150);
		predit_shift = 1;
		if( nop_move_pause_flag ==1)
		{
			process_nop_move_pause(1);
		} 
		if(para.x_origin_mode == AUTO_FIND_START_POINT)
		{
			if (already_auto_find_start_point == 0)
			{
				find_start_point_x();
				already_auto_find_start_point = 1;
			}
		}
		if( nop_move_pause_flag ==1)
		{
			process_nop_move_pause(1);
		} 
		if( sys.error == 0)
		 go_commandpoint(comx_step,comy_step);
		delay_ms(500);	
		predit_shift = 0;
		need_action_once = 0;//跳转以后不能直接加固，要判定位置
		return_from_setout = 0;
		coor_com = 0;
	}
			
	if (PointShiftFlag == 1)
	{
		if( already_in_origin ==0 )
		{
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
	      		sys.error = ERROR_46; 
				predit_shift = 0;  
				single_flag = 0;
				return;
		}

		if(super_pattern_flag != 1)
		{
					temp8 = detect_position();	            
			        if(temp8 == OUT)    
			        {
					    find_dead_center();
			        }
					footer_both_down();
					inpress_up();
					delay_ms(120);
					tempx_step = 0;
					tempy_step = 0;
					pat_point = (PATTERN_DATA *)(pat_buf);
					for(i=0;i<PointShiftNumber;i++)
					{
						process_data();
						if( end_flag ==1 )
						{
							//pat_point -- ;
							do_pat_point_sub_one();
							break;
						}
						tempx_step += xstep_cou;
						tempy_step += ystep_cou;
					}
					go_commandpoint(tempx_step,tempy_step);
		}
		else
		{	
					pat_point = (PATTERN_DATA *)(pat_buf) + PointShiftNumber%6000;
					pat_buff_total_counter = PointShiftNumber;
					PointShiftNumber = 0;
				  
		}
		PointShiftFlag = 0;
	}
	if(StatusChangeLatch != SLACK) 
	{
 		predit_shift = 0;
		sys.status = StatusChangeLatch;
	}
	
	pattern_number = 0;
//	serail_number =0;	 
}
//--------------------------------------------------------------------------------------
//  Name:		checki03_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of checki03 stauts---check input signal
//--------------------------------------------------------------------------------------	
void checki03_status(void)
{	
	if(StatusChangeLatch != CHECKI03) 
	{
 		predit_shift = 0;
		sys.status = StatusChangeLatch;
	}
}
//--------------------------------------------------------------------------------------
//  Name:		checki04_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of checki04 stauts---check servo motor
//--------------------------------------------------------------------------------------	
void checki04_status(void)
{	
	UINT8 temp8 ,j;
	UINT16 spd_tmp,i;

	motor.dir = 0; 
	//--------------------------------------------------------------------------------------
  	// config main motor driver
  	//-------------------------------------------------------------------------------------- 
  	if(sys.status == ERROR)
  	{
    	return;
  	}  
	//--------------------------------------------------------------------------------------
  	//  find origin
  	//-------------------------------------------------------------------------------------- 
	if(origin_com == 1)
	{		
    	origin_com = 0;        		
  	}
    
	//--------------------------------------------------------------------------------------
  	// servo motor run and stop
  	//-------------------------------------------------------------------------------------- 	
	if(foot_flag == 1)
	{		 
	  	footer_both_down();
	}
	else
	{
	   if( aging_selection == 1)//单主轴老化
       {
		 if( smotor_speed != 0 )
		 {
		   if( 	aging_delay == 0)//连续模式
		   {
			   if( running_flag == 0)
				{
					aging_mode_counter_1 = 0;
					running_flag = 1;
					aging_old_counter = 0;
				}
				else
				{
				   if( aging_mode_counter_1 >= 60000)//1 min
				   {
				   	   aging_mode_counter_1 = 0;
					   aging_mode_counter_2++;
					   if( aging_mode_counter_2 >= 60 )// 1 hour
					   {
						   aging_old_counter ++;
						   if( aging_old_counter >= AgingLasting )
						   {
							   smotor_speed = 0;
							   sewing_stop();		
								while(motor.stop_flag == 0)    
								{
								    rec_com();                	
								} 	  
								inpress_up();
								delay_ms(150);
								running_flag = 0;
						   }
					   }
				   }
				}
		   }
		   else
		   {
				if( running_flag == 0)
				{
					aging_mode_counter_1 = 0;
					running_flag = 1;
					aging_old_counter = 0;
					
				}
				else
				{
					if( aging_mode_counter_1 >= 1000)//
					{
						aging_mode_counter_1 = 0;
						aging_old_counter++;
						//if( aging_old_counter%2)
						//    turnon_buz();
						//else
						//    turnoff_buz();
						if( aging_old_counter >= AgingLasting )
						{
							
							sewing_stop();	
							aging_old_counter = 0;	
							running_flag = 0;
							while(motor.stop_flag == 0)    
							{
							    rec_com();                	
							} 
							
							for( i= 0;i<aging_delay ;i++)
							{
									delay_ms(1000);
									if( smotor_speed == 0)
									break;
							}
					
							aging_mode_counter_1 = 0;
							running_flag = 1;
							aging_old_counter = 0;
						}
					}
				}
		    }
			if( (motor.stop_flag == 1)&&( smotor_speed != 0) )
			{
				inpress_down(inpress_high_base);
				if( smotor_speed > MAXSPEED0)
				    smotor_speed = MAXSPEED0;
					
				for(i=0;i<5;i++)
		    	{	  	  	  		     
					if(i==0)
					   spd_tmp = u10;	
					else if(i==1)  
					   spd_tmp = u11;	
					else if(i==2)
					   spd_tmp = u12 ;	
					else if(i==3)
					   spd_tmp = u13;	
					else if(i==4)
					   spd_tmp = u14;
					   
					if(spd_tmp >= smotor_speed)
					   motor.spd_obj = smotor_speed*100;
					else
					   motor.spd_obj = spd_tmp*100;
					delay_ms(10);  
					
					while(motor.angle_adjusted >= 16)
			    		{
			    			rec_com();    						                
			    		}  	 
					while(motor.angle_adjusted < 16)
			    		{
			    			rec_com();    						                
			    		} 
					if(smotor_speed ==0)
					   break;
						 
				}
				if(smotor_speed !=0)
				   motor.spd_obj = smotor_speed * 100;				
			}	
		}
		else
		{
			if(motor.stop_flag == 0) 
			{
				for(i=0;i<5;i++)//减速停车
		    	{	  	  	  		     
					if(motor.spd_obj <= 1200)
					{
						sewing_stop();	
						break;
					}	 
					else if( motor.spd_obj < 1800)
					{
						motor.spd_obj = 1000;      
					}
					else if(motor.spd_obj < 2500 )
					{
						motor.spd_obj = 1600;      
					}
					else if(motor.spd_obj >= 2500)
					{
						motor.spd_obj = 2300;      
					}	
					delay_ms(15);  
					
					while(motor.angle_adjusted >= 16)
			    	{
			    		rec_com();    						                
			    	}  	 
					while(motor.angle_adjusted < 16)
			    	{
			    		rec_com();    						                
			    	} 
						 
				}
				while(motor.stop_flag == 0)    
				{
				    rec_com();    // communication with panel                  	
				} 	  
				inpress_up();
				delay_ms(150);
			}	
		}
	 }
	 else
	 {
		if(smotor_speed != 0)
		{
			if(motor.stop_flag == 1)
			{
				inpress_down(inpress_high_base);
				if( smotor_speed > MAXSPEED0)
				    smotor_speed = MAXSPEED0;
					
				for(i=0;i<5;i++)
		    	{	  	  	  		     
					if(i==0)
					  spd_tmp = u10;	
					else if(i==1)  
					  spd_tmp = u11;	
					else if(i==2)
					  spd_tmp = u12 ;	
					else if(i==3)
					  spd_tmp = u13;	
					else if(i==4)
					  spd_tmp = u14;
					if(spd_tmp >= smotor_speed)
					   motor.spd_obj = smotor_speed*100;
					else
					   motor.spd_obj = spd_tmp*100;
					delay_ms(10);  
					while(motor.angle_adjusted >= 16)
			    		{
			    			rec_com();    						                
			    		}  	 
					while(motor.angle_adjusted < 16)
			    		{
			    			rec_com();    						                
			    		} 
					if(smotor_speed ==0)
					   break;
						 
				}
				if(smotor_speed !=0)
				   motor.spd_obj = smotor_speed * 100;				
			}	
		}	
		else
		{
			if(motor.stop_flag == 0) 
			{
				for(i=0;i<5;i++)//减速停车
		    	{	  	  	  		     
					if(motor.spd_obj <= 1200)
					{
						sewing_stop();	
						break;
					}	 
					else if( motor.spd_obj < 1800)
					{
						motor.spd_obj = 1000;      
					}
					else if(motor.spd_obj < 2500 )
					{
						motor.spd_obj = 1600;      
					}
					else if(motor.spd_obj >= 2500)
					{
						motor.spd_obj = 2300;      
					}	
					delay_ms(15); 
					//delay_ms(5);  
					
					while(motor.angle_adjusted >= 16)
			    	{
			    		rec_com();    						                
			    	}  	 
					while(motor.angle_adjusted < 16)
			    	{
			    		rec_com();    						                
			    	} 
						 
				}	
				while(motor.stop_flag == 0)    
				{
				    rec_com();    // communication with panel                  	
				} 	  
				inpress_up();
				delay_ms(150);
			}	
		}	
	  }
	}	
	
	if(pause_flag == 1)//press sotp button when stepper motor moving     2011-9-9
	{ 
		sys.status = ERROR;
		StatusChangeLatch = ERROR;
		sys.error = ERROR_02;
		status_now = CHECKI04;
		smotor_speed = 0;
		sewing_stop();		
		while(motor.stop_flag == 0)    
		{
			rec_com();                     	
		} 	  
		inpress_up();
		delay_ms(150);
	}
	if(StatusChangeLatch != CHECKI04) 
	{
 		predit_shift = 0;
		sys.status = StatusChangeLatch;
	}
}
//--------------------------------------------------------------------------------------
//  Name:		checki05_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of checki05 stauts---check output signal
//--------------------------------------------------------------------------------------	
void checki05_status(void)
{	
  	UINT16 i,flag,test_flag;
	UINT16 running_time,interval_time;

	test_flag = 0;
  	//--------------------------------------------------------------------------------------
  	// output move
  	//--------------------------------------------------------------------------------------
	switch(output_com)
	{
		//--------------------------------------------------------------------------------------      
    	//  wiper move
    	//--------------------------------------------------------------------------------------
    	case 1: //wiper
		    /*if(inpress_flag == 1)  // foot is up
	    	{
	    		if( u224 ==2 )
				{
					go_origin_zx();
					delay_ms(100);
				}
			    inpress_down(0);
				delay_ms(100);
	    	}*/
			//SNT_H = 1;  
			FW = 1;
			delay_ms(wiper_end_time);
			FW = 0;			
	        output_com = 0;
	        break;
    	//--------------------------------------------------------------------------------------      
    	//  trimmer move
    	//--------------------------------------------------------------------------------------
    	case 2: 

			FA = 1;       
			trim_io_control(ON);
			delay_ms(200);
			trim_io_control(OFF);
			FA = 0;
			delay_ms(200);

	        output_com = 0;
	        break;
	  	//--------------------------------------------------------------------------------------      
    	//  foot move
    	//--------------------------------------------------------------------------------------
    	case 3: 
			if(foot_flag == 1) 
    	    {
				foot_down();
    	   	}
    	    else               
    	    {
    	    	foot_up();
    	    }
		    delay_ms(200);
	        output_com = 0;
	        break;
	  	//--------------------------------------------------------------------------------------      
    	//  infoot move
    	//--------------------------------------------------------------------------------------
   	 	case 4: 
			if(inpress_flag == 1)  
    	    {
    	    	inpress_down(0);
    	    }
    	    else                  
    	    {
    	    	inpress_up();
    	    }    	      
	        output_com = 0;
	        break;
		case 5:	//thread tension
			SNT_H = 1;
			da0 = 250;
			delay_ms(200);
			da0 = 0;
			SNT_H = 0;
	        output_com = 0;
	        break;
		case 6:
			FA =1;
            delay_ms(200);
			FA = 0;
	        delay_ms(200);
			output_com = 0;

		break;
		
		case 7:
			 EXTEND = 1;
			 T_DIR_EXTEND = 1;
			 delay_ms(200);
			 EXTEND = 0;
			 T_DIR_EXTEND = 0;
			 delay_ms(200);
			 output_com = 0;
			 #if DA0_TEST_FUN_ENABLE
			 test_flag = 1;
			 running_time = 18000;
			 interval_time = 1000;
			 #endif
		break;
		case 8:                         //fuzhuqifa2
		     FR_ON = 1;
			 delay_ms(200);
			 FR_ON = 0;
			 delay_ms(200);
			 output_com = 0;
			 #if DA0_TEST_FUN_ENABLE
			test_flag = 1;
			running_time = 24000;
			interval_time = 1000;
			#endif
		break;
		case 9:
		     FK_OFF = 1;
			 delay_ms(200);
			 FK_OFF = 0;
			 delay_ms(200);
			 output_com = 0;
			 #if DA0_TEST_FUN_ENABLE
			test_flag = 1;
			running_time = 36000;
			interval_time = 1000;
			#endif
		break;
		case 10:
		     T_HALF = 1;
			 delay_ms(200);
			 T_HALF = 0;
			 delay_ms(200);
			 output_com = 0;
			 #if DA0_TEST_FUN_ENABLE
			test_flag = 1;
			running_time = 18000;
			interval_time = 500;
			#endif
		break;
		case 11:
		     T_DIR = 1;
			 delay_ms(200);
			 T_DIR = 0;
			 delay_ms(200);
			 output_com = 0;
		break;
		case 12:
		     T_CLK = 1;  
			 delay_ms(200);
			 T_CLK = 0;
			 delay_ms(200);
			 output_com = 0;
		break;
       
	}	
	output_com = 0;
	#if DA0_TEST_FUN_ENABLE
	if( test_flag == 1)
	{
			flag = 1;
			while(flag)
			{
				rec_com();				
				temp_tension = tension_release_value;
				tension_open_counter = 0;
				da0 = 250;
				tension_open_switch =1;
				for(i=0;i<running_time;i++)//打开一分钟
				{
					delay_ms(1);
					if (PAUSE == PAUSE_ON)
					{
						delay_ms(10);                           
						if(PAUSE == PAUSE_ON)
						{
							flag = 0;
							break;	
						}
					}
				}
				tension_open_switch =0;
				da0 = 0;
				if(flag == 1)
				{
					for(i=0;i<interval_time;i++)//间隔3秒
					{
						delay_ms(1);
						if (PAUSE == PAUSE_ON)
						{
							delay_ms(10);                           
							if(PAUSE == PAUSE_ON)
							{
								flag = 0;
								break;	
							}
						}
					}
				}				
			} 
	}
	#endif
	
	if(StatusChangeLatch != CHECKI05) 
	{
 		predit_shift = 0;
		sys.status = StatusChangeLatch;
	}
}
//--------------------------------------------------------------------------------------
//  Name:		checki06_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of checki06 stauts---check X and Y sensor
//--------------------------------------------------------------------------------------	
void checki06_status(void)
{	
	UINT8 temp8;
  	if(sys.status == ERROR)
  	{
    	return;
  	}

 	if(foot_com == 0)
	{		 
	  	footer_both_down();       
		foot_com = 2;
	}
	
	if(origin_com == 1)
	{		
		opl_origin_flag = 0;
	  	go_origin_allmotor();	 
		status_now = CHECKI06;  
	  	origin_com = 0; 
		footer_both_down();
		delay_ms(50);       		
  	}
 	//--------------------------------------------------------------------------------------
	//  start sensor
	//--------------------------------------------------------------------------------------  
	if(DVA == para.dvab_open_level)           							
	{
			delay_ms(10);
			if(DVA == para.dvab_open_level)
			{				
				go_origin_xy();    						
				while(DVA == para.dvab_open_level)
		    	{
		      		rec_com();       					
		    	}	 	      	    
		  	}
	}
	if(DVB == para.dvab_open_level)           				
	{
		delay_ms(10);
		if(DVB == para.dvab_open_level && DVBLastState != para.dvab_open_level)
		{				
			if ( foot_flag == 0)
			{
			    footer_both_up();
			}
			else 
				footer_both_down();
		}
	}
	DVBLastState = DVB;
	//=======================================
	#if DEBUG_TEST_NOPMOVE
	if (PAUSE == PAUSE_ON)
	{
			delay_ms(10);                           
			if(PAUSE == PAUSE_ON)
			{				
				if(DVA == para.dvab_open_level)           							
				{
					delay_ms(10);
					if(DVA == para.dvab_open_level)
					{				
						test_quickmove_program();  						
						while( (DVA == para.dvab_open_level)||(PAUSE == PAUSE_ON) )
				    	{
				      		rec_com();       					
				    	}	 	      	    
				  	}
			  	}	  			  	
		  	}
	}
	#endif
	
  	//--------------------------------------------------------------------------------------
  	//  manual shift step  
  	//--------------------------------------------------------------------------------------
	if((shift_flag == 0x88)&&(shift_flag_old&0xf0))
	{
		shift_flag = shift_flag_old;
		shift_low_flag = 1;		
	}
	shift_func(shift_flag);
  	predit_shift = 0; 
	if(StatusChangeLatch != CHECKI06) 
	{
 		predit_shift = 0;
		sys.status = StatusChangeLatch;
	}
}
//--------------------------------------------------------------------------------------
//  Name:		checki07_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of checki07 stauts---check press motor and sensor
//--------------------------------------------------------------------------------------	
void checki07_status(void)
{	
	if(sys.status == ERROR)
  	{
    	return;
  	}
	DisMotorAngle = ((INT32)motor.angle * 23040)>>14;
	
	if(StatusChangeLatch != CHECKI07) 
	{
 		predit_shift = 0;
		sys.status = StatusChangeLatch;
	}	
}
//--------------------------------------------------------------------------------------
//  Name:		checki08_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of checki08 stauts---check clamp thread motor and sensor
//--------------------------------------------------------------------------------------	
void checki08_status(void)
{	
	if( inpress_first_flag == 1)
	{
	    go_origin_zx();
		inpress_first_flag = 0;
	}
	if( inpress_flag == 1)//2013-3-19
	{
		if(u224 == 2)
		{
		  foot_down();
		  delay_ms(200);
		}
		inpress_down(0);
	}
	if(StatusChangeLatch != CHECKI08) 
	{
 		predit_shift = 0;
		sys.status = StatusChangeLatch;
	}
}
//--------------------------------------------------------------------------------------
//  Name:		checki10_status 
//  Parameters:	None
//  Returns:	None
//  Description: control machines of checki10 stauts---check inpress motor and sensor
//--------------------------------------------------------------------------------------	
void checki10_status(void)
{	
	UINT8 temp8;

  	//--------------------------------------------------------------------------------------
  	// config main motor driver
  	//-------------------------------------------------------------------------------------- 
  	if(sys.status == ERROR)
  	{
  	  predit_shift = 0;
  	  return;
  	}	
   	//--------------------------------------------------------------------------------------
  	//  find origin
  	//-------------------------------------------------------------------------------------- 

	if(origin_com == 1) 
	{		
	   	origin_com = 0;     
   	}
		//--------------------------------------------------------------------------------------
	  	// inpress motor
	  	//-------------------------------------------------------------------------------------- 
	if(inpress_action_flag == 1) 
	{
			 if(inpress_high < 0)
			 {
			     inpress_down(inpress_high_base);
			 }
			 else if(inpress_high == 81)
			 {
			 	 temp8 = detect_position();	
			     if(temp8 == OUT)     
			     {
					 find_dead_center();
			     }
				 inpress_up();
			 }
			 else
			 {
				R_AIR = 1;
				delay_ms(200);
			 	inpress_to(inpress_high);
			 }
			 delay_ms(120);
			 inpress_action_flag = 0;
			 predit_shift = 0;
	}
		//------------------------------------------------------------------------------------
		//stitch up command
		//------------------------------------------------------------------------------------	
	 	if(StitchUpFlag ==1)
		{
			temp8 = detect_position();	
	    	if(temp8 == OUT)     
	      	{
					find_dead_center();
	      	}
			StitchUpFlag = 71;
			predit_shift = 0;
		}
		else if(StitchUpFlag ==0)
		{
			if(inpress_flag ==1)
			{
				inpress_down(inpress_high_base); 
				delay_ms(150);                   
				inpress_high = 0;
			}
			needle_down();
			StitchUpFlag = 71;
			predit_shift = 0;
		}
    	if(DVB == para.dvab_open_level)           								// start sensor is pushed
		{
			delay_ms(10);
			if(DVB == para.dvab_open_level)
			{
				R_AIR ^= 1;
			}	
		}
		//--------------------------------------------------------------------------------------
	  	//  start sensor
	  	//--------------------------------------------------------------------------------------  
	  	if(DVA == para.dvab_open_level)           								// start sensor is pushed
		{
			delay_ms(10);
			if(DVA == para.dvab_open_level)
			{				
				temp8 = detect_position();	        //2012-2-29 add
		        if(temp8 == OUT)    // out of range 
  		         {
			       find_dead_center();
  		          }			
				go_origin_zx();   
				//go_origin_yj();
      			stepmotor_state = 0x00;
      			stepmotor_comm = 0xff;
				inpress_high = inpress_origin;
				while(DVA == para.dvab_open_level)
		    	{
		      		rec_com();       						// communication with panel	
		    	}	 	      	    
		  	}
	  	}
	
  	//--------------------------------------------------------------------------------------
  	//  manual shift step  
  	//--------------------------------------------------------------------------------------
  	switch(stepmotor_comm)
  	{
  		case 0x00: 
				go_origin_zx();
				delay_ms(230);
				inpress_high =inpress_position; 
				stepmotor_state = 0;  
				stepmotor_comm = 0xff;  
				predit_shift = 0;
				break;   // origin 
  		case 0x01: 
				inpress_to(71); 
				delay_ms(230); 
				inpress_high =inpress_position; 
				stepmotor_state = 1;  
				stepmotor_comm = 0xff;  
				predit_shift = 0;
				break;   // 7.0mm   		            							
  	  	case 0x02: 
	      	  	R_AIR = 1;
				delay_ms(100);
				inpress_to(35);  
				delay_ms(230);
				inpress_high =inpress_position; 
				stepmotor_state = 2;  
				stepmotor_comm = 0xff;  
				predit_shift = 0;
				break;   // 3.5mm     
  	  	case 0x03: 
		        R_AIR = 1;
				delay_ms(100);
				inpress_to(0);   
				delay_ms(230);
				inpress_high =inpress_position; 
				stepmotor_state = 3;  
				stepmotor_comm = 0xff;  
				predit_shift = 0;
				break;   // 0.0mm                       		            							   
  	  	case 0x04: 
				inpress_to(inpress_high_base);  
				delay_ms(30);
		        inpress_high =inpress_position; 
				stepmotor_state = 4;  
				stepmotor_comm = 0xff;  
				predit_shift = 0;
				break;   // -X.Xmm                             	     		            									  		            								    
  	  	default:                                                                  break;	
  	}
  	//--------------------------------------------------------------------------------------
  	//  single shift step 09.3.26 wr add 
  	//--------------------------------------------------------------------------------------
  	switch(stepmotor_single)
  	{
  		case 0x00://-ccw
  			movestep_zx(-1,1);
  		  	delay_ms(1);
			//movestep_yj(-1,1);
  			stepmotor_single = 0xff;
			predit_shift = 0;
  	  		break; 
  		case 0x01://+cw 
  			movestep_zx(1,1);
  		  	delay_ms(1);
			//movestep_yj(1,1);
  			stepmotor_single = 0xff;
			predit_shift = 0;
  			break;              	     		            									  		            								    
  	  	default:
  	  	    break;	
  	}
	delay_ms(20);
	predit_shift = 0;
	if(StatusChangeLatch != CHECKI10) 
	{
 		predit_shift = 0;
		sys.status = StatusChangeLatch;
	}
}
#if ROTATE_CUTTER_ENABLE  //旋转切刀测试
void checki11_status(void)
{
	UINT8 temp8;

   	if(sys.status == ERROR)
  	{
  	  predit_shift = 0;
  	  return;
  	}	
	
 	if(origin_com == 1) 
	{		
	   	origin_com = 0;     
   	}
	
	//--------------------------------------------------------------------------------------
	//  start sensor
	//--------------------------------------------------------------------------------------  
	if( DVA == para.dvab_open_level )           					
	{
		delay_ms(10);
		if(DVA == para.dvab_open_level)
		{				
			temp8 = detect_position();
		    if(temp8 == OUT)  
  		    {
			   find_dead_center();
  		    }	
					
			go_origin_qd();
      		stepmotor_state = 0x00;
      		stepmotor_comm = 0xff;
			while(DVA == para.dvab_open_level)
		    {
		      	rec_com();  
		    }	 	      	    
		}
	}
	
  	//--------------------------------------------------------------------------------------
  	//  manual shift step  
  	//--------------------------------------------------------------------------------------
  	switch(stepmotor_comm)
  	{
  		case 0x00: 
				go_origin_qd();
				stepmotor_state = 0;  
				stepmotor_comm = 0xff;  
				predit_shift = 0;
				break;   
  		case 0x01: 
				//movestep_yj(-stepper_cutter_move_range ,stepper_cutter_move_time);
				movestep_qd(-90 ,stepper_cutter_move_time);
				cutter_delay_counter = stepper_cutter_move_time;
				cutter_delay_flag = 1;
				stepmotor_state = 1;  
				stepmotor_comm = 0xff;  
				predit_shift = 0;
				break;   	            							
  	  	case 0x02: 
				//movestep_yj(stepper_cutter_move_range ,stepper_cutter_move_time);
				movestep_qd(90 - stepper_cutter_move_range ,20);//断线位置
				cutter_delay_counter = stepper_cutter_move_time;
				cutter_delay_flag = 1;
				stepmotor_state = 2;  
				stepmotor_comm = 0xff;  
				predit_shift = 0;
				break;     
  	  	case 0x03: 
				movestep_qd(stepper_cutter_move_range ,stepper_cutter_move_time);
				cutter_delay_counter = stepper_cutter_move_time;
				cutter_delay_flag = 1;
				stepmotor_state = 3;  
				stepmotor_comm = 0xff;  
				predit_shift = 0;
				break;                        		            							   
  	  	case 0x04: 
			    go_origin_qd();
				stepmotor_state = 4;  
				stepmotor_comm = 0xff;  
				predit_shift = 0;
				break;                              	     		            									  		            								    
  	  	default: break;	
  	}
  	//--------------------------------------------------------------------------------------
  	//  single shift step 09.3.26 wr add 
  	//--------------------------------------------------------------------------------------
  	switch(stepmotor_single)
  	{
  		case 0x00://-ccw
  			movestep_qd(-1,1);
  			stepmotor_single = 0xff;
			predit_shift = 0;
  	  		break; 
  		case 0x01://+cw 
  			movestep_qd(1,1);
  			stepmotor_single = 0xff;
			predit_shift = 0;
  			break;              	     		            									  		            								    
  	  	default:
  	  	    break;	
  	}
	delay_ms(20);
	predit_shift = 0;
	if(StatusChangeLatch != CHECKI11) 
	{
 		predit_shift = 0;
		sys.status = StatusChangeLatch;
	}
}
#else  //剪线电机调试
void checki11_status(void)
{
	UINT8 temp8;

   	if(sys.status == ERROR)
  	{
  	  predit_shift = 0;
  	  return;
  	}	
	
 	if(origin_com == 1) 
	{		
	   	origin_com = 0;     
   	}
	
	//--------------------------------------------------------------------------------------
	//  start sensor
	//--------------------------------------------------------------------------------------  
	if( DVA == para.dvab_open_level )           					
	{
		delay_ms(10);
		if(DVA == para.dvab_open_level)
		{				
			temp8 = detect_position();
		    if(temp8 == OUT)  
  		    {
			   find_dead_center();
  		    }			
			go_origin_yj();
      		stepmotor_state = 0x00;
      		stepmotor_comm = 0xff;
			while(DVA == para.dvab_open_level)
		    {
		      	rec_com();  
		    }	 	      	    
		}
	}
	
  	//--------------------------------------------------------------------------------------
  	//  manual shift step  
  	//--------------------------------------------------------------------------------------
  	switch(stepmotor_comm)
  	{
  		case 0x00: 
				go_origin_yj();
				stepmotor_state = 0;  
				stepmotor_comm = 0xff;  
				predit_shift = 0;
				break;   
  		case 0x01: 
				#if MACHINE_14090_MASC_PLUS
				movestep_yj(stepper_cutter_move_range ,stepper_cutter_move_time);
				#else
				movestep_yj(-90 ,stepper_cutter_move_time);
				#endif
				cutter_delay_counter = stepper_cutter_move_time;
				cutter_delay_flag = 1;
				stepmotor_state = 1;  
				stepmotor_comm = 0xff;  
				predit_shift = 0;
				break;   	            							
  	  	case 0x02: 
				#if MACHINE_14090_MASC_PLUS
				movestep_yj(-stepper_cutter_move_range ,stepper_cutter_move_time);
				#else
				movestep_yj(90 - stepper_cutter_move_range ,20);//断线位置
				#endif
				cutter_delay_counter = stepper_cutter_move_time;
				cutter_delay_flag = 1;
				stepmotor_state = 2;  
				stepmotor_comm = 0xff;  
				predit_shift = 0;
				break;     
  	  	case 0x03: 				
				movestep_yj(stepper_cutter_move_range ,stepper_cutter_move_time);
				cutter_delay_counter = stepper_cutter_move_time;
				cutter_delay_flag = 1;
				stepmotor_state = 3;  
				stepmotor_comm = 0xff;  
				predit_shift = 0;
				break;                        		            							   
  	  	case 0x04: 
			    go_origin_yj();
				stepmotor_state = 4;  
				stepmotor_comm = 0xff;  
				predit_shift = 0;
				break;                              	     		            									  		            								    
  	  	default: break;	
  	}
  	//--------------------------------------------------------------------------------------
  	//  single shift step 09.3.26 wr add 
  	//--------------------------------------------------------------------------------------
  	switch(stepmotor_single)
  	{
  		case 0x00://-ccw
  			movestep_yj(-1,1);
  			stepmotor_single = 0xff;
			predit_shift = 0;
  	  		break; 
  		case 0x01://+cw 
  			movestep_yj(1,1);
  			stepmotor_single = 0xff;
			predit_shift = 0;
  			break;              	     		            									  		            								    
  	  	default:
  	  	    break;	
  	}
	delay_ms(20);
	predit_shift = 0;
	if(StatusChangeLatch != CHECKI11) 
	{
 		predit_shift = 0;
		sys.status = StatusChangeLatch;
	}
}
#endif
typedef unsigned char ( *pt2FunctionErase)(unsigned long, unsigned short * );
#define BOOTLOADER_ADDR 0xfb000L
void download_status(void)
{
	pt2FunctionErase fp;

	UINT32 far* ptr;

	UINT32 entry;
	
	ptr = (unsigned long far *)( BOOTLOADER_ADDR );
	if(*ptr != 0xffffffff)
	{
		//==================protect
		U=1;V=1;W=1;
	  	prcr = 0x02;
		inv03 = 0;
		U_=0;V_=0;W_=0;
	  	prcr = 0x00;
		OUTPUT_ON = 1;
		//=================
		entry = BOOTLOADER_ADDR ;
		fp = (pt2FunctionErase)entry;
		fp( 0, 0);
	}
}

void single_move_func(void)
{
	UINT8 temp8,i;
	if( (single_flag!=0)&&(already_in_origin ==0) )
	{
		sys.status = ERROR;
		StatusChangeLatch = ERROR;
		sys.error = ERROR_46;
		predit_shift = 0;
		single_flag = 0;
		return;
	}
	if(single_flag!=0)
	{
		if(((foot_flag==1) || (footer_working_mode != 0 && foot_half_flag ==1)) && (u202 ==0))
		{
			sys.status = ERROR;
			StatusChangeLatch = ERROR;
			status_now = READY;
			sys.error = ERROR_45;
			return;
		}
	   if( second_start_switch == 1)//试缝以后，二次启动的功能仍然有效
		   second_start_counter = 0;
		   
	   if( FootRotateFlag == 1)
	   {
		   if( (single_flag == 2)||(single_flag ==7) )
		   		process_making_pen_signal(1);
		   else if( (single_flag == 1)||(single_flag ==6) )
		   		process_making_pen_signal(0);
				
		   FootRotateFlag = 0;
		   predit_shift = 0;
		   single_flag = 0;
		   return;
	   }
	   if(para.x_origin_mode == AUTO_FIND_START_POINT)
	   {
			if (already_auto_find_start_point == 0)
			{
				find_start_point_x();
				already_auto_find_start_point = 1;
				allx_step = 0;
				ally_step = 0;
				delay_ms(100);
				if( sys.error == 0)
				go_startpoint();
			}
	   }
		 
	}

	switch(single_flag)
	{
			case 0:
					   if(origin_com != 1)
					   {
						   predit_shift = 0;
					   }
			break;
			case 1:
						predit_shift = 1;
						
						return_from_setout = 0;
						not_in_origin_flag = 1;
						allx_step = allx_step + xstep_cou;
						ally_step = ally_step + ystep_cou;

						if(allx_step < -RESOLUTION*u213 || allx_step > RESOLUTION*u214 || ally_step < -RESOLUTION*u216 || ally_step > RESOLUTION*u215)
						{
							allx_step = allx_step - xstep_cou;
							ally_step = ally_step - ystep_cou;
							do_pat_point_sub_one();
							single_flag = 0;
							move_flag = 0;
							nopmove_flag = 0;
							xstep_cou = 0;	
							ystep_cou = 0;
							sys.error = ERROR_15;
							StatusChangeLatch = ERROR;
						}
						else
						{
							SewTestStitchCounter++;
							allx_step = allx_step - xstep_cou;
							ally_step = ally_step - ystep_cou;
							if(StopStatusFlag == 1)
							{
								temp8 = detect_position();
								if(temp8 == OUT)
								{
									find_dead_center();
									delay_ms(100);
								}
							}
							
							if( (motor.angle_adjusted>=440) &&(motor.angle_adjusted <=1000) )//110-250
							  	 find_dead_center();
							if( nop_move_pause_flag ==1)
							{
								process_nop_move_pause(1);
								single_flag = 0;
							} 
					     	else
						 	{
								if( inpress_follow_flag == 1)
								{
									if(nopmove_flag == 1)
									{
										if(inpress_flag == 0)
										{
											inpress_up();
											delay_ms(100);
										}
									}
									if(move_flag == 1)
									{
										if((inpress_flag == 1)&&(making_pen_actoin_flag == 0))
										{
											inpress_down(last_inpress_position);
											delay_ms(100);
										}
									}
								}
								else
								{
									if(inpress_flag == 0)  
									{
										inpress_up();
										delay_ms(100);
									}
								}
								#if ENABLE_JUMP_NOPMOVE
								if(nopmove_flag == 1)
								{
									do_pat_point_sub_one();
									go_beginpoint(0);
									single_flag = 0; 
								}
								else if(move_flag == 1)
									move_next();
								#else
									move_next();
								#endif
							}
						}

						predit_shift = 0;
						break;
			case 2:
						predit_shift = 1;
						return_from_setout = 0;
						not_in_origin_flag = 1;
						SewTestStitchCounter--;
						if( (motor.angle_adjusted>=440) &&(motor.angle_adjusted <=1000) )//110-250
							 find_dead_center();
						if( inpress_follow_flag == 1)
						{
							 if(nopmove_flag == 1)
							{
								if(inpress_flag == 0)
								{
									inpress_up();
									delay_ms(100);
								}
							}

							if(move_flag == 1)
							{
								if((inpress_flag == 1)&&(making_pen_actoin_flag == 0))
								{
									inpress_down(last_inpress_position);
									delay_ms(100);
								}
							}
						}
						else
						{
							if(inpress_flag == 0)  
							{
								inpress_up();
								delay_ms(100);
							}
						}
						if( nop_move_pause_flag ==1) 
						{
					    	process_nop_move_pause(2);
							finish_nopmove_pause_flag = 0;
							single_flag = 0;
						}
						else
						{
							#if ENABLE_JUMP_NOPMOVE
							if(nopmove_flag == 1)
							{
								back_endpoint();
								nopmove_flag = 0; 
								nop_move_pause_flag = 0;
								single_flag = 0;
							}
							else if(move_flag == 1)	
								move_back();
							#else
								move_back();
							#endif
						}
						predit_shift = 0;
						break;
			case 3:
						predit_shift = 1;
						PEN_SIGNAL = 0;
						LASER_SIGNAL =0;
						temp8 = detect_position();
						if(temp8 == OUT)
						{
							find_dead_center();
						}
						if(inpress_flag == 0)  	
						{
							inpress_up();
							delay_ms(100);
						}
						move_startpoint();
						
						FootRotateFlag = 0;
						StopStatusFlag = 0;
						stop_flag = 0;
						nopmove_flag = 0;
						move_flag = 0;
						cut_flag = 0;
						origin2_lastmove_flag = 0;
						SewingStopFlag = 0;
						RotateFlag = 0;
						end_flag = 0;
						foot_com = 1;
						predit_shift = 0;
						break;
			//==================================================
				case 6:
						predit_shift = 1;
						single_edit_continue_next();
						if( inpress_follow_flag == 1)
						{
							if(nopmove_flag == 1)
							{
								if(inpress_flag == 0)
								{
									inpress_up();
									delay_ms(100);
								}
							}

							if(move_flag == 1)
							{
								if((inpress_flag == 1)&&(making_pen_actoin_flag == 0))
								{
									inpress_down(last_inpress_position);
									delay_ms(100);
								}
							}
						}
						else
						{
							if(inpress_flag == 0)  
							{
								inpress_up();
								delay_ms(100);
							}
						}
						if(end_flag == 1)
						{
							single_flag = 0;
							single_reply = 0x51;
							end_flag = 0;
							break;
						}
						#if ENABLE_JUMP_NOPMOVE
						if(nopmove_flag == 1)
						{
							nopmove_flag = 0; 
							do_pat_point_sub_one();
							go_beginpoint(0);
						}
						else if(move_flag == 1)
							move_continue_next();
						#else
							move_continue_next();
						#endif
						for(i=0;i<movestepxy_time-3;i++)
						{
							delay_ms(1);
						}
						predit_shift = 0;
						delay_ms(3);
						
					break;
			case 7:
					predit_shift = 1;
					single_edit_continue_back();
					if( inpress_follow_flag == 1)
					{
						if(nopmove_flag == 1)
						{
							if(inpress_flag == 0)
							{
								inpress_up();
								delay_ms(100);
							}
						}

						if(move_flag == 1)
						{
							if((inpress_flag == 1)&&(making_pen_actoin_flag == 0))
							{
								inpress_down(last_inpress_position);
								delay_ms(100);
							}
						}
					}
					else
					{
						if(inpress_flag == 0)  
						{
							inpress_up();
							delay_ms(100);
						}
					}
					if(start_flag ==1)
				   {
						start_flag = 0;
						single_flag = 0;
						single_reply = 0x51;
						break;
				   }
				   #if ENABLE_JUMP_NOPMOVE
					if( nopmove_flag == 1)
					{
						back_endpoint();
						nopmove_flag = 0; 
						movestepxy_time = 10;
					}
					else if(move_flag == 1)
						move_continue_back();
					#else
						move_continue_back();
					#endif
					for(i=0;i<movestepxy_time-3;i++)
					{
						delay_ms(1);
					}
					predit_shift = 0;
					delay_ms(3);
				break;
			case 8:
				single_flag = 0;
			break;
		default:
			break;
	  }
}

void shift_func(UINT8 shift_num)
{
	if( sys.status == PREEDIT)
	{
		switch(shift_num)
	  	{
	  		case 0x0C: shift_12();                    break;    		            							
    		case 0x01: shift_01();                    break;    		            							
	    	case 0x03: shift_03(); 	                  break;  
	    	case 0x04: shift_04();                    break;
	    	case 0x06: shift_06();   				  break;    		            							
	    	case 0x07: shift_07();                    break;
	    	case 0x09: shift_09();                    break;
	    	case 0x0A: shift_10();                    break;    		            									  		            								
	    	case 0xEC: remove_12();                   break;
	    	case 0xE1: remove_01();                   break;    		            							
	    	case 0xE3: remove_03();                   break;  
	    	case 0xE4: remove_04();                   break;
	    	case 0xE6: remove_06();                   break;    		            							
	    	case 0xE7: remove_07();                   break;
	    	case 0xE9: remove_09();                   break;
	    	case 0xEA: remove_10();                   break; 	
	    	case 0x88: remove_stop();                 break; 		
	    	case 0x5C: remove_12();   delay_process();    break;//Y-
	    	case 0x51: remove_01();   delay_process();    break;//X-Y-    		            							
	    	case 0x53: remove_03();   delay_process();    break;//X-  
	    	case 0x54: remove_04();   delay_process();    break;//X-Y+
	    	case 0x56: remove_06();   delay_process();    break;//Y+    		            							
	    	case 0x57: remove_07();   delay_process();    break;//X+Y+
	    	case 0x59: remove_09();   delay_process();    break;//X+
	    	case 0x5A: remove_10();   delay_process();    break;//X+Y-     		
	    	default:   				                     break;	
	  	}
	}
	else
	{
		switch(shift_num)
	  	{
	  		case 0x06: shift_12();                    break;    		            							
    		case 0x07: shift_01();                    break;    		            							
	    	case 0x09: shift_03(); 	                  break;  
	    	case 0x0a: shift_04();                    break;
	    	case 0x0c: shift_06();   				  break;    		            							
	    	case 0x01: shift_07();                    break;
	    	case 0x03: shift_09();                    break;
	    	case 0x04: shift_10();                    break;
			    		            									  		            								
	    	case 0xE6: remove_12();                   break;
	    	case 0xE7: remove_01();                   break;    		            							
	    	case 0xE9: remove_03();                   break;  
	    	case 0xEA: remove_04();                   break;
	    	case 0xEC: remove_06();                   break;    		            							
	    	case 0xE1: remove_07();                   break;
	    	case 0xE3: remove_09();                   break;
	    	case 0xE4: remove_10();                   break; 	
	    	case 0x88: remove_stop();                 break; 		
	    	case 0x56: remove_12();   delay_process();    break;
	    	case 0x57: remove_01();   delay_process();    break;    		            							
	    	case 0x59: remove_03();   delay_process();    break;  
	    	case 0x5A: remove_04();   delay_process();    break;
	    	case 0x5C: remove_06();   delay_process();    break;    		            							
	    	case 0x51: remove_07();   delay_process();    break;
	    	case 0x53: remove_09();   delay_process();    break;
	    	case 0x54: remove_10();   delay_process();    break;     		
	    	default:   				                     break;	
	  	}
	}
	predit_shift = 0; 
	if( (shift_num !=0 )&&(sys.status != PREEDIT) )//already left the origin point,you cann't change pattern
		already_in_origin = 0;
}


//--------------------------------------------------------------------------------------
void download_drv_status(void)
{
	UINT16 delay,delay_t;
	switch(sys.status)
	{
		case DOWNLOAD_DRV1:
			 download_drv_flag = 1;
	 	break;
		case DOWNLOAD_DRV2:
			 download_drv_flag = 2; 
		break;
		case DOWNLOAD_DRV3:
			 download_drv_flag = 3;
	 	break;
		case DOWNLOAD_DRV4:
			 download_drv_flag = 4; 
		break;
	}
	delay_t = 13000;
	if(DVB == para.dvab_open_level)  
	{
			delay_ms(10);
			if(DVB == para.dvab_open_level)
			{	
				delay_t = 25000;			
		  	}
	}
	if(DVA == para.dvab_open_level)           								// start sensor is pushed
	{
			delay_ms(10);
			if(DVA == para.dvab_open_level)
			{	
				delay_t = 45000;			
		  	}
		
	}
	if(5 == predit_shift) 
	{  
	    if(1 == erase_falg)
		  jump_to_begin();
		if(((stepversion1 >= 60000)&&(sys.status == DOWNLOAD_DRV1)) ||((stepversion2 >= 60000)&&(sys.status == DOWNLOAD_DRV2))
			||  ((stepversion3 >= 60000)&&(sys.status == DOWNLOAD_DRV3)) || sys.status == DOWNLOAD_DRV4)//DSP4版本上不做限定，直接升级
		{
			send_stepmotor_up_drv();
			if(1 == erase_falg)
			     delay_ms(delay_t); 
			else  
			    delay_ms(40);         
		    drv_satus = read_stepmotor_up_drv();
			//0XA0    功能码错误
			//0XA1    文件校验错误
			//0XA2    数据包自校验错误
			//0XA3    SPI通信校验错误
			//0XA4    flash擦除错误
			//0XA5    flash烧写错误
			//0XA6    flash校验错误
			//0XA7    数据包crc校验错误
			//0XA8    解锁错误
			//0xa9    超时
			delay = 0;
			while(0x00 != drv_satus)
			{
				rec_com();
				delay_ms(5);
				drv_satus = read_stepmotor_up_drv();
				delay++;
				if( delay >1000)//超时
				{
				    sys.status = ERROR;
					sys.error = ERROR_86;
					de_bug.test1 = 0x00;
					de_bug.test2 = 0xa9;
					break;	
				}
				if( (drv_satus >=0xa0) && (drv_satus <=0xaf) )
				{
					sys.status = ERROR;
					sys.error = ERROR_86; //50 号错误？？？？？？？？？？？
					de_bug.test1 = 0x00;
					de_bug.test2 = drv_satus;
					break;
				}
			}
			predit_shift = 0;

		}
		else
		{
			predit_shift = 0;
		}
	}
	if(6 == predit_shift ) //2升级结束校验证
	{  
		if(((stepversion1 >= 60000)&&(sys.status == DOWNLOAD_DRV1)) ||((stepversion2 >= 60000)&&(sys.status == DOWNLOAD_DRV2))
		  ||  ((stepversion3 >= 60000)&&(sys.status == DOWNLOAD_DRV3)) || sys.status == DOWNLOAD_DRV4)
		{
			send_stepmotor_end_drv();
		    predit_shift = 0;
			ta0s = 0;  			      // stop timer A0 
			ta0ic = 0;
			predit_shift = 0;	
		}
		else
		{
			predit_shift = 0;
		}
	}
    rec_com();		
}

void download_multipul_program_status(void)
{
  UINT16 delay;
  if(5 == predit_shift) 
  {  
	    if(1 == erase_falg)
	       multipule_program_beginning(4);
	   
	    send_multipule_program_data(4);//发送升级程序内容
	    if(1 == erase_falg)
	        delay_ms(2500); 
	    else  
	    	delay_ms(60);
	
	    drv_satus = read_multipule_program_status(4);//读取升级状态
	    delay = 0;
	    while(0x00 != drv_satus)
	    {
	        rec_com();
	        delay_ms(5);
	        drv_satus = read_multipule_program_status(4);//读取升级状态
	        delay++;
	        if( delay >1000)//超时
	        {
	          sys.status = ERROR;
	          sys.error = ERROR_86;
	          de_bug.test1 = 0x00;
	          de_bug.test2 = 0xa9;
	          break;	
	        }
	        if( (drv_satus >=0xa0) && (drv_satus <=0xaf) )
	        {
	          sys.status = ERROR;
	          sys.error = ERROR_86; //50 号错误？？？？？？？？？？？
	          de_bug.test1 = 0x00;
	          de_bug.test2 = drv_satus;
	          break;
	        }
	     }
      	 predit_shift = 0;
  }
  
  if(6 == predit_shift ) //2升级结束校验证
  {  
      multipule_program_end(4);
      predit_shift = 0;
  }
  rec_com();		
}
void continue_status(void)
{
	
	INT16 temp16_xo,temp16_yo;
	
	allx_step = 0;
	ally_step = 0;	
	
	pat_point = (PATTERN_DATA *)(pat_buf);
	
	while(1)
	{				
  		process_data();	
	
    	if(nopmove_flag == 1)
    	{	  
    		     
      		allx_step = allx_step + xstep_cou;
			ally_step = ally_step + ystep_cou; 			
      		
			              
			if(allx_step < -RESOLUTION*u213 || allx_step > RESOLUTION*u214 || ally_step < -RESOLUTION*u216 || ally_step > RESOLUTION*u215)
			{
				do_pat_point_sub_one();		
				nopmove_flag = 0;				
				sys.error = ERROR_15;
				StatusChangeLatch = ERROR;
				allx_step = allx_step - xstep_cou;
  				ally_step = ally_step - ystep_cou;
				OutOfRange_flag = 1;
				return;
			}
			  
      		if(lastmove_flag == 1)   /* the last nop move */
      		{    	  
    	  		nopmove_flag = 0;
    	  		lastmove_flag = 0;
				process_data();  //2012-6-27
				do_pat_point_sub_one();				   
				  
    	  		break;   	    	    	
      		}       
      		nopmove_flag = 0;
    	}
		
		if(origin2_lastmove_flag == 1 || StopStatusFlag == 1)
		  {
		   origin2_lastmove_flag = 0;
		   stop_flag = 0;
		   //StopStatusFlag = 0;
		   process_data();
		  } 
		
    	if(move_flag ==1)
    	{
			do_pat_point_sub_one();  	    	
    		break;   	
    	}	
    	rec_com();    // communication with panel                                       	        
  	}
	
   if(allx_step == allx_step_last && ally_step == ally_step_last)
     {
		 process_data();		     
				
		 if(FootRotateFlag == 1 || origin2_lastmove_flag == 1)
			{
				if(FootRotateFlag ==1)
				{
				   process_making_pen_signal(0);
				   FootRotateFlag = 0;	
				   if( sys.error != 0 )
				      return;
				}
				process_data();			
			}
		move_next();					        	
	   	 
	 }
	 else
	 {
		 temp16_xo = allx_step;
		 temp16_yo = ally_step;
		 
		 allx_step = allx_step_last;
		 ally_step = ally_step_last;
		 
		 go_commandpoint(temp16_xo,temp16_yo);		 
		 
	 }
	  
	 
	 if(stop_flag == 1)
		{
			sys.status = READY;
			StatusChangeLatch = READY;
			StopStatusFlag = 1;
			if(u208 == 1)
			{
				footer_both_up();
			}
		    if(pat_point->para == 0)          //5-5
		       footer_both_down();
		    else if(pat_point->para == 1)
		       footer_both_up();
		}
	else
		{
	       	sys.status = RUN;
			StatusChangeLatch = RUN;
			StopStatusFlag = 0;	 
	       	
		} 			
 	
}
//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
