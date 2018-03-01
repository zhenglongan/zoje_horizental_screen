//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : variables.h
//  Description: external variables declaration
//  Version    Date     Author    Description
//  0.01     03/07/07   pbb        created
//  ... 
//  ...
//--------------------------------------------------------------------------------------
#ifndef VARIABLES_H
#define VARIABLES_H
//--------------------------------------------------------------------------------------
// 	Includes
//--------------------------------------------------------------------------------------
#include "typedef.h"      //Data type define
#include "common.h"
//--------------------------------------------------------------------------------------
//  External variables declaration
//--------------------------------------------------------------------------------------
extern UINT8 MAIN_MOTOR_TYPE;
extern INT16 Ud_i,Uq_i,d_iq_last;//scx
extern SYS_STRU sys;
extern MOTOR_STRU motor;
extern PI_SPD_IQ Run_SpdIq;//scx
extern PATTERN_DATA *pat_point;
extern PATTERN_DATA *sta_point,*origin2_point;
extern PATTERN_DATA *TempEnd_point;
extern PATTERN_DATA *TempStart_point;
extern PATTERN_DATA *TempPat_point;
extern PATTERN_DATA PatStart;
extern PATTERN_DATA PatEnd;
extern UINT8 EditCurrentCoorIndex;
extern INT16 EditCurrentCoorLatch[255];
extern INT16 ElementPointPositionLatch[255];

extern STEPVERSION Step1Version;
extern STEPVERSION Step2Version;
extern STEPVERSION Step3Version;
extern STEPVERSION Step4Version;

extern UINT8 CourseBackStartFlag;

extern UINT8 flag_1ms;
extern UINT16 ms_counter; 
extern UINT16 ms_scan_counter; 
extern UINT16 foot_ready_counter_dva;
extern UINT16 foot_ready_counter_dvb;
extern UINT16 wipe_time; 
extern UINT16 us_counter;  
extern UINT8 pat_buf[]; 

extern UINT8 *recpat_point;

extern UINT8 pedal_state;
extern UINT8 pedal_last_state;
extern UINT16 cut_start_angle;
extern UINT16 cut_end_angle;
extern UINT16 tension_start_angle;
extern UINT16 tension_end_angle;
extern UINT16 wiper_start_time;
extern UINT16 wiper_end_time;
extern UINT8 pedal_style;
extern UINT8 foot_flag;     
extern UINT8 foot_half_flag;  
extern UINT8 inpress_flag;   
extern INT16 inpress_position;    
extern UINT8 clamp_flag;
extern UINT8 OutOfRange_flag;
extern UINT8 DVSMLastState;
 
extern UINT8 zpl_pass;   
  
extern UINT8 origin_com; 
extern UINT8 wind_com;    
extern UINT8 repeat_com;       
extern UINT8 manual_com; 
extern UINT8 foot_com; 
extern UINT8 foot_half_com;
extern UINT8 inpress_com; 
extern UINT8 coor_com;   
extern UINT8 cooradjust_com;      // coordinate compensation command 
   
extern UINT16 stitch_num; 
extern UINT16 stitch_counter;
extern INT16 allx_step,ally_step;
extern INT16 xstep_cou,ystep_cou;
extern INT16 sox_step,soy_step;   
extern INT16 comx_step,comy_step; 

extern INT16 allx_coor_temp,ally_coor_temp;
extern INT16 allx_step_temp,ally_step_temp;
extern UINT8 predit_shift;
extern UINT16 DelayCounter;
extern UINT32 DelayTime;
extern INT16 SewingCounter;
extern UINT8 SewingTestFlag;
extern UINT8 MotorPositionSet;

extern UINT8 PatternDelayFlag;
extern UINT16 PattenrDelayTime;
extern UINT8 RotateFlag;

extern UINT8 SewingSpeedValue;

extern UINT8 SewingStopFlag;
extern UINT8 SewingStopValue;
extern UINT8 MotorSpeedRigister;

extern UINT8 StitchUpFlag;
extern UINT8 PointShiftFlag;
extern UINT8 PointShiftDirection;
extern UINT16 PointShiftNumber;

extern UINT16 SewTestStitchCounter;
extern UINT16 PatternShiftValue;

extern UINT8 ShapePointFlag;
extern UINT8 SewDataFlag;
extern UINT8 ElementPointFlag;
extern UINT8 ElementIndex;
extern UINT8 ElementIndexLatch;
extern UINT8 TestStatusChangeFlag;
extern UINT8 MoveToCurrentCoorFlag;
extern INT16 CurrentXCoor;
extern INT16 CurrentYCoor;
extern UINT8 EditEntranceFlag;
extern UINT8 SewingTestEndFlag;
extern INT16 StepDrive1Version;
extern INT16 StepDrive2Version;
extern UINT8 StatusChangeLatch;
extern UINT8 CurrentPointShiftFlag;
extern INT16 CurrentPointShiftPosition;
extern UINT8 EditElementCommand;
extern UINT8 CancelOverlabPointFlag;
extern UINT8 FootUpCom;
extern UINT8 DVBLastState;
extern UINT8 DVALastState;
extern UINT8 FootRotateFlag;

extern INT16 DestinationPointShiftPosition;
extern UINT8 MotionSet;
extern UINT8 MotiongSetFlag;
extern UINT8 StopStatusFlag;

extern UINT8 move_flag;   
extern UINT8 movex_flag;		   // move x flag in pattern process
extern UINT8 movey_flag;		   // move y flag in pattern process
extern UINT8 movestep_x_flag;	   // move x step motor flag
extern UINT8 movestep_y_flag;     // move y step motor flag        
extern UINT8 lastmove_flag; 
extern UINT8 origin2_lastmove_flag; 
extern UINT8 NopMoveSpd_flag;   
extern UINT8 nopmove_flag;
extern UINT8 cut_flag;
extern UINT8 laststitch_flag;   
extern UINT8 cut_start_flag;
extern UINT8 cut_end_flag;
extern UINT8 tension_start_flag;
extern UINT8 tension_end_flag;
extern UINT8 wipe_start_flag;
extern UINT8 wipe_end_flag;
extern UINT8 inpress_start_flag;
extern UINT8 inpress_end_flag;
extern UINT8 wipe_start_time;
extern UINT8 wipe_end_time; 
extern UINT8 stop_flag;   
extern UINT8 stop_number;
extern UINT8 first_stitch_flag;      
extern UINT8 end_flag;
extern UINT8 machine_stop_flag;
extern UINT8 slow_flag;
extern UINT8 check_flag;   
extern UINT8 process_flag;               
extern UINT8 calculate_flag; 
extern UINT8 start_flag; 
extern UINT8 connect_flag; 
extern UINT8 motorconfig_flag;    
extern UINT8 stepconfig_flag; 
extern UINT8 InpresserIni_flag;    
extern UINT16 DelayMsCount;
extern UINT8 StitchStartFlag;
     
extern INT16 movestep_angle; 
extern INT16 movestep_time;  
extern UINT8 movestepx_time;
extern UINT8 movestepy_time;
extern INT16 movestepx_angle;      // move x step angle
extern INT16 movestepy_angle;      // move y step angle
extern INT16 movect_angle;   
extern INT16 last_speed;

extern INT16 allyj_step;          
extern INT16 allin_step;          
extern INT16 allct_step;     

extern UINT8 emermove_high;    
extern UINT8 clamp_stepflag;  
extern UINT8 tb1_flag;     

extern UINT8 temp_tension; 
extern UINT8 status_now;
extern UINT8 status_15;
extern UINT8 FindZeroFlag;
extern UINT8 EncoderZ_flag;
extern UINT8 motor_stuck_flag;
//--------------------------------------------------------------------------------------
// sewing parameter
//-------------------------------------------------------------------------------------- 
extern INT16 sew_speed;        
extern UINT8 tension;   
extern UINT8 u15;         
   
extern UINT16 sew_stitch; 
extern UINT8 findorigin_flag;
extern UINT8 u71; 
extern UINT8 u51; 
extern UINT8 u49; 
extern UINT8 u02;                  
extern UINT8 u03;                  
extern UINT8 u04;                  
extern UINT8 u05;                  
extern UINT8 u06;                  
extern UINT8 u07;                  
extern UINT8 u08;                  
extern UINT8 u09;                  
extern UINT8 u10;                  
extern UINT8 u11;                  
extern UINT8 u12;                  
extern UINT8 u13;                  
extern UINT8 u14;  
extern UINT8 u201;
extern UINT8 u202;  
extern UINT8 u203;
extern UINT8 u204;
extern INT8 u16; 
extern UINT8 u26; 
extern UINT8 u33; 
extern UINT8 u34;               
extern UINT8 clamp_com;
extern UINT8 u36; 
extern UINT8 u37;  
extern UINT8 u38; 
extern UINT8 u39;
extern UINT8 u40;
extern UINT8 u205;
extern UINT8 u206;
extern UINT8 u207;
extern UINT8 u208;
extern UINT8 u209;
extern UINT8 u210;
extern UINT16 u211;
extern UINT8 u212;
extern INT16 u213;
extern INT16 u214;
extern INT16 u215;
extern INT16 u216;
extern UINT8 u217;
extern UINT8 u218;
extern UINT8 u219;
extern UINT8 u220;
extern UINT8 u221;
extern UINT8 u222;
extern UINT16 u223;
extern UINT8 u224;
extern UINT8 u225;
extern UINT8 u226;
extern UINT8 u227;
extern UINT8 u228;
extern UINT8 u229;
extern UINT8 u230;
extern UINT8 u231;
extern UINT8 u232;
extern UINT8 u233;
extern UINT8 u234;
extern UINT8 u235;
extern UINT16 u236;
extern UINT8 u237;
extern INT16 u238;
extern UINT8 u239;
extern INT16 u240;
extern INT16 u241;
extern UINT8 u242;
extern INT16 AdjustAngleSet;
extern UINT8 u41; 
extern UINT8 u42;   
extern UINT8 u46;   
extern UINT8 u48;              
extern UINT8 u68;                 
extern UINT8 u69;                 
extern UINT8 u70;
extern UINT8 u94;
extern UINT8 u97;                 
extern UINT8 u101; 
extern UINT8 u103;                
extern UINT8 u104;                
extern UINT8 u105;                
extern UINT8 u112; 
extern UINT8 u89; 
extern UINT8 aging_flag; 
extern UINT8 aging_delay; 
extern UINT8 u72; 
extern UINT8 u73;
extern UINT8 u35;
extern INT16 x_bios;
extern INT16 y_bios;
extern INT16 DisMotorAngle;
extern INT16 AdjustAngle;
//--------------------------------------------------------------------------------------
// self test
//-------------------------------------------------------------------------------------- 
extern UINT8 smotor_speed;

extern UINT8 output_com; 
 
extern UINT8 stepmotor_comm; 
extern UINT8 stepmotor_state;  
//--------------------------------------------------------------------------------------
// 
//-------------------------------------------------------------------------------------- 
extern UINT8 single_reply;   
extern UINT8 single_flag;     

extern UINT8 shift_reply;         
extern UINT8 shift_flag;  
extern UINT8 shift_flag_old;        
extern UINT8 shift_low_flag;
extern UINT8 alarmled_flag;         
extern UINT8 alarmbuz_flag;  
extern UINT16 sound_count;
extern INT16 power_off_count;
extern UINT8 pause_count;   
extern UINT8 pause_flag;      
extern UINT8 stay_flag;      
extern UINT8 thbrk_count;         
extern UINT8 thbrk_flag; 
extern UINT8 brkdt_flag; 
extern UINT8 adtc_count;   
extern UINT8 adtc_flag;                        
extern UINT8 sfsw_count,sfsw_count1;  
extern UINT8 sfsw_flag;      

extern UINT8 aging_com;                         
//--------------------------------------------------------------------------------------
//  1900a 
//--------------------------------------------------------------------------------------
extern UINT8 spi_flag;
extern UINT8 timer_x;
extern UINT8 timer_y;
extern UINT8 timer_yj;
extern UINT8 timer_zx;


//******
extern INT16 tactic_flag_last ;
extern INT16 tactic_flag ;
extern INT16 iq_max_tester;
extern INT16 speed_min_tester;
extern INT16 door_ac;
extern INT16 door_dt;
extern INT16 spider_man;
extern INT16 spider_lim;
extern UINT8 m_status;
extern UINT8 pwm_forbid_flag;
extern UINT8 over_spd_flag;
//*********

extern UINT8 DAActionFlag,CutActionFlag,test_action_flag;
extern INT16 DAActionCounter,CutActionCounter,test_action_counter;
extern INT8 u243;

extern UINT32 counter_1ms;
extern UINT16  pattern_change_counter;

extern UINT8 origin_adjust_flag,origin_flag;

extern UINT8 wind_mode;

extern UINT8 footer_working_mode;
extern UINT8 LRfooter_down_mode;
extern UINT8 LRfooter_up_mode;
extern UINT8 origin_footer_status;
extern UINT8 LTR_trim_option;

extern INT16 nop_move_delay;
extern UINT16 one_step_delay;
extern INT16 edit_move_delay;

extern UINT8 FootNumber;
extern UINT8 LastPatternHaveSOP;

extern UINT8 PatternSpeedLimited;
extern UINT16 SpeedRange[10];

extern UINT8 find_deadpoint_flag;//0 for do not find deadpoint or already found deadpoint,1 for finding
extern UINT8  x_step_current_level;
extern UINT8  y_step_current_level;
extern UINT8  motor_para[10];

extern UINT8 already_in_origin,not_in_origin_flag;
extern UINT16 stepstatus1;                              
extern UINT8 serail_config_flag,rfid_config_flag;                           
extern UINT8 nop_move_k;
extern UINT16 stepstatus2;

extern UINT8 allow;
extern UINT8 atum_flag,atum_data,hevi_flag,hevi_data,fun_flag,fun_code;

extern UINT8 cut_mode;
extern UINT8 manual_cut_flag;
//2011-4-20
extern UINT8 x_sensor_pos;
extern UINT8 stop_foot_status;
extern UINT8 delay_of_inpress_up;
extern UINT8 delay_of_wipper_down;
extern UINT8 delay_of_nop_move;
extern UINT8 delay_of_go_setout;
extern UINT8 dead_point_degree;
extern INT16 m_spd_ref_n;
extern INT16 m_spd_n;
extern INT16 m_spd_n_last;
extern UINT8 stretch_foot_flag;
extern UINT8 stretch_foot_enable;
extern UINT8 MoveMode,HighSpeedStitchLength;
extern UINT16 OverCurrentOneCycle,OutOfPositionTimes;
extern UINT8 StitchSpeedCurve;
extern UINT16 MoveStartAngle;
extern UINT8 sm_overload_counter;
extern UINT8 sm_overload_flag;
extern UINT16 sm_overload_value;
extern UINT16 iit_warning_value;

extern INT16 before_down_speed;
extern UINT8 setout_emermove_flag;
extern UINT16 inpress_origin;
extern INT16 inpress_real_delta_runing;
extern INT16 inpress_high_base;
extern INT16 inpress_high;
extern UINT8 inpress_high_flag; 
extern UINT8 stepmotor_single; 

extern UINT8 inpress_high_limits;
extern UINT8 inpress_high_whole;
extern UINT8 inpress_down_delay;
extern UINT8 go_origin_speed;
extern UINT8 single_move_speed;
extern INT16 movestepxy_time;
extern UINT16 tension_open_counter;
extern UINT8 tension_open_switch;
extern UINT8 inpress_action_flag;
extern UINT8 cut_move_flag;
extern UINT8 int0_finish_flag;
extern UINT8 int3_finish_flag;

extern UINT8 inpress_first_flag;
extern INT16 last_inpress_position;
extern UINT8 fun_default_flag;
extern UINT8 speed_display_mode;
extern UINT8 movex_delay_counter,movex_delay_flag;
extern UINT8 movey_delay_counter,movey_delay_flag;
extern UINT8 inpress_type;
extern UINT8 stretch_out_delay;
extern UINT8 stretch_up_delay;
extern UINT8 stretch_down_delay;
extern UINT8 smotor_direction;
extern UINT16	theta_adjust;
extern UINT8 led_light_adjust;
extern UINT8 motor_lock_flag;
extern UINT8 x_motor_dir;
extern UINT8 y_motor_dir;
extern UINT8 z_motor_dir;
extern UINT8 yj_motor_dir;

extern UINT16 emergency_counter;
extern UINT16 caculate_stitch_step;
extern UINT16 check_data_speed;

extern UINT8 before_nopmove_cut_flag;
extern UINT8 finish_cut_flag;
extern UINT8 fw_flag,L_AIR_flag,da0_flag;
extern UINT8 aging_mode,unaging_flag;
extern INT16 tension_release_value,x_origin_offset,y_origin_offset;
extern UINT16 tension_release_time;
extern UINT16 steper_footer_range,steper_footer_position;

extern INT16 pdl_val_now;	
extern INT16 pdl_val_old;
extern UINT16 pedal_pos0;
extern UINT16 pedal_pos1;
extern UINT16 pedal_pos2;
extern UINT16 pedal_pos3;

extern UINT8 manual_operation_flag;

extern UINT8 PAUSE_ON,PAUSE_OFF;


extern UINT8 inpress_speed;
extern UINT8 cut_pwm_counter,fw_counter;
extern UINT8 special_machine_type;
extern UINT8 autoprecess_footer3_flag,IORGLastState,PSENSLastState,autoprocess_inout_flag;
extern UINT8 PSENScounter,DVSMcounter;

extern UINT8 inpress_follow_flag;
extern UINT8 svpara_buf[];
extern UINT8 svpara_trans_flag;
extern UINT8 svpara_disp_buf[];
extern UINT8 opl_origin_flag;


extern INT16 hold_enable;//1:allowed ,0:forbidden

extern UINT8 test_flag,test_flag2,test_flag3;
extern UINT8 test_da;
extern UINT8 find_communication_start_flag; 
extern UINT8 autoprocess_action_flag,return_from_setout;
extern UINT8 autoprocess_put_down_flag,put_down_counter;
extern UINT8 inpress_offset,ready_go_setout_com;

extern UINT8 jiting_flag;
extern INT16 last_xstep_cou,last_ystep_cou;
extern INT16 sewing_ren_flag;
extern UINT8 cut_pwmHIGH_counter,FWendCounter;
extern UINT8 waitingforomove_flag,x_movedone,y_movedone;

extern UINT8 cut_test_flag,cut_pwm_counter;
extern INT8 inpress_mod_remain;
extern UINT8 back_from_checki08;
extern UINT16 stretch_action_counter;
extern UINT8 stretch_action_flag;
extern UINT8 auto_select_flag,auto_function_flag;
extern UINT16 pattern_number,last_pattern_number; 

extern UINT8 special_footer_type;
extern UINT8 zero_speed_flag;
extern UINT8 pattern_delay,pattern_change_flag,pattern_change_finish;
extern UINT8 new_pattern_done,PORG_action_flag ;
extern UINT8 return_origin_flag,course_next_flag;
extern UINT8 new_pattern_done;
extern UINT8 marking_speed,marking_flag,marking_finish_flag;
extern UINT8 formwork_identify_device;
extern INT16 x_bios_offset,y_bios_offset;

extern INT16 angle_test_x,angle_test_y;
extern UINT8 speed_down_stitchs,start_to_speed_down,speed_down_counter;
extern UINT16 ratio_array[],Corner_deceleration_speed;
extern UINT16 serail_number;
extern UINT8  serail_module_sleep;


extern UINT8 super_pattern_flag;
extern UINT16 pat_buff_write_offset;
extern UINT16 pat_buff_total_counter,bakeup_total_counter,pat_buff_total_counter_temp;
extern UINT8 requset_which_side,pat_writting_flag,send_flag;

extern UINT8 setout_flag;
extern UINT8 origin_check_flag;


extern UINT8  speed_limit;
extern UINT8  count_temp;
extern UINT8  sevro_curve_num;

extern UINT16 pat_buff_total_counter_temp;

extern UINT8  pause_inpress_flag;
extern UINT16 u555;
extern UINT8  cut_nopmove_flag;
extern UINT8  thread_switch;

extern UINT16 dsp1_message,dsp2_message;
extern UINT16 dsp3_message,dsp4_message;

extern UINT8 movezx_delay_counter,movezx_delay_flag;
extern UINT16 err_num_dsp1,err_num_dsp2;
extern UINT16 err_num_dsp3,err_num_dsp4;

extern UINT8  main_control_lock_flag;
extern UINT8	OW_RomID[8];
extern UINT8   identify_pattern_counter;
extern UINT32 pattern_byte_num;

extern UINT8 baseline_alarm_flag;
extern UINT16 baseline_alarm_stitchs;

extern UINT8  down_up_stitch;
extern UINT8  x_step_curve,y_step_curve;

extern UINT8  stay_end_flag;

extern UINT16 total_counter_temp;
extern UINT8  pattern_buf_write_flag;
extern UINT8  delay_start_function;
extern UINT8 delay_start_time;
extern UINT8  after_trim_stop_angle_adjust;
extern UINT16 start_angle_speed_curve8;
extern UINT16 end_angle_speed_curve8;

extern INT8 sewingcontrol_flag,sewingcontrol_stitchs,need_backward_sewing;
extern UINT8 sewingcontrol_tail_flag,need_action_once,need_action_two;

extern PATTERN_DATA *target_pat_point;
extern INT16 target_allx_step,target_ally_step; 
extern UINT16 target_total_counter;

extern INT8 mode0_time,mode1_x_time,mode1_y_time;
extern INT16 mode0_angle,mode1_x_angle,mode1_y_angle;

extern UINT8 inflection_flag;     
extern UINT8 inflection_point_dis;  //
extern UINT8  inflection_pass_flag;

extern UINT8 cover_position_flag,cover_position_flag1,pause_inpress_flag1;

extern UINT16 test_counet_ms;
extern UINT8  test_brk_flag;

extern INT16 test_nop_x,test_nop_y,next_nop_x,next_nop_y;
extern UINT8 waiting_motor_flag,waiting_motor_counter;

extern UINT8 flag_start_waitcom;
extern UINT8 counter_wait_com;
extern UINT8 flag_wait_com;  

extern UINT8 slowdown_stitchs;

extern UINT8 step_movetype;
extern UINT8 num_stepcomerr;
	#if INPRESS_FOLLOW_ACTION
		extern UINT16 inpress_follow_down_angle,inpress_follow_up_angle;
		extern UINT16 inpress_follow_down_end_angle,inpress_follow_up_end_angle;
		extern UINT8 inpress_follow_down_speed,inpress_follow_up_speed,enable_inpress_follow;
		extern INT16 inpress_follow_range;
		extern UINT8 movezx_delay_flag,movezx_delay_counter;
	#endif
#endif
extern UINT8 need_down_flag;

extern UINT16 smotor_speed_counter;
extern UINT8 smotor_speed_flag;

extern UINT8 k03; //0: Mechanical Type; 1: Electrical Type
extern UINT8 temp_tension_last,base_tension,cut_tension,sewing_tenion;

extern UINT8 fw_action_flag;
extern UINT16 fw_action_counter;

extern UINT8  thread_holding_switch,thread_holding_current;
extern UINT16 thread_holding_start_angle,thread_holding_end_angle,thread_holding_cut_angle;
extern UINT8 second_start_switch,second_start_counter;
extern UINT8 download_drv_flag;
extern UINT8 recieve_flag;
extern UINT8 data_length_drv;
extern UINT8 erase_falg;
extern UINT8 DRV_DATA_LEN;
extern UINT8 drv_satus ;
extern FAULT_DIAG de_bug;


extern UINT8 bobbin_change_switch;
extern UINT8 baseline_detect_switch;


extern INT16	allx_step_last,ally_step_last;
extern UINT8 operate_dsp1ordsp2_flag;

extern UINT16 aging_old_counter;
extern UINT8 needle_cool_flag,running_flag;

extern UINT16 CoolingInterval,CoolingDuration,CoolingIntervalCounter,CoolingDurationCounter,Cooling1sCounter;
extern UINT8 AgingLasting,CoolingActionFlag;

extern INT16 read_step_x,read_step_y;
extern INT16 nop_move_remainx,nop_move_remainy;
extern UINT8 nop_move_pause_flag;
extern PATTERN_DATA *last_pattern_point;
extern INT16 last_allx_step,last_ally_step;
extern UINT8 special_go_allmotor_flag;
extern UINT8 finish_nopmove_pause_flag;


extern UINT8  fk_action_flag;
extern UINT16 fk_action_counter;

extern UINT8 bobbin_change_done_flag;
extern UINT8 sewingcontrol_tail_stitches;
extern UINT8 aging_selection;

extern UINT8 making_pen_actoin_flag,making_pen_status;
extern INT16 spd_monitor;

extern UINT8 tail_sewing_flag;
extern UINT16 speed_up_value;

extern UINT16 aging_mode_counter_1,aging_mode_counter_2;
extern UINT8  making_pen_offset_done;

extern UINT8 bobbin_change_in_progress;

extern UINT8  tail_sewing_speed_flag;
extern UINT16 tail_sewing_speed;

extern UINT8 barcoder_time_between_same_code;
extern UINT8 wirte_stepermotor_para_flag;
extern UINT8 write_stepmotor_curve_flag;

extern UINT16 inpress_follow_down_angle,inpress_follow_up_angle;
extern UINT8 inpress_follow_down_speed,inpress_follow_up_speed;
extern INT16 inpress_follow_range;
extern UINT8 inpress_follow_high_flag;

extern UINT16 rec1_datalength,rec1_package_length;

extern UINT8  rec1_status_machine;

extern SYSTEM_PARA para;
extern UINT8 already_auto_find_start_point;
extern UINT8 write_eeprom_para_flag;

extern UINT8 cutter_delay_counter,cutter_delay_flag;

extern UINT8 yj_step_current_level;
extern INT16 stepper_cutter_move_range;
extern INT8  stepper_cutter_move_time;
extern INT16  follow_up_inpresser_time_adj;
extern INT16  follow_up_inpresser_angle_adj;
extern UINT16 fw_start_angle;
extern UINT8  inpress_lower_stitchs,inpress_lower_steps;
extern INT16 inpress_delta,inpress_follow_delta;
extern UINT16 holding_bobbin_start_angle;
extern UINT8 software_key_footer,software_key_run,software_key_pause,software_key_bobbin;
extern UINT8 tra1_buf[]; 
extern UINT16 tra1_ind_r; 
extern UINT16 tra1_ind_w; 
extern UINT16 rec1_total_counter;
extern UINT8  laser_cutter_aciton_flag;
extern UINT16  Rfid_Nom;
extern UINT8  rc522_write_falg;
extern UINT8  rc522_write_ret_falg;
extern UINT8  rc522_control_falg;
extern UINT8  making_pen_nopmove_flag;
extern UINT16  dsp1_step_crc;
extern UINT16  dsp2_step_crc;

#if AUTO_CHANGE_FRAMEWORK 
	extern UINT8  left_footer_action_flag,left_start_action_flag,left_footer_counter,left_start_counter,left_footer_lock_flag,left_start_lock_flag;
	extern UINT8  right_footer_action_flag,right_start_action_flag,right_footer_counter,right_start_counter,right_footer_lock_flag,right_start_lock_flag;
	extern UINT8  left_footer_status,left_second_footer_status,left_quest_running;
	extern UINT8  right_footer_status,right_second_footer_status,right_quest_running;
	extern UINT8  waitting_for_pattern_done;
	extern UINT8 left_footer_delay_flag,left_footer_delay_counter,right_footer_delay_flag,right_footer_delay_counter;
	extern UINT8 power_on_allow_keypress;
	extern UINT8 testpin;

#endif
	
extern UINT8 blow_air_action_flag;
extern UINT16 blow_air_counter;
extern UINT8 waitting_for_point_command;
extern UINT8 auto_function_skip_flag;
extern UINT8 main_control_lock_setup;
extern UINT8 already_up_flag;

extern UINT8 rfid_alarm_flag;
extern UINT16 rfid_alarm_counter,request_rfid_number;

extern UINT16 stepversion1,stepversion2;
extern UINT16 stepversion3,stepversion4;

extern UINT8 cool_air_action_flag;
extern UINT16 cool_air_counter,cool_air_action_counter,cool_air_1_sec;
extern UINT16 cool_air_close_time,cool_air_open_time;

extern UINT8  led_turn_green_flag;
extern UINT16 led_stay_green_counter,led_stay_1s_counter;
extern UINT8  first_power_on_flag;
//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
