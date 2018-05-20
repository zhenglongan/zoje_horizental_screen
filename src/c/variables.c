//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : variables.c
//  Description: external variables define
//  Version    Date     Author    Description
//  ...
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
// 	Includes
//--------------------------------------------------------------------------------------
#include "..\..\include\typedef.h"      //Data type define
#include "..\..\include\common.h"       //External variables declaration

//--------------------------------------------------------------------------------------
//  External variables define
//--------------------------------------------------------------------------------------
SYS_STRU sys; 		         // systerm variable structure
MOTOR_STRU motor;          // motor variable structure
PATTERN_DATA *pat_point;   // pattern point variable structure
PATTERN_DATA *sta_point,*origin2_point;   // pattern point variable structure
PATTERN_DATA *TempEnd_point;
PATTERN_DATA *TempStart_point;
PATTERN_DATA *TempPat_point;


STEPVERSION Step1Version = {0xa,1,0,0,2};
STEPVERSION Step2Version = {0xa,1,0,0,2};
STEPVERSION Step3Version = {0xa,1,0,0,2};
STEPVERSION Step4Version = {0xa,1,0,0,2};

INT16 Ud_i,Uq_i,d_iq_last;//SCX
UINT8 MAIN_MOTOR_TYPE;//0 550W 1:1730  2:2530

UINT8 flag_1ms;            // 1ms flag
UINT16 ms_counter;         // 1ms delay counter
UINT16 ms_scan_counter;
UINT16 foot_ready_counter_dva;
UINT16 foot_ready_counter_dvb;
UINT16 wipe_time;

UINT16 us_counter;         // 100us delay counter

UINT8 pat_buf[24004];		//5000 


UINT8 *recpat_point;       // receive pattern point

UINT8 pedal_state;
UINT8 pedal_last_state;
UINT16 cut_start_angle;
UINT16 cut_end_angle;
UINT16 tension_start_angle;
UINT16 tension_end_angle;
UINT16 wiper_start_time;
UINT16 wiper_end_time;
UINT8 pedal_style;
UINT8 foot_flag;           // foot flag  0=down  1=up
UINT8 foot_half_flag;
UINT8 inpress_flag;        // inpress flag  0=down  1=up
INT16 inpress_position; 
UINT8 clamp_flag;          // clamp out flag   0=in  1=out
UINT8 OutOfRange_flag;
UINT8 DVSMLastState;

UINT8 zpl_pass;            // ZPL pass flag
//UINT8 stitch_counter_tmp;

UINT8 origin_com;          // all motor find origin command
//UINT8 commandpoint_com;    // go to the commamd point in edit status
UINT8 wind_com;            // wind command
UINT8 repeat_com;          // repeat command
UINT8 manual_com;          // manual command
UINT8 foot_com;            // foot command    0=down  1=up  2=no move
UINT8 foot_half_com;
UINT8 inpress_com;         // inpress command 0=down  1=up  2=no move
UINT8 coor_com;            // coordinate command    
UINT8 cooradjust_com;      // coordinate compensation command 

UINT16 stitch_num;         // sewing stitch number
UINT16 stitch_counter;     // sewing stitch counter


INT16 allx_step,ally_step; // all step counter
INT16 xstep_cou,ystep_cou; // x and y move step counter
INT16 sox_step,soy_step;   // start point step counter
INT16 comx_step,comy_step; // command move step counter

INT16 allx_coor_temp,ally_coor_temp;
INT16 allx_step_temp,ally_step_temp;
UINT8 predit_shift ;
UINT16 DelayCounter;
UINT32 DelayTime = 0;
INT16 SewingCounter = 0;
UINT8 SewingTestFlag = 0;
UINT8 MotorPositionSet;

UINT8 PatternDelayFlag;
UINT16 PattenrDelayTime;
UINT8 RotateFlag;

UINT8 SewingSpeedValue;

UINT8 SewingStopFlag;
UINT8 SewingStopValue;
UINT8 MotorSpeedRigister;

UINT8 StitchUpFlag;
UINT8 PointShiftFlag;
UINT8 PointShiftDirection;
UINT16 PointShiftNumber;

UINT16 SewTestStitchCounter;
UINT16 PatternShiftValue;

UINT8 ShapePointFlag;
UINT8 SewDataFlag;
UINT8 ElementPointFlag;
UINT8 ElementIndex;
UINT8 ElementIndexLatch;
UINT8 TestStatusChangeFlag;
UINT8 MoveToCurrentCoorFlag;
INT16 CurrentXCoor;
INT16 CurrentYCoor;
UINT8 EditEntranceFlag;
UINT8 SewingTestEndFlag;
INT16 StepDrive1Version;
INT16 StepDrive2Version;
UINT8 StatusChangeLatch;
UINT8 CurrentPointShiftFlag;
INT16 CurrentPointShiftPosition;
UINT8 EditElementCommand;
UINT8 CancelOverlabPointFlag;
UINT8 FootUpCom;
UINT8 DVBLastState;
UINT8 DVALastState;
UINT8 FootRotateFlag;

INT16 DestinationPointShiftPosition;
UINT8 MotionSet;
UINT8 MotiongSetFlag;
UINT8 StopStatusFlag;


UINT8 move_flag;           // move stepper motor flag
UINT8 movex_flag;		   // move x flag in pattern process
UINT8 movey_flag;		   // move y flag in pattern process
UINT8 movestep_x_flag;	   // move x step motor flag
UINT8 movestep_y_flag;     // move y step motor flag
UINT8 lastmove_flag;       // move stepper motor flag
UINT8 origin2_lastmove_flag;
UINT8 NopMoveSpd_flag;
UINT8 nopmove_flag;        // nop move stepper motor flag
UINT8 cut_flag;            // cut thread flag
UINT8 laststitch_flag;	   // the last stitch flag in line/circle sewing without cut code
UINT8 cut_start_flag;
UINT8 cut_end_flag;
UINT8 tension_start_flag;
UINT8 tension_end_flag;
UINT8 wipe_start_flag;
UINT8 wipe_end_flag;
UINT8 inpress_start_flag;
UINT8 inpress_end_flag;
UINT8 wipe_start_time;
UINT8 wipe_end_time;
UINT8 stop_flag;           // stop flag
UINT8 stop_number;
UINT8 end_flag;            // pattern end flag
UINT8 machine_stop_flag;

UINT8 first_stitch_flag;   // the first stitch in the pattern
UINT8 slow_flag;           // slow flag
UINT8 check_flag;          // check data flag
UINT8 process_flag;        // process data flag
UINT8 calculate_flag;      // calculat angle flag
UINT8 start_flag;          // pattern start flag
UINT8 connect_flag;        // panel connect flag
UINT8 motorconfig_flag;    // main motor config flag
UINT8 stepconfig_flag;     // stepping motor config flag
UINT8 InpresserIni_flag = 0;
UINT16 DelayMsCount;
UINT8 StitchStartFlag;

INT16 movestep_angle;      // move step angle
INT16 movestep_time;       // move step time
UINT8 movestepx_time;      // move step x time
UINT8 movestepy_time;      // move step y time
INT16 movestepx_angle;      // move x step angle
INT16 movestepy_angle;      // move y step angle
INT16 movect_angle;        // move ct angle
INT16 last_speed;          // sewing speed

INT16 allyj_step;          // all yj step counter
INT16 allin_step;          // all in step counter
INT16 allct_step;          // all ct step counter

UINT8 emermove_high;       // inpresser high 
UINT8 clamp_stepflag;      // clamp step flag
UINT8 tb1_flag;            // timer b1 flag

UINT8 temp_tension;        // temp tension  
UINT8 status_now;
UINT8 status_15;
UINT8 FindZeroFlag;
UINT8 EncoderZ_flag;
UINT8 motor_stuck_flag;
//--------------------------------------------------------------------------------------
// sewing parameter
//-------------------------------------------------------------------------------------- 
INT16 sew_speed;           // sewing speed                            
UINT8 tension;             // tension                                 
UINT8 u15;                 // first stitch tension                      
              
UINT16 sew_stitch;         // sewing stitch counter                    
UINT8 findorigin_flag;     // x and y find origin flag when sewing end
UINT8 u71;                 // thread breakage detection select                
UINT8 u51;                 // wiper enable                            
UINT8 u49;                 // wind speed                              
UINT8 u02;                 // 1 speed catch                           
UINT8 u03;                 // 2 speed catch                           
UINT8 u04;                 // 3 speed catch                           
UINT8 u05;                 // 4 speed catch                           
UINT8 u06;                 // 5 speed catch                           
UINT8 u07;                 // first stitch tension clamp thread                         
UINT8 u08;                 // cut thread tension                      
UINT8 u09;                 // cut thread tension time                 
UINT8 u10;                 // 1 speed no catch                        
UINT8 u11;                 // 2 speed no catch                        
UINT8 u12;                 // 3 speed no catch                        
UINT8 u13;                 // 4 speed no catch                        
UINT8 u14;                 // 5 speed no catch  
UINT8 u201;                // go original when enenergy  
UINT8 u202;  			   // sewing when pedal is up
UINT8 u203;				   // go original forbidden when pedal is up
UINT8 u204;				   // go original forbidden when taken-up is down
UINT8 u205;				   // wind switch
UINT8 u206;				   // wiper switch
UINT8 u207;				   // taken-up position when pause buttern is on
UINT8 u208;				   // pedal action when pause buttern is on
UINT8 u209;				   // puase switch style
UINT8 u210;				   // cut switch
UINT16 u211; 			   // cut speed
UINT8 u212;				   // cancel sewing sera potection	
INT16 u213;				   // x left limit
INT16 u214;				   // x right limit	
INT16 u215;				   // y up limit
INT16 u216;                // y down limit
UINT8 u217;				   // set the high speed
UINT8 u218;				   // set the low speed
UINT8 u219;				   // set the mid-high speed
UINT8 u220;				   // set the mid-low speed	
UINT8 u221;				   // single pedal
UINT8 u222;				   // trim on delay 
UINT16 u223;				   // tension start delay
UINT8 u224; 			   // presser style
UINT8 u225;				   // presser wright
UINT8 u226;			       // light presser
UINT8 u227;				   // mid presser
UINT8 u228;				   // heavy presser
UINT8 u229;				   // sewing material style
UINT8 u230;				   // thin material
UINT8 u231;				   // mid material
UINT8 u232;				   // thick material
UINT8 u233;				   // check origin style when aging
UINT8 u234;				   // pedal action times when aging	
UINT8 u235;				   // inpresser current setting 
UINT16 u236;				// stop angle setting   
UINT8 u237;				   // sewing machine type
INT16 u238;				   // rotate device control
UINT8 u239;				   // origin check with rotate device
INT16 u240;				   // origin check without rotate device	
INT16 u241;				   // Y axis minimum distance 
UINT8 u242;				   // two or three pedal 
INT16 AdjustAngleSet;      
INT8 u16;                  // thread tension changeover timing at the time of sewing start           
UINT8 u26;                 // high of presser at 2 step
UINT8 u33;                 // number of stitchs of thread clamp release
UINT8 u34;                 // clamping timing of thread clamp                                                    
UINT8 clamp_com;           // thread clamp command                    
UINT8 u36;                 // feed motion timing 
UINT8 u37;                 // state of the presser after end of sewing                  
UINT8 u38;                 // presser lifting motion at end of sewing  
UINT8 u39;				   // check origin when sewing end(normal)	
UINT8 u40;                 // check origin when sewing end(c pattern)                                  
UINT8 u41;                 // state of the presser when emergency stop
UINT8 u42;                 // up position or upper dead point  
UINT8 u46;                 // thread trimming disable 
UINT8 u48;				   // setting the find origin style        
UINT8 u68;                 // thread tension output time
UINT8 u69;                 // bend position of thread clamp
UINT8 u70;                 // thread clamp and thread clamp position
UINT8 u94;				   // find dead center when find origin or reset 
UINT8 u97;                 // emergency and thread trimming operation
UINT8 u101;                // main motor and X/Y feed synchronized control 
UINT8 u103;                // inpresser with or without control
UINT8 u104;                // inpresser lowering timing
UINT8 u105;                // inpresser and wiper position
UINT8 u112;                // inpresser down limit                
UINT8 u89;                 // jog move funtion mode    
UINT8 aging_flag;          // aging flag 
UINT8 aging_delay;         // aging delay time 
UINT8 u72;                 // number of invalid stitches at start of sewing of thread breakage detection 
UINT8 u73;                 // number of invalid stitches during sewing of thread breakage detection 
UINT8 u35;                 // have clamp thread motor
INT16 x_bios;
INT16 y_bios;
INT16 DisMotorAngle;	   // motor mechanical angle for display
INT16 AdjustAngle;		   // motor set adjust angle
//--------------------------------------------------------------------------------------
// self test
//--------------------------------------------------------------------------------------    
UINT8 smotor_speed;        // smotor speed

UINT8 output_com;          // output command 1---do   0---stop      
UINT8 fw_flag;
UINT8 L_AIR_flag;
UINT8 da0_flag;

UINT8 stepmotor_comm;      // stepping motor command
UINT8 stepmotor_state;     // stepping motor state
UINT8 stepmotor_single; 
//--------------------------------------------------------------------------------------
// 
//-------------------------------------------------------------------------------------- 
UINT8 single_reply;        // single move step reply
UINT8 single_flag;         // single move step flag

UINT8 shift_reply;         // manual shift step reply
UINT8 shift_flag;          // manual shift step flag
UINT8 shift_flag_old; 
UINT8 shift_low_flag;

UINT8 alarmled_flag;       // alarm led flag
UINT8 alarmbuz_flag;       // alarm buzzer flag
UINT16 sound_count;        // sound counter
INT16 power_off_count;     // power off counter
UINT8 pause_count;         // pause counter
UINT8 pause_flag;          // pause flag
UINT8 stay_flag;           // stay flag
UINT8 thbrk_count;         // thread breakage counter
UINT8 thbrk_flag;          // thread breakage flag
UINT8 thbrk_flag;          // thread breakage flag
UINT8 brkdt_flag;          // breakage detection flag
UINT8 adtc_count;          // adtc counter
UINT8 adtc_flag;           // adtc flag
UINT8 sfsw_count,sfsw_count1;          // sfsw counter
UINT8 sfsw_flag;           // sfsw flag

UINT8 aging_com;           // aging command 
//--------------------------------------------------------------------------------------
//  1900a 
//--------------------------------------------------------------------------------------
UINT8 spi_flag;


//*******
INT16 tactic_flag_last = 0;
INT16 tactic_flag = 0;
INT16 iq_max_tester = 0;
INT16 speed_min_tester = 0;
INT16 door_ac = 1;
INT16 door_dt = 1;
INT16 spider_man;
INT16 spider_lim;
UINT8 m_status;
UINT8 pwm_forbid_flag;
UINT8 over_spd_flag;
//********
UINT8 DAActionFlag,CutActionFlag,test_action_flag;
INT16 DAActionCounter,CutActionCounter,test_action_counter;
INT8 u243;				   // step motor shift angle 

UINT32 counter_1ms;
UINT16  pattern_change_counter;

UINT8 origin_adjust_flag,origin_flag;

UINT8 wind_mode;

UINT8 footer_working_mode;
UINT8 LRfooter_down_mode;
UINT8 LRfooter_up_mode;
UINT8 origin_footer_status;
UINT8 LTR_trim_option;

INT16 nop_move_delay;
UINT16 one_step_delay;
INT16 edit_move_delay;

UINT8 FootNumber;

UINT8 LastPatternHaveSOP;

UINT8 PatternSpeedLimited;
UINT16 SpeedRange[10];

//2010-8-17
UINT8 find_deadpoint_flag;//0 for do not find deadpoint or already found deadpoint,1 for finding
UINT8  x_step_current_level;
UINT8  y_step_current_level;
UINT8  motor_para[10];

UINT8 already_in_origin,not_in_origin_flag;
UINT8 serail_config_flag,rfid_config_flag;

UINT8 nop_move_k;
UINT16 stepstatus1;                              
UINT16 stepstatus2;

UINT8 allow;//0 for not allow ,1 for allow
UINT8 atum_flag,atum_data,hevi_flag,hevi_data,fun_flag,fun_code;

UINT16 stepversion1;                              
UINT16 stepversion2;
UINT16 stepversion3;                              
UINT16 stepversion4;
UINT8 manual_cut_flag;
UINT8 x_sensor_pos;
UINT8 stop_foot_status;
UINT8 cut_mode;
UINT8 delay_of_inpress_up;
UINT8 delay_of_wipper_down;
UINT8 delay_of_nop_move;
UINT8 delay_of_go_setout;
UINT8 dead_point_degree;
INT16 m_spd_ref_n;
INT16 m_spd_n;
INT16 m_spd_n_last;
UINT8 stretch_foot_flag;
UINT8 stretch_foot_enable;
UINT8 MoveMode,HighSpeedStitchLength;
UINT16 OverCurrentOneCycle,OutOfPositionTimes;
UINT8 StitchSpeedCurve;
UINT16 MoveStartAngle;
UINT8 sm_overload_counter;
UINT8 sm_overload_flag;
UINT16 sm_overload_value;
UINT16 iit_warning_value;
INT16 before_down_speed;
UINT8 setout_emermove_flag;
UINT16 inpress_origin;
INT16 inpress_real_delta_runing;
INT16 inpress_high_base;
INT16 inpress_high;
UINT8 inpress_high_flag; 
UINT8 inpress_high_limits;
UINT8 inpress_high_whole;
UINT8 inpress_down_delay;
UINT8 go_origin_speed;
UINT8 single_move_speed;
INT16 movestepxy_time;
UINT16 tension_open_counter;
UINT8 tension_open_switch;
UINT8 inpress_action_flag;
UINT8 cut_move_flag;
UINT8 int0_finish_flag;
UINT8 int3_finish_flag;
UINT8 inpress_first_flag;
INT16 last_inpress_position;
UINT8 fun_default_flag;
UINT8 speed_display_mode;
UINT8 movex_delay_counter,movex_delay_flag;
UINT8 movey_delay_counter,movey_delay_flag;
UINT8 movezx_delay_counter,movezx_delay_flag;
UINT8 inpress_type;
UINT8 stretch_out_delay;
UINT8 stretch_up_delay;
UINT8 stretch_down_delay;
UINT8 smotor_direction;
UINT16	theta_adjust;
UINT8 led_light_adjust;
UINT8 motor_lock_flag;
UINT8 opl_origin_flag;
UINT8 x_motor_dir;
UINT8 y_motor_dir;
UINT8 z_motor_dir;
UINT8 yj_motor_dir;

UINT16 emergency_counter;
UINT16 caculate_stitch_step;
UINT16 check_data_speed;
UINT8 svpara_buf[100];
UINT8 svpara_trans_flag;
UINT8 svpara_disp_buf[256];		

INT16 hold_enable;//1:allowed ,0:forbidden
UINT8 before_nopmove_cut_flag;
UINT8 finish_cut_flag;
UINT8 aging_mode,unaging_flag;
INT16 tension_release_value,x_origin_offset,y_origin_offset;
UINT16 tension_release_time;

UINT16 steper_footer_range,steper_footer_position;

INT16 pdl_val_now;	
INT16 pdl_val_old;  
UINT16 pedal_pos0;
UINT16 pedal_pos1;
UINT16 pedal_pos2;
UINT16 pedal_pos3;

UINT8 manual_operation_flag;
UINT8 origin_adjust_action;
UINT8 cut_pwm_counter;
UINT8 PAUSE_ON,PAUSE_OFF;
UINT8 inpress_speed;
UINT8 special_machine_type;
UINT8 autoprecess_footer3_flag,IORGLastState,PSENSLastState,autoprocess_inout_flag;
UINT8 PSENScounter,DVSMcounter;

UINT8 inpress_follow_flag;

UINT8 test_flag,test_flag2,test_flag3;
UINT8 test_da;
UINT8 autoprocess_action_flag,return_from_setout;
UINT8 autoprocess_put_down_flag,put_down_counter;
UINT8 inpress_offset,ready_go_setout_com;

UINT8 jiting_flag;

INT16 last_xstep_cou,last_ystep_cou;
INT16 sewing_ren_flag ;
UINT8 FWendCounter;

UINT8 waitingforomove_flag,x_movedone,y_movedone;

UINT8 find_communication_start_flag;
UINT8 cut_test_flag,cut_pwm_counter;
INT8 inpress_mod_remain;
UINT8 back_from_checki08;
UINT16 stretch_action_counter;
UINT8 stretch_action_flag;
UINT8 auto_select_flag,auto_function_flag;
UINT16 pattern_number,last_pattern_number;  

UINT8 special_footer_type;
UINT8 zero_speed_flag;

UINT8 pattern_delay,pattern_change_flag,pattern_change_finish;

UINT8 new_pattern_done,PORG_action_flag;

UINT8 return_origin_flag,course_next_flag;
UINT8 new_pattern_done;

UINT8 marking_speed,marking_finish_flag;
UINT8 formwork_identify_device;  
INT16 x_pen_offset,y_pen_offset;
INT16 x_laser_offset,y_laser_offset; 
INT16 x_bios_offset,y_bios_offset;
UINT16 serail_number;   /* ÌõÂëÊ¶±ðºÅ */
UINT8  serail_module_sleep;


INT16 angle_test_x,angle_test_y;
UINT8 speed_down_stitchs,start_to_speed_down,speed_down_counter;
UINT16 ratio_array[17];

UINT8 super_pattern_flag;

UINT16 pat_buff_write_offset;
UINT16 pat_buff_total_counter,bakeup_total_counter,pat_buff_total_counter_temp;
UINT8 requset_which_side,pat_writting_flag,send_flag;

UINT8 setout_flag;

UINT8 origin_check_flag;


UINT8  speed_limit;
UINT8  count_temp;
UINT8  sevro_curve_num;
UINT8  pause_inpress_flag;

UINT8  cut_nopmove_flag;
UINT8  thread_switch;

UINT16 dsp1_message,dsp2_message;
UINT16 dsp3_message,dsp4_message;

UINT16 err_num_dsp1,err_num_dsp2;
UINT16 err_num_dsp3,err_num_dsp4;

UINT8  main_control_lock_flag;
UINT8	OW_RomID[8];


UINT8 baseline_alarm_flag;
UINT16 baseline_alarm_stitchs;

UINT8  down_up_stitch;
UINT8  x_step_curve,y_step_curve;

UINT8  stay_end_flag;
UINT16 total_counter_temp;
UINT8  pattern_buf_write_flag;

UINT8  delay_start_function;
UINT8  delay_start_time;
UINT8  after_trim_stop_angle_adjust;
UINT16 start_angle_speed_curve8;
UINT16 end_angle_speed_curve8;

INT8 sewingcontrol_flag,sewingcontrol_stitchs,need_backward_sewing;
UINT8 sewingcontrol_tail_flag,need_action_once,need_action_two;
PATTERN_DATA *target_pat_point;
INT16 target_allx_step,target_ally_step; 
UINT16 target_total_counter;

INT8 mode0_time,mode1_x_time,mode1_y_time;
INT16 mode0_angle,mode1_x_angle,mode1_y_angle;


UINT8 cover_position_flag,cover_position_flag1,pause_inpress_flag1;

UINT16 test_counet_ms;
UINT8  test_brk_flag;

INT16 test_nop_x,test_nop_y,next_nop_x,next_nop_y;

UINT8 flag_start_waitcom;
UINT8 counter_wait_com;
UINT8 flag_wait_com;
UINT8 slowdown_stitchs;
UINT8 need_down_flag;
UINT16 smotor_speed_counter;
UINT8 smotor_speed_flag;
UINT8 k03; //0: Mechanical Type; 1: Electrical Type
UINT8 temp_tension_last,base_tension,cut_tension,sewing_tenion;

UINT8  fw_action_flag;
UINT16 fw_action_counter;

UINT8  thread_holding_switch,thread_holding_current;

UINT16 thread_holding_start_angle,thread_holding_end_angle,thread_holding_cut_angle;
UINT8 second_start_switch,second_start_counter;
UINT8 download_drv_flag;
UINT8 recieve_flag;
UINT8 data_length_drv;
UINT8 erase_falg;
UINT8 DRV_DATA_LEN;

UINT8 drv_satus ;
FAULT_DIAG de_bug;


UINT8 bobbin_change_switch,baseline_detect_switch;



INT16 allx_step_last,ally_step_last;
UINT8 operate_dsp1ordsp2_flag;

UINT16 aging_old_counter;
UINT8 needle_cool_flag,running_flag;

UINT16 CoolingInterval,CoolingDuration,CoolingIntervalCounter,Cooling1sCounter,CoolingDurationCounter;
UINT8 AgingLasting,CoolingActionFlag;


UINT8 nop_move_pause_flag;
INT16 read_step_x,read_step_y;
INT16 nop_move_remainx,nop_move_remainy;
PATTERN_DATA *last_pattern_point;
INT16 last_allx_step,last_ally_step;
UINT8 special_go_allmotor_flag;
UINT8 finish_nopmove_pause_flag; 

UINT8  fk_action_flag;
UINT16 fk_action_counter;

UINT8 bobbin_change_done_flag;
UINT8 sewingcontrol_tail_stitches;
UINT8 aging_selection;

UINT8 making_pen_actoin_flag,making_pen_status;

INT16 spd_monitor;
UINT8 tail_sewing_flag;

UINT16 speed_up_value;

UINT16 aging_mode_counter_1,aging_mode_counter_2;

UINT8  making_pen_offset_done;

UINT8 bobbin_change_in_progress;

UINT8  tail_sewing_speed_flag;
UINT16 tail_sewing_speed;

UINT8 barcoder_time_between_same_code;

UINT8 wirte_stepermotor_para_flag;
UINT8 write_stepmotor_curve_flag;

UINT16 inpress_follow_down_angle,inpress_follow_up_angle;
UINT8 inpress_follow_down_speed,inpress_follow_up_speed;
INT16 inpress_follow_range;
UINT8 inpress_follow_high_flag;

UINT16 rec1_datalength,rec1_package_length;
UINT8  rec1_status_machine;

SYSTEM_PARA para;
UINT8 already_auto_find_start_point;

UINT8 write_eeprom_para_flag;

UINT8 cutter_delay_counter,cutter_delay_flag;

UINT8  yj_step_current_level;
INT16  stepper_cutter_move_range;
INT8   stepper_cutter_move_time;
INT16   follow_inpresser_time_adj;
INT16  follow_inpresser_angle_adj;
UINT16 fw_start_angle;
UINT8  inpress_lower_stitchs,inpress_lower_steps;

INT16 inpress_delta,inpress_follow_delta;
UINT16 holding_bobbin_start_angle;
UINT8 software_key_footer,software_key_run,software_key_pause,software_key_bobbin;
UINT8  tra1_buf[255];      
UINT16 tra1_ind_r; 
UINT16 tra1_ind_w; 
UINT16 rec1_total_counter;
UINT8  laser_cutter_aciton_flag;
UINT16  Rfid_Nom;
UINT8  rc522_write_falg;
UINT8  rc522_write_ret_falg;
UINT8  rc522_control_falg;
UINT8  rc522_ok_flag;
UINT8  making_pen_nopmove_flag;
UINT16  dsp1_step_crc;
UINT16  dsp2_step_crc;

#if AUTO_CHANGE_FRAMEWORK 
	UINT8  left_footer_action_flag,left_start_action_flag,left_footer_counter,left_start_counter,left_footer_lock_flag,left_start_lock_flag;
	UINT8  right_footer_action_flag,right_start_action_flag,right_footer_counter,right_start_counter,right_footer_lock_flag,right_start_lock_flag;
	UINT8  left_footer_status,left_second_footer_status,left_quest_running;
	UINT8  right_footer_status,right_second_footer_status,right_quest_running;
	UINT8  waitting_for_pattern_done;
	
	UINT8 left_footer_delay_flag,left_footer_delay_counter,right_footer_delay_flag,right_footer_delay_counter;
	UINT8 power_on_allow_keypress;
	
	UINT8 testpin;

#endif 

UINT8  blow_air_action_flag;
UINT16 blow_air_counter;
	
UINT8 waitting_for_point_command;
UINT8 main_control_lock_setup;

UINT8 already_up_flag;

UINT8 rfid_alarm_flag;
UINT16 rfid_alarm_counter,request_rfid_number;

UINT8 cool_air_action_flag;
UINT16 cool_air_counter,cool_air_action_counter,cool_air_1_sec;
UINT16 cool_air_close_time,cool_air_open_time;

UINT8  led_turn_green_flag;
UINT16 led_stay_green_counter,led_stay_1s_counter;
UINT8  first_power_on_flag;
UINT8 rotated_function_flag;
INT16 rotated_position;
INT16 rotated_abs_angle;

UINT16 monitor_predit_shift_flag_conter;
UINT8  monitor_predit_shift_flag_value;
UINT16 inpress_follow_speed;

UINT8 auto_lock_flag;
UINT16 now_pattern_number; 
//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
