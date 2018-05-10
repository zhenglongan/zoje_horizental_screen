/*
--------------------------------------------------------------------------------------
         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
                     ALL RIGHTS RESERVED 
  Project Number: sewing_machine_controller 
  File Name : initial.c
  Description: Core program to control UART communication and protocol
  Version    Date     Author    Description
--------------------------------------------------------------------------------------
*/
//--------------------------------------------------------------------------------------
// 	Includes
//--------------------------------------------------------------------------------------
#include "..\..\include\sfr62p.h"           // M16C/62P special function register definitions
#include "..\..\include\typedef.h"          // data type define
#include "..\..\include\common.h"           // External variables declaration
#include "..\..\include\variables.h"        // External variables declaration
#include "..\..\include\initial.h"          // sine table 
#include "..\..\include\delay.h"          // delay time definition    
#include "..\..\include\stepmotor.h"        // stepmotor driver
#include "..\..\include\iic_bus_eeprom.h"

void init_para_variables(void);
//--------------------------------------------------------------------------------------
//  Name:		init_ta0 
//  Parameters:	None
//  Returns:	None
//  Description: initial timer A0
//--------------------------------------------------------------------------------------
void init_ta0(void)
{
  	ta0mr = 0x00;         // 0000 0000 
                          // |||| |||+- must always be 0 in timer mode
                          // |||| ||+-- must always be 0 in timer mode
                          // |||| |+--- 0: pulse is not output at pin TA0out
                          // |||| |     1: pulse is output at pin TA0out
                          // |||| |        TA0out is automatically  output
                          // |||| +---- 0: gate function: timer counts only 
                          // ||||          when TA0in is held "L"
                          // ||||       1: gate function: timer counts only
                          // ||||          when TA0in is held "H"
                          // |||+------ 0: gate function not available
                          // |||        1: gate function available
                          // ||+------- must always be 0 in timer mode
                          // |+-------- count source select bits:
                          // +--------- count source select bits:
                          //            00:  f1
                          //            01:  f8
                          //            10:  f32
                          //            11:  fc32
  
  	ta0 = (FX*1/1000-1);	// Set up Timer A0 Reload Register for 1 ms interval interrupt
  	ta0s = 1;  			    // start timer A0 
	ta0ic = 0;				// set timer A0 interrupt priority
}
//--------------------------------------------------------------------------------------
//  Name:		init_tb3 
//  Parameters:	None
//  Returns:	None
//  Description: initial timer B3
//--------------------------------------------------------------------------------------
void init_tb3(void)
{
  	tb3mr = 0x00;         // 0000 0000 
                          // |||| |||+- must always be 0 in timer mode
                          // |||| ||+-- must always be 0 in timer mode
                          // |||| |+--- 0: pulse is not output at pin TA0out
                          // |||| |     1: pulse is output at pin TA0out
                          // |||| |        TA0out is automatically  output
                          // |||| +---- 0: gate function: timer counts only 
                          // ||||          when TA0in is held "L"
                          // ||||       1: gate function: timer counts only
                          // ||||          when TA0in is held "H"
                          // |||+------ 0: gate function not available
                          // |||        1: gate function available
                          // ||+------- must always be 0 in timer mode
                          // |+-------- count source select bits:
                          // +--------- count source select bits:
                          //            00:  f1
                          //            01:  f8
                          //            10:  f32
                          //            11:  fc32
  
  	tb3 = (FX*1/1000-1);	// Set up Timer B3 Reload Register for 1 ms interval interrupt
	tb3ic = TB3_IPL;		// set timer B3 interrupt priority
	tb3s = 0;     			// stop timer B3 
}
//--------------------------------------------------------------------------------------
//  Name:		init_tb4 
//  Parameters:	None
//  Returns:	None
//  Description: initial timer B4
//--------------------------------------------------------------------------------------
void init_tb4(void)
{
  	tb4mr = 0x00;         // 0000 0000 
                          // |||| |||+- must always be 0 in timer mode
                          // |||| ||+-- must always be 0 in timer mode
                          // |||| |+--- 0: pulse is not output at pin TA0out
                          // |||| |     1: pulse is output at pin TA0out
                          // |||| |        TA0out is automatically  output
                          // |||| +---- 0: gate function: timer counts only 
                          // ||||          when TA0in is held "L"
                          // ||||       1: gate function: timer counts only
                          // ||||          when TA0in is held "H"
                          // |||+------ 0: gate function not available
                          // |||        1: gate function available
                          // ||+------- must always be 0 in timer mode
                          // |+-------- count source select bits:
                          // +--------- count source select bits:
                          //            00:  f1
                          //            01:  f8
                          //            10:  f32
                          //            11:  fc32
  
  	tb4 = (FX*1/10000-1);	// Set up Timer B4 Reload Register for 100 us interval interrupt
	tb4ic = TB4_IPL;		  // set timer B4 interrupt priority
	tb4s = 0;  			      // stop timer B4 
}
//--------------------------------------------------------------------------------------
//  Name:		init_ad 
//  Parameters:	None
//  Returns:	None
//  Description: initial ADC
//--------------------------------------------------------------------------------------
void init_ad(void)
{
	INT16 i;
  	// A-D conversion initial setting 
  	adcon0 = 0x98;                      // sweep repetition mode 0 
  	adcon1 = 0x29;                      // AN0 to AN3,10 bit mode ,sweep 0 
  	adcon2 = 0x04;                      // P0 group is selected
  	for(i = 0 ; i < 24 ; i++);          // wait for more than 1us 
  		adcon0 |= 0x40;                     // A-D conversion start 
}
//--------------------------------------------------------------------------------------
//  Name:		init_da 
//  Parameters:	None
//  Returns:	None
//  Description: initial DAC
//--------------------------------------------------------------------------------------
void init_da(void)
{
	prcr = 0x04;                        // Protect mode reg    
	da1e = 1;
	da1 = 0;
	da0e = 1;
	da0 = 0;
  	prcr = 0x00;                        // Protect mode reg       
}
//--------------------------------------------------------------------------------------
//  Name:		init_clk 
//  Parameters:	None
//  Returns:	None
//  Description: initial cpu main clock
//--------------------------------------------------------------------------------------
void init_clk(void)
{
  	INT32 i;
  //24MHz (12M*2 PLL)
	prcr=0x07;                            // Protect mode reg    
	pm0=0x00;                             // Processor mode reg0 
	pm1=0x08;                             // Processor mode reg1 
	pm2=0x00;                             // Processor mode reg2 
  	cm0=0x00;                             // System clock control register 0 
  	cm1=0x20;                             // System clock control register 1 
  	cm2=0x00;                             // System clock control register 2 
  	plc0=0x11;                            // 12MHz X 2
  	plc0|=0x80;
	for(i=0;i<480000;i++)				  // delay 20ms
	{
	}
  	cm1 |= 0x02;
  	pclkr=0x03;                           // f1,Peripheral Clock Select Register 
  	prcr=0x00;                            // Protect mode reg    
}
//--------------------------------------------------------------------------------------
//  Name:		init_io 
//  Parameters:	None
//  Returns:	None
//  Description: initial IO direction    pdX=0---input  pdX=1---output
//--------------------------------------------------------------------------------------
void init_io(void)
{
		
	if( para.platform_type == FIFTH_GENERATION)
	{
	#if	MULTIPULE_IO_ENABLE
  	p0  = 0x10; 
  	pd0 = 0x10;     // set p0.0-p0.7 to input p0.4 output
	#else
	p0  = 0x00; 
  	pd0 = 0x10;     // set p0.0-p0.7 to input  p0.4 to output
    #endif
  	p1  = 0x00;     // SNT_ON=1  SNT_H=0
  	pd1 = 0x43;     // set p1.0  p1.1 p1.6 to output   set p1.2-p1.7 to intput 
    POWER_OFF = 0;
	
  	p2  = 0x00; 
  	pd2 = 0x00;     // set p2.0-p2.7 to input
    #if MULTIPULE_IO_ENABLE
  	p3  = 0x40;     // FW=0  FL_ON=0  LM_AIR=0  R_AIR=0  L_AIR=0
  	pd3 = 0xF8;     // set p3.0 p3.1 p3.2 to inupt   set p3.3-p3.7 to output
	#else
	p3  = 0x00;     // FW=0  FL_ON=0  LM_AIR=0  R_AIR=0  L_AIR=0
  	pd3 = 0xF8;     // set p3.0 p3.1 p3.2 to inupt   set p3.3-p3.7 to output
    #endif
  	p4  = 0x01;//0x21;     // OUTPUT_ON=1  FL=0  FA=0  T_CLK=0  T_DIR=0  T_HALF=0//1  BACKUP1=0  BACKUP2=0
  	pd4 = 0xFF;     // set p4.0-p4.7 to output 
    
  	p5_1  = 0;      // ALARM_LED=0
  	pd5_1 = 1;      // set p5.1 to output 
  	p5_2  = 1;      // PWR_LED=1
  	pd5_2 = 1;      // set p5.2 to output 
  	p5_3  = 0;      // SUM=0
  	pd5_3 = 1;      // set p5.3 to output   
	p5_4  =0 ;      //FK_OFF=0 
    pd5_4 =1 ;      // set p5.4 to output
    
    p6_0 = 0;		// RDSET = 0            //09.5.14 wr add for 485
  	pd6_0 = 1;      // set p6.0 to output   //09.5.14 wr add for 485
  	
  	p6_1  = 1;      // RST_PN=1
  	pd6_1 = 1;      // set p6.1 to output

  	p7  = 0x3c;     // V=1  \V\=1  W=1  \W\=1
  	pd7 = 0x3e;		  // set p7.7 p7.6 p7.0 to input   set p7.1-p7.5 to output
    
  	p8 = 0x43;      // U=1  \U\=1
  	pd8 = 0x43;     // set p8.0,p8.1,p8.6 to output   set p8.2-p8.7 to input
    
  	p9 = 0x40;      // DA0=0  DA1=0  SPI_CLK=0  SPI_OUT=0 
  	prcr = 0x04;    // protect disable
  	pd9 = 0x60;     // set p9.0-p9.4 and p9.7 to input   set p9.5 and p9.6 to output
  	prcr = 0x00;    // protect enable
	
	p10 = 0x03;     // SPI_CS1=1  SPI_CS2=1
	pd10 = 0x43;    // set p10.0 and p10.1 to output   set p10.2-p10.7 to input
	}
	else
	{
  	p0  = 0x00; 
  	pd0 = 0x08;     // set p0.0-p0.7 to input except p0.3 as output
    
  	p1  = 0x01;     // SNT_ON=1  SNT_H=0
  	//pd1 = 0x03;     // set p1.0 and p1.1 to output   set p1.2-p1.7 to output
	pd1 = 0x43; 
    
  	p2  = 0x00; 
  	pd2 = 0x00;     // set p2.0-p2.7 to input
    
  	p3  = 0x00;     // FW=0  FL_ON=0  LM_AIR=0  R_AIR=0  L_AIR=0
  	pd3 = 0xF8;     // set p3.0 p3.1 p3.2 to inupt   set p3.3-p3.7 to output
    
  	p4  = 0x01;//0x21;     // OUTPUT_ON=1  FL=0  FA=0  T_CLK=0  T_DIR=0  T_HALF=0//1  BACKUP1=0  BACKUP2=0
  	pd4 = 0xFF;     // set p4.0-p4.7 to output 
    
  	p5_1  = 0;      // ALARM_LED=0
  	pd5_1 = 1;      // set p5.1 to output 
  	p5_2  = 1;      // PWR_LED=1
  	pd5_2 = 1;      // set p5.2 to output 
  	p5_3  = 0;      // SUM=0
  	pd5_3 = 1;      // set p5.3 to output   
	p5_4  =0 ;      //FK_OFF=0 
    pd5_4 =1 ;      // set p5.4 to output
    
    p6_0 = 0;		// RDSET = 0            //09.5.14 wr add for 485
  	pd6_0 = 1;      // set p6.0 to output   //09.5.14 wr add for 485
  	
  	p6_1  = 1;      // RST_PN=1
  	pd6_1 = 1;      // set p6.1 to output

  	p7  = 0x3c;     // V=1  \V\=1  W=1  \W\=1
  	pd7 = 0x3e;		  // set p7.7 p7.6 p7.0 to input   set p7.1-p7.5 to output
    
  	p8 = 0x03;      // U=1  \U\=1
  	pd8 = 0x03;     // set p8.0 and p8.1 to output   set p8.2-p8.7 to input
    
  	p9 = 0x40;      // DA0=0  DA1=0  SPI_CLK=0  SPI_OUT=0 
  	prcr = 0x04;    // protect disable
  	pd9 = 0x60;     // set p9.0-p9.4 and p9.7 to input   set p9.5 and p9.6 to output
  	prcr = 0x00;    // protect enable
	
	p10 = 0x03;     // SPI_CS1=1  SPI_CS2=1
	pd10 = 0x03;    // set p10.0 and p10.1 to output   set p10.2-p10.7 to input
	}
}
//--------------------------------------------------------------------------------------
//  Name:		init_var 
//  Parameters:	None
//  Returns:	None
//  Description: initial external variables
//--------------------------------------------------------------------------------------
void init_var(void)
{
	UINT8 i;
	//--------------------------------------------------------------------------------------
  	// system variable
  	//--------------------------------------------------------------------------------------
  	sys.status = READY;        // 
  	sys.error = 0;            // clear error
  	sys.uzk_val = 0;          // clear voltage
  	sys.u24_val = 0;          // clear voltage
  	//--------------------------------------------------------------------------------------
  	// motor variable
  	//--------------------------------------------------------------------------------------  
	motor.iq = 0;
	motor.iq_last = 0;		//scx
	Run_SpdIq.Sum_Iq = 0;	//scx
	motor.max_spd = 2500;     // max speed limit
	motor.min_spd = 30;       // min speed limit
	motor.acc = 30;           // 30 rpm/ms
	motor.dec = 30;           // 30 rpm/ms
	motor.acc_curve = 0;
  	motor.spd = 0;            // speed 
  	motor.angle = 0;
	motor.dir = 0;            // clockwise
  	motor.spd_obj = 0;
  	motor.spd_ref = 0;
	motor.stop_angle = 0;
	motor.stop_flag	= 1;      // motor sotp	
	wipe_time = 0;	
	StitchStartFlag = 0;	
	//--------------------------------------------------------------------------------------
  	// sewing parameter
  	//-------------------------------------------------------------------------------------- 
  	sew_speed = 800;          // sewing speed
  	tension = 50;             // tension
  	inpress_high = 20;        // inpresser high
  	sew_stitch = 0;           // sewing stitch counter
  	u15 = 0;  		            // first stitch tension
  	findorigin_flag = 1;  	  // x and y find origin flag when sewing end
  	u71 = 0;                  // thread breakage detection select
  	u51 = 1;                  // wiper enable
  	u49 = 8;                 // wind speed
  	u02 = 15;                 // 1 speed catch
  	u03 = 27;                 // 2 speed catch
  	u04 = 27;                 // 3 speed catch
  	u05 = 27;                 // 4 speed catch
  	u06 = 27;                 // 5 speed catch
  	u07 = 200;                // first stitch tension clamp thread  
  	u08 = 0;                  // cut thread tension 
  	u09 = 0;                  // cut thread tension time
  	u10 = 2;                  // 1 speed no catch
  	u11 = 6;                  // 2 speed no catch
  	u12 = 10;                 // 3 speed no catch
  	u13 = 15;                 // 4 speed no catch
  	u14 = 20;                 // 5 speed no catch
  	u16 = -5;                 // thread tension changeover timing at the time of sewing start
  	u26 = 70;                 // high of presser at 2 step
  	u33 = 2;                  // number of stitchs of thread clamp release
  	u34 = 0;                  // clamping timing of thread clamp   
  	clamp_com = 0;            // thread clamp command
  	u36 = 3;                  // feed motion timing 
  	u37 = 0;                  // state of the presser after end of sewing 
  	u38 = 0;                  // presser lifting motion at end of sewing  
  	u41 = 0;                  // state of the presser when emergency stop 
  	u42 = 0;                  // up position or upper dead point
  	u46 = 0;                  // thread trimming disable
	u48 = 1;                  // setting the find origin style 
  	u68 = 0;                  // thread tension output time
  	u69 = 1;                  // bend position of thread clamp
  	u70 = 0;                  // thread clamp and thread clamp position
	u94 = 1;
  	u97 = 1;                  // emergency and thread trimming operation
  	u101 = 0;                 // main motor and X/Y feed synchronized control
  	u103 = 1;                 // inpresser with or without control
  	u104 = 0;                 // inpresser lowering timing
  	u105 = 0;                 // inpresser and wiper position
  	u112 = 35;                // inpresser down limit        
  	u89 = 2;                  // jog move funtion mode    
  	aging_flag = 0;           // aging flag
  	aging_delay = 20;         // aging delay time   
  	u72 = 8;                  // number of invalid stitches at start of sewing of thread breakage detection 
  	u73 = 3;                  // number of invalid stitches during sewing of thread breakage detection 
  	u35 = 0;                  // have clamp thread motor
	u211 = 40;				  // cut speed 
	u235 = 8;				  // inpresser current setting
	u236 = 170;				  // motor stop angle setting 
	u224 = 0;				  // pedal style choose,
	AdjustAngle = 0;
	OutOfRange_flag = 0;
	x_bios = 0;
	y_bios = 0;
	single_flag = 0;
	laststitch_flag = 0;
	cut_flag = 0;
	stop_flag = 0;
	move_flag = 0;
	end_flag = 0;
	comx_step = 0;
	comy_step = 0;
	//--------------------------------------------------------------------------------------
  	// global variable
  	//--------------------------------------------------------------------------------------
	pedal_style = 2;
	pedal_last_state = 1; 
  	sound_count = 0;               // clear sound counter
	power_off_count = 0;           // clear power off counter
	pause_count = 0;               // clear pause counter
  	alarmled_flag = 0;             // alarm led is off
  	alarmbuz_flag = 0;             // alarm buzzer is off
  	pause_flag = 0;                // pause flag clear
  	stay_flag = 0;                 // emergency break flag clear
  	connect_flag = 0;              // panel connect flag
  	motorconfig_flag = 0;          // main motor config flag
  	stepconfig_flag = 0;           // stepping motor config flag
	InpresserIni_flag = 0;
  	manual_com = 0;                // manual command
  	inpress_position = IN_ORIGIN;  // inpresser position clear
  	foot_com = 2;                  // foot command
  	coor_com = 0;                  // coordinate command
	cooradjust_com = 0;			   // coordinate adjust command	
  	thbrk_count = 0;               // clear thread breakage counter
  	thbrk_flag = 0;                // clear thread breakage flag
  	brkdt_flag = 0;                // clear thread breakage detection flag
  	adtc_count = 0;                // clear adtc counter
  	adtc_flag = 0;                 // clear adtc detection flag
  	sfsw_count = 0;                // clear sfsw counter
  	sfsw_flag = 0;                 // clear sfsw detection flag
    sfsw_count1 = 0;
  	aging_com = 0;                 // aging command   
  	first_stitch_flag = 0;  
  	foot_flag = 1;                 // foot down for jack
	foot_half_flag = 0;
	foot_half_com = 1;
	inpress_flag = 0;              // inpress_down
	inpress_com = 0;               // foot command
	FindZeroFlag = 0;			   // find motor encoder z phase
	EncoderZ_flag = 1;
	motor_stuck_flag = 0;
	tension_start_angle = 880;
	SewingTestFlag = 0;
	MotorPositionSet = 0;
	machine_stop_flag = 0;
	StitchUpFlag = 71;
	PointShiftFlag = 0;
	SewTestStitchCounter = 0;
	ShapePointFlag = 0;
	SewDataFlag = 0;
	ElementPointFlag = 0;
	ElementIndex = 0;
	ElementIndexLatch = 0;
	TestStatusChangeFlag = 0;
	MoveToCurrentCoorFlag = 0;
	EditEntranceFlag = 0;
	SewingTestEndFlag = 0;
	StatusChangeLatch = READY;
	CurrentPointShiftFlag = 0;
	CurrentPointShiftPosition = 0;
	EditElementCommand = 0;
	CancelOverlabPointFlag = 0;
	FootUpCom = 0;
	FootRotateFlag = 0;
	u238 = 1;
	u239 = 17;
	u242 = 0;
	MotiongSetFlag = 0;
	StopStatusFlag = 0; 
	u243 =0;
	CutActionFlag =0;
	DAActionFlag = 0;
	da1 = 0;
	counter_1ms = 0;
	pattern_change_counter = 0;
	origin_adjust_flag = 0;
	origin_flag = 0;	
	pat_point = (PATTERN_DATA *)(pat_buf);
	sta_point = pat_point;
	TempStart_point = pat_point;
	allx_step =0;
	ally_step =0;
	FootNumber = 0;	
	SewingStopFlag = 0;
	PatternDelayFlag = 0;	
	LastPatternHaveSOP = 0;
	PatternSpeedLimited = 0;
	motor_para[0] = 40;
	already_in_origin = 0;	
	serail_config_flag = 0;
	rfid_config_flag = 0;
	nop_move_k = 1;
	manual_cut_flag = 0;
	x_sensor_pos = 0;
	stop_foot_status = 0;
	cut_mode = 0; 
	delay_of_inpress_up = 0;
	delay_of_wipper_down = 0;
	delay_of_nop_move = 4;
	delay_of_go_setout = 4;
	dead_point_degree = 10;
	stretch_foot_flag = 0;
	stretch_foot_enable = 0;
	MoveMode = 0;
	HighSpeedStitchLength = 0;
	OverCurrentOneCycle = 0;
	OutOfPositionTimes = 0;
	StitchSpeedCurve = 5;
	MoveStartAngle = 135;
	u201 = 0;

    inpress_high_limits = 200;
    inpress_high_whole = 0;
    inpress_down_delay = 0;
    go_origin_speed = 5;
    single_move_speed = 30;
	tension_open_counter = 0;
	tension_open_switch = 0;
	inpress_action_flag =0;
	status_now = READY;      
	one_step_delay = 0; 
	cut_move_flag = 0; 
	stepmotor_comm = 0xff ;
	stepmotor_single = 0xff;
	inpress_first_flag = 1;    
	last_inpress_position = inpress_high_base;
	fun_default_flag = 0;
	speed_display_mode = 0;	    
	movex_delay_counter = 0;
	movex_delay_flag = 0;	
	movey_delay_counter = 0;
	movey_delay_flag = 0;	
	movezx_delay_counter = 0;
	movezx_delay_flag = 0;
	inpress_type = 1 ;
	stretch_out_delay = 0;
	stretch_up_delay = 0;
	stretch_down_delay = 0;
	smotor_direction = 0;
	led_light_adjust =0;
	motor_lock_flag =0;	
	x_motor_dir = 1;
	y_motor_dir = 0;
	z_motor_dir = 0;	
	emergency_counter = 0;
	caculate_stitch_step = 0;
	check_data_speed = 0;
	inpress_first_flag = 1;   
	last_inpress_position = inpress_high_base;
	speed_display_mode = 0;
    opl_origin_flag = 0;
	svpara_trans_flag = 0;

	before_nopmove_cut_flag = 0;
	finish_cut_flag =0;
	aging_mode = 0;
	unaging_flag =0;
	tension_release_time = 0;
	tension_release_value = 25;
	x_origin_offset = 0;
	y_origin_offset = 0;
	steper_footer_range = 280;
	steper_footer_position = 0;

	pdl_val_old = 0;
	pdl_val_now = 0;
	pedal_pos1 = 70*8;
	pedal_pos2 = 120*5;
	pedal_pos3 = 185*5 - 25;
	if(pedal_pos2 <= pedal_pos1)  
	   pedal_pos2 = pedal_pos1 + 40;
	if(pedal_pos3 <= pedal_pos2)  
	   pedal_pos3 = pedal_pos2 + 300;
	manual_operation_flag = 0;
	PAUSE_OFF = 0;
	PAUSE_ON = 1;	
	inpress_speed = 13;	

	special_machine_type = 0;
	autoprecess_footer3_flag = 0;
	IORGLastState = 1;
	PSENSLastState = 1;
	PSENScounter = 0;
	DVSMcounter = 0;
	autoprocess_inout_flag = 1;
	inpress_follow_flag = 0;	
	test_flag = 0;
	test_flag2 = 0;
	test_flag3 = 0;
	autoprocess_action_flag = 0;
	return_from_setout = 1;
	autoprocess_put_down_flag =0;
	put_down_counter = 0;
	inpress_offset = 0;
	ready_go_setout_com = 0;	

	movestepx_angle = 0;
	movestepy_angle = 0;
	sewing_ren_flag = 0;	
	waitingforomove_flag = 0;
	x_movedone = 0;
	y_movedone = 0;	
	cut_test_flag = 0;
   
	inpress_mod_remain = 0;
	back_from_checki08 = 0;
	special_footer_type = 0;
	auto_select_flag = 0;
	auto_function_flag = 0;	
	pattern_number=0;
	zero_speed_flag = 0;	
	last_pattern_number = 0;
	pattern_delay = 0;
	pattern_change_flag = 0;	
	new_pattern_done = 0;
	pattern_change_finish = 0 ;
	PORG_action_flag = 0 ;
	return_origin_flag = 0;
	course_next_flag = 0;

	ready_go_setout_com = 0;
	marking_speed = 1;
	x_bios_offset = 0;
	y_bios_offset = 0;
	x_pen_offset = 0;
	y_pen_offset = 0;
	x_laser_offset = 0;
	y_laser_offset = 0;
	marking_finish_flag = 1;
	speed_down_stitchs = 0;
	start_to_speed_down= 0;
	speed_down_counter = 0;
	serail_number = 0;
	formwork_identify_device = 0; 
	serail_module_sleep = 0;
	angle_test_x = 0;
	angle_test_y = 0;
	super_pattern_flag = 0;
	pat_buff_write_offset = 0;
	pat_buff_total_counter = 0;

	pat_writting_flag = 0;
	send_flag = 0;
	setout_flag = 0;
	origin_check_flag = 0;
	inpress_high_flag = 0; 
	speed_limit = 20;
	sevro_curve_num = 0;
	pause_inpress_flag = 0; 	
	cut_nopmove_flag = 0;	
	thread_switch = 0;	
	dsp1_message = 0;
	dsp2_message = 0;	
	dsp3_message = 0;
	dsp4_message = 0;
	err_num_dsp1 = 0;
	err_num_dsp2 = 0;
	err_num_dsp3 = 0;
	err_num_dsp4 = 0;
	baseline_alarm_flag = 0;
    baseline_alarm_stitchs =0;	
	down_up_stitch = 0;	
	x_step_curve = 0;
	y_step_curve = 0;	
	stay_end_flag = 0;	
	total_counter_temp = 0;
	pattern_buf_write_flag = 0;	
	sewingcontrol_flag = 0;
	sewingcontrol_stitchs =0;
	need_backward_sewing = 0;
	sewingcontrol_tail_flag = 0;
	need_action_once = 0;
	need_action_two  = 0;	
	target_pat_point = 0;
    target_allx_step = 0;
	target_ally_step = 0;	
	mode0_time = 0;
	mode1_x_time = 0;
	mode1_y_time = 0;
    mode0_angle = 0;
	mode1_x_angle = 0;
	mode1_y_angle = 0;	

	cover_position_flag = 0;
	cover_position_flag1 = 0;
	pause_inpress_flag1 = 0;
	test_counet_ms = 0;
	test_brk_flag = 0;	
	test_nop_x = -16000;
	test_nop_y = -16000;	
	next_nop_x = 16000;
	next_nop_y = 16000;	
	
	flag_start_waitcom = 0;
	counter_wait_com = 0;
	flag_wait_com = 0;  	
	slowdown_stitchs = 5;

	after_trim_stop_angle_adjust =0;
	not_in_origin_flag = 0;	
	need_down_flag = 1;
    smotor_speed_counter = 0;
    smotor_speed_flag = 0;
	k03 = MECHANICAL; 					//0: Mechanical Type; 1: Electrical Type
	temp_tension_last = 0;
	temp_tension = 0;
	base_tension = 0;
	cut_tension = 0;
	sewing_tenion = 0;
	
	DelayCounter = 0;
	fw_action_flag = 0;
    fw_action_counter = 0;
	thread_holding_switch = 0;
	thread_holding_current = 0;
	
	thread_holding_start_angle = 0;
	thread_holding_end_angle = 0;
	thread_holding_cut_angle = 0;
	//if( u227 == 55)
	//    second_start_switch = 1;
	//else
	    second_start_switch = 0;
	second_start_counter = 1;
	
	erase_falg = 0;
	DRV_DATA_LEN=0;
	stepversion1 = 0;
	stepversion2 = 0;
	stepversion3 = 0;
	stepversion4 = 0;
	find_communication_start_flag = 0; 
	jiting_flag = 0;
	MAIN_MOTOR_TYPE = 1;
	motor_para[0] = 8;
	motor_para[1] = 10;
	MotorPreInit();
	
	#if BOBBIN_CHANGER_ENABLE
	bobbin_change_switch = 0;
	#endif
	#if BOBBIN_THREAD_DETECT
	baseline_detect_switch = 0;
	#endif
	
	allx_step_last = 0;
	ally_step_last = 0;
	operate_dsp1ordsp2_flag = 0;
	
	aging_old_counter = 0;
	needle_cool_flag = 0;
	running_flag = 0;
	
	CoolingInterval = 0;
	CoolingIntervalCounter = 0;
	CoolingDuration = 0;
	CoolingDurationCounter = 0;
	CoolingActionFlag = 0;
	Cooling1sCounter = 0;
    AgingLasting = 0;
	
	
	special_go_allmotor_flag = 0;
	finish_nopmove_pause_flag =0;
	nop_move_pause_flag = 0;
	
	read_step_x = 0;
	read_step_y = 0;
    nop_move_remainx = 0;
	nop_move_remainy = 0;
	
	last_pattern_point =0;
    last_allx_step = 0;
	last_ally_step = 0;
	bakeup_total_counter = 0;
	
	fk_action_flag = 0;
    fk_action_counter = 0;
	bobbin_change_done_flag = 0;
	sewingcontrol_tail_stitches = 1;
	aging_selection = 0;
	
	making_pen_actoin_flag = 0;
	making_pen_status = 0;
	
	spd_monitor = 0;
	tail_sewing_flag = 0;
	speed_up_value = 0;
	aging_mode_counter_1 = 0;
	aging_mode_counter_2 = 0;
	
	making_pen_offset_done = 0;
	
	bobbin_change_in_progress = 0;
    tail_sewing_speed = 3;
	tail_sewing_speed_flag = 0;
	
	barcoder_time_between_same_code = 0;
	
	wirte_stepermotor_para_flag = 0;
	write_stepmotor_curve_flag = 0;
	
	inpress_follow_down_angle =30;
	inpress_follow_up_angle = 210;
	inpress_follow_down_speed = 12 ;
	inpress_follow_up_speed = 12;
    inpress_follow_range = 32;
	inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
	yj_motor_dir = 0;
	
	rec1_datalength = 0;
	rec1_status_machine = 0;
	rec1_package_length = 0;
	
	already_auto_find_start_point = 0;
	write_eeprom_para_flag = 0;
	
	
	cutter_delay_counter = 0;
	cutter_delay_flag = 0;
	
	yj_step_current_level = 0;
	stepper_cutter_move_range = 100;
	stepper_cutter_move_time = 45;
	
	follow_up_inpresser_time_adj = 0;
	fw_start_angle = 1000;
	inpress_lower_stitchs = 0;
	inpress_lower_steps = 0;
	follow_up_inpresser_angle_adj = 0;
	inpress_delta = 0;
	holding_bobbin_start_angle = 0;
	shift_low_flag = 0;
	software_key_footer = 0;
	software_key_run = 0;
	software_key_pause = 0;
	software_key_bobbin = 0;
	rec1_total_counter = 0;
	laser_cutter_aciton_flag = 0;
	rc522_write_falg = 0;
	rc522_write_ret_falg = 1;
	rc522_control_falg = 0;
	foot_ready_counter_dva = 0;
	foot_ready_counter_dvb = 0;
	making_pen_nopmove_flag = 0;
	
#if AUTO_CHANGE_FRAMEWORK 
	left_footer_action_flag = 0;
	left_start_action_flag = 0;
	left_footer_counter = 0;
	left_start_counter = 0;
	right_footer_action_flag = 0;
	right_start_action_flag = 0;
	right_footer_counter = 0;
	right_start_counter = 0;
	left_footer_lock_flag = 0;
	left_start_lock_flag = 0;
	right_footer_lock_flag = 0;
	right_start_lock_flag = 0;
	left_footer_status = 0;
	left_second_footer_status = 0;
	left_quest_running = 0;
	right_footer_status = 0;
	right_second_footer_status = 0;
	right_quest_running = 0;
	waitting_for_pattern_done = 0;
	
	left_footer_delay_flag = 0;
	left_footer_delay_counter = 0;
	right_footer_delay_flag = 0;
	right_footer_delay_counter = 0;
	power_on_allow_keypress = 0;
	
	blow_air_action_flag = 0;
	blow_air_counter = 0;
#endif
	waitting_for_point_command = 0;
	auto_function_skip_flag = 1;
	main_control_lock_setup = 0;
	
	already_up_flag = 0;
	rfid_alarm_flag = 0;
	rfid_alarm_counter = 0;
	request_rfid_number = 0;
	
	cool_air_action_flag = 0;
    cool_air_counter = 0;
	cool_air_action_counter = 0;
	cool_air_1_sec = 0;
	
	cool_air_close_time = 10;
	cool_air_open_time = 5;
	
	led_turn_green_flag = 0;
	led_stay_green_counter = 0;
	led_stay_1s_counter = 0;
	
	first_power_on_flag = 0;
}			
//--------------------------------------------------------------------------------------
//  Name:		initial 
//  Parameters:	None
//  Returns:	None
//  Description: system initialization 
//--------------------------------------------------------------------------------------
void initial(void)
{
	INT16  count;
	//--------------------------------------------------------------------------------------
  	// disable interrupts
  	//--------------------------------------------------------------------------------------
  	asm("fclr I");
	//--------------------------------------------------------------------------------------
  	// call intial cpu clock function
  	//--------------------------------------------------------------------------------------
	init_clk();	
	p6_1  = 1;      // RST_PN=1
  	pd6_1 = 1;      
	p3  = 0x00;     // PAUSE
  	pd3 = 0xF8;
	init_eeprom();	
	init_para_variables();
	//--------------------------------------------------------------------------------------
  	// call initial IO direction fucntion
  	//--------------------------------------------------------------------------------------
  	init_io();	
	//--------------------------------------------------------------------------------------
  	// call initial timer A0 function
  	//--------------------------------------------------------------------------------------
  	init_ta0();
  	//--------------------------------------------------------------------------------------
  	// call initial timer B3 function
  	//--------------------------------------------------------------------------------------
  	init_tb3();
  	//--------------------------------------------------------------------------------------
  	// call initial timer B4 function
  	//--------------------------------------------------------------------------------------
  	init_tb4();
  	//--------------------------------------------------------------------------------------
  	// call initial ADC function
  	//--------------------------------------------------------------------------------------
  	init_ad();
  	//--------------------------------------------------------------------------------------
  	// call initial DAC function
  	//--------------------------------------------------------------------------------------
  	init_da();
  	//--------------------------------------------------------------------------------------
  	// call initial communication function
  	//--------------------------------------------------------------------------------------
	init_comm();	
	//--------------------------------------------------------------------------------------
  	// call initial stepmotor function
  	//--------------------------------------------------------------------------------------
	init_stepmotor_drv();	
	//--------------------------------------------------------------------------------------
  	// call initial external variables function
  	//--------------------------------------------------------------------------------------
  	init_var();	
	
	//--------------------------------------------------------------------------------------
  	// delay 1000ms  wait power on
  	//--------------------------------------------------------------------------------------
	count = 0;
	while(1)
	{
    	if(ir_ta0ic)        //test 1ms timer
    	{
			ir_ta0ic = 0;
		  	count++;
			if(count >= 1500)
			break;
		}
		#if USE_SC013K_PALTFORM
		#else
	    if( para.platform_type == FOURTH_GENERATION)
		{
	    	if((PWR_ON == 1) && (BLDC_ON == 0))   // if 120V < AC input < 290V and  BLDC ON 
		  		break;
		}
		#endif

  	}
	//--------------------------------------------------------------------------------------
  	// initial watch and motor
  	//--------------------------------------------------------------------------------------
	#if USE_SC013K_PALTFORM
	
	#else
 	if((PWR_ON == 0) && (AC_OVDT == 1))     // if 300V is overvoltage 
		sys.error = ERROR_05;
	else
	#endif
	{
		//call initial system watch module function     function is in the watch.c
	  	init_watch();	
	  	//set interrupt priority
		int1ic = INT1_IPL | 0x10; 			// rising  edge select           
	  	int2ic = INT2_IPL;              	// falling edge select 
	
	  	// enable interrupts
  		asm("fset I");

		//set motor PWM port output intial value
		U=1;U_=1;V=1;V_=1;W=1;W_=1;
		prcr = 0x02;
		inv03 = 0;
		prcr = 0x00;
	
		//output enable  
		OUTPUT_ON = 0;

		//delay 10ms
		count = 0;
		while(1)
		{
	    	if(ir_ta0ic)        //test 1ms timer
	    	{
				ir_ta0ic = 0;
			  	count++;
				if(count >= 10)
			  		break;
			}
		}

		//call system watch of initialize function 
		if(sys.error == 0)
			sys.error = watch_initialization();	 
	
		//disable interrupts
  		asm("fclr I");

		//call initial motor control module function
		init_motor();
	
		//set interrupt priority
		tb2ic = TB2_IPL;			      // falling edge select 
	  	int0ic = 0;//INT0_IPL;          // falling edge select 
	  	int3ic = 0;//INT3_IPL;          // scxadd falling edge select 
		s0tic = UART_TRANSMIT_IPL;	// falling edge select 
		s0ric = UART_RECEIVE_IPL;	  // falling edge select 
		s1tic = UART1_TRANSMIT_IPL;  // UART0 TX: TXR transmit
        s1ric = UART1_RECEIVE_IPL_7;   // UART0 RX: TXR receive 

	}
	//--------------------------------------------------------------------------------------
  	// set interrupt priority
  	//--------------------------------------------------------------------------------------
	if( para.platform_type == FOURTH_GENERATION)
	{
		PWR_LED = 0;	   
	  	ALARM_LED = 0;	
	}
	//--------------------------------------------------------------------------------------
  	// enable interrupts
  	//-------------------------------------------------------------------------------------- 
  	asm("fset I"); 

}

void mymemcpy(UINT8 *src,UINT8 *dest,UINT16 len)
{
	while( len > 0)
	{
		*dest++ = *src++;
		len--;
	}
}

UINT16 string2int(UINT8 *src)
{
	return(  (((UINT16)src[0])<<8 ) + src[1] );
}

void int2string( UINT8 *src,UINT16 dat)
{
	src[0] = (dat>>8) &0xff;
	src[1] =  dat&0xff;
}

void restore_para_from_eeprom(void)
{
	UINT16 index;
	index = 0;
	read_para_group(100,svpara_disp_buf,205);

	para.DSP1_para_1F   = string2int(&svpara_disp_buf[index]);	index +=2;  //0,1
	para.DSP1_para_20   = string2int(&svpara_disp_buf[index]);	index +=2;  //2,3
	para.DSP1_para_21   = string2int(&svpara_disp_buf[index]);	index +=2;
	para.DSP1_para_22   = string2int(&svpara_disp_buf[index]);	index +=2;
	para.DSP1_para_23   = string2int(&svpara_disp_buf[index]);  index +=2;
	para.DSP1_para_27   = string2int(&svpara_disp_buf[index]);	index +=2;
	para.DSP1_para_28H  = string2int(&svpara_disp_buf[index]);	index +=2;
	para.DSP1_para_28M1 = string2int(&svpara_disp_buf[index]);	index +=2;
	para.DSP1_para_28M2 = string2int(&svpara_disp_buf[index]);	index +=2;
	para.DSP1_para_28L  = string2int(&svpara_disp_buf[index]);	index +=2;
	
	para.DSP2_para_1F   = string2int(&svpara_disp_buf[index]);	index +=2;
	para.DSP2_para_20   = string2int(&svpara_disp_buf[index]);	index +=2;
	para.DSP2_para_21   = string2int(&svpara_disp_buf[index]);	index +=2;
	para.DSP2_para_22   = string2int(&svpara_disp_buf[index]);	index +=2;
	para.DSP2_para_23   = string2int(&svpara_disp_buf[index]);	index +=2;
	para.DSP2_para_27   = string2int(&svpara_disp_buf[index]);	index +=2;
	para.DSP2_para_28H  = string2int(&svpara_disp_buf[index]);	index +=2;
	para.DSP2_para_28M1 = string2int(&svpara_disp_buf[index]);	index +=2;
	para.DSP2_para_28M2 = string2int(&svpara_disp_buf[index]);	index +=2;
	para.DSP2_para_28L  = string2int(&svpara_disp_buf[index]);	index +=2;
	
	para.dsp1A_half_current = svpara_disp_buf[index++];
	para.dsp1B_half_current = svpara_disp_buf[index++];
	para.dsp2A_half_current = svpara_disp_buf[index++];
	para.dsp2B_half_current = svpara_disp_buf[index++];
		
	para.platform_type = svpara_disp_buf[index++];			
	para.mainmotor_type = svpara_disp_buf[index++];
	para.x_origin_mode = svpara_disp_buf[index++];		
	para.yj_org_direction = svpara_disp_buf[index++];
	para.Corner_deceleration_speed = svpara_disp_buf[index++];
	para.wipper_type = svpara_disp_buf[index++];
	para.x_sensor_open_level = svpara_disp_buf[index++];
	para.y_sensor_open_level = svpara_disp_buf[index++];
	para.laser_function_enable = svpara_disp_buf[index++];
	para.last_9_speed = svpara_disp_buf[index++];
	para.last_8_speed = svpara_disp_buf[index++];
	para.last_7_speed = svpara_disp_buf[index++];
	para.last_6_speed = svpara_disp_buf[index++];
	para.last_5_speed = svpara_disp_buf[index++];
	para.last_4_speed = svpara_disp_buf[index++];
	para.last_3_speed = svpara_disp_buf[index++];
	para.last_2_speed = svpara_disp_buf[index++];
	para.last_1_speed = svpara_disp_buf[index++];
	para.dvab_open_level = svpara_disp_buf[index++]; 
	para.dsp1_step_crc   = string2int(&svpara_disp_buf[index]);			index +=2;
	para.dsp2_step_crc   = string2int(&svpara_disp_buf[index]);			index +=2;
	
	para.y_backward_dis   = string2int(&svpara_disp_buf[index]);		index +=2;
	para.x_take_offset   = string2int(&svpara_disp_buf[index]);			index +=2;
	para.x_take_offset2   = string2int(&svpara_disp_buf[index]);		index +=2;
	
	para.left_barcode_position   = string2int(&svpara_disp_buf[index]);	index +=2;
	para.right_barcode_position   = string2int(&svpara_disp_buf[index]);index +=2;
	para.catch_delay_time   = string2int(&svpara_disp_buf[index]);		index +=2;
	
	para.y_barcode_position = string2int(&svpara_disp_buf[index]);	index +=2;
	para.blow_air_counter = string2int(&svpara_disp_buf[index]);	index +=2;
	para.cut_air_counter = string2int(&svpara_disp_buf[index]);		index +=2;
	
	para.dsp3_step_crc   = string2int(&svpara_disp_buf[index]);			index +=2;
	para.dsp4_step_crc   = string2int(&svpara_disp_buf[index]);			index +=2;
	
	para.DSP3_para_1F   = string2int(&svpara_disp_buf[index]);	index +=2;
	para.DSP3_para_20   = string2int(&svpara_disp_buf[index]);	index +=2;
	para.DSP3_para_21   = string2int(&svpara_disp_buf[index]);	index +=2;
	para.DSP3_para_22   = string2int(&svpara_disp_buf[index]);	index +=2;
	para.DSP3_para_23   = string2int(&svpara_disp_buf[index]);	index +=2;
	para.DSP3_para_27   = string2int(&svpara_disp_buf[index]);	index +=2;
	para.DSP3_para_28H  = string2int(&svpara_disp_buf[index]);	index +=2;
	para.DSP3_para_28M1 = string2int(&svpara_disp_buf[index]);	index +=2;
	para.DSP3_para_28M2 = string2int(&svpara_disp_buf[index]);	index +=2;
	para.DSP3_para_28L  = string2int(&svpara_disp_buf[index]);	index +=2;
	
	para.dsp3A_half_current = svpara_disp_buf[index++];
	para.dsp3B_half_current = svpara_disp_buf[index++];
	para.qd_org_direction = svpara_disp_buf[index++];
}

void cpy_para_buff(void)
{
	UINT16 index;
	index = 0;
	int2string(&svpara_disp_buf[index],para.DSP1_para_1F);  index += 2;
	int2string(&svpara_disp_buf[index],para.DSP1_para_20);  index += 2;
	int2string(&svpara_disp_buf[index],para.DSP1_para_21);  index += 2;
	int2string(&svpara_disp_buf[index],para.DSP1_para_22);  index += 2;
	int2string(&svpara_disp_buf[index],para.DSP1_para_23);  index += 2;
	int2string(&svpara_disp_buf[index],para.DSP1_para_27);  index += 2;
	int2string(&svpara_disp_buf[index],para.DSP1_para_28H); index += 2;
	int2string(&svpara_disp_buf[index],para.DSP1_para_28M1);index += 2;
	int2string(&svpara_disp_buf[index],para.DSP1_para_28M2);index += 2;
	int2string(&svpara_disp_buf[index],para.DSP1_para_28L); index += 2;

	int2string(&svpara_disp_buf[index],para.DSP2_para_1F);  index += 2;
	int2string(&svpara_disp_buf[index],para.DSP2_para_20);  index += 2;
	int2string(&svpara_disp_buf[index],para.DSP2_para_21);  index += 2;
	int2string(&svpara_disp_buf[index],para.DSP2_para_22);  index += 2;
	int2string(&svpara_disp_buf[index],para.DSP2_para_23);  index += 2;
	int2string(&svpara_disp_buf[index],para.DSP2_para_27);  index += 2;
	int2string(&svpara_disp_buf[index],para.DSP2_para_28H); index += 2;
	int2string(&svpara_disp_buf[index],para.DSP2_para_28M1);index += 2;
	int2string(&svpara_disp_buf[index],para.DSP2_para_28M2);index += 2;
	int2string(&svpara_disp_buf[index],para.DSP2_para_28L); index += 2;	
	
	svpara_disp_buf[index++] = para.dsp1A_half_current;
	svpara_disp_buf[index++] = para.dsp1B_half_current;
	svpara_disp_buf[index++] = para.dsp2A_half_current;
	svpara_disp_buf[index++] = para.dsp2B_half_current;
		
	svpara_disp_buf[index++] = para.platform_type;			
	svpara_disp_buf[index++] = para.mainmotor_type;
	svpara_disp_buf[index++] = para.x_origin_mode;		
	svpara_disp_buf[index++] = para.yj_org_direction;
	svpara_disp_buf[index++] = para.Corner_deceleration_speed;
	svpara_disp_buf[index++] = para.wipper_type;
	svpara_disp_buf[index++] = para.x_sensor_open_level;
	svpara_disp_buf[index++] = para.y_sensor_open_level;
	svpara_disp_buf[index++] = para.laser_function_enable;
	svpara_disp_buf[index++] = para.last_9_speed;
	svpara_disp_buf[index++] = para.last_8_speed;
	svpara_disp_buf[index++] = para.last_7_speed;
	svpara_disp_buf[index++] = para.last_6_speed;
	svpara_disp_buf[index++] = para.last_5_speed;
	svpara_disp_buf[index++] = para.last_4_speed;
	svpara_disp_buf[index++] = para.last_3_speed;
	svpara_disp_buf[index++] = para.last_2_speed;
	svpara_disp_buf[index++] = para.last_1_speed;
	svpara_disp_buf[index++] = para.dvab_open_level;
	
	int2string(&svpara_disp_buf[index],para.dsp1_step_crc);  index += 2;
	int2string(&svpara_disp_buf[index],para.dsp2_step_crc);  index += 2;//66,67
	
	int2string(&svpara_disp_buf[index],para.y_backward_dis);  index += 2;//68,69
	int2string(&svpara_disp_buf[index],para.x_take_offset);  index += 2; //70,71
	int2string(&svpara_disp_buf[index],para.x_take_offset2);  index += 2;//72,73

	int2string(&svpara_disp_buf[index],para.left_barcode_position);  index += 2; //74,75
	int2string(&svpara_disp_buf[index],para.right_barcode_position);  index += 2;//76,77

	int2string(&svpara_disp_buf[index],para.catch_delay_time);  index += 2;//78,79
	int2string(&svpara_disp_buf[index],para.y_barcode_position);  index += 2;//80,81
	
	int2string(&svpara_disp_buf[index],para.blow_air_counter);  index += 2;//82,83
	int2string(&svpara_disp_buf[index],para.cut_air_counter);  index += 2;//
	
	int2string(&svpara_disp_buf[index],para.dsp3_step_crc);  index += 2;
	int2string(&svpara_disp_buf[index],para.dsp4_step_crc);  index += 2;
	
	int2string(&svpara_disp_buf[index],para.DSP3_para_1F);  index += 2;
	int2string(&svpara_disp_buf[index],para.DSP3_para_20);  index += 2;
	int2string(&svpara_disp_buf[index],para.DSP3_para_21);  index += 2;
	int2string(&svpara_disp_buf[index],para.DSP3_para_22);  index += 2;
	int2string(&svpara_disp_buf[index],para.DSP3_para_23);  index += 2;
	int2string(&svpara_disp_buf[index],para.DSP3_para_27);  index += 2;
	int2string(&svpara_disp_buf[index],para.DSP3_para_28H); index += 2;
	int2string(&svpara_disp_buf[index],para.DSP3_para_28M1);index += 2;
	int2string(&svpara_disp_buf[index],para.DSP3_para_28M2);index += 2;
	int2string(&svpara_disp_buf[index],para.DSP3_para_28L); index += 2;	
	
	svpara_disp_buf[index++] = para.dsp3A_half_current;
	svpara_disp_buf[index++] = para.dsp3B_half_current;
	svpara_disp_buf[index++] = para.qd_org_direction;
	
	svpara_disp_buf[index++] = 55;
	svpara_disp_buf[index++] = 66;
}

void init_para_variables(void)
{
	restore_para_from_eeprom();
	if((para.DSP1_para_1F ==0XFFFF)||(para.DSP1_para_1F ==0X0000))
	{
		para.platform_type = FIFTH_GENERATION;
	
		para.DSP1_para_1F   = 0x0101;
		para.DSP1_para_20   = 1000;
		para.DSP1_para_21   = 1000;
		para.DSP1_para_22   = 20047;
		para.DSP1_para_23   = 4096;

		para.DSP1_para_27   = 100;
		para.DSP1_para_28H  = (128<<6)+40;
		para.DSP1_para_28M1 = (128<<6)+40;
		para.DSP1_para_28M2 = (3<<12)+4095;
		para.DSP1_para_28L  = 3<<12;
	
		para.DSP2_para_1F   = 0x0101;
		para.DSP2_para_20   = 400;
		para.DSP2_para_21   = 1000;
		para.DSP2_para_22   = 8192;
		para.DSP2_para_23   = 32768;

		para.DSP2_para_27   = 100;
		para.DSP2_para_28H  = (128<<6)+40;
		para.DSP2_para_28M1 = (128<<6)+40;
		para.DSP2_para_28M2 = (3<<12)+4095;
		para.DSP2_para_28L  = (3<<12)+4095;
		para.x_origin_mode = ENABLE_X_ORIGIN_FUN;
		para.Corner_deceleration_speed = 12;
		para.wipper_type = SOLENOID_WIPPER;
		para.x_sensor_open_level = 0;
		para.y_sensor_open_level = 0;
		para.laser_function_enable = 0;
		para.last_1_speed = 4;
		para.last_2_speed = 6;
		para.last_3_speed = 8;
		para.last_4_speed = 10;
		para.last_5_speed = 13;
		para.last_6_speed = 16;
		para.last_7_speed = 18;
		para.last_8_speed = 22;
		para.last_9_speed = 24;
		para.dvab_open_level = 0;
		para.dsp1_step_crc = 0;
		para.dsp2_step_crc = 0;
		para.y_backward_dis = 0;
		para.x_take_offset = 0;
		para.x_take_offset2 = 0;
		para.catch_delay_time = 0;
		para.y_barcode_position = 0;
		para.blow_air_counter = 0;
		para.cut_air_counter = 0;
		para.DSP3_para_1F   = 0x0202;
		para.DSP3_para_20   = 1000;
		para.DSP3_para_21   = 1000;
		para.DSP3_para_22   = 8192;
		para.DSP3_para_23   = 32768;

		para.DSP3_para_27   = 100;
		para.DSP3_para_28H  = (128<<6)+40;
		para.DSP3_para_28M1 = (128<<6)+40;
		para.DSP3_para_28M2 = (3<<12)+4095;
		para.DSP3_para_28L  = (3<<12)+4095;
	    para.qd_org_direction = 0;
		//mymemcpy((UINT8 *)&para,svpara_disp_buf,sizeof(SYSTEM_PARA));
		cpy_para_buff();
		write_para_group(100,svpara_disp_buf,205);	
	
	}	
}
//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xindahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
