//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : motor.c
//  Description: motor control arithmetic
//  Version    Date     Author    Description
//  0.01     03/07/07   pbb        created
//  ...
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
// 	Includes
//--------------------------------------------------------------------------------------
#include "..\..\include\sfr62p.h"            //M16C/62P special function register definitions
#include "..\..\include\typedef.h"           //data type define
#include "..\..\include\common.h"           //External variables declaration
#include "..\..\include\variables.h"        //External variables declaration
#include "..\..\include\motor_def.h"        //constants definition
#include "..\..\include\math_tab.h"          //sine table     
#include "..\..\include\math.h"         

#define VOL_ADJUST
//scxadd
static INT16 order_spd_last;//scx
static INT16 order_spd_this;//scx
static INT16 err_order_spd_this_last;//scx
static INT16 order_spd_state_1up_2cst_3dwn;//scx  后期可改成order_speed_mode
static INT16 order_spd_0_p_9;//s
static INT16 constant_spd_cnt;//scx

//scxadd
//--------------------------------------------------------------------------------------
//  ROM constants declaration - the values are declared at the end of the code
//--------------------------------------------------------------------------------------
/*
	30,32,35,34,36,35,37,37,36,36,
	32,35,36,35,37,36,37,38,37,37,//1
	34,35,37,35,37,37,37,38,37,38,
	34,35,37,35,37,37,38,38,38,38,//2
	35,35,37,35,37,38,38,38,38,38,
	35,35,37,35,37,38,38,38,38,38,//3
	36,35,37,35,37,38,38,38,38,38,
	36,35,37,35,37,38,38,38,38,37,//4
	37,35,37,35,37,38,38,38,38,37,
	38,35,37,35,37,38,38,38,38,37,//5
	39,35,37,35,37,38,38,38,38,37,
	40,35,37,35,37,38,38,38,38,37,//6
	43,35,37,35,37,38,38,38,37,37,
	46,35,37,35,37,38,38,38,36,38,//7
	46,35,37,35,37,38,38,38,36,39,
	46,35,37,35,37,38,38,38,36,39,//8
	47,36,37,35,37,38,38,38,36,39,
	48,36,37,35,37,38,38,37,36,39,//9
	49,36,38,35,37,38,38,36,36,39,
	50,37,38,35,37,38,37,35,36,39,//10
	50,37,38,35,37,38,37,34,36,39,
	51,38,39,35,37,38,36,34,36,39,//11
	51,39,39,35,37,38,36,34,36,39,
	51,39,39,36,37,38,36,35,36,39,//12
	51,40,39,36,37,38,36,36,36,39,
	51,40,39,36,37,38,36,37,36,39,//13
	51,40,39,36,37,38,36,38,36,39,
	51,40,39,36,37,38,36,38,36,39,//14
	50,40,39,36,37,38,36,39,36,39*/
const UINT8 etablenew1[29][10] =
{
//	
	30,32,35,34,36,35,34,35,36,36,
	32,35,36,35,37,36,34,35,37,37,//1
	34,35,36,35,37,37,34,35,37,38,
	34,35,36,35,37,37,35,35,38,38,//2
	35,35,36,35,37,38,35,35,38,38,
	35,35,36,35,37,38,35,35,38,38,//3
	36,35,36,35,37,38,35,35,38,38,
	36,35,36,35,37,38,35,35,38,37,//4
	37,35,36,35,37,38,36,35,38,37,
	38,35,36,35,37,38,37,36,38,37,//5
	39,35,37,35,37,38,38,36,38,37,
	40,35,37,35,37,38,38,37,38,37,//6
	43,35,37,35,37,38,38,38,37,37,
	46,35,37,35,37,38,38,38,36,38,//7
	46,35,37,35,37,38,38,38,36,39,
	46,35,37,35,37,38,38,38,36,39,//8
	47,36,37,35,37,38,38,38,36,39,
	48,36,37,35,37,38,38,37,36,39,//9
	49,36,38,35,37,38,38,36,36,39,
	50,37,38,35,37,38,37,35,36,39,//10
	50,37,38,35,37,38,37,34,36,39,
	51,38,39,35,37,38,36,34,36,39,//11
	51,39,39,35,37,38,36,34,36,39,
	51,39,39,36,37,38,36,35,36,39,//12
	51,40,39,36,37,38,36,36,36,39,
	51,40,39,36,37,38,36,37,36,39,//13
	51,40,39,36,37,38,36,38,36,39,
	51,40,39,36,37,38,36,38,36,39,//14
	52,40,39,36,37,38,36,39,36,39
};
const UINT8 etablenew2[10][6] =
{
	10,14,20,22,24,29,
	12,14,20,24,26,29,
	14,14,20,24,26,29,
	15,16,20,24,26,29,
	16,17,22,25,26,29,
	17,18,23,26,27,29,
	18,19,24,27,28,30,
	19,19,24,28,29,31,
	20,19,23,28,29,31,
	21,19,23,27,29,32
};
//--------------------------------------------------------------------------------------
//  Global variables define
//--------------------------------------------------------------------------------------
//************
static INT8 kk;
static INT16 sum_spd_s;
static UINT8 current_response_speed;
//************

static INT16 m_count;
static INT16 ta3z_elec;
static INT16 ta3z_mach;
static INT16 alpha;
static INT16 vol; 
static INT16 m_pos_ref;	    //motor referense position normalize
static UINT16 m_pos_ref_add;
static INT32 a_m_spd;
static INT32 a_m_spdlast;
static INT16 count_spd;

static INT32 sum_err_s;
static INT16 sum_err_p_stop;

static UINT8 start_status;
static INT16 ta3_start;

static UINT8 start_count;
static INT16 ta3_startdirect;//1225

static INT16 s_count;
static INT32 Y_k;//SCX  由16位改为32
//static INT16 Y_k;
static UINT8 ta3_ind;

static INT16 ta3_buf[1<<SAMPLES_BIT];

static INT8	stop_status;
static UINT8 l_ism;
INT32 eso_z1 = 0;
INT32 eso_z2 = 0;

static VECTOR v_vector = V_VECTOR_DEF;//scxadd 
PI_SPD_IQ Run_SpdIq ={
	
	0,//INT16 SetSpeed;
	0,//INT16 ActualSpeed;
	0,//INT16 Err_This;
	0,//INT16 Err_Last;
	0,//INT16 Err_LLast;
	1,//INT16 Kp;
	1,//INT16 Ki;
	1,
	0//INT16 Run_SpdIq.Sum_Iq;
	
};//scxadd

//****************************
//****************************
static INT16 rpm_keeper = 0;
static INT16 counter_H = 0;
static INT16 counter_L = 0;
static INT16 tactic_allowed;			
static INT16 delta_s;		
static INT16 pos = 0;
static INT16 transfer_sp = 0;
static INT16 transfer_si = 0;
//*****************************
//*****************************
volatile INT16 spd_tmp;
static INT16 spd_last_value;
static INT16 TaccTmp; 

static INT16 ta3n;
static UINT16 top = 255;
static UINT16 bottom = 0;
static UINT16 error = 1;
static UINT16 mid = 0;

static INT16 ta3_last;
static INT16 ta3_begin;
static INT16 delta_l;
static INT16 e_start;
static INT16 real_dir;
static INT16 zero_vol_counter;
static INT16 stop_change_counter;
static INT16 vol_last;
static INT16 compensation_flag = 0;
static INT8 e;

static INT16 ele_angle = 0;
static INT16 en;//SCX
static INT16 a_uzkval = 0;//scx

UINT8 r;
UINT8 l;
INT16 AccFlag ;
PISTRUCT PI_Pre;
UINT8 dectable[10];
UINT8 acctable[10];
//--------------------------------------------------------------------------------------
//  Internal functions and subroutines declaration
//--------------------------------------------------------------------------------------
#pragma	INTERRUPT/E pwm_int
void pwm_int(void);
#pragma	INTERRUPT/E int0_int
void int0_int(void);
#pragma	INTERRUPT/E int3_int
void int3_int(void);

void motor_control(void);
void m_start(INT16 ta3_t);
void m_startn(INT16 ta3_t);


void cal_forward(INT16 theta_input, INT16 u);
void cal_forwardn(INT16 theta_input, INT16 u);

INT16 cal_pid_s(INT16 pos_spd_ref, INT16 sp, INT16 si);
void cal_spd_ref(INT16 speed, INT16 acc_spd, INT16 dcc_spd);
//*********************
INT16 dither_tactic();
INT16 acc_tactic();
INT16 normal_control();
INT16 eso_normal_control();
INT16 pdff_control(UINT8 Kfr,UINT8 Kp,UINT8 Ki);
INT16 fall_back_tactic();
UINT8 oldmotoretable(INT16 spd,INT16 iq,UINT8 quadrant);
void MotorPreInit(void);
//*********************
//--------------------------------------------------------------------------------------
// 	Constants definition
//--------------------------------------------------------------------------------------
#define A_1 4096//scxadd 30711//27853  //(25Hz,0x71b5),(50Hz,0x664d),(100Hz,0x5532),(120Hz,0x4fdb),(150Hz,0x48fa),(200Hz,0x3fd8),(220Hz,0x3cc8),(250Hz,0x38b8),(280Hz 27853)                        
#define B0_1 2057//4915 //(25Hz,0x0e4a),(50Hz,0x19B2),(100Hz,0x2acd),(120Hz,0x3024),(150Hz,0x3705),(200Hz,0x4027),(220Hz,0x4337),(250Hz,0x4747),(280Hz 4915)

//--------------------------------------------------------------------------------------
//  Name:		init_tb0
//  pars:	None
//  Returns:	None
//  Description: initial timer B0
//-------------------------------------
//-------------------------------------------------
void init_tb0(void)
{
    //tb0mr = 0x4a;
	tb0mr = 0x42;       // XX0X XX00 
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
	tb0 = 0; 
	tb0s = 1;  //	1: start timer B0 (count flag)
    tb0ic = 0;
} 

//--------------------------------------------------------------------------------------
//  Name:		init_motor
//  Parameters:	None
//  Returns:	None
//  Description: initial motor control module 
//--------------------------------------------------------------------------------------
void init_motor(void)
{ 
    INT16 i; 
    
    //QEP_initialization
    ta3s = 0;                           // count disable,tabsr_addr.bit.b3 
    ta3ic = 0;                          // TA3 interrupt disable
    ta3mr = 0xd1;                       // 4tuple-phase event count mode,free_run type 
    udf |= 0x40;                        // two-phase pulse signal processing function enable  
    onsf |= 0x20;                       // Z-phase input is valid 
    trgsr &= 0xcf;                      // input on TA3IN is selected
    ta3 = 0;                            // counter clear 
    ta3ic = 0x00;                       // TA3 interrupt disable 
    ifsr0 = 0;                          // rising/falling edge select,single edge 
    int0ic = 0;			                // falling edge select 
    int3ic = 0;			                // scx add falling edge select 
	ifsr3 = 0;                          // rising/falling edge select
	
    ta3s = 1;                           // count enable 
        
    //PWM_initialization
    ictb2 = 1;                          // one TB2 underflow interrupt 
    prcr  = 0x02;
    invc0 = 0x16;                       // triangular wave modulation mode, no two active at an instance 
    invc1 = 0x02;                       // short-circuit protection time is valid, L is active 
    prcr  = 0x00;
    idb0 = 0x3f;                        // set three-phase output buffer register 0 
    idb1 = 0x3f;                        // set three-phase output buffer register 1 
    dtt = DTT_CNT;                      //16*(1/16*10^6) short-circuit protection time setting 
    prcr  = 0x02;
    tb2sc = 0x00;
    prcr  = 0x00;
    ta1mr = 0x12;                       // one-shot pulse mode 
    ta2mr = 0x12;                       // one-shot pulse mode 
    ta4mr = 0x12;                       // one-shot pulse mode 
    tb2mr = 0x00;                       // timer mode 
    trgsr |= 0x45;                      // trigger select register TB2 trigger 
    tb2 = CARR_CNT - 1;                 // carrier cycle 
    ta4 = 1;
	ta1 = 1;
	ta2 = 1;
    ta41 = CARR_CNT-1;
	ta11 = CARR_CNT-1;
	ta21 = CARR_CNT-1;

    tb2ic = 0;	                	// TB2 interrupt enable 
	prcr=0x02;
	
    inv03 = 0;
	prcr=0x00;
	
    m_count = 0;
	
    ta3z_elec = 0;
    ta3z_mach = 0;

    m_pos_ref = 0;
    m_spd_ref_n = 0;
	m_spd_n = 0;
	a_m_spdlast = 0;
	a_m_spd = 0;
	count_spd = 0;
	
    sum_err_s = 0; 

    TaccTmp = 0;
	spd_tmp = 0;
    alpha = 0;
    vol = 0;
	
	
	init_tb0();
	stop_status = 0;
    start_status = 0;
	ta3_start = 0;

    idb0 = 0x2a;                      
    idb1 = 0x15;                      	    
    tabsr |= 0x96;
	m_pos_ref_add = 0;
	
	s_count = 0;
	
	Y_k = 0;
    for(i=0;i<(1<<SAMPLES_BIT);i++)
        ta3_buf[i] = 0;
    ta3_ind = 0;
	m_status = INIT;
	l_ism = 0;
	m_spd_n_last = 0;
	
	//******************
	kk = 0;
	sum_spd_s = 0;
	current_response_speed = 0;
	pwm_forbid_flag = 0;
	over_spd_flag = 0;
	//******************
	find_deadpoint_flag = 0;
	allow = 1;
	ta3_last = 0;
	ta3_begin = 0;
	delta_l = 0;
	e_start = 0;
	real_dir = 0;
	zero_vol_counter = 0;
	stop_change_counter = 0;
	vol_last = 0;
//iit 
	sm_overload_counter = 0;
	sm_overload_flag = 0;
	sm_overload_value = 0;
	iit_warning_value = 200;
	
	e = 30;

	en = 492;//scx 30;

	//MAIN_MOTOR_TYPE = 1;
	theta_adjust = 840;
	AccFlag = 1;
}
void MotorPreInit(void)
{
	
	if(MAIN_MOTOR_TYPE == 2)//新电机2530
	{
		r = 39;//39
		l = 14;
		//da0 = 50;
		if(DIDT_SPEED_RANGE == 0)
		{
			PI_Pre.kp_l = 41;//40
			PI_Pre.ki_l = 16;//10
			PI_Pre.kp_up_l = 25;//30
			PI_Pre.ki_up_l = 30;//3;//5				
			PI_Pre.kp_transup_l = 37;//27
			PI_Pre.ki_transup_l = 20;//3;//3
		}
		else
		{
			PI_Pre.kp_l = 21;//40
			PI_Pre.ki_l = 16;//10
			PI_Pre.kp_up_l = 25;//30
			PI_Pre.ki_up_l = 30;//3;//5				
			PI_Pre.kp_transup_l = 27;//27
			PI_Pre.ki_transup_l = 20;//3;//3
		}
		PI_Pre.kp_m = 10;//3
		PI_Pre.ki_m = 20;//11
		PI_Pre.kp_h = 7;//5
		PI_Pre.ki_h = 40 ;//40


		PI_Pre.kp_up_m = 20;//9
		PI_Pre.ki_up_m = 20;//3;//3
		PI_Pre.kp_up_h = 20;//10
		PI_Pre.ki_up_h = 15;//3;//2
	
		PI_Pre.kp_down_l = 18;//18
		PI_Pre.ki_down_l = 2;//5
		PI_Pre.kp_down_h = 26;//26
		PI_Pre.ki_down_h = 2;
	
		PI_Pre.kp_transup_m = 10;//10
		PI_Pre.ki_transup_m = 20;//3;//3
		PI_Pre.kp_transup_h = 8;//7
		PI_Pre.ki_transup_h = 20;//3;//2
	
	
		PI_Pre.kp_transdown_l = 16;//3
		PI_Pre.ki_transdown_l = 2;//2
		PI_Pre.kp_transdown_h = 16;//7
		PI_Pre.ki_transdown_h = 2;//3

		PI_Pre.kp_stop0 = 8;
		PI_Pre.ki_stop0 = 8;
		PI_Pre.kp_stop1 = 8;
		PI_Pre.ki_stop1 = 8;
		PI_Pre.kp_stop2 = motor_para[0] ;
		PI_Pre.ki_stop2 = motor_para[1] ;
		PI_Pre.kp_stop = 25;
		PI_Pre.ki_stop = 5;
		
				
		PI_Pre.kpp_stop31 = STOP_KPp_1 - 2;
		PI_Pre.kps_stop31 = STOP_KPs_1  + 1;
		PI_Pre.kpp_stop32 = STOP_KPp_2 ;
		PI_Pre.kps_stop32 = STOP_KPs_2 ;
		
		dectable[0] = 2;
		dectable[1] = 4;
		dectable[2] = 6;
		dectable[3] = 8;
		dectable[4] = 15;
		dectable[5] = 16;
		dectable[6] = 18;
		dectable[7] = 4;
		dectable[8] = 10;
		dectable[9] = 10;
		
		acctable[0] = 3;//4
		acctable[1] = 3;//6
		acctable[2] = 2;//7
		acctable[3] = 3;//10
	}
	else //if (MAIN_MOTOR_TYPE == 1) 550W
	{
		
		r = 27;
		l = 12;
		//da0 = 100;
		
		if(DIDT_SPEED_RANGE == 0)
		{
			PI_Pre.kp_l = 41;//40
			PI_Pre.ki_l = 16;//10
			PI_Pre.kp_up_l = 30;//30
			PI_Pre.ki_up_l = 5;//5				
			PI_Pre.kp_transup_l = 37;//27
			PI_Pre.ki_transup_l = 3;//3
		}
		else
		{
			PI_Pre.kp_l = 21;//40
			PI_Pre.ki_l = 16;//10
			PI_Pre.kp_up_l = 30;//30
			PI_Pre.ki_up_l = 5;//5				
			PI_Pre.kp_transup_l = 27;//27
			PI_Pre.ki_transup_l = 3;//3
		}
		
		
		PI_Pre.kp_m = 10;//3
		PI_Pre.ki_m = 20;//11
		PI_Pre.kp_h = 7;//5
		PI_Pre.ki_h = 40 ;//40
		

		PI_Pre.kp_up_m = 20;//9
		PI_Pre.ki_up_m = 5;//3
		PI_Pre.kp_up_h = 20;//10
		PI_Pre.ki_up_h = 5;//2
	
		PI_Pre.kp_down_l = 18;//18
		PI_Pre.ki_down_l = 2;//5
		PI_Pre.kp_down_h = 26;//26
		PI_Pre.ki_down_h = 2;
	
		PI_Pre.kp_transup_m = 10;//10
		PI_Pre.ki_transup_m = 3;//3
		PI_Pre.kp_transup_h = 8;//7
		PI_Pre.ki_transup_h = 3;//2
	
	
		PI_Pre.kp_transdown_l = 16;//3
		PI_Pre.ki_transdown_l = 2;//2
		PI_Pre.kp_transdown_h = 16;//7
		PI_Pre.ki_transdown_h = 2;//3

		PI_Pre.kp_stop0 = 8;
		PI_Pre.ki_stop0 = 8;
		PI_Pre.kp_stop1 = 8;
		PI_Pre.ki_stop1 = 8;
		PI_Pre.kp_stop2 =  motor_para[0];
		PI_Pre.ki_stop2 =  motor_para[1];
		PI_Pre.kp_stop = 25;
		PI_Pre.ki_stop = 5;
		
				
		PI_Pre.kpp_stop31 = STOP_KPp_1 - 2;
		PI_Pre.kps_stop31 = STOP_KPs_1  + 1;
		PI_Pre.kpp_stop32 = STOP_KPp_2 ;
		PI_Pre.kps_stop32 = STOP_KPs_2 ;
		
		dectable[0] = 2;
		dectable[1] = 4;
		dectable[2] = 6;
		dectable[3] = 8;
		dectable[4] = 15;
		dectable[5] = 16;
		dectable[6] = 18;
		dectable[7] = 4;
		dectable[8] = 10;
		dectable[9] = 10;
		
		acctable[0] = 3;//4
		acctable[1] = 3;//6
		acctable[2] = 2;//7
		acctable[3] = 3;//10
	}

}
//--------------------------------------------------------------------------------------
//  Name:	pwm_int
//  Parameters:	None
//  Returns:	None
//  Description: pwm interrupt routine
//--------------------------------------------------------------------------------------
void pwm_int(void)
{   

    INT16 temp16,temp16c,spd,dir;
    INT16 l_theta_elec;                  //Q15,Motor Electrical Angle
	INT16 delta_p;
	INT16 leading_angle;
	INT16 tempzeroflag;
	INT16 tzeroflag;
	
//	if(sys.error != 0)
//	  return;
	

	//20120620
	//if(smotor_direction)	// anti-clockwise
	//	theta_adjust = 790;
	//else					// clockwise
	//	theta_adjust = 890;
   
    //save TA3 register value
	ta3n = (INT16)ta3;
	//calculate speed
    delta_p = ta3n - ta3_buf[ta3_ind];
    ta3_buf[ta3_ind++] = ta3n;
    ta3_ind &= SAMPLES;//8
	
	
	delta_l = ta3 - ta3_begin;
	
	tempzeroflag = EXTSM;
	tzeroflag = ISM;
		
//scxchange	
	if(abs(delta_p) < 11)//scx
	{
		if(ir_tb0ic)
		{
			ir_tb0ic = 0;
			if(mr3_tb0mr == 0)
			{
				
				//spd = (((INT32)SPD_K)/((INT16)(tb0>>1)))>>1;
				spd = ((INT32)SPD_K)/((INT16)(tb0>>1));
				if(delta_p < 0)
				    spd = -spd;
			}
			else
			{
				tb0s = 0;
				tb0 = 0;
				tb0s = 1;
				mr3_tb0mr = 0;
				spd = ((INT32)delta_p * RPM_K)>>RPM_Q;
			}
		}
		else
		{
			spd = ((INT32)delta_p * RPM_K)>>RPM_Q;
		}
	}
	else
	{
		spd = ((INT32)delta_p * RPM_K)>>RPM_Q;
	}


	if(MAIN_MOTOR_TYPE==1)
	{
		Y_k = Y_k + (( (INT32)spd*9 )/25 - m_spd_n)*A_1 ;
	}
	else
	{
		Y_k = Y_k + ((INT32)spd - m_spd_n)*A_1;
	}

	m_spd_n = Y_k >> 15;
//scxchange	
	
//	da1 = m_spd_ref_n >> 4;
//	da1 = m_spd_n >> 4;
//	da0 = stop_status*50;
//	da0 = abs(m_spd_n)>>5;

#if (DA1_OUTPUT_ANGLE==1)

	da1 = motor.angle_adjusted >> 3;
    
	if( (test_flag>0)&&(test_flag<10) )
	{
		da1 =0;
		test_flag++;
	}
	else if( (test_flag>20)&&(test_flag<30) )
	{
		da1 =255;
		test_flag++;
	}
	else
	{
		/*
		temp16c = motor.angle_adjusted;
	  	if((temp16c >= 440) &&(temp16c <= 460))//109~113
	  	{
			  da1 = 0;	                                                    
	  	}
		temp16c = motor.angle_adjusted;
	  	if( (temp16c >= 1000) && (temp16c <= 1020) )//249~253
	  	{
			  da1 = 0;	                                                    
	  	}
		*/
	}
	
#endif	

	switch(m_status)
    {
		case INIT:	//dummy status
         	ta3_start = ta3n;
		 	ta3_buf[ta3_ind] = ta3n;

			break;
			
	    case OPEN_START:     //start status
		
		if(MAIN_MOTOR_TYPE==1)
		{
			m_startn(ta3);
			temp16 = ta3n - ta3_startdirect;

		if(motor.dir == 1)		// clockwise
		{
			if(temp16 < 0 && tempzeroflag == 0)
			{
				U=1;
				U_=1;
				V=1;
				V_=1;
				W=1;
				W_=1;
				prcr = 0x02;
				inv03 = 0;
				prcr = 0x00;
				U_=0;
				V_=0;
				W_=0;
				ta3z_elec = ta3n;
				ta3z_mach = ta3z_elec;
				motor.spd_obj = 0;
				motor.dir = 0;
				motor.iq = 0;
				m_status = STOP;
				int3ic = INT3_IPL; 
				FindZeroFlag = 1;
			}
		}
		else
		{
			if(temp16 > 0 && tempzeroflag == 1)
			{
				U=1;
				U_=1;
				V=1;
				V_=1;
				W=1;
				W_=1;
				prcr = 0x02;
				inv03 = 0;
				prcr = 0x00;
				U_=0;
				V_=0;
				W_=0;
				ta3z_elec = ta3n;
				ta3z_mach = ta3z_elec;
				motor.spd_obj = 0;
				motor.dir = 0;
				motor.iq = 0;
				m_status = STOP;
				int3ic = INT3_IPL; 
				FindZeroFlag = 1;
			}
		}

			Run_SpdIq.Sum_Iq = 0;	//scx

			
		}
		else
		{
			m_startn(ta3);
			temp16 = ta3n - ta3_startdirect;
			
			if(motor.dir == 1)		// clockwise
			{
				if(temp16 < 0 && tzeroflag == 0)
				{
					U=1;
					U_=1;
					V=1;
					V_=1;
					W=1;
					W_=1;
					prcr = 0x02;
					inv03 = 0;
					prcr = 0x00;
					U_=0;
					V_=0;
					W_=0;
					ta3z_elec = ta3n;
					ta3z_mach = ta3z_elec;
					motor.spd_obj = 0;
					motor.dir = 0;
					motor.iq = 0;
					m_status = STOP;
					
					int0ic = INT0_IPL | 0x10;//2530 
					
					FindZeroFlag = 1;
				}
			}
			else
			{
				if(temp16 > 0 && tzeroflag == 1)
				{
					U=1;
					U_=1;
					V=1;
					V_=1;
					W=1;
					W_=1;
					prcr = 0x02;
					inv03 = 0;
					prcr = 0x00;
					U_=0;
					V_=0;
					W_=0;
					ta3z_elec = ta3n;
					ta3z_mach = ta3z_elec;
					motor.spd_obj = 0;
					motor.dir = 0;
					motor.iq = 0;
					m_status = STOP;
					
					int0ic = INT0_IPL | 0x10;//2530 
					
					FindZeroFlag = 1;
				}
			}

			Run_SpdIq.Sum_Iq = 0;	//scx
		}			
/*		else
		{
			int0ic = INT0_IPL | 0x10;//2530 

			m_start(ta3);
			temp16 = ta3 - ta3_start;
			
			while(temp16 >= ENCODER)
			{
				temp16 -= ENCODER;
			}
			while(temp16 < 0)
			{
				temp16 += ENCODER;
			}
			leading_angle = alpha;
			leading_angle = ((INT32)leading_angle * ENCODER) >> 10;
		
			while(leading_angle >= ENCODER)
			{
				leading_angle -= ENCODER;
			}
			while(leading_angle < 0)
			{
				leading_angle += ENCODER;
			}			

			l_theta_elec = (temp16 << 1) + theta_adjust + leading_angle;//temp16当前角度
		
			while(l_theta_elec >= ENCODER)
			{
				l_theta_elec -= ENCODER;
			}
			while(l_theta_elec < 0)
			{
				l_theta_elec += ENCODER;
			}
				
            cal_forward(l_theta_elec, vol);	

			temp16 = ISM;
				
			if(motor.dir)		
			{
				if( temp16 == 0 && delta_l < 0 )
				{
					U=1;U_=1;V=1;V_=1;W=1;W_=1;
					prcr = 0x02;
				    inv03 = 0;
					prcr = 0x00;
					U_=0;V_=0;W_=0;
					ta3z_elec = ta3n;
					ta3z_mach = ta3z_elec;
					FindZeroFlag = 1;
					motor.spd_obj = 0;
					motor.dir = 0;
					motor.iq = 0;
					m_status = STOP;
				}
			}
			else
			{
				if( temp16 == 1 && delta_l > 0)
				{
					U=1;U_=1;V=1;V_=1;W=1;W_=1;
					prcr = 0x02;
				    inv03 = 0;
					prcr = 0x00;
					U_=0;V_=0;W_=0;
					ta3z_elec = ta3n;
					ta3z_mach = ta3z_elec;
					FindZeroFlag = 1;
					motor.spd_obj = 0;
					motor.dir = 0;
					motor.iq = 0;
					m_status = STOP;
				}
			}
			l_ism = temp16;
			
		}
		
*/		
			break;
			
        case CLOSE_RUN:     //run of closed loop status
			//calculate mechanical angle
			if(MAIN_MOTOR_TYPE==1)
			{
				temp16 = ta3z_mach;
				temp16 = ta3n - temp16;//20150323 compatiable with new encoder
	
		
				while(temp16 >= LINE_4NUM)
				{
					temp16 -= LINE_4NUM;
				}
				while(temp16 < 0)
				{
					temp16 += LINE_4NUM;
				}
				motor.angle_up = ((INT32)temp16*5898)>>14;//scx=9/25
				motor.angle = - motor.angle_up;

				while(motor.angle < 0)
				{
					motor.angle += ENCODER;
				}
				while(motor.angle >= ENCODER)
				{
					motor.angle -= ENCODER;
				}
			
				motor.angle_adjusted =  motor.angle - AdjustAngle; //2010-4-7
			
			
	//			停车角度不对导致？
				while(motor.angle_adjusted < 0)
				{
					motor.angle_adjusted = motor.angle_adjusted + ENCODER;
				}
			
				while(motor.angle_adjusted > ENCODER)
				{
					motor.angle_adjusted = motor.angle_adjusted - ENCODER;
				}
			
				ele_angle = (motor.angle_up<<1);

				ele_angle = ele_angle + (((INT32)m_spd_n * 65) >> 14);//-(INT16)u226; //12=3度表示Z沿与电流波形零点的差值，如果正好对应则去掉该12。+abs(UserPara.first3StitchingFeedingSynModifyAngle_269)-3;//	+	(u228-128)	;//-360 ;//122934 + u227 -128;

				while(ele_angle < 0)
				{
					ele_angle = ele_angle + ENCODER;
				}
				while(ele_angle >=ENCODER)
				{
					ele_angle = ele_angle - ENCODER;
				}

				l_theta_elec = v_vector.angle;				//scx	
					l_theta_elec = ele_angle + l_theta_elec ;	//scx
		
				while(l_theta_elec < 0)						//scx
				{
			     	l_theta_elec = l_theta_elec + ENCODER;
				}
				while(l_theta_elec >=ENCODER)
				{
			     	l_theta_elec = l_theta_elec - ENCODER;
				}
						
	        	cal_forwardn(l_theta_elec,v_vector.mudulus);	//scx
			
				
			}
			else
			{
				temp16 = ta3z_mach;
				temp16 = ta3 - temp16;//20150323 compatiable with new encoder
				while(temp16 >= ENCODER)
				{
					temp16 -= ENCODER;
				}
				while(temp16 < 0)
				{
					temp16 += ENCODER;
				}
				motor.angle = temp16;

				motor.angle_adjusted = motor.angle - AdjustAngle; //2010-4-7
				while(motor.angle_adjusted < 0)
				{
					motor.angle_adjusted = motor.angle_adjusted + ENCODER;
				}
			
				//calculate electronical angle
				temp16 = ta3z_elec;
				temp16 = ta3 - temp16;//20150323 compatiable with new encoder
				while(temp16 >= ENCODER)
				{
					temp16 -= ENCODER;
				}
				while(temp16 < 0)
				{
					temp16 += ENCODER;
				}
			
				leading_angle = alpha;
				leading_angle = ((INT32)leading_angle * ENCODER) >> 10;
			
				while(leading_angle >= ENCODER)
				{
					leading_angle -= ENCODER;
				}
				while(leading_angle < 0)
				{
					leading_angle += ENCODER;
				}			
			
				l_theta_elec = (temp16 << 1) + theta_adjust + leading_angle;
			
				while(l_theta_elec >= ENCODER)
				{
					l_theta_elec -= ENCODER;
				}
				while(l_theta_elec < 0)
				{
					l_theta_elec += ENCODER;
				}
				
				cal_forward(l_theta_elec,vol);		
				
			}		
	        break;
			
	    case STOP:     //stopped status
		
			
            sum_err_s = 0;		//clear 
            sum_spd_s = 0;     
			motor.iq = 0;
//-------------------360ppr-750w--------------------//
			if(MAIN_MOTOR_TYPE==1)
			{
				motor.iq = 0;	//scx?
				motor.iq_last = 0;//scx?
			    Run_SpdIq.Sum_Iq = 0;	//scx?
				Ud_i = 0; 			//scx?
				Uq_i = 0; 			//scx?
				d_iq_last = 0; 		//scx?

//-------------------360ppr-750w--------------------//
				temp16 = ta3z_mach;
				temp16 = ta3n - temp16;
				//temp16 = ta3n - ta3z_mach;
				while(temp16 < 0)
				{
			     	temp16 = temp16 + LINE_4NUM;//ENCODER;
				}
				while(temp16 >= LINE_4NUM ) //ENCODER)
				{
			     	temp16 = temp16 - LINE_4NUM;//ENCODER;
				}						
				//scx1k motor.angle = temp16;
				motor.angle_up = ((INT32)temp16*5898)>>14;//ADJ_CODE_LINE_PAR(temp16);//(temp16*9)/25;//=temp16*720/2000=36/100   16384*9/25  /16384
				motor.angle = - motor.angle_up;

				while(motor.angle < 0)
					{
						motor.angle += ENCODER;
					}
				while(motor.angle >= ENCODER)
					{
						motor.angle -= ENCODER;
					}
				motor.angle_adjusted =  motor.angle - AdjustAngle;


				while(motor.angle_adjusted < 0)
					{
						motor.angle_adjusted += ENCODER;
					}
				while(motor.angle_adjusted >= ENCODER)
					{
						motor.angle_adjusted -= ENCODER;
					}
				
			}
			else
			{
				temp16 = ta3z_mach;
				temp16 = ta3 - temp16;
				while(temp16 >= ENCODER)
				{
					temp16 -= ENCODER;
				}
				while(temp16 < 0)
				{
					temp16 += ENCODER;
				}
				motor.angle = temp16;
			
				motor.angle_adjusted = motor.angle - AdjustAngle;//2010-4-7

				while(motor.angle_adjusted < 0)
				{
					motor.angle_adjusted = motor.angle_adjusted + ENCODER;
				}
				
			}
	        break;
    }	
}

//--------------------------------------------------------------------------------------
//  Name:	cal_forward
//  pars:	None
//  Returns:None
//  Description: 
//--------------------------------------------------------------------------------------
void cal_forward(INT16 theta_input, INT16 u)
{	  
    INT16 sin1,sin2;                    //Q15,sin of theta
    INT16 Ta,Tb,Tc;                          //duty ratio
    INT16 t0,t1,t2;
	
	if(theta_input < 720)
	{
		if(theta_input < 240)
		{
			if(theta_input >= 0)
			{
				sin1 = sin_tab[theta_input];
			    t2 = ((INT32)u*sin1) >> 15;   //Q14
			    sin2 = sin_tab[240-theta_input];
			    t1 = ((INT32)u*sin2) >> 15;   //Q14
			    t0 = (CARR_CNT-t1-t2)>>1;
			
                Ta = t0;
				Tb = Ta + t1;
				Tc = Tb + t2;
			}
		}
		else
		{
			if(theta_input < 480)
			{	
				theta_input -= 240;
				sin1 = sin_tab[theta_input];
				t1 = ((INT32)u*sin1) >> 15;   //Q14
			    sin2 = sin_tab[240-theta_input];
				t2 = ((INT32)u*sin2) >> 15;   //Q14
			    t0 = (CARR_CNT-t1-t2)>>1;
			
                Tb = t0;
				Ta = Tb + t1;
				Tc = Ta + t2;
				
			}
			else 
			{
				theta_input -= 480;
				sin1 = sin_tab[theta_input];
			    t2 = ((INT32)u*sin1) >> 15;   //Q14
			    sin2 = sin_tab[240-theta_input];
			    t1 = ((INT32)u*sin2) >> 15;   //Q14
			    t0 = (CARR_CNT-t1-t2)>>1;
			
				Tb = t0;
				Tc = Tb + t1;
				Ta = Tc + t2;
			}
		}
	}
	else
	{
		if(theta_input < 1200)
		{
			if(theta_input < 960)
			{
				theta_input -= 720;
				sin1 = sin_tab[theta_input];
                t1 = ((INT32)u*sin1) >> 15;   //Q14
			    sin2 = sin_tab[240-theta_input];
				t2 = ((INT32)u*sin2) >> 15;   //Q14
			    t0 = (CARR_CNT-t1-t2)>>1;
			
				Tc = t0;
				Tb = Tc + t1;
				Ta = Tb + t2;
			}
			else 
			{
				theta_input -= 960;
				sin1 = sin_tab[theta_input];
			    t2 = ((INT32)u*sin1) >> 15;   //Q14
			    sin2 = sin_tab[240-theta_input];
			    t1 = ((INT32)u*sin2) >> 15;   //Q14
			    t0 = (CARR_CNT-t1-t2)>>1;
			
				Tc = t0;
				Ta = Tc + t1;
				Tb = Ta + t2;
			}
		}
		else 
		{
			if(theta_input < 1440)
			{
				theta_input -= 1200;
				sin1 = sin_tab[theta_input];
				t1 = ((INT32)u*sin1) >> 15;   //Q14
			    sin2 = sin_tab[240-theta_input];
				t2 = ((INT32)u*sin2) >> 15;   //Q14
			    t0 = (CARR_CNT-t1-t2)>>1;
			
				Ta = t0;
				Tc = Ta + t1;
				Tb = Tc + t2;
			}
   		}
	}
	    
	if(Ta <= 0)
	    Ta = 1;
	else if(Ta >= CARR_CNT)	
	    Ta = CARR_CNT - 1;
	

	if(Tb <= 0)
	    Tb = 1;
	else if(Tb >= CARR_CNT)	
	    Tb = CARR_CNT - 1;
		
	if(Tc <= 0)
	    Tc = 1;
	else if(Tc >= CARR_CNT)	
	    Tc = CARR_CNT - 1;
	
	
    ta4 = Ta;
    ta1 = Tb;
    ta2 = Tc;                           
    ta41 = CARR_CNT - Ta;
    ta11 = CARR_CNT - Tb;
    ta21 = CARR_CNT - Tc;
    asm("nop");
    ta41 = CARR_CNT - Ta;
    ta11 = CARR_CNT - Tb;
    ta21 = CARR_CNT - Tc;
	
}

//--------------------------------------------------------------------------------------
//  Name:	cal_forward
//  pars:	None
//  Returns:None
//  Description:
//--------------------------------------------------------------------------------------
void cal_forwardn(INT16 theta_input, INT16 u)
{	  
    INT16 sin1,sin2;                    //Q15,sin of theta
    INT16 Ta,Tb,Tc;                     //duty ratio
    INT16 t0,t1,t2;
	INT16 udead_offset,vdead_offset,wdead_offset,dead_dir;//scx
	
	if(theta_input < 720)
	{
		if(theta_input < 240)
		{
			if(theta_input >= 0)
			{
				sin1 = sin_tab[theta_input];
			    t2 = ((INT32)u*sin1) >> 15;   //Q14
			    sin2 = sin_tab[240-theta_input];
			    t1 = ((INT32)u*sin2) >> 15;   //Q14
			    t0 = (CARR_CNT-t1-t2)>>1;
			
                Ta = t0;
				Tb = Ta + t1;
				Tc = Tb + t2;
			}
			else
			{
				U=1;U_=1;V=1;V_=1;W=1;W_=1;
	   			prcr = 0x02;
			    inv03 = 0;
	   			prcr = 0x00;
				U_=0;V_=0;W_=0;
				m_status = STOP;
			}
		}
		else
		{
			if(theta_input < 480)
			{	
				theta_input -= 240;
				sin1 = sin_tab[theta_input];
				t1 = ((INT32)u*sin1) >> 15;   //Q14
			    sin2 = sin_tab[240-theta_input];
				t2 = ((INT32)u*sin2) >> 15;   //Q14
			    t0 = (CARR_CNT-t1-t2)>>1;
			
                Tb = t0;
				Ta = Tb + t1;
				Tc = Ta + t2;
				
			}
			else 
			{
				theta_input -= 480;
				sin1 = sin_tab[theta_input];
			    t2 = ((INT32)u*sin1) >> 15;   //Q14
			    sin2 = sin_tab[240-theta_input];
			    t1 = ((INT32)u*sin2) >> 15;   //Q14
			    t0 = (CARR_CNT-t1-t2)>>1;
			
				Tb = t0;
				Tc = Tb + t1;
				Ta = Tc + t2;
			}
		}
	}
	else
	{
		if(theta_input < 1200)
		{
			if(theta_input < 960)
			{
				theta_input -= 720;
				sin1 = sin_tab[theta_input];
                t1 = ((INT32)u*sin1) >> 15;   //Q14
			    sin2 = sin_tab[240-theta_input];
				t2 = ((INT32)u*sin2) >> 15;   //Q14
			    t0 = (CARR_CNT-t1-t2)>>1;
			
				Tc = t0;
				Tb = Tc + t1;
				Ta = Tb + t2;
			}
			else 
			{
				theta_input -= 960;
				sin1 = sin_tab[theta_input];
			    t2 = ((INT32)u*sin1) >> 15;   //Q14
			    sin2 = sin_tab[240-theta_input];
			    t1 = ((INT32)u*sin2) >> 15;   //Q14
			    t0 = (CARR_CNT-t1-t2)>>1;
			
				Tc = t0;
				Ta = Tc + t1;
				Tb = Ta + t2;
			}
		}
		else 
		{
			if(theta_input < 1440)
			{
				theta_input -= 1200;
				sin1 = sin_tab[theta_input];
				t1 = ((INT32)u*sin1) >> 15;   //Q14
			    sin2 = sin_tab[240-theta_input];
				t2 = ((INT32)u*sin2) >> 15;   //Q14
			    t0 = (CARR_CNT-t1-t2)>>1;
			
				Ta = t0;
				Tc = Ta + t1;
				Tb = Tc + t2;
			}
			else
			{
				U=1;U_=1;V=1;V_=1;W=1;W_=1;
	   			prcr = 0x02;
			    inv03 = 0;
	   			prcr = 0x00;
				U_=0;V_=0;W_=0;
				m_status = STOP;

			}
   		}
	}


//****************V波形DTT_COMP=36********************

	if( (ele_angle>(NO_COMP_OFFSET+120) ) && ( ele_angle <(840-NO_COMP_OFFSET) ) )// 120-30  840-210
	{
		vdead_offset = -DTT_COMP;//DTT_COMP=36
	}
	else if( (ele_angle>(840+NO_COMP_OFFSET))|| (ele_angle<(120-NO_COMP_OFFSET)))
	{ 
		vdead_offset = DTT_COMP;
	}
	else
	{
			vdead_offset=0;
	}
//****************U波形********************

	if(  (ele_angle>(1080+NO_COMP_OFFSET)) || (ele_angle<(360-NO_COMP_OFFSET))	)// 
	{
		udead_offset = -DTT_COMP;
	}
	else if( (ele_angle>(360+NO_COMP_OFFSET)) &&   (ele_angle< (1080-NO_COMP_OFFSET) ) )
	{ 
		udead_offset = DTT_COMP;
	}
	else
	{
			udead_offset=0;
	}

//****************W波形********************
	
	if( (ele_angle>(600+NO_COMP_OFFSET)) && (ele_angle < (1320-NO_COMP_OFFSET)))// 
	{
		wdead_offset = -DTT_COMP;
	}
	else if((ele_angle>(1320+NO_COMP_OFFSET))||((ele_angle<(600-NO_COMP_OFFSET))))
	{ 
		wdead_offset = DTT_COMP;
	}
	else
	{
        wdead_offset=0;
	}
	
//scx xiug DTT_COMP=36

	if(motor.iq_last > IQ_MIN_VALUE)
	{
		dead_dir = 1;
	}
	else if(motor.iq_last < -IQ_MIN_VALUE)
	{
		dead_dir = -1;
	}
	else
	{
		dead_dir = 0;	
	}
	
//	da0_test=	vdead_offset * dead_dir *2+ 128;

//	dead_dir = 0;
	
	Ta += (udead_offset * dead_dir);
	Tb += (vdead_offset * dead_dir);
	Tc += (wdead_offset * dead_dir);


//scx	 CARR_CNT=1200	    

	    
	if(Ta <= 0)
	    Ta = 1;
	else if(Ta >= CARR_CNT)	
	    Ta = CARR_CNT - 1;
	

	if(Tb <= 0)
	    Tb = 1;
	else if(Tb >= CARR_CNT)	
	    Tb = CARR_CNT - 1;
		
	if(Tc <= 0)
	    Tc = 1;
	else if(Tc >= CARR_CNT)	
	    Tc = CARR_CNT - 1;
	
	
    ta4 = Ta;
    ta1 = Tb;
    ta2 = Tc;                           
    ta41 = CARR_CNT - Ta;
    ta11 = CARR_CNT - Tb;
    ta21 = CARR_CNT - Tc;
    asm("nop");
    ta41 = CARR_CNT - Ta;
    ta11 = CARR_CNT - Tb;
    ta21 = CARR_CNT - Tc;
	
}

//scx20161031
PI_SPD_IQ Spd_Iq_PID(PI_SPD_IQ Pi_Spd_Iq)
{
			//定义PI结构体变量
	
	INT16 En_Ki_Flag;				//定义是否开启积分项标志
	INT32 Increment_Iq;				//Iq的调节量
	INT32 ABS_SpdErr;
	INT32 Pi_tmp,Di_tmp,Sum_Iq_tmp;
	
	PI_SPD_IQ Pi_Spd_Iq_temp = Pi_Spd_Iq;

//计算当前偏差	
	Pi_Spd_Iq_temp.Err_This		=  ( Pi_Spd_Iq_temp.SetSpeed - Pi_Spd_Iq_temp.ActualSpeed );

	ABS_SpdErr					= abs(Pi_Spd_Iq_temp.Err_This);

	if( ABS_SpdErr> EN_KI_VALUE)		//偏差太大，取消Ki项
	{
		En_Ki_Flag = 0;					// 
	}
	else								//偏差在规定范围内，启动Ki项
	{
		En_Ki_Flag = 1;	
	}
	
//		Δu(k)   = 	kp * 	( error(k) - error(k-1) )		 +		  ki *	error(k)        // + kd*( error(k)-2*error(k-1) + error(k-2) )
		
	Pi_tmp  = ( Pi_Spd_Iq_temp.Err_This * En_Ki_Flag * Pi_Spd_Iq_temp.Ki )<<6;
	
	Di_tmp	= ( ( Pi_Spd_Iq_temp.Err_This - 2*Pi_Spd_Iq_temp.Err_Last + Pi_Spd_Iq_temp.Err_LLast )* Pi_Spd_Iq_temp.Kd )>>8;
	
	Increment_Iq =	((Pi_Spd_Iq_temp.Err_This <<13)- Pi_Spd_Iq_temp.Err_Last ) * Pi_Spd_Iq_temp.Kp + Pi_tmp	+ Di_tmp;
	//如果把Pi_Spd_Iq_temp.Kp移动到括号里，合并err_last，则在KP变化时，不能保证this和last同步变化
	
	if(	 Increment_Iq > DETA_IQ_PID_MAX_V )//超过极限值还发送之前的调节量
	{
		Increment_Iq	=	DETA_IQ_PID_MAX_V;
	}
	if(	 Increment_Iq < -DETA_IQ_PID_MAX_V )//超过极限值还发送之前的调节量
	{
		Increment_Iq	=	-DETA_IQ_PID_MAX_V;
	}

	//实际结果	=	u(k) + 增量调节值Δu(k) 
	Sum_Iq_tmp = Pi_Spd_Iq_temp.Sum_Iq + Increment_Iq;
//总量限幅	
	if(Sum_Iq_tmp > IQ_PID_MAX_V )//超过极限值还发送之前的调节量
	{
		Sum_Iq_tmp	=	IQ_PID_MAX_V;
	}
	
	if( Sum_Iq_tmp < -IQ_PID_MAX_V )//超过极限值还发送之前的调节量
	{
		Sum_Iq_tmp	=	-IQ_PID_MAX_V;
	}

	Increment_Iq = Sum_Iq_tmp - Pi_Spd_Iq_temp.Sum_Iq;
	
	Pi_Spd_Iq_temp.Err_LLast = Pi_Spd_Iq_temp.Err_Last;//PID-D	
	
	Pi_Spd_Iq_temp.Err_Last += (Increment_Iq - Pi_tmp - Di_tmp)/Pi_Spd_Iq_temp.Kp;//( Pi_Spd_Iq_temp.Err_This * En_Ki_Flag * Pi_Spd_Iq_temp.Ki );//  /KP是为了防止Kp发生变化时，Pi_Spd_Iq_temp.Err_Last初次使用的不是0的情况	
	


	Pi_Spd_Iq_temp.Sum_Iq = Sum_Iq_tmp;
	
	return Pi_Spd_Iq_temp;//(INT16)((Pi_Spd_Iq.Sum_Iq)>>13);

}

//scx20161031


INT16 cal_pid_s(INT16 pos_spd_ref, INT16 sp, INT16 si)
{	
 INT32 preSatOut;
	
	
	pos = pos_spd_ref;
	transfer_sp = sp;
	transfer_si = si;
	//da1 = transfer_sp;
	tactic_flag_last = tactic_flag;

	tactic_allowed = 0;
	/*	
	if( (tactic_allowed == 1) && ( (m_spd_ref_n <= 2000) && (m_spd_ref_n >= 400) && (allow == 1)) )
	{
		if(tactic_flag == FB)
		{
			if(m_spd_n <= ((INT32)m_spd_ref_n<<1))
			{
				preSatOut = normal_control();
			}
			else
			{ 
				preSatOut = fall_back_tactic();
			}
		}
		else
		{
			if(door_ac == 1)
			{
				if(door_dt == 1)	
				{
					if(tactic_flag == DT)
					{
						if(m_spd_n < ( m_spd_ref_n * 3 ))//modified by zz on 20100311
						{
							preSatOut = dither_tactic();
						}
						else			
						{	
							preSatOut = fall_back_tactic();
						}
					}
					else				
					{
						if(iq_max_tester >= 20) 
						{
							preSatOut = dither_tactic();
						}
					
						else				
						{
			
							if(m_spd_n < 60)	
							{
								
								if(speed_min_tester >= 30)
								{
									speed_min_tester = 0;
									preSatOut = dither_tactic();
								}
								else
								{
									preSatOut = acc_tactic();
								}
				
							}
			
							else				
							{
								switch(tactic_flag)
								{
									case AC:	
					
										if(m_spd_n > ( ((INT32)m_spd_ref_n * 10) >> 3))//modified by zz on 20100311
										{
							
											if(m_spd_n_last > ( ((INT32)m_spd_ref_n * 10) >> 3))//modified by zz on 20100311
											{
												rpm_keeper++;
											}
											else				
											{
												rpm_keeper = 0;
											}
																								
											if(rpm_keeper >= AC_HOLD_TIME)		
											{
												rpm_keeper = 0;
												preSatOut = fall_back_tactic();
											}
											else				
											{
												preSatOut = acc_tactic();
											}
							
										}
										else				 
										{
											preSatOut = acc_tactic();
										}
						
										break;
					
									case NC:	
						
										if(m_spd_n < ( (m_spd_ref_n * 6) >> 3))	
										{
											transfer_sp = 15;//20
											transfer_si = 20;
											preSatOut = normal_control();
										}
										else				
										{
											if(m_spd_n < ( ((INT32)m_spd_ref_n * 12) >> 3))//modified by zz on 20100311	
											{
												preSatOut = normal_control();
											}
											else				
											{
												transfer_sp = 15;//20
												transfer_si = 20;
												preSatOut = normal_control();
											}
										}				
										break;
								}
				
							}
						}
					}
				}
				else
				{
					preSatOut = normal_control();
				}
			}
			else
			{
				preSatOut = normal_control();	
			}
		
		}
	}
	else	*/			
	{
		if(stop_status == 2 && motor.spd_obj == 0)
			preSatOut = eso_normal_control();	
		else
		    preSatOut = normal_control();
	}



	
	m_spd_n_last = m_spd_n;

	return (INT16)preSatOut;
}

//*************************
//normal_control()
//*************************
INT16 normal_control()
{
	INT32 preSatOut;
    INT16 err;
	INT32 up;
	INT16 pos_spd_ref;
	INT16 sp;
	INT16 si;
	
	pos_spd_ref = pos;
	sp = transfer_sp;
	si = transfer_si;
	
	//tactic_flag = NC;
 
if(motor.spd_obj != 0)
  {
	eso_z1 = 0;
	eso_z2 = 0;	  
  } 
 
	err = m_spd_ref_n  - m_spd_n + pos_spd_ref ;
	if(motor.spd_obj <=300 && motor.spd_obj > 0)
	   {
		 if(abs(err) < 100) 
		   {
		   sp = transfer_sp;//1
	       si = transfer_si;//1	  			   
		   }
		 else if(abs(err) < 200) 
		   {
		   sp = transfer_sp*1;//
	       si = transfer_si*2;//	  			   
		   }
		 else if(abs(err) < 300) 
		   {
		   sp = transfer_sp*1;//
	       si = transfer_si*2;//	  			   
		   }
         else
		   {
		   sp = transfer_sp*1;//
	       si = transfer_si*2;//			   
		   }		   
	   }
	else if(motor.spd_obj <=800 && err >= 600 && motor.spd_obj > 0)
	   {
		 sp = transfer_sp*1;
	     si = transfer_si*2;	 		   
	   }
	up = (((INT32)err)*sp)>>2;//这尼玛
	//up = ((INT32)err)*sp ;
	//sum_err_s = sum_err_s + ((((INT32)si)*up)>>10);	
    sum_err_s = sum_err_s + ((((INT32)si)*up)>>12);

	if((sum_err_s) > 10600)//8000
		sum_err_s = 10600;
	else if((sum_err_s) < -10600)	
		sum_err_s = -10600;
	//da1 = 	sum_err_s >> 2;	
	preSatOut = up + sum_err_s;
	
	if(preSatOut > PID_SPD_MAX_P)
	    { 
			preSatOut = PID_SPD_MAX_P;
		}

	if(preSatOut < -PID_SPD_MAX_N)
		{
	 		preSatOut = -PID_SPD_MAX_N;	
		}
	
	return (INT16)preSatOut;	
}
INT16 eso_normal_control()
{
	INT32 preSatOut;    INT16 err;	INT32 up;
	INT16 pos_spd_ref;	INT16 sp;	INT16 si;
	
	INT32 eso_e ;
	INT32 eso_y = 1;
	INT32 eso_u = 1;
	
	INT32 eso_h = 1;
	INT32 eso_b = 3;

	INT32 beta01 = 3;
	INT32 beta02 = 3;
	INT32 temp_z2 = 0;
	
	
	
eso_y = (INT32)m_spd_n << 10 ;         //eso_spd.y=pid1_spd.Fdb;// 9
eso_u = (INT32)motor.iq;        //eso_spd.u=filterEsospd.lOutPut;  
	
eso_e = eso_z1 - eso_y;  //v->e=v->z1-v->y;
eso_e = eso_e >> 10;
/*
eso_z1 = eso_z1+ (eso_z2 >> 5) - eso_e * beta01 + (eso_u * eso_b >> 5) ;
eso_z2 = eso_z2 - beta02 * eso_e;
//_IQmpy((v->h),(v->z2)) - _IQmpy((v->beta01),(v->e)) + _IQmpy((v->u),_IQmpy((v->h),(v->b)));   
*/
eso_z1 = eso_z1+ ((eso_z2>>10) * eso_h) - eso_e * beta01 * 1024 + (eso_u * eso_b * eso_h) ;//_IQmpy((v->h),(v->z2))-_IQmpy((v->beta01),(v->e))+_IQmpy((v->u),_IQmpy((v->h),(v->b)));   
eso_z2 = eso_z2 - beta02 * eso_e * 1024;

temp_z2 = eso_z2 >> 10;

if(temp_z2 > 3000)
   temp_z2 = 3000;
else if (temp_z2 < -3000)
   temp_z2 = -3000;

//da0 = motor.iq >> 5;
//da1 = temp_z2 >> 4;

/*          
//pid1_iq.Ref = pid1_spd.Out- _IQmpy(eso_spd.z2,_IQ(0.5));	
*/	
	pos_spd_ref = pos;
	sp = transfer_sp;
	si = transfer_si;	
 
	err = m_spd_ref_n  - m_spd_n + pos_spd_ref ;

	up = (((INT32)err)*sp)>>2;//这尼玛	
    sum_err_s = sum_err_s + ((((INT32)si)*up)>>12);

	if((sum_err_s) > 8000)
		sum_err_s = 8000;
	else if((sum_err_s) < -8000)	
		sum_err_s = -8000;

//	preSatOut = up + sum_err_s; //temp_z2
	preSatOut = up + sum_err_s ;//- (temp_z2 >> 1);
	
	if(preSatOut > PID_SPD_MAX_P)
	    { 
			preSatOut = PID_SPD_MAX_P;
		}
	if(preSatOut < -PID_SPD_MAX_N)
		{
	 		preSatOut = -PID_SPD_MAX_N;	
		}
	
	return (INT16)preSatOut;	
}


//Kfr(0-32 reference 0to 1)
INT16 pdff_control(UINT8 Kfr,UINT8 Kp,UINT8 Ki)
{
	INT32 temp32;
    //INT16 temp16;
	INT16 out;

	temp32= m_spd_ref_n  - m_spd_n;
	
	temp32 = temp32 * Kp;
	temp32 = temp32 >> 2;
	
	temp32 = temp32 * Ki;
	temp32 = temp32 >> 12;
		
	sum_err_s = sum_err_s + temp32;
	
	if((sum_err_s) > 9000)
		sum_err_s = 9000;
		
	if((sum_err_s) < -9000)	
		sum_err_s = -9000;
	//da0 = 	sum_err_s >> 2;	
	
	//temp32 = Kfr * m_spd_ref_n;
	//temp32 = Kp * temp32;
	//temp32 = temp32 /32;
	temp32 = Kp * m_spd_ref_n;
	temp32 = temp32 * Kfr;
	temp32 = temp32 / 32;
	out = temp32;
	
	temp32 = m_spd_n * Kp;
	
	out = out - temp32;
	
	out +=sum_err_s;
	
	if(out > PID_SPD_MAX_P)
	{ 
		out = PID_SPD_MAX_P;
	}

	if(out < -PID_SPD_MAX_N)
	{
	 	out = -PID_SPD_MAX_N;	
	}
		
	return out;	
}
//****************************
//acc_tactic()
//****************************
INT16 acc_tactic()
{	
	INT16 preSatOut;
	
	current_response_speed = 4;
	
	tactic_flag = AC;
	
	preSatOut = PID_SPD_MAX_P;

	return preSatOut;
}


//****************************
//dither_tactic()
//****************************
INT16 dither_tactic()
{	
	INT16 preSatOut;
	INT16 move_bit;
	
		move_bit = 4;
	current_response_speed = 4;
	
	tactic_flag = DT;
	
	if(motor.iq == PID_SPD_MAX_P)	
	{
		if(motor.iq_last == PID_SPD_MAX_P)	
		{
			counter_H++;
		}
		else								
		{
			counter_H = 0;
		}
		if(counter_H > H_time)		
		{
			counter_H = 0;
			preSatOut = PID_SPD_MAX_P >> move_bit;
		}
		else					
		{
			preSatOut = PID_SPD_MAX_P;
		}
		
	}
	else							
	{
		if(motor.iq_last == (PID_SPD_MAX_P >> move_bit))	
		{
			counter_L++;
		}
		else
		{
			counter_L = 0;
		}
		
		if(counter_L >= L_time)
		{
			counter_L = 0;
			preSatOut = PID_SPD_MAX_P;
		}
		else
		{
			preSatOut = PID_SPD_MAX_P >> move_bit;
		}
	}
	
	return preSatOut;
	
}


//****************************
//fall_back_tactic()
//****************************
INT16 fall_back_tactic()
{
	INT16 preSatOut;
	
	current_response_speed = 4;
	
	tactic_flag = FB;
	
	preSatOut = -4000;
	
	return preSatOut;
}

INT16 cal_pid_p_stopn(INT16 pp, INT16 ps, INT16 is)
{
	INT16 preSatOut;
	INT16 err_p;
	INT16 spd_err;
	INT16 temp16;
	err_p = -motor.angle_hold;
	spd_err = err_p * pp - m_spd_n;

	sum_spd_s = sum_spd_s + spd_err;
	sum_spd_s = (ps * is * sum_spd_s)>>10;
	limit(sum_spd_s,1000);//原来4000

	preSatOut = spd_err * ps + sum_spd_s;

	limit(preSatOut,PID_SPD_MAX_P);

	motor.stop_angle1 = motor.stop_angle1 +  2 * kk;
	if(kk >= 0)
	{
		if(motor.stop_angle1 >= motor.stop_angle )
			motor.stop_angle1 = motor.stop_angle ;
	}
	else
	{
		if(motor.stop_angle1 <= motor.stop_angle )
			motor.stop_angle1 = motor.stop_angle ;
	}
	return (INT16)preSatOut;
}


//--------------------------------------------------------------------------------------
//  Name:	cal_pid_p_stop
//  Parameters:	None
//  Returns:	None
//  Description: calculate position loop stop PID
//--------------------------------------------------------------------------------------
INT16 cal_pid_p_stop(INT16 pp, INT16 ps, INT16 is)
{
	INT16 preSatOut;
	INT16 err_p;
	INT16 spd_err;
	err_p =  motor.stop_angle1- motor.angle_adjusted;  //2010-4-7
    //err_p =  motor.stop_angle - motor.angle_adjusted;

	//da1 = err_p * 4 + 128;
	spd_err = err_p * pp;
	//if(spd_err > ((STOP_SPD)>>1))
	//{
	//	spd_err = ((STOP_SPD)>>1);	
	//}
	spd_err = spd_err - m_spd_n;

	/*sum_spd_s = sum_spd_s + spd_err;
	sum_spd_s = (ps * is * sum_spd_s)>>10;
	limit(sum_spd_s,500);
	preSatOut = spd_err * ps + sum_spd_s;*/
	sum_err_s = sum_err_s +	spd_err;
	sum_err_s = (ps * is * sum_err_s)>>10;
	
	limit(sum_err_s,1000);
	preSatOut = spd_err * ps + sum_err_s;
	
    limit(preSatOut,PID_SPD_MAX_P);
	if (kk > 0)
	{
		if(motor.stop_angle1 < motor.stop_angle )	
		{
				if(STOP_SPD <= 300)
					motor.stop_angle1 = motor.stop_angle1 + 1;//2;//1
				else	
					motor.stop_angle1 = motor.stop_angle1 + 1;//3;//1
		}
		
		if(motor.stop_angle1 > motor.stop_angle )	
			motor.stop_angle1 = motor.stop_angle ;
	}
	if(kk < 0)
	{
		if(motor.stop_angle1 > motor.stop_angle )	
		{
				if(STOP_SPD < 300)
					motor.stop_angle1 = motor.stop_angle1 - 6;
				else	
					motor.stop_angle1 = motor.stop_angle1 - 8;
		}
		
		if(motor.stop_angle1 < motor.stop_angle )	
			motor.stop_angle1 = motor.stop_angle ;
	}
	/*if(abs(motor.stop_angle - motor.angle_adjusted ) <= 6)
		motor.stop_angle1 = motor.stop_angle;
		
	if ( abs(motor.stop_angle1 - motor.stop_angle) <= 6 )
		motor.stop_angle1 = motor.stop_angle;
	else
    	motor.stop_angle1 = motor.stop_angle1+ 5 * kk;*/
	if(m_spd_n < 10)
		motor.stop_angle1 = motor.stop_angle;
	
	//da1 = motor.stop_angle1 ;
	
	return (INT16)preSatOut;
}

void m_startn(INT16 ta3_t)
{
	INT16 delta_ta3;
	INT16 temp16;


	m_count++;
	delta_ta3 = ta3_t - ta3_start;
	if(motor.dir)
	{
		if(m_count >= COUNT_S)                       //10ms
		{
			alpha -= 5;								 // angle change 5/4=1.25du
			if(alpha < 0)
				alpha += 1440;
			motor.iq += DELTA_I;                      //1A,Q12,1365 motor.iq changes 1000 and angle changes 1.25du @1ms scx
			m_count = 0;
		}
		switch(start_status)
		{
			case 0:									
				if(delta_ta3 < 0)
				{
					start_status = 1;
				}
				else if(delta_ta3 > 0)
				{
					alpha -= 720;
					if(alpha < 0)
						alpha += 1440;
					motor.iq = motor.iq<<1;
					start_status = 1;
					ta3_start = ta3_t;
				}
				break;
			case 1:
				if(delta_ta3 < 0)
				{
					ta3_start = ta3_t;
					start_count++;
					if(start_count > 10)
						start_status = 2;
				}
				else if(delta_ta3 > 0)
				{
					start_count++;
					if((start_count > 200) && (delta_ta3 > 5))
					{
						alpha -= 720;
						if(alpha < 0)
							alpha += 1440;
						motor.iq = motor.iq<<1;
						start_status = 2;
						start_count = 0;
					}
				}
				break;
		}
	}
	else
	{
		if(m_count >= COUNT_S)                       //10ms
		{
			alpha += 5;
			if(alpha >= 1440)
				alpha -= 1440;
			motor.iq += DELTA_I;                      //1A,Q12,1365
			m_count = 0;
		}
		switch(start_status)
		{
			case 0:
				if(delta_ta3 > 0)
				{
					start_status = 1;
				}
				else if(delta_ta3 < 0)
				{
					alpha += 720;
					if(alpha >= 1440)
						alpha -= 1440;
					motor.iq = motor.iq<<1;
					start_status = 1;
					ta3_start = ta3_t;
				}
				break;
			case 1:
				if(delta_ta3 > 0)
				{
					ta3_start = ta3_t;
					start_count++;
					if(start_count > 10)
						start_status = 2;
				}
				else if(delta_ta3 < 0)
				{
					start_count++;
					if( (start_count > 200) || (delta_ta3 < -5) )
					{
						alpha += 720;
						if(alpha >= 1440)
							alpha -= 1440;
						motor.iq = motor.iq<<1;
						start_status = 2;
						m_count = 0;
					}
				}
				break;
		}
	}

	limit(motor.iq,I_MAX_O);                 //10A,5412

	temp16 = ((INT32)motor.iq*20)>> 10 ; //R  CURRENT_Q
#ifdef VOL_ADJUST
	//voltage adjust
	temp16 = ((INT32)temp16*DC300) / sys.uzk_val;   //Q12
#endif
	temp16 += 54;//scx 72;
	if(temp16 > 1200) //U_MAX
		vol = 1200;
	else
		vol = temp16;
	cal_forwardn(alpha, vol);
}


//--------------------------------------------------------------------------------------
//  Name:	m_start
//  Parameters:	None
//  Returns:	None
//  Description: motor start function
//--------------------------------------------------------------------------------------
void m_start(INT16 ta3_t)
{
    INT16 delta_ta3;
	INT16 temp16;
    INT16 Ud_r;
    INT16 Uq_r;
    INT16 i;
	

	if(abs(delta_l) >= ((ENCODER>>1) + DEGREE_20))
	{
		motor_stuck_flag = 1;//ERROR_14
	}
	
	zero_vol_counter ++;
	if(zero_vol_counter >= 2000)
		zero_vol_counter = 2000;
	m_count++;
//	da1 = ta3_start	>> 3;
	if(stop_change_counter <= 20)//400ms
	{	
		if(m_count >= 200)//20ms
		{
			m_count = 0;
			delta_ta3 = ta3 - ta3_last;
			ta3_last = ta3;
			if(delta_ta3 != 0)
			{
				if(delta_ta3 > 0)
					real_dir = 0;
				else if(delta_ta3 < 0)
					real_dir = 1;
			
				if(motor.dir ^ real_dir)
				{
					stop_change_counter ++;

					ta3_start = ta3_start + (M_CODER - 70);//70/4=17
				
					if(ta3_start >= ENCODER)
					{	
						ta3_start -= ENCODER;
					}
				}
				else
				{
					
					if((abs(delta_ta3) <= 2*ENCODER/360) && (zero_vol_counter >= 2000))//
					{
						stop_change_counter ++;
						ta3_start = ta3_start + (M_CODER >> 4);
						if(ta3_start >= ENCODER)
						{	
							ta3_start -= ENCODER;
						}
					}
				}
			}
			else
			{
				stop_change_counter ++;

				ta3_start = ta3_start + (M_CODER - 70);
				if(ta3_start >= ENCODER)
				{	
					ta3_start -= ENCODER;
				}

			}
			
		}
		else//<20ms
		{
			delta_ta3 = ta3 - ta3_last;

			if(motor.dir == 1 && delta_ta3 >= 2*ENCODER/360)//转向不对
			{
				stop_change_counter ++;
				ta3_last = ta3;
				m_count = 0;
				ta3_start = ta3_start + (M_CODER - 70);
				if(ta3_start >= ENCODER)
				{	
					ta3_start -= ENCODER;
				}
			}
			else if(motor.dir == 0 && delta_ta3 <= -2*ENCODER/360)
			{
				stop_change_counter ++;
				ta3_last = ta3;
				m_count = 0;
				ta3_start = ta3_start + (M_CODER - 70);
				if(ta3_start >= ENCODER)
				{	
					ta3_start -= ENCODER;
				}
			}
			//else?
		}
	}		
	cal_spd_ref(50,0,0);
	
	 motor.iq = cal_pid_s(0,30,16);//8);
	 
	 if(motor.iq >= 0)
		e_start = 40;
	else
		e_start = 6;	
	
	Ud_r = (((INT32)(motor.iq) * L *m_spd_n)>>(LQ_Q+CURRENT_Q)); 

	Uq_r = ((INT32)(motor.iq) * R>>CURRENT_Q) + (((INT32)(m_spd_n) * e_start)>>(KE_Q)) ;
		
    temp16 = ((INT32)Ud_r<<7)/Uq_r;

	if(temp16 > 0)
	{					
		while(abs(top - bottom) > error)
		{
			mid = (top + bottom);
			mid = mid>>1;
				
			if(temp16 > tan_tab[mid])
				bottom = mid;
			else
				top = mid;	
		}
		i = mid;
	}
	else
	{
		temp16 = abs(temp16);	
		while(abs(top - bottom) > error)
		{
			mid = (top + bottom);
			mid = mid>>1;
					
			if(temp16 > tan_tab[mid])
			    bottom = mid;
			else
			    top = mid;	
		}
			i = mid;
			i = -i;
	}
					
	if(motor.dir)
		i = -i;
  
     temp16 = (((abs(Ud_r))>>2) + abs(Uq_r));					
	if(Uq_r>0)
	{
		alpha = 0xff+i; 					
	}
	else
	{
		alpha = 0x2ff+i;
	}
		
	if(abs(motor.iq) > 2000)			
		temp16 += 90;        				    
	else if(abs(motor.iq) > 1000)			
		temp16 += 85;
	else if(abs(motor.iq) > 300)			
		temp16 += 80;
	else if(abs(motor.iq) > 100)
		temp16 += 75;
	else if(abs(motor.iq) > 50)
		temp16 += 65;
	else if(abs(motor.iq) > 10)
		temp16 += 20;
       
#ifdef VOL_ADJUST
			temp16 = ((INT32)(temp16*(INT32)DC300))/ (sys.uzk_val);   //Q12
#endif
          

	if(zero_vol_counter >= 20)
	{  
		if(temp16 > U_MAX)
			vol = U_MAX;
		else
			vol = temp16; 
	}	
	else
	{
		vol = 0;
	}		

	vol = (vol_last*7 + vol ) >> 3;
	vol_last = vol;		

}

//--------------------------------------------------------------------------------------
//  Name:	cal_spd_ref
//  Parameters:	None
//  Returns:	None
//  Description: calculate reference speed
//--------------------------------------------------------------------------------------

void cal_spd_ref(INT16 speed, INT16 acc_spd, INT16 dcc_spd)
{
	static INT16 W1max;
	static INT16 AlphaMax;   // Maximum acceleration
	static INT16 Wmax;       // Maximum speed according to the acceleration 
	static INT16 spd_tmp_test;
	static INT16 Dec;
	static INT16 Acc;
	//static UINT8 test = 0;
	
	//test ++;
	//da1 = test;
	if(speed < STOP_SPD)
	{
		motor.spd_ref = speed;
	}
	
	if(speed == motor.spd_ref)
    {
		if(motor.dir)
	        m_spd_ref_n = -motor.spd_ref<<1;   
	    else
	        m_spd_ref_n = motor.spd_ref<<1;
		spd_tmp = 0;
		TaccTmp = 0;
        return; //直接跳出函数
    }
		
			
	spd_tmp_test = abs(spd_tmp);
	
	if(spd_tmp < 0)
	{
	    if(speed == STOP_SPD)
	    {
			if(spd_tmp_test < 800)
				Dec = dectable[0]; 
			else if(spd_tmp_test < 1200)
				Dec = dectable[1]; 
			else if(spd_tmp_test < 1500 )
				Dec = dectable[2];
			else if(spd_tmp_test < 1800 )
				Dec = dectable[3];
			else if(spd_tmp_test < 2200 )
				Dec = dectable[4];
			else
				Dec = dectable[5];
	        Wmax = 1600;
	    }
		else if(speed < STOP_SPD)
	    {
	        Dec = dectable[1];
	        Wmax = 600;
	    }
		else
		{
			Wmax = 1800;
			if(speed < 800)
				Dec = dectable[2];
			else
				Dec = dectable[3];
		}
	}	
	else
	{
		if(speed < 800)
			Acc = acctable[0];
		else if(speed < 1800)
			Acc = acctable[1];
		else
			Acc = acctable[2];
		Wmax = 800;
	}
	
	if(spd_tmp_test > (Wmax<<1))
		W1max = spd_tmp_test - (Wmax<<1);
	else
	{
		W1max = 0;
		Wmax = spd_tmp_test>>1;
	}
 
	if(spd_tmp >= 1 || spd_tmp <= -10)
	{
		if(speed - motor.spd_ref != 0)
		{		
		    if(speed - motor.spd_ref > 1)
			{
				if(speed - motor.spd_ref >= W1max + Wmax)
				{
					TaccTmp = TaccTmp + Acc;
					
					//motor.spd_ref = motor.spd_ref + TaccTmp;
					//da1 = 50;
				}
				//else if(speed - motor.spd_ref < W1max + Wmax && speed - motor.spd_ref >= Wmax )
				{
				//	motor.spd_ref = motor.spd_ref + TaccTmp;
				}
				//else if(speed - motor.spd_ref < Wmax)
				if(speed - motor.spd_ref < Wmax)
				{
					TaccTmp = TaccTmp - Acc;
					if(TaccTmp <= 10)
					{
						TaccTmp = 3;
					}
											
				}
				motor.spd_ref = motor.spd_ref + TaccTmp;
					
				if((speed - motor.spd_ref) <= 3)
				{
					motor.spd_ref = speed;
					spd_tmp = 0;
					TaccTmp = 0;

				}
			}
			else if(speed - motor.spd_ref < -10)
			{
				if(speed - motor.spd_ref <= -(W1max + Wmax))
				{
					TaccTmp = TaccTmp - Dec;
					//motor.spd_ref = motor.spd_ref + TaccTmp;
				}
			/*	else if(speed - motor.spd_ref > -(W1max + Wmax) && speed - motor.spd_ref <= -Wmax )
				{
					motor.spd_ref = motor.spd_ref + TaccTmp;
				}*/
				//else if(speed - motor.spd_ref > -Wmax)
				if(speed - motor.spd_ref > -Wmax)
				{
					TaccTmp = TaccTmp + Dec;
					if(TaccTmp >= -2*Dec)
					{
						TaccTmp = -2*Dec;
					}
					
				}
				motor.spd_ref = motor.spd_ref + TaccTmp;
		
				if((motor.spd_ref - speed) <= 5)//
				{
					motor.spd_ref = speed;
					spd_tmp = 0;
					TaccTmp = 0;
				}
			}
			else
			{
				motor.spd_ref = speed;
			    spd_tmp = 0;
			    TaccTmp = 0;
			}
		}
		else
		{
			motor.spd_ref = speed;
			spd_tmp = 0;
			TaccTmp = 0;
		}
	}
	else
	{
		spd_tmp = speed - motor.spd_ref;//when there is no spd_tmp caculate before call 'cal_spd_ref()',it works
	}
		
	
    if(motor.dir)
        m_spd_ref_n = -motor.spd_ref<<1;
    else
        m_spd_ref_n = motor.spd_ref<<1;
		

}

void int0_int (void)//2530
{
	
	if((motor.angle < 25) || (motor.angle > 1000))
	{	
		ta3z_elec = ta3;
		ta3z_mach = ta3z_elec;		
		
		//-----------------------------------------------------------------------	
	    // modify for 210E 
	    //-----------------------------------------------------------------------	
	    zpl_pass = 1;                // ZPL have passed
	}
}

void int3_int (void)
{
	if((motor.angle < 120) || (motor.angle > 1200))
	{	
		ta3z_elec = (INT16)ta3;
		ta3z_mach = ta3z_elec;		
		//-----------------------------------------------------------------------	
	    // modify for 210E 
	    //-----------------------------------------------------------------------	
	    zpl_pass = 1;                // ZPL have passed
	}
}


//--------------------------------------------------------------------------------------
//  Name:	motor_start
//  Parameters:	None
//  Returns:	None
//  Description: start motor
//--------------------------------------------------------------------------------------
void motor_start(void)
{
	if(m_status >= OPEN_START)
		return;

	if(MAIN_MOTOR_TYPE==1)
	{
		l_ism = EXTSM;
	}
	else
	{
		l_ism = ISM;
	}
	
	if(l_ism)
		motor.dir = 1;
	else
		motor.dir = 0;

	U=1;U_=1;V=1;V_=1;W=1;W_=1;
	prcr = 0x02;
	inv03 = 1;
	prcr = 0x00;
	ta3_start = ta3;
	ta3_startdirect = ta3;
	
	m_status = OPEN_START;
	ta3_begin = ta3;

}


//--------------------------------------------------------------------------------------
//  Name:	holding_axes
//  Parameters:	None
//  Returns:	None
//  Description: holding_axes
//--------------------------------------------------------------------------------------
INT16 holding_axes(INT16 stop_angle, INT16 angle)
{
	INT16 angle_hold;


	//holding axes  m_coder  (m_coder<<1)  ((m_coder<<1)-1)
	if(stop_angle >= 0 && stop_angle <= ((M_CODER<<1)-1) )
		{
			if(angle >= 0 && angle <= (stop_angle + (M_CODER<<1)))
				{
					angle_hold = stop_angle - angle;
				}
			else//(angle <= 1023 && angle > (stop_angle + 512))
				{
					angle_hold =  (M_CODER<<2) - angle + stop_angle;
				}
			while(angle_hold > (M_CODER<<1))
				{
					angle_hold -= (M_CODER<<1);
				}
			//while(angle_hold < -((m_coder<<1)-1))
			while(angle_hold < -(M_CODER<<1))
				{
					angle_hold += (M_CODER<<1);
				}
		}
	else//(stop_angle >= 512 && stop_angle <= 1023)
		{
			if(angle > (stop_angle - (M_CODER<<1)) && angle <=((M_CODER<<2)-1))
				{
					angle_hold = stop_angle - angle;
				}
			else//(angle >= 0 && angle <= (stop_angle - 512))
				{
					angle_hold = stop_angle - (M_CODER<<2) - angle ;
				}
			while(angle_hold > (M_CODER<<1))
				{
					angle_hold -= (M_CODER<<1);
				}
			//while(angle_hold < -((m_coder<<1)-1))
			while(angle_hold < -(M_CODER<<1))
				{
					angle_hold += (M_CODER<<1);
				}

		}

	return angle_hold;

}


VECTOR rc_pc(INT16 h_y, INT16 h_x)//scx 反正切， 输出值 为 角度值的4倍。模值经过电压修正 
{
	INT16 h_yabs,h_xabs;
	INT16 alpha_out;
	INT8  U_sign;
	INT16 temp16;
	VECTOR vector_out;
	U_sign = 0;
	if(h_y >= 0)
	{
		h_yabs = h_y;			   
	}
	else
	{
		h_yabs = -h_y;
		U_sign |= 0x04;			   
	}
	
	if(h_x >= 0)
	{
		h_xabs = h_x;			   
	}
	else
	{
		h_xabs = -h_x;
		U_sign |= 0x02;			   
	}
				 
	if(h_y == 0)
	{
		alpha_out = 0;
		temp16 = abs(h_x);
	}
	else
	{

		if(h_yabs <= h_xabs)
		{
			temp16 = ((INT32)h_yabs<<7)/h_xabs;	  //计算正切值
			alpha_out = atan_tab[temp16];         //计算反正切值
			temp16 = ((INT32)h_xabs * csc_tab[alpha_out])>>14; //计算斜边值
		}
		else
		{
			temp16 = ((INT32)h_xabs<<7)/h_yabs;
			alpha_out = atan_tab[temp16];
			temp16 = ((INT32)h_yabs * csc_tab[alpha_out])>>14;	
			U_sign |= 0x01;
		}	
		
	}
	switch(U_sign)
	{ 
		//case 0: ;break;
		case 1: alpha_out =  360 - alpha_out;break;
		case 2: alpha_out =  720 - alpha_out;break;//x<0,y>=0
		case 3: alpha_out =  360 + alpha_out;break;
		case 4: alpha_out = 1440 - alpha_out;break;  //x>=0,y<0
		case 5: alpha_out = 1080 + alpha_out;break;
		case 6: alpha_out =  720 + alpha_out;break;  //x<0,y<0
		case 7: alpha_out = 1080 - alpha_out;break;		
	}
	vector_out.angle = alpha_out;
				
#ifdef VOL_ADJUST
	//voltage adjust
	//temp16 = ((INT32)(temp16*(INT32)DC300))/ (sys.uzk_val);   //Q12
	temp16 = ((INT32)temp16*a_uzkval)>>14;   //Q12  2^14=16384
#endif	   
	if(temp16 > 1200)//U_MAX)
		vector_out.mudulus = 1200;//U_MAX;
	else
		vector_out.mudulus = temp16;
		
	return vector_out;
}

//--------------------------------------------------------------------------------------
//  Name:	motor_control
//  Parameters:	None
//  Returns:	None
//  Description: motor control function
//--------------------------------------------------------------------------------------

UINT8 down_stage;//1 for up ,0 for down
UINT8 LineStage;
void motor_control(void)
{
    INT16 temp16,dir;
    INT16 i;
    INT16 Ud_r;
    INT16 Uq_r;
	INT16 iq_r;
	INT32 didt;
	INT16 object_speed;
	UINT8 quadrant_flag;
  	UINT32 iqtemp;
	UINT16 spdtemp;
	INT16 angletemp;
	UINT8 iqstep;
	UINT8 spdstep;
	static INT16 nn;
	static INT16 unlock_mainmotor;
	//static UINT8 testout = 0;

	INT16 Speed_0_32_Seg_Index;//scx
	INT16 Inductance_Value;//scx
	INT16 Resistance_Value;//scx
	
	INT16 Abs_m_spd_n;//scx	
	INT16 Uq_rn,Ud_rn,diq_temp;//scx
	INT16 d_iq = 0;//scx
	
	a_uzkval = ((INT32)620*16384)/sys.uzk_val;//scx

	
    top = 255;
	bottom = 0;
	//testout ++;
	//da1 = testout;
	//if()
	object_speed = motor.spd_obj;
	//da1 = object_speed >> 4;
	//da0 = motor.angle_adjusted >> 3;
	//da1 = m_spd_ref_n >> 5;//geiding
	//da1 = motor.spd_ref>>4;
	//da0 = iqtemp >> 4;
//	da1 = motor.spd_obj >> 4;
	//da1 = motor.iq >> 2;
//	da0 = m_spd_n >> 5;
//	   da1 = m_spd_n >> 5;//shishi
//	else 
//	   da1 = motor.spd_obj >> 4;
    //save the histroy 	
	if(object_speed - spd_last_value != 0)
	{
		spd_tmp = object_speed - motor.spd_ref;
	}	
	spd_last_value = object_speed;
	
    a_m_spd += abs(m_spd_n);		
	count_spd ++;
    if(count_spd >= 512)
    {
         count_spd = 0;
         motor.spd = (a_m_spd+512)>>10;
         a_m_spd = 0;
    }

	if(MAIN_MOTOR_TYPE==1)
	{
		if(m_status > OPEN_START)
		{
			if(m_status < HOLD)
			{
				if(object_speed > 0)
				{
					motor.stop_flag = 0;
					stop_status = 0;
					pwm_forbid_flag = 0;
			
					if(m_status == CLOSE_RUN)
				    {
					//scx
					   
						    if(motor.dir)
						        m_spd_ref_n = motor.spd_obj<<1;	//指令速度，及升速时的台阶指令速度如1500、2000
						    else
						        m_spd_ref_n = -motor.spd_obj<<1;
					//PI分段scx	
		
							order_spd_this	=	motor.spd_obj;
		
							err_order_spd_this_last = order_spd_this-order_spd_last;//abs( abs_m_spd_this - abs_m_spd_last );

							//2*（上次指令速度+（这次指令速度-上次指令速度）*0.9 ） 0.9=115/128
		
							if( err_order_spd_this_last > 0)	//升速
							{
						 
								tactic_allowed = 0;
	//
								if(abs(order_spd_this)<PID_UP_SEG_1)
								{
									Run_SpdIq.Kp = (INT8)KPspd_UP1; 
									Run_SpdIq.Ki = (INT8)KIspd_UP1;
									Run_SpdIq.Kd = (INT8)KDspd_UP1;
								}
								else
								{
									Run_SpdIq.Kp = (INT8)KPspd_UP2; 
									Run_SpdIq.Ki = (INT8)KIspd_UP2;
									Run_SpdIq.Kd = (INT8)KDspd_UP2;
							
								}

							
								order_spd_state_1up_2cst_3dwn = SPD_UP;
								order_spd_0_p_9	 = (2*order_spd_last)+ ( (err_order_spd_this_last*2*115)>>7 );	
		 					
							}
							else if(err_order_spd_this_last < 0)//降速
							{
								tactic_allowed = 0;
								if(abs(order_spd_this)<PID_DWN_SEG_1)
								{
									Run_SpdIq.Kp = (INT8)KPspd_DWN1; 
									Run_SpdIq.Ki = (INT8)KIspd_DWN1;
									Run_SpdIq.Kd = (INT8)KDspd_DWN1;
								}
								else
								{
									Run_SpdIq.Kp = (INT8)KPspd_DWN2; 
									Run_SpdIq.Ki = (INT8)KIspd_DWN2;
									Run_SpdIq.Kd = (INT8)KDspd_DWN2;
							
								}
						
								order_spd_state_1up_2cst_3dwn = SPD_DWN;
								order_spd_0_p_9	 = (2*order_spd_last)+ ( (err_order_spd_this_last*2*115)>>7 );
			 
							}
							else//容易满足，但速度如果没跟上来，仍旧保持up或dwn的P I参数
							{
								if( order_spd_state_1up_2cst_3dwn == SPD_UP ) 
								{
									 if( abs(m_spd_n) > order_spd_0_p_9 ) // >原来+0.9差值  只有速度跟上来且持续COSTANT_SPD_VALUE次，才切换到匀速
									 {
										constant_spd_cnt++;
										if( constant_spd_cnt >= COSTANT_SPD_VALUE ) //消抖
										{
											constant_spd_cnt=0;
					
	//										Run_SpdIq.Kp = (INT8)KPspd_CONSTANT; 
	//										Run_SpdIq.Ki = (INT8)KIspd_CONSTANT; 
	//										Run_SpdIq.Kd = (INT8)KDspd_CONSTANT;
											order_spd_state_1up_2cst_3dwn = SPD_CST;
					 					 
										}
									}
									else
									{
										constant_spd_cnt = 0;	 
									}							
								}
								else if( order_spd_state_1up_2cst_3dwn == SPD_DWN ) // 降速
								{
				 
								 
								 //避免由于降速期间出现的匀速与低速匀速时大穿透力PI参数干扰20161130	 
									 if( abs(m_spd_n) < order_spd_0_p_9 ) 		 // 	< 原来+0.9差值!!!scx
									 {
									
										constant_spd_cnt++;
										//da1=120+2*constant_spd_cnt;
									
									
										if( constant_spd_cnt >= COSTANT_SPD_VALUE ) //消抖
										{
											constant_spd_cnt=0;
					
											motor.spd_ref = motor.spd_obj;
											// 屏蔽该语句是因为打算由 KPspd_DWN1 来控制降速及降速期间的匀速性能。order_spd_state_1up_2cst_3dwn = SPD_CST;
						 			 
										}
					 
									 }
									 else
									 {
											constant_spd_cnt = 0;
									 }
							 
						   
					
								}
								else	//匀速
								{
								
									if(abs(order_spd_this)<PID_CST_SEG_1)
									{

										Run_SpdIq.Kp = (INT8)KPspd_CONSTANT1; 
										Run_SpdIq.Ki = (INT8)KIspd_CONSTANT1; 
										Run_SpdIq.Kd = (INT8)KDspd_CONSTANT1;
									}
									else 
									{
										Run_SpdIq.Kp = (INT8)KPspd_CONSTANT2; 
										Run_SpdIq.Ki = (INT8)KIspd_CONSTANT2; 
										Run_SpdIq.Kd = (INT8)KDspd_CONSTANT2;
									
									}
								
								 	order_spd_state_1up_2cst_3dwn = SPD_CST;
									constant_spd_cnt = 0;
									motor.spd_ref = motor.spd_obj;
				 
								}
							}

							order_spd_last = order_spd_this;// 
					//PI分段scx						
							//scx
							Run_SpdIq.SetSpeed    = m_spd_ref_n;
							Run_SpdIq.ActualSpeed = m_spd_n;
						
							Run_SpdIq =	Spd_Iq_PID( Run_SpdIq );
							motor.iq  = ((Run_SpdIq.Sum_Iq)>>13);
					
				    }
					else
					{
						m_status = CLOSE_RUN;
						U=1;U_=1;V=1;V_=1;W=1;W_=1;
			   			prcr = 0x02;
					    inv03 = 1;
			   			prcr = 0x00;
					}
	 			}
				else 
				{
					tactic_allowed = 0;
					if(m_status == CLOSE_RUN)
					{

						switch(stop_status)
						{
				        
								case 0:  
		//scx
								Run_SpdIq.Kp = (INT8)KPspd1_CUT_LINE;//UserPara.start3rdStitchingFeedingSynModify_263 ;	//KPspd_DWN; 
								Run_SpdIq.Ki = (INT8)KIspd1_CUT_LINE;//UserPara.first3StitchingFeedingSynModifyAngle_269 ;//KIspd_DWN;
								Run_SpdIq.Kd = (INT8)KDspd1_CUT_LINE;
							 
								if(motor.spd_ref <= DEC_SPD)//<=800 速度实际是400 进入case 2  如果剪线阶段速度稳定性不好，就会在上阶段的motor.spd_ref不更新从而按800进入
					        	{
								    spd_tmp = STOP_SPD;		//=200
						        	stop_status = 2;		//
					        	}
					        	else						//>800
					        	{
					        		spd_tmp = DEC_SPD;		//=800
									stop_status = 1;		//进入case 1
								}
						 
								if(motor.dir)
							        m_spd_ref_n = spd_tmp<<1;	//按指令速度给PI参数  // 及升速时的台阶指令速度如1500、2000
							    else
							        m_spd_ref_n = -spd_tmp<<1;
				
								Run_SpdIq.SetSpeed    = m_spd_ref_n;
								Run_SpdIq.ActualSpeed = m_spd_n;

								Run_SpdIq =	Spd_Iq_PID( Run_SpdIq );
								motor.iq  = ((Run_SpdIq.Sum_Iq)>>13);
		//scx						
									unlock_mainmotor  = 0;
									break;
							
								case 1:  
		//scx							
									if(motor.dir)
								        m_spd_ref_n = STOP_SPD<<1;	//指令速度，及升速时的台阶指令速度如1500、2000
								    else
								        m_spd_ref_n = -STOP_SPD<<1;
				
									Run_SpdIq.SetSpeed    = m_spd_ref_n;
									Run_SpdIq.ActualSpeed = m_spd_n;
				
									Run_SpdIq.Kp = (INT8)KPspd2_CUT_LINE; 
									Run_SpdIq.Ki = (INT8)KIspd2_CUT_LINE; 
									Run_SpdIq.Kd = (INT8)KDspd2_CUT_LINE; 
									Run_SpdIq 	 =	Spd_Iq_PID( Run_SpdIq );
									motor.iq     = ((Run_SpdIq.Sum_Iq)>>13);
		//scx							
		//匀速条件判断
								order_spd_0_p_9	 = (2*motor.spd_ref)+ ( ( ( m_spd_ref_n - 2*motor.spd_ref) *115) >>7 );
								//发停车指令速度之前的指令速度motor.spd_ref,给定的停车指令速度m_spd_ref_n
								if( abs(m_spd_n) < order_spd_0_p_9 ) 		 // 	< 原来+0.9差值
								{
									constant_spd_cnt++;
							
									if( constant_spd_cnt >= COSTANT_SPD_VALUE ) //消抖
									{
										constant_spd_cnt=0;

										if(	motor.angle_adjusted >= (4*240) )
										{
											stop_status = 2;		//
										}
									}
 
								}
								else
								{
									constant_spd_cnt = 0;	 
								}
		//匀速条件判断
									break;
								case 2: 
						
									motor.angle_hold = holding_axes(motor.stop_angle, motor.angle_adjusted);

									if(abs(motor.angle_hold) <= 40)//scx 改为40，停车角度为53，否则一直54、55 temp16)//2010-4-7//20150727ANGLE_P
									{
										if(motor.angle_hold > 0)//2010-4-7
											kk = 1;//(stop_angle-adjust) > 40:不到停止角度
										else
											kk = -1;//(stop_angle-adjust)< 40：超过了停止角度
										motor.stop_angle1 = motor.angle_adjusted  + 4 * kk;//motor.stop_angle1 = motor.angle_adjusted  - 40 * kk;
										stop_status = 3;
										m_spd_ref_n = 0;
										m_count = 0;
									

									}
		//scx
									if(motor.dir)
								        m_spd_ref_n = STOP_SPD<<1;	//指令速度，及升速时的台阶指令速度如1500、2000
								    else
								        m_spd_ref_n = -STOP_SPD<<1;
					
									Run_SpdIq.SetSpeed    = m_spd_ref_n;
									Run_SpdIq.ActualSpeed = m_spd_n;
					
									Run_SpdIq.Kp = (INT8)KPspd2_CUT_LINE; 
									Run_SpdIq.Ki = (INT8)KPspd2_CUT_LINE; 
									Run_SpdIq.Kd = (INT8)KPspd2_CUT_LINE; 
			
									Run_SpdIq =	Spd_Iq_PID( Run_SpdIq );
									motor.iq  = ((Run_SpdIq.Sum_Iq)>>13);
		//scx							
									break;
						
								case 3: 
							
									motor.angle_hold = holding_axes(motor.stop_angle1, motor.angle_adjusted);
									compensation_flag = 1;
									m_count++;
									if(m_count < 80)//150)
									{
										motor.iq = cal_pid_p_stopn((STOP_KPp_1),(STOP_KPs_1 + 5),0);
									}
									else
									{
										motor.iq = cal_pid_p_stopn(0,0,0);
									}
									//motor.iq = cal_pid_p_stop(STOP_KPp_2,STOP_KPs_2,0);
									if( m_count > 40)  //停车标志位给出前的延时
									{
									
										if(sys.status == RUN || sys.status == PREWIND || sys.status == WIND)
										{
											motor.stop_flag = 1;
											motor.spd_ref = 0;
										
										
											if(motorconfig_flag == 1)
											{
												if(inpress_flag == 0)
												{
													MotorPositionSet = 1;
												}
												else if(inpress_flag == 1)
												{
													MotorPositionSet = 0;
												}
											}
											else
											{
												MotorPositionSet = 0;
											}
										}
										else
										{
											motor.stop_flag = 1;
											motor.spd_ref = 0;
											MotorPositionSet = 0;
										
										}
								
								
									}
							
									if((MotorPositionSet == 0) && (motor.stop_flag == 1) && (m_count >= 80))//400))
									{
										U=1;
										U_=1;
										V=1;
										V_=1;
										W=1;
										W_=1;
										prcr = 0x02;
										inv03 = 0;
										prcr = 0x00;
										U_=0;
										V_=0;
										W_=0;
										stop_status = 5;
										TaccTmp = 0;
										s_count = 0;
										motor.spd_ref = 0;
										m_status = STOP;
										motor.iq = 0;
										spider_man = 0;
										spider_lim = 0;
								
										Run_SpdIq.Err_Last=0;	//scx
										Run_SpdIq.Err_LLast=0;	//scx
										order_spd_last=0;		//scx
								
									}
									break;
									case  4://for motor hold
										//motor.iq = cal_pid_p_stop(PI_Pre.kpp_stop31,PI_Pre.kps_stop31,0);
										//if(m_count < 30)
										//	current_response_speed = 2;
									break;
							}
				
					}
					else 
					{
						m_pos_ref_add = motor.angle_adjusted<<6;
						m_pos_ref = m_pos_ref_add >> 6;
						motor.stop_flag = 1;
						stop_status = 4;
					
					
					}
				}
			}
		
			Abs_m_spd_n = abs(m_spd_n);

			Speed_0_32_Seg_Index = ( Abs_m_spd_n >>8 );//速度段

			if( Speed_0_32_Seg_Index >= 32 )
			{
				Resistance_Value = Resistance_tab[32];
				Inductance_Value = Inductance_tab[32];
			}
			else
			{
				//A=X>>8
				//R=Y[A+1]-Y[A]/256 *(X-A);
				//两点直线方程（x,y）在（x1,y1）和（x2,y2）中间（x1<x2）
						// (y-y1)/(x-x1)==(y2-y1)/(x2-x1)也即 y=(y2-y1)*(x-x1)/(x2-x1)+ y1
				Resistance_Value = ((( Resistance_tab[Speed_0_32_Seg_Index+1] - Resistance_tab[Speed_0_32_Seg_Index] )* ( Abs_m_spd_n - (Speed_0_32_Seg_Index<<8))	)>>8) +Resistance_tab[Speed_0_32_Seg_Index];
				Inductance_Value = ((( Inductance_tab[Speed_0_32_Seg_Index+1] - Inductance_tab[Speed_0_32_Seg_Index] )* ( Abs_m_spd_n - (Speed_0_32_Seg_Index<<8)) 	)>>8) +Inductance_tab[Speed_0_32_Seg_Index];
			}
	//		Current_A=(3*238*Current_A_cnt);

	//	motor.iq = 0;//Current_A;//Current_A ;//dhd  1/0.0014  -(INT16)
	//SCX pid		

			d_iq = ( motor.iq - motor.iq_last );//iq 这次变换量
		
			diq_temp = d_iq +d_iq_last;//iq 实际变换值=【上次变换量 】+ 【iq这次变换量】
		
			Ud_i += ((INT32)diq_temp); //Ud=R*id-w*L*iq，我的理解正常情况下id是要为0的， 
			Uq_i += ((INT32)diq_temp);//modified by zz on 20100311

			Ud_rn = ( (INT32)Ud_i  * Inductance_Value * m_spd_n)	>> ( CURRENT_Qn + LQ_Qn +1);//scx对于电感Inductance和以前相比扩大了一倍所以左移也多加了一

			if((order_spd_state_1up_2cst_3dwn == SPD_CST)&&(motor.spd_obj >1000))
			{
				Uq_rn = ( ( (INT32)Uq_i * Resistance_Value ) >> (CURRENT_Qn +1) ) + ( ( (INT32)(m_spd_n) * en) >> KE_Qn );// + (((INT32)d_iq * Inductance_Value * 5 )>>10);
			}
			else
			{
				Uq_rn = ( ( (INT32)Uq_i * Resistance_Value ) >> (CURRENT_Qn +1) ) + ( ( (INT32)(m_spd_n) * en) >> KE_Qn ) + (((INT32)d_iq * Inductance_Value * 5 )>>10);
			}

			v_vector = rc_pc(Ud_rn,Uq_rn);
		
			motor.iq_last = motor.iq;		//将iq值赋值给上次iq值	 
			d_iq_last = d_iq;				//将iq这次变换量赋值给上次变换量
				
		}
		else if(m_status < OPEN_START)
		{
			if(object_speed > 0)
			{	
				U=1;U_=1;V=1;V_=1;W=1;W_=1;
				prcr = 0x02;
				inv03 = 1;
				prcr = 0x00;
				m_status = OPEN_START;
			}
			m_pos_ref_add = motor.angle_adjusted<<6;
			m_pos_ref = m_pos_ref_add >> 6;
		}
		else
		{
			m_pos_ref_add = motor.angle_adjusted<<6;
			m_pos_ref = m_pos_ref_add >> 6;
		}
	}
	else
	{
		if(m_status > OPEN_START)
		{
			if(object_speed > 0)
			{
				motor.stop_flag = 0;
				stop_status = 0;
				pwm_forbid_flag = 0;
			
				if(DIDT_SPEED_RANGE == 0)
				{
					if(m_spd_ref_n <= 2000)
						current_response_speed = 4;
					else
						current_response_speed = 0;
				}
				else
				{
					current_response_speed = 0;
				}
				if(m_status == CLOSE_RUN)
			    {
					LineStage = 0;
						if((object_speed != motor.spd_ref))
						{
					
							cal_spd_ref(object_speed, motor.acc, motor.dec);
							s_count = 0;
						
		               		if (spd_tmp > 0)
							{
					
								down_stage = 1;
						      	if (m_spd_ref_n < 1500)
							    {	
									tactic_allowed = 0;
									motor.iq = cal_pid_s(0, PI_Pre.kp_up_l, PI_Pre.ki_up_l);  
							    }
								else if(m_spd_ref_n < 3000)
								{
									tactic_allowed = 0;
									motor.iq = cal_pid_s(0, PI_Pre.kp_up_m, PI_Pre.ki_up_m); 
								}
							    else
							    {	
									tactic_allowed = 0;
								    motor.iq = cal_pid_s(0, PI_Pre.kp_up_h, PI_Pre.ki_up_h); 
								}
							}
							else if (spd_tmp < 0)
							{
							
								down_stage = 0;
								if (m_spd_n <= 2000)
							  	{
								  	tactic_allowed = 0;
									motor.iq = cal_pid_s(0, PI_Pre.kp_down_l, PI_Pre.ki_down_l); 
						        }
						    	else
						        {	
								   	tactic_allowed = 0;
					 				motor.iq = cal_pid_s(0, PI_Pre.kp_down_h, PI_Pre.ki_down_h);
								 }
			 				}
						}
						else//object_speed != motor.spd_ref
						{
						
							cal_spd_ref(object_speed, motor.acc, motor.dec);
							if(s_count >= 30)
							{
								LineStage = 1;
								if(m_spd_ref_n <= 2000)
									tactic_allowed = 1;
								else
									tactic_allowed = 0;
								
								if (m_spd_ref_n < 1500)
								{	
									motor.iq = cal_pid_s(0, PI_Pre.kp_l, PI_Pre.ki_l);								
								}
							    else if(m_spd_ref_n < 3000)
								{	
									motor.iq = cal_pid_s(0, PI_Pre.kp_m, PI_Pre.ki_m);
								}
								else
								{	
									motor.iq = cal_pid_s(0, PI_Pre.kp_h, PI_Pre.ki_h);
									//motor.iq = pdff_control(20,4,40);
								}
							
							}
							else
							{
								s_count++;
							 	if(down_stage) //up
								{  
								
									if (m_spd_ref_n < 1500)
									{
										tactic_allowed = 0;
										motor.iq = cal_pid_s(0 , PI_Pre.kp_transup_l, PI_Pre.ki_transup_l); //reach objtect speed for 50ms

									}
								    else if(m_spd_ref_n < 3000)
									{
				   		     			tactic_allowed = 0;
				   		    			motor.iq = cal_pid_s(0,  PI_Pre.kp_transup_m, PI_Pre.ki_transup_m); //reach objtect speed for 50ms    	
									}
								    else
									{
				   		     			tactic_allowed = 0;
				   		    			motor.iq = cal_pid_s(0,  PI_Pre.kp_transup_h, PI_Pre.ki_transup_h);//reach objtect speed for 50ms    	
									}
								}
								else//down
								{
									if (m_spd_n < 2000)
									{
										tactic_allowed = 0;
										motor.iq = cal_pid_s(0, PI_Pre.kp_transdown_l, PI_Pre.ki_transdown_l); //reach objtect speed for 50ms
									}
								    else
									{
				   		     			tactic_allowed = 0;
				   		    			motor.iq = cal_pid_s(0, PI_Pre.kp_transdown_h, PI_Pre.ki_transdown_h); //reach objtect speed for 50ms    	
									}
								}
							}
						}
				
					
			    }
				else
				{
					m_status = CLOSE_RUN;
					U=1;U_=1;V=1;V_=1;W=1;W_=1;
		   			prcr = 0x02;
				    inv03 = 1;
		   			prcr = 0x00;
				}
			}
			else 
			{
				current_response_speed = 0;
				tactic_allowed = 0;
				if(m_status == CLOSE_RUN)
				{
					//da0 = 128;
	//				da1 = m_spd_n >> 3;
					switch(stop_status)
					{
				        
							case 0:
					        	if(find_deadpoint_flag == 0)
								{
						        	if(motor.spd_ref <= 800)
						        	{
										if(motor.spd_ref > STOP_SPD && RotateFlag == 0)
										{
											if(motor.angle_adjusted >= DEGREE_165 && motor.angle_adjusted <= DEGREE_202)
											{
												spd_tmp = STOP_SPD - motor.spd_ref;
								        		cal_spd_ref(STOP_SPD, motor.acc, motor.dec);
								        		stop_status = 2;
											}//else case1
										}
										else
										{
											spd_tmp = STOP_SPD - motor.spd_ref;
								        	cal_spd_ref(STOP_SPD, motor.acc, motor.dec);
								        	stop_status = 2;
										}
						        	}
						        	else
						        	{
						        		if(motor.angle_adjusted > DEGREE_202)
										{
											spd_tmp = DEC_SPD - motor.spd_ref;
											cal_spd_ref(DEC_SPD, motor.acc, motor.dec);
											stop_status = 1;	
										}
									}
									motor.iq = cal_pid_s(0, PI_Pre.kp_stop0, PI_Pre.ki_stop0);
								}
								else
								{
									cal_spd_ref(DEADPOINT_SPD, motor.acc, motor.dec);
									stop_status = 2;
									motor.iq = cal_pid_s(0, PI_Pre.kp_stop, PI_Pre.ki_stop);
								}
							
								//da0 = 50;
								unlock_mainmotor  = 0;
								break;
							case 1:
								//da1 = 50;
								if(motor.spd_ref <= 800)
								{
									if(jiting_flag == 1)
									{
										if(motor.angle_adjusted >= DEGREE_165 && motor.angle_adjusted <= DEGREE_320)
										{
											spd_tmp = STOP_SPD - motor.spd_ref;
										    cal_spd_ref(STOP_SPD, motor.acc, motor.dec);
										    stop_status = 2;
											motor.iq = cal_pid_s(0, PI_Pre.kp_stop1, PI_Pre.ki_stop1);
										}
										else
										{
											cal_spd_ref(DEC_SPD, motor.acc, motor.dec);
											motor.iq = cal_pid_s(0, PI_Pre.kp_stop1, PI_Pre.ki_stop1);
										}
									}
									else
									{
										if(motor.angle_adjusted >= DEGREE_165 && motor.angle_adjusted <= DEGREE_202)
										{
											spd_tmp = STOP_SPD - motor.spd_ref;
										    cal_spd_ref(STOP_SPD, motor.acc, motor.dec);
										    stop_status = 2;
											motor.iq = cal_pid_s(0, PI_Pre.kp_stop1, PI_Pre.ki_stop1);
										}
										else
										{
											cal_spd_ref(DEC_SPD, motor.acc, motor.dec);
											motor.iq = cal_pid_s(0, PI_Pre.kp_stop1, PI_Pre.ki_stop1);
										}
									}
					        	}
								else
								{
									cal_spd_ref(DEC_SPD, motor.acc, motor.dec);
									motor.iq = cal_pid_s(0, PI_Pre.kp_l, PI_Pre.ki_l);
								}
					        	break;	
							case 2:
								//da1 = 128;
								//holding axes  M_CODER  (M_CODER<<1)  ((M_CODER<<1)-1)
								if(motor.stop_angle >= 0 && motor.stop_angle <= ((M_CODER<<1)-1) )
								{
									if(motor.angle_adjusted >= 0 && motor.angle_adjusted <= (motor.stop_angle + (M_CODER<<1)))
									{
										motor.angle_hold = motor.stop_angle - motor.angle_adjusted;
									}
									else//(motor.angle_adjusted <= 1023 && motor.angle_adjusted > (motor.stop_angle + 512))
									{
										motor.angle_hold =  (M_CODER<<2) - motor.angle_adjusted + motor.stop_angle;
									}
								
								}
								else//(motor.stop_angle >= 512 && motor.stop_angle <= 1023)
								{
									if(motor.angle_adjusted > (motor.stop_angle - (M_CODER<<1)) && motor.angle_adjusted <=((M_CODER<<2)-1))
									{
										motor.angle_hold = motor.stop_angle - motor.angle_adjusted;
									}
									else//(motor.angle_adjusted >= 0 && motor.angle_adjusted <= (motor.stop_angle - 512))
									{
										motor.angle_hold = motor.stop_angle - (M_CODER<<2) - motor.angle_adjusted ;
									}
							
								}
							
								while(motor.angle_hold > (M_CODER<<1))
								{
									motor.angle_hold -= (M_CODER<<1);
								}
								while(motor.angle_hold < -((M_CODER<<1)-1))
								{
									motor.angle_hold += (M_CODER<<1);
								}
							
								if(motor.dir)
								{
									temp16 = -motor.angle_hold;
								}
								else
								{
									temp16 = motor.angle_hold;
								}
							
								if(find_deadpoint_flag == 0)
								{
									if(STOP_SPD <= 150)
										angletemp = 28;//10;//20 							
									else if(STOP_SPD <= 240)//200
										angletemp = 32;//14;//20 							    
									else if(STOP_SPD <= 300)
										angletemp = 43;//25;//20  
									else if(STOP_SPD <= 400) 
										angletemp = 53;//35;//38
									else
										angletemp = 56;//38;//49  38  43
								}
								else
								{
									angletemp = 12;//10;//20
								}
								temp16 = abs(motor.angle_hold);
								if(temp16 <= angletemp && temp16 > 0)//2010-4-7//20150706,when cut speed is over 400rpm ,the angle run in case 3 should be more big --by li
								{ 
								 	if(motor.angle_hold > 0)//2010-4-7
								 		kk = 1;
								 	else
										kk = -1;
									angletemp = angletemp - temp16;
									angletemp = angletemp * 3;
									/*
									if(STOP_SPD < 300)
										angletemp += 5;
									else if (STOP_SPD < 400)
									 	angletemp += 35; 
									else
										angletemp += 32;
									*/	
									motor.stop_angle1 = motor.angle_adjusted + kk;//4*kk;
									motor.iq = cal_pid_p_stop(PI_Pre.kpp_stop31,PI_Pre.kps_stop31,0);
									// motor.stop_angle1 = motor.stop_angle;
									stop_status = 3;
									m_spd_ref_n = 0;  
									m_count = 0;
									compensation_flag = 0;
									jiting_flag = 0;
														
								}
							
								if(find_deadpoint_flag == 0)
								{
									current_response_speed = DIDT_CUT_THREAD;
					
									cal_spd_ref(STOP_SPD, motor.acc, motor.dec);

								}
								else
								{
									current_response_speed = DIDT_CUT_THREAD;
									cal_spd_ref(DEADPOINT_SPD, motor.acc, motor.dec);
								}
								if(find_deadpoint_flag == 0)//
								{
					
									if(STOP_SPD <= 150)//300
										motor.iq = cal_pid_s(0, (PI_Pre.kp_stop2 + 30), (PI_Pre.ki_stop2 + 20)); //17 , 20
									else
										motor.iq = cal_pid_s(0, (PI_Pre.kp_stop2 + 17), (PI_Pre.ki_stop2 + 20)); //17 , 20
								}
								else
								{
									motor.iq = cal_pid_s(0, PI_Pre.kp_stop, PI_Pre.ki_stop);//KPs_deadpoint, KIs_deadpoint);
									//da0 = 200;
								}

								break;
						
							case 3:
								//current_response_speed = DIDT_CUT_THREAD3;
								nn = abs(motor.stop_angle - motor.angle_adjusted);
							//	da0 = motor.stop_angle1;
								//da1 = nn*4 + 128;
								//da1 = motor.iq >> 5;
								//da0 = 
								//da0 = 150;
								m_count++;
				
							
								if(m_count < 150)  
								{
									if(STOP_SPD <= 180)
									   motor.iq = cal_pid_p_stop((PI_Pre.kpp_stop31),PI_Pre.kps_stop31,0);
									else
									motor.iq = cal_pid_p_stop(PI_Pre.kpp_stop31,PI_Pre.kps_stop31,0);
									if(m_count < 30)
										current_response_speed = 3;
									else
										current_response_speed = 0;
								}
								else
								{
									motor.iq = cal_pid_p_stop(0,0,0);
								}
								if(m_count > 50)
								{
								
									motor.stop_flag = 1;
									motor.spd_ref = 0;	
									if(sys.status == RUN || sys.status == PREWIND || sys.status == WIND)
									{
								
							
										if(motorconfig_flag == 1)
										{
											if(inpress_flag == 0)
											{
												MotorPositionSet = 1;
											}
											else if(inpress_flag == 1)
											{
												MotorPositionSet = 0;
											}
										}
										else
										{
											MotorPositionSet = 0;
										}
									}
									else
									{
										//motor.stop_flag = 1;
										//motor.spd_ref = 0;
										MotorPositionSet = 0;
									}
								}
						
								if(MotorPositionSet == 0 && motor.stop_flag == 1 && m_count >= 400)
								{
									m_count = 400;
									if(1)//unlock_mainmotor == 1)
									{								
										U=1;U_=1;V=1;V_=1;W=1;W_=1;
						    			prcr = 0x02;
									    inv03 = 0;
										prcr = 0x00;	    			
										U_=0;V_=0;W_=0;
										stop_status = 5;
										TaccTmp = 0;
										s_count = 0;
										motor.spd_ref = 0;
										m_spd_ref_n = 0;
										m_status = STOP;
										motor.iq = 0;
										spider_man = 0;
										spider_lim = 0;
		//								da0 = 0;
									}
									else
									{
										if(motor.iq > LOCK_IQ)
										{
											motor.iq = LOCK_IQ;
										}
										else if(motor.iq < -LOCK_IQ)
										{
											motor.iq = -LOCK_IQ;
										}
										if(abs(nn) > LOCK_THETA)
										{
											unlock_mainmotor = 1;
										}
									}
								}
								break;
								case  4://for motor hold
									//motor.iq = cal_pid_p_stop(PI_Pre.kpp_stop31,PI_Pre.kps_stop31,0);
									//if(m_count < 30)
									//	current_response_speed = 2;
								break;
						}
				
				}
				else 
				{
					m_pos_ref_add = motor.angle_adjusted<<6;
					m_pos_ref = m_pos_ref_add >> 6;
					motor.stop_flag = 1;
					stop_status = 4;
				}
			}

			if((m_spd_n >= 0 && motor.iq >= 0) || (m_spd_n <=0 && motor.iq <= 0))//motor run in 1st & 3rd quadrant;
				quadrant_flag = 1;
			if((m_spd_n > 0 && motor.iq < 0) || (m_spd_n <0 && motor.iq > 0))//motor run in 2nd & 4th quadrant;
				quadrant_flag = 0;

			/*if(LineStage)
			{
				didt = motor.iq + motor.iq_last;//79Hz
				motor.iq = didt >>1;
			}*/
			//da1 = abs();
	//------------------360ppr-750w------------------------//
			iqtemp = abs(motor.iq);
			spdtemp = abs(m_spd_n);
	//		da1 = spdtemp >> 5;
			//da1 = motor.iq >> 6;
			if(MAIN_MOTOR_TYPE==2)
			{
				if(motor.iq >= 0)
				{
					iqstep = iqtemp / 345;
					if(iqstep > 28)
						iqstep = 28;
	               	spdstep = spdtemp / 600;
					if( spdstep > 9)
						spdstep = 9;
					if((motor.spd_ref >= 2300 && motor.spd_ref <= 2500) && (s_count >= 30) )
					{
						spdtemp = abs(m_spd_ref_n);
						e = 36;//36
					}
					else
					{
						e = etablenew1[iqstep][spdstep];
					}
				}
				else//iq<0
				{
					iqstep = iqtemp / 690;
					if(iqstep > 9)
						iqstep = 9;
				
					spdstep = spdtemp / 1000;
				
					if(spdstep > 5)
						spdstep = 5;
					
					e = etablenew2[iqstep][spdstep];
					//da1 = 100;
				}
			//	e= oldmotoretable(m_spd_n,motor.iq,quadrant_flag);	
				
	
			}//老电机
			else//Iq<0
			{
				e = oldmotoretable(m_spd_n,motor.iq,quadrant_flag);			
		
			}
		
			didt = (INT32)(motor.iq-motor.iq_last);			

			if(didt >= 0)
			{
				if(didt > DERV)
					didt = DERV;
			}
			else
			{
				if(tactic_flag == 0)
				{
					didt = 0;
				}
				else
				{
					if(didt < (-DERV))
						didt = (-DERV);
				}
			}	
		
			Ud_r = (((INT32)(motor.iq) * l*m_spd_n)>>(LQ_Q+CURRENT_Q)); 
			if( stop_status == 2)
				Uq_r = ((INT32)(motor.iq) * r>>CURRENT_Q) + (((INT32)(m_spd_ref_n) * e)>>(KE_Q)) + (((((INT32)didt*l*5)>>9)*current_response_speed)>>2);
			else
		    	Uq_r = ((INT32)(motor.iq) * r>>CURRENT_Q) + (((INT32)(m_spd_n) * e)>>(KE_Q)) + (((((INT32)didt*l*5)>>9)*current_response_speed)>>2);//modified by zz on 20100311
		

				motor.iq_last = motor.iq;	
		   
			    temp16 = ((INT32)Ud_r<<7)/Uq_r;

				if(temp16 > 0)
				{					
					while(abs(top - bottom) > error)
					{
						mid = (top + bottom);
						mid = mid>>1;
					
						if(temp16 > tan_tab[mid])
						    bottom = mid;
						else
						    top = mid;	
				    }
				    i = mid;
				}
				else
				{
					temp16 = abs(temp16);	
					while(abs(top - bottom) > error)
					{
						mid = (top + bottom);
						mid = mid>>1;
					
						if(temp16 > tan_tab[mid])
						    bottom = mid;
						else
						    top = mid;	
					}
					i = mid;
					i = -i;
				}
					
				if(motor.dir)
					i = -i;
       
	        temp16 = (((abs(Ud_r))>>2) + abs(Uq_r));
					
			if(Uq_r>0)
			{
				alpha = 0xff+i;
			}
			else
			{
				alpha = 0x2ff+i;
			}
		
				if(abs(motor.iq) > 2000)			
					temp16 += 90;        				    
				else if(abs(motor.iq) > 1000)			
					temp16 += 85;
				else if(abs(motor.iq) > 300)			
					temp16 += 80;
				else if(abs(motor.iq) > 100)
					temp16 += 75;
				else if(abs(motor.iq) > 50)
					temp16 += 65;
				else if(abs(motor.iq) > 10)
					temp16 += 50;
				//else if(abs(motor.iq) > 10)
					//temp16 += 20;
				/*if(Ud_r > 0)
					temp16 += 60;
				if(Ud_r < 0)
			   		temp16 += 60;*/
				
	#ifdef VOL_ADJUST
			    //voltage adjust
				temp16 = ((INT32)(temp16*(INT32)DC300))/ (sys.uzk_val);   //Q12
	#endif
          
		   
				if(temp16 > U_MAX)
					vol = U_MAX;
				else
					vol = temp16; 
				
		}
		else if(m_status < OPEN_START)
		{
			if(object_speed > 0)
			{	
				U=1;U_=1;V=1;V_=1;W=1;W_=1;
				prcr = 0x02;
				inv03 = 1;
				prcr = 0x00;
				m_status = OPEN_START;
			}
			m_pos_ref_add = motor.angle_adjusted<<6;
			m_pos_ref = m_pos_ref_add >> 6;
		}
		else
		{
			m_pos_ref_add = motor.angle_adjusted<<6;
			m_pos_ref = m_pos_ref_add >> 6;
		}
	
		
		
	}
	
	
//	da1scx=(abs(m_spd_n)>>5);
//	da1scx=motor.angle_adjusted>>2;
//	da1scx=abs(m_spd_ref_n)>>6;//motor.spd_obj>>5;	


}
UINT8 oldmotoretable(INT16 spd,INT16 iq,UINT8 quadrant)
{
	UINT16 spdtemp;
	UINT16 iqtemp;
	UINT8 etemp;
	spdtemp = abs(spd);
	iqtemp = abs(iq);
	if(spdtemp < 250)
	{
			if(quadrant)
			{
				if(iqtemp < 70)
					etemp = 27;
				else if(iqtemp < 90)
					etemp = 28;
				else if(iqtemp < 120)
					etemp = 29;
				else if(iqtemp < 160)
					etemp = 30;
				else if(iqtemp < 200)
					etemp = 31;
				else if(iqtemp < 300)
					etemp = 32;
				else if(iqtemp < 400)
					etemp = 33;
				else if(iqtemp < 450)
					etemp = 34;
				else if(iqtemp < 650)
					etemp = 35;	
				else if(iqtemp < 850)
					etemp = 37;
				else if(iqtemp < 4050)
					etemp = 35;
				else if(iqtemp < 6000)
					etemp = 36;
				else if(iqtemp < 7000)
					etemp = 38;
				else if(iqtemp < 7500)
					etemp = 39;
				else if(iqtemp < 7800)
					etemp = 40;
				else if(iqtemp < 8550)
					etemp = 42;
				else
					etemp = 43;	//for test
			}
			else
			{
				if(iqtemp < 200)
					etemp = 25;
				else if(iqtemp < 500)
					etemp = 20;
				else if(iqtemp < (4000))
					etemp = 18;
				else
					etemp = 10;
			}
		}		
		else if(spdtemp < 500)//with motor.spd_obj = 200,where actual speed is 175rpm
		{
			if(quadrant)
			{
				if(iqtemp < 70)
					etemp = 27;
				else if(iqtemp < 90)
					etemp = 28;
				else if(iqtemp < 120)
					etemp = 29;
				else if(iqtemp < 160)
					etemp = 30;
				else if(iqtemp < 200)
					etemp = 31;
				else if(iqtemp < 300)
					etemp = 32;
				else if(iqtemp < 400)
					etemp = 33;
				else if(iqtemp < 450)
					etemp = 34;
				else if(iqtemp < 650)
					etemp = 35;	
				else if(iqtemp < 850)
					etemp = 37;
				else if(iqtemp < 4050)
					etemp = 35;
				else if(iqtemp < 6000)
					etemp = 36;
				else if(iqtemp < 7000)
					etemp = 37;
				else if(iqtemp < 7500)
					etemp = 38;
				else if(iqtemp < 7800)
					etemp = 39;
				else if(iqtemp < 8550)
					etemp = 40;
				else
					etemp = 41;	//for test
			}
			else
			{
				if(iqtemp < (200))
					etemp = 25;
				else if(iqtemp < (500))
					etemp = 20;
				else if(iqtemp < (4000))
					etemp = 18;
				else
					etemp = 10;
			}
		}		
		else if(spdtemp < 900)//with motor.spd_obj = 400,where actual speed is 375rpm
		{
			if(quadrant)
			{
				if(iqtemp < 80)
					etemp = 29;
				else if(iqtemp < 120)
					etemp = 30;
				else if(iqtemp < 200)
					etemp = 31;
				else if(iqtemp < 300)
					etemp = 32;
				else if(iqtemp < 400)
					etemp = 33;
				else if(iqtemp < 450)
					etemp = 34;
				else if(iqtemp < 650)
					etemp = 35;	
				else if(iqtemp < 850)
					etemp = 37;
				else if(iqtemp < 4050)
					etemp = 35;
				else if(iqtemp < 6000)
					etemp = 36;
				else if(iqtemp < 7000)
					etemp = 37;
				else if(iqtemp < 7500)
					etemp = 38;
				else if(iqtemp < 8000)
					etemp = 39;
				else if(iqtemp < 8550)
					etemp = 40;
				else
					etemp = 41;	//for test
			}
			else
			{
				if(iqtemp < (200))
					etemp = 25;
				else if(iqtemp < (500))
					etemp = 20;
				else if(iqtemp < (4000))
					etemp = 18;
				else
					etemp = 14;
			}
		}
		else if(spdtemp < 1400)//with motor.spd_obj = 600 ,where actual speed is 575rpm
		{
			if(quadrant)
			{
				if(iqtemp < 80)
					etemp = 29;
				else if(iqtemp < 120)
					etemp = 30;
				else if(iqtemp < 180)
					etemp = 31;
				else if(iqtemp < 300)
					etemp = 32;
				else if(iqtemp < 420)
					etemp = 33;
				else if(iqtemp < 550)
					etemp = 34;
				else if(iqtemp < 650)
					etemp = 35;
				else if(iqtemp < 4550)
					etemp = 36;
				else if(iqtemp < 6550)
					etemp = 37;
				else if(iqtemp < 8000)
					etemp = 38;
				else if(iqtemp < 8550)
					etemp = 39;
				else 
					etemp = 37;
			}
			else
			{
				if(iqtemp < (200))
					etemp = 25;
				else if(iqtemp < (500))
					etemp = 20;
				else if(iqtemp < (4000))
					etemp = 18;
				else
					etemp = 14;
			}
				
		}
		else if(spdtemp < 1700)//with motor.spd_obj = 800 ,where actual speed is 780rpm
		{
			if(quadrant)
			{
				if(iqtemp < 80)
					etemp = 30;
				else if(iqtemp < 120)
					etemp = 31;
				else if(iqtemp < 180)
					etemp = 32;
				else if(iqtemp < 300)
					etemp = 33;
				else if(iqtemp < 420)
					etemp = 34;
				else if(iqtemp < 3050)
					etemp = 35;
				else if(iqtemp < 4450)
					etemp = 36;
				else if(iqtemp < 6850)
					etemp = 37;
				else if(iqtemp < 8550)
					etemp = 38;
				else
					etemp = 36;
			}
			else
			{
				if(iqtemp < (200))
					etemp = 25;
				else if(iqtemp < (500))
					etemp = 20;
				else if(iqtemp < (4000))
					etemp = 18;
				else
					etemp = 14;
			}
		}
		else if(spdtemp < 2100)//with motor.spd_obj = 1000 ,where actual speed is 990rpm
		{
			if(quadrant)
			{
				if(iqtemp < 80)
					etemp = 30;
				else if(iqtemp < 120)
					etemp = 31;
				else if(iqtemp < 180)
					etemp = 32;
				else if(iqtemp < 300)
					etemp = 33;
				else if(iqtemp < 420)
					etemp = 34;
				else if(iqtemp < 650)
					etemp = 35;
				else if(iqtemp < 2050)
					etemp = 36;
				else if(iqtemp < 6250)
					etemp = 37;
				else if(iqtemp < 8550)
					etemp = 38;
				else 
					etemp = 36;
			}
			else
			{
				if(iqtemp < (200))
					etemp = 25;
				else if(iqtemp < (500))
					etemp = 20;
				else if(iqtemp < (4000))
					etemp = 18;
				else
					etemp = 14;
			}
		}
		else if(spdtemp < 2600)//with motor.spd_obj = 1200 ,where actual speed is 1199rpm
		{
			if(quadrant)
			{
				if(iqtemp < 80)
					etemp = 30;
				else if(iqtemp < 120)
					etemp = 31;
				else if(iqtemp < 180)
					etemp = 32;
				else if(iqtemp < 300)
					etemp = 33;
				else if(iqtemp < 420)
					etemp = 34;
				else if(iqtemp < 650)
					etemp = 35;
				else if(iqtemp < 2050)
					etemp = 36;
				else if(iqtemp < 4550)
					etemp = 37;
				else if(iqtemp < 8550)
					etemp = 38;
				else
					etemp = 36;
			}
			else
			{
				if(iqtemp < (200))
					etemp = 25;
				else if(iqtemp < (500))
					etemp = 20;
				else if(iqtemp < (4000))
					etemp = 18;
				else
					etemp = 14;
			}
		}
		else if(spdtemp < 3000)//with motor.spd_obj = 1400 ,where actual speed is 1380rpm
		{
			if(quadrant)
			{
				if(iqtemp < 80)
					etemp = 30;
				else if(iqtemp < 120)
					etemp = 31;
				else if(iqtemp < 180)
					etemp = 32;
				else if(iqtemp < 300)
					etemp = 33;
				else if(iqtemp < 420)
					etemp = 34;
				else if(iqtemp < 650)
					etemp = 35;
				else if(iqtemp < 2050)
					etemp = 36;
				else if(iqtemp < 3550)
					etemp = 37;
				else if(iqtemp < 8550)
					etemp = 38;
				else 
					etemp = 36;
			}
			else
			{
				if(iqtemp < (200))
					etemp = 25;
				else if(iqtemp < (500))
					etemp = 20;
				else if(iqtemp < (4000))
					etemp = 18;
				else
					etemp = 14;
			}
		}
		else if(spdtemp < 3400)//with motor.spd_obj = 1600 ,where actual speed is 1590rpm
		{
			if(quadrant)
			{
				if(iqtemp < 80)
					etemp = 30;
				else if(iqtemp < 120)
					etemp = 31;
				else if(iqtemp < 180)
					etemp = 32;
				else if(iqtemp < 300)
					etemp = 33;
				else if(iqtemp < 420)
					etemp = 34;
				else if(iqtemp < 650)
					etemp = 35;
				else if(iqtemp < 2050)
					etemp = 36;
				else if(iqtemp < 3550)
					etemp = 37;
				else if(iqtemp < 4550)
					etemp = 38;
				else if(iqtemp < 8550)	
					etemp = 39;
				else 
					etemp = 37;
			}
			else
			{
				if(iqtemp < (200))
					etemp = 31;
				else if(iqtemp < (500))
					etemp = 28;
				else if(iqtemp < (1000))
					etemp = 25;
				else if(iqtemp < (2000))
					etemp = 21;
				else if(iqtemp < (4000))
					etemp = 19;
				else
					etemp = 16;
			}
		}
		else if(spdtemp < 3800)//with motor.spd_obj = 1800 ,where actual speed is 1770rpm
		{
			if(quadrant)
			{
				if(iqtemp < 80)
					etemp = 30;
				else if(iqtemp < 120)
					etemp = 31;
				else if(iqtemp < 180)
					etemp = 32;
				else if(iqtemp < 300)
					etemp = 33;
				else if(iqtemp < 420)
					etemp = 34;
				else if(iqtemp < 1000)
					etemp = 35;
				else if(iqtemp < 2250)
					etemp = 36;
				else if(iqtemp < 3050)
					etemp = 37;
				else if(iqtemp < 5550)
					etemp = 38;
				else if(iqtemp < 8550)
					etemp = 39;
				else 
					etemp = 37;//why smaller: make sure actual current will not over protect line;
			}
			else
			{
				if(iqtemp < (200))
					etemp = 31;
				else if(iqtemp < (500))
					etemp = 28;
				else if(iqtemp < (1000))
					etemp = 25;
				else if(iqtemp < (2000))
					etemp = 21;
				else if(iqtemp < (4000))
					etemp = 19;
				else
					etemp = 16;
			}
		}
		else if(spdtemp < 4200)//with motor.spd_obj = 2000 ,where actual speed is 1980rpm
		{
			if(quadrant)
			{
				if(iqtemp < 80)
					etemp = 30;
				else if(iqtemp < 120)
					etemp = 31;
				else if(iqtemp < 180)
					etemp = 32;
				else if(iqtemp < 300)
					etemp = 33;
				else if(iqtemp < 420)
					etemp = 34;
				else if(iqtemp < 1000)
					etemp = 35;
				else if(iqtemp < 2250)
					etemp = 36;
				else if(iqtemp < 3050)
					etemp = 37;
				else if(iqtemp < 4550)
					etemp = 38;
				else if(iqtemp < 8550)
					etemp = 39;
				else
					etemp = 37;
			}
			else
			{
				if(iqtemp < (200))
					etemp = 31;
				else if(iqtemp < (500))
					etemp = 28;
				else if(iqtemp < (1000))
					etemp = 25;
				else if(iqtemp < (2000))
					etemp = 21;
				else if(iqtemp < (4000))
					etemp = 19;
				else
					etemp = 16;
			}
		}
		else if(spdtemp < 4600)//with motor.spd_obj = 2200 ,where actual speed is 2190rpm
		{
			if(quadrant)
			{
				if(iqtemp < 80)
					etemp = 30;
				else if(iqtemp < 120)
					etemp = 31;
				else if(iqtemp < 180)
					etemp = 32;
				else if(iqtemp < 300)
					etemp = 33;
				else if(iqtemp < 420)
					etemp = 34;
				else if(iqtemp < 1000)
					etemp = 35;
				else if(iqtemp < 2250)
					etemp = 36;
				else if(iqtemp < 3050)
					etemp = 37;
				else if(iqtemp < 8550)
					etemp = 38;
				else
					etemp = 36;
			}
			else
			{
				if(iqtemp < (200))
					etemp = 32;
				else if(iqtemp < (500))
					etemp = 29;
				else if(iqtemp < (1000))
					etemp = 26;
				else if(iqtemp < (2000))
					etemp = 22;
				else if(iqtemp < (4000))
					etemp = 20;
				else
					etemp = 16;
			}
		}
		else if(spdtemp < 5000)//with motor.spd_obj = 2400 ,where actual speed is 2395rpm
		{
			if(quadrant)
			{
				if(iqtemp < 80)
					etemp = 30;
				else if(iqtemp < 120)
					etemp = 31;
				else if(iqtemp < 180)
					etemp = 32;
				else if(iqtemp < 300)
					etemp = 33;
				else if(iqtemp < 420)
					etemp = 34;
				else if(iqtemp < 1000)
					etemp = 35;
				else if(iqtemp < 2250)
					etemp = 36;
				else if(iqtemp < 3050)
					etemp = 37;
				else if(iqtemp < 8550)
					etemp = 38;
				else
					etemp = 35;
			}
			else
			{
				if(iqtemp < (200))
					etemp = 32;
				else if(iqtemp < (500))
					etemp = 29;
				else if(iqtemp < (1000))
					etemp = 26;
				else if(iqtemp < (2000))
					etemp = 22;
				else if(iqtemp < (4000))
					etemp = 20;
				else
					etemp = 16;
			}
		
		}
		else if(spdtemp < 5400)//with motor.spd_obj = 2600 ,where actual speed is rpm
		{
			if(quadrant)
			{
				if(iqtemp < 80)
					etemp = 30;
				else if(iqtemp < 120)
					etemp = 31;
				else if(iqtemp < 180)
					etemp = 32;
				else if(iqtemp < 300)
					etemp = 33;
				else if(iqtemp < 420)
					etemp = 34;
				else if(iqtemp < 1000)
					etemp = 35;
				else if(iqtemp < 2250)
					etemp = 36;
				else if(iqtemp < 3050)
					etemp = 37;
				else if(iqtemp < 6000)
					etemp = 38;
				else if(iqtemp < 6500)
					etemp = 39;
				else if(iqtemp < 7000)
					etemp = 40;
				else if(iqtemp < 8550)
					etemp = 41;
				else
					etemp = 39;
			}
			else
			{
				if(iqtemp < (200))
					etemp = 32;
				else if(iqtemp < (500))
					etemp = 29;
				else if(iqtemp < (1000))
					etemp = 26;
				else if(iqtemp < (2000))
					etemp = 22;
				else if(iqtemp < (4000))
					etemp = 20;
				else
					etemp = 16;
			}
		}
		else //if(m_spd_n < 5800)//with motor.spd_obj = 2800 ,where actual speed is rpm
		{
			if(quadrant)
			{
				if(iqtemp < 80)
					etemp = 30;
				else if(iqtemp < 120)
					etemp = 31;
				else if(iqtemp < 180)
					etemp = 32;
				else if(iqtemp < 300)
					etemp = 33;
				else if(iqtemp < 420)
					etemp = 34;
				else if(iqtemp < 1000)
					etemp = 35;
				else if(iqtemp < 2250)
					etemp = 36;
				else if(iqtemp < 3050)
					etemp = 37;
				else if(iqtemp < 6000)
					etemp = 38;
				else if(iqtemp < 6500)
					etemp = 39;
				else if(iqtemp < 7000)
					etemp = 40;
				else if(iqtemp < 8550)
					etemp = 41;
				else
					etemp = 39;
			}
			else
			{
				if(iqtemp < (200))
					etemp = 32;
				else if(iqtemp < (500))
					etemp = 29;
				else if(iqtemp < (1000))
					etemp = 26;
				else if(iqtemp < (2000))
					etemp = 22;
				else if(iqtemp < (4000))
					etemp = 20;
				else
					etemp = 16;
			}
		
		}
		return etemp;	
}

//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
