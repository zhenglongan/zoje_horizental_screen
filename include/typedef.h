 //--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : typedef.h
//  Description: data type define
//  Version    Date     Author    Description
//  0.01     03/07/07   pbb        created
//  0.02     02/08/07   lm         modify
//  ... 
//  ...
//--------------------------------------------------------------------------------------
#ifndef TYPEDEF_H
#define TYPEDEF_H
//--------------------------------------------------------------------------------------      
//  type define
//--------------------------------------------------------------------------------------
typedef long INT32;
typedef unsigned long UINT32;
typedef short INT16;
typedef unsigned short UINT16;
typedef signed char INT8;
typedef unsigned char UINT8;

typedef struct  
{
  UINT8 status;      // system status
  UINT8 u24_val;     // vol of U24
  UINT16 error;      // system error
  INT16 uzk_val;     // vol of UZK
} SYS_STRU;

typedef struct  
{
  	UINT8 dir;         			// motor direction
  	UINT8 stop_flag;
	INT16 iq;          			// motor current
	INT16 iq_last;
	INT16 max_spd;     			// motor max speed
	INT16 min_spd;     			// motor max speed
	INT16 acc;         			// motor accelerate
	INT16 dec;         			// motor decelerate
	INT16 acc_curve;   			// motor accelerate cure type
  	INT16 spd;         			// motor feedback speed
  	INT16 angle;       			// motor angle
	INT16 angle_adjusted;
	INT16 l_angle;     			// motor angle history value
  	INT16 spd_obj;     			// motor object speed
  	INT16 spd_ref;     			// motor referense speed
	INT16 stop_angle;  			// stop posion
	INT16 stop_angle1;
	INT16 angle_hold;
	INT16 angle_up;
} MOTOR_STRU;

typedef struct//scx
{	
	INT16 SetSpeed;
	INT16 ActualSpeed;
	INT32 Err_This;
	INT32 Err_Last;
	INT32 Err_LLast;
	INT16 Kp;
	INT16 Ki;
	INT16 Kd;
	
	INT32 Sum_Iq;
	
} PI_SPD_IQ;
typedef struct 
{
	UINT8 func;        // fuction code
	UINT8 para;
	INT8 xstep;        // X 
	INT8 ystep;        // Y
} PATTERN_DATA;

typedef struct 
{
	UINT8 MachineType;
	UINT8 FatherVersion;
	UINT8 ChildVersion;
	UINT8 SVNVersion;
	UINT8 DSPNumber;
}STEPVERSION;
//--------------------------------------------------------------------------------------      
//  1900a type define
//--------------------------------------------------------------------------------------
struct two_byte
{
	UINT8	byte2;
	UINT8	byte1;	
};

union TRANS
{
	struct	two_byte	byte;
	UINT16 	word;
};

union RECV
{
	struct	two_byte	byte;
	UINT16 	word;
};
typedef struct
{
	UINT8 test1;
	UINT8 test2;
	UINT8 test3;
	UINT8 test4;
}FAULT_DIAG;

typedef struct
{
	UINT16 DSP1_para_1F;
	UINT16 DSP1_para_20;
	UINT16 DSP1_para_21;
	UINT16 DSP1_para_22;
	UINT16 DSP1_para_23;
	UINT16 DSP1_para_27;
	UINT16 DSP1_para_28H;
	UINT16 DSP1_para_28M1;
	UINT16 DSP1_para_28M2;
	UINT16 DSP1_para_28L;
	
	UINT16 DSP2_para_1F;
	UINT16 DSP2_para_20;
	UINT16 DSP2_para_21;
	UINT16 DSP2_para_22;
	UINT16 DSP2_para_23;
	UINT16 DSP2_para_26;
	UINT16 DSP2_para_27;
	UINT16 DSP2_para_28H;
	UINT16 DSP2_para_28M1;
	UINT16 DSP2_para_28M2;
	UINT16 DSP2_para_28L;
	
	UINT8  dsp1A_half_current;
	UINT8  dsp1B_half_current;
	UINT8  dsp2A_half_current;
	UINT8  dsp2B_half_current;
		
	UINT8  platform_type;			
	UINT8  mainmotor_type;
	UINT8  x_origin_mode;		
	UINT8  yj_org_direction;
	UINT8  Corner_deceleration_speed;
    UINT8  wipper_type;
	UINT8  x_sensor_open_level;
	UINT8  y_sensor_open_level;
	UINT8  laser_function_enable;
	UINT8  last_10_speed;
	UINT8  last_9_speed;
	UINT8  last_8_speed;
	UINT8  last_7_speed;
	UINT8  last_6_speed;
	UINT8  last_5_speed;
	UINT8  last_4_speed;
	UINT8  last_3_speed;
	UINT8  last_2_speed;
	UINT8  last_1_speed;
	UINT8  dvab_open_level;
	UINT16  dsp1_step_crc;
	UINT16  dsp2_step_crc;
	UINT16 y_backward_dis;
    UINT16 x_take_offset;
	UINT16 x_take_offset2;
	UINT16 left_barcode_position;
	UINT16 right_barcode_position;
	UINT16 catch_delay_time;
	UINT16 y_barcode_position;
	UINT16 blow_air_counter;
	UINT16 cut_air_counter;
}SYSTEM_PARA;

#endif
//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
