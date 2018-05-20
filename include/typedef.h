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
	UINT16 DSP1_para_1F;		//1,2
	UINT16 DSP1_para_20;		//3,4
	UINT16 DSP1_para_21;		//5,6
	UINT16 DSP1_para_22;		//7,8
	UINT16 DSP1_para_23;		//9,10
	UINT16 DSP1_para_27;		//11,12
	UINT16 DSP1_para_28H;		//13,14
	UINT16 DSP1_para_28M1;		//15,16
	UINT16 DSP1_para_28M2;		//17,18
	UINT16 DSP1_para_28L;		//19,20
	
	UINT16 DSP2_para_1F;		//21,22
	UINT16 DSP2_para_20;		//23,24
	UINT16 DSP2_para_21;		//25,26
	UINT16 DSP2_para_22;		//27,28
	UINT16 DSP2_para_23;		//29,30

	UINT16 DSP2_para_27;		//31,32
	UINT16 DSP2_para_28H;		//33,34
	UINT16 DSP2_para_28M1;		//35,36
	UINT16 DSP2_para_28M2;		//37,38
	UINT16 DSP2_para_28L;		//39,40
	
	UINT8  dsp1A_half_current;	//41
	UINT8  dsp1B_half_current;	//42
	UINT8  dsp2A_half_current;	//43
	UINT8  dsp2B_half_current;	//44
		
	UINT8  platform_type;		//45	
	UINT8  mainmotor_type;      //46
	UINT8  x_origin_mode;		//47
	UINT8  yj_org_direction;    //48
	UINT8  Corner_deceleration_speed;//49
    UINT8  wipper_type;			//50
	UINT8  x_sensor_open_level; //51
	UINT8  y_sensor_open_level;	//52
	UINT8  laser_function_enable;//53
	UINT8  last_9_speed;		//54
	UINT8  last_8_speed;		//55
	UINT8  last_7_speed;		//56
	UINT8  last_6_speed;		//57
	UINT8  last_5_speed;		//58
	UINT8  last_4_speed;		//59
	UINT8  last_3_speed;		//60
	UINT8  last_2_speed;		//61
	UINT8  last_1_speed;		//62
	UINT8  dvab_open_level;		//63
	UINT16  dsp1_step_crc;		//64,65
	UINT16  dsp2_step_crc;		//66,67
	UINT16 y_backward_dis;		//68,69
    UINT16 x_take_offset;		//70,71
	UINT16 x_take_offset2;		//72,73
	UINT16 left_barcode_position;	//74,75
	UINT16 right_barcode_position;	//76,77
	UINT16 catch_delay_time;	//78,79
	UINT16 y_barcode_position;	//80,81
	UINT16 blow_air_counter;	//82,83
	UINT16 cut_air_counter;		//84,85
	UINT16  dsp3_step_crc;		//86,87
	UINT16  dsp4_step_crc;		//88,89
	
	UINT16 DSP3_para_1F;		//90,91
	UINT16 DSP3_para_20;		//92,93
	UINT16 DSP3_para_21;		//94,95
	UINT16 DSP3_para_22;		//96,97
	UINT16 DSP3_para_23;		//98,99

	UINT16 DSP3_para_27;		//100,101
	UINT16 DSP3_para_28H;		//102,103
	UINT16 DSP3_para_28M1;		//104,105
	UINT16 DSP3_para_28M2;		//106,107
	UINT16 DSP3_para_28L;		//108,109
	
	UINT8  dsp3A_half_current;	//110
	UINT8  dsp3B_half_current;	//111
	UINT8  qd_org_direction;    //112
	UINT8  zx_curver;			//113
	UINT8  slow_start_mode;		//114 慢速启动模式
	UINT8  Corner_deceleration_speed1; //115 拐点后第一针
	UINT8  Corner_deceleration_speed2; //116 拐点后第二针
	UINT8  Corner_deceleration_speed3; //117 拐点后第三针
	UINT8  Corner_deceleration_speed4; //118 拐点后第四针


}SYSTEM_PARA;

#endif
//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
