//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : eeprom.h
//  Description: eeprom driver external function define
//  Version    Date     Author    Description
//  0.01     03/07/07   pbb        created
//  ...
//  ... 
//  ...
//--------------------------------------------------------------------------------------

#ifndef MOTOR_DEF_H
#define MOTOR_DEF_H

//--------------------------------------------------------------------------------------
// 	Includes
//--------------------------------------------------------------------------------------
#include "typedef.h" //data type define
#include "common.h"  //External variables declaration
//--------------------------------------------------------------------------------------
// 	Constants definition
//--------------------------------------------------------------------------------------


typedef struct
{
	INT16 angle;	
	INT16 mudulus;
} VECTOR;//scx

enum { SPD_UP=1,SPD_CST,SPD_DWN };//SCX

#define V_VECTOR_DEF {0,0}//scx


//motor status
#define INIT		0
#define OPEN_START	1
#define CLOSE_RUN	2
#define STOP		3
#define HOLD		4

#define DC300 620

#define STOP_SPD (10*u211)       //2010-7-2
#define DEC_SPD  800
#define DEC1_SPD  (10*u211)      //2010-4-7

#define PP_NUM 2
#define M_CODER 360   
#define ENCODER (M_CODER<<2)

#define LINE_4NUM (1000<<2)   


#define DEGREE_165	(165*ENCODER/360)
#define DEGREE_202	(230*ENCODER/360)//202*ENCODER/360//20150701for 1 round stop, by li
#define DEGREE_320	(320*ENCODER/360)//202*ENCODER/360//20150701for 1 round stop, by li
#define DEGREE_20	(20*ENCODER/360)

#define LINEDEGREE_20	(20*LINE_4NUM/360)

/*QEP*/

#define SPD_K (60*FX/M_CODER/8)



#define L 12//10  
#define E 55     
#define R 27     

#define THETA_INI 890//840
#define LQ_Q 10   
#define CURRENT_Q 10   
#define KE_Q 8 

#define LQ_Qn 		10   //scx
#define CURRENT_Qn 	11//scx10   
#define KE_Qn 		12	//scx8 
                                            
/*SPEED_FRQ*/
#define SAMPLES_BIT 3
#define SAMPLES ((1<<SAMPLES_BIT)-1)
#define RPM_K  ((60*1000*10*32/((M_CODER<<2)))*PWM_FREQ/10000)//(360*PWM_FREQ/10000)////  
#define RPM_Q  (SAMPLES_BIT+4)  
                                             
                                             
                                            
#define DTT_CNT (FX/1000000*3)
#define DTT_COMP (36)//((INT16)u228)//((INT16)(DTT_CNT>>1)) scx

#define CARR_CNT (FX/(2*PWM_FREQ))

#define IIT_CONT 262144		//2^18
//******************   
#define BEGIN 1
#define END 0
#define FB 3
#define DT 2
#define AC 1
#define NC 0    
#define H_time 7
#define L_time 25   
#define LIMIT_TIME_AC 80    //the least time between 2 acc_tactic   
#define LIMIT_TIME_DT 30    //the least time between DT and the other two status 
#define AC_HOLD_TIME 4		//the time when switch status from ac to normal        
//****************** 

#define COUNT_S (10*PWM_FREQ/10000)
#define DELTA_I 1000         
#define I_MAX_O 5000//5000//3000         


#define U_MAX 1385     		//300v
//scx 20161115
//#define A_CNT_MAX 			 (u228)//scx
#define NO_COMP_OFFSET		 (3*4)//死区不补偿角度即过零点左右各3度
#define IQ_MIN_VALUE	 	 100

//转速电流PID参数宏定义 scx

#define N_A_ONCE			 (5)
#define DETA_IQ_PID_MAX_V0	 (714*N_A_ONCE)			//iq的增量最【大】值限幅714*1/2/3

//#define IQ_PID_MAX_V0		 (12*714)//(8000)  //14A的话速度失控 机器大幅度异响！

#define DETA_IQ_PID_MAX_V	 (DETA_IQ_PID_MAX_V0<<13)			//iq的增量最【大】值限幅

#define IQ_PID_MAX_V	 	 ((14*714L)<<13) //(IQ_PID_MAX_V0<<13)		//iq的最【大】值限幅

#define SPD_IQ_PID_KP	 	 1				//IQ的PID调节【P】参数默认值，可以u226代替
#define SPD_IQ_PID_KI	 	 1				//IQ的PID调节【I】参数默认值，可以u227代替
#define SPD_IQ_PID_KD	 	 1				//IQ的PID调节【I】参数默认值，可以u227代替

#define EN_KI_VALUE			 (700)//+10*UserPara.first3StitchingFeedingSynModifyAngle_269)				//积分分离的【I】参数起作用的【误差值】，误差小于该值【I】参数起作用！

//#define OBJ_SPD_VALUE		 (1500*2)

#define COSTANT_SPD_VALUE	  (12)//后期可作为一个参数

#define PID_UP_SEG_1	550

#define PID_DWN_SEG_1	1050

#define PID_CST_SEG_1	550
//scx
#define KPspd_UP1	 10//3//8//14//(u227)//	(10) //14//(10)//15	 //    7
#define KIspd_UP1	 4//5//20//(u228)//	(20)//30//(14)//30	 //    7
#define KDspd_UP1	 0	 //    7

#define KPspd_UP2	 10//3//8//14//(u227)//	(10) //14//(10)//15	 //    7
#define KIspd_UP2	 4//5//20//(u228)//	(20)//30//(14)//30	 //    7
#define KDspd_UP2	 0	 //    7

#define KPspd_CONSTANT1	 10//8//(14)//15	 //    7
#define KIspd_CONSTANT1	 4//20//(30)//30	 //    7
#define KDspd_CONSTANT1	 0//0	 	 //    7

#define KPspd_CONSTANT2	 10//8//(14)//15	 //    7
#define KIspd_CONSTANT2	 4//20//(30)//30	 //    7
#define KDspd_CONSTANT2	 0//0	 	 //    7




#define KPspd_DWN1	 10//10//8//(11)//(u227)//(13)//(10)//12	 //    7
#define KIspd_DWN1	 4//10//20//(10)//(u228)//(30)//(14)//30	 //    7
#define KDspd_DWN1	 0	 //    7

#define KPspd_DWN2	 10//10//8//(11)//(u227)//(13)//(10)//12	 //    7
#define KIspd_DWN2	 4//10//20//(10)//(u228)//(30)//(14)//30	 //    7
#define KDspd_DWN2	 0	 //    7


#define KPspd1_CUT_LINE	 10//(u226)//(11)//(11)//16	 //    7
#define KIspd1_CUT_LINE	 10//(u227)////(10)//20	 //    7
#define KDspd1_CUT_LINE	 0	 //    7

#define KPspd2_CUT_LINE	 10//(u226)//10//14//(u227)//(11)//16	 //    7
#define KIspd2_CUT_LINE	 10//(u227)//10//30//(u228)//(10)//20	 //    7
#define KDspd2_CUT_LINE	 0	 //    7


//scx


typedef struct 
{
	UINT8 kp_h;
	UINT8 ki_h;	
	UINT8 kp_m;
	UINT8 ki_m;
	UINT8 kp_l;
	UINT8 ki_l;
	
	UINT8 kp_up_h;
	UINT8 ki_up_h;	
	UINT8 kp_up_m;
	UINT8 ki_up_m;	
	UINT8 kp_up_l;
	UINT8 ki_up_l;
	
	UINT8 kp_down_h;
	UINT8 ki_down_h;
	UINT8 kp_down_l;
	UINT8 ki_down_l;
	
	UINT8 kp_transup_h;
	UINT8 ki_transup_h;
	UINT8 kp_transup_m;
	UINT8 ki_transup_m;
	UINT8 kp_transup_l;
	UINT8 ki_transup_l;
	
	UINT8 kp_transdown_h;
	UINT8 ki_transdown_h;
	UINT8 kp_transdown_l;
	UINT8 ki_transdown_l;
	
	UINT8 kp_stop0;
	UINT8 ki_stop0;
	UINT8 kp_stop1;
	UINT8 ki_stop1;
	UINT8 kp_stop2;
	UINT8 ki_stop2;
	UINT8 kp_stop;
	UINT8 ki_stop;

	UINT8 kpp_stop31;
	UINT8 kps_stop31;
	UINT8 kpp_stop32;
	UINT8 kps_stop32;
	
} PISTRUCT;

/*PI*/

#define KPs_H 15 			//      >=1500rpm
#define KIs_H 7				//
#define KPs_M 18        		//		>1000 && <1500rpm
#define KIs_M 5			//8

#if HIGH_PENETRATION
#define KPs_L 20			    //8			<=1000rpm
#define KIs_L 15			//
#else
#define KPs_L 40			    //8			<=1000rpm
#define KIs_L 10			//
#endif

#if 1 

#define KPs_up_L 30    		//50	20101014;			<750rpm
#define KIs_up_L 5			//4
#define KPs_up_M 20    		//50	20101014;			<1600rpm
#define KIs_up_M 3
#define KPs_up_H 24			//24//4			>=750rpm
#define KIs_up_H 4			//4//4

#define KPs_trans_L_up 27    //27	//80	20101011;	50	20101014;	<500rpm
#define KIs_trans_L_up 3	//3	//20
#define KPs_trans_M_up 27    //20	//24		<1000rpm
#define KIs_trans_M_up 3	//4	//20
#define KPs_trans_H_up 27	//27//15	//4		>=1000rpm
#define KIs_trans_H_up 2	//2//4	//2

#endif

#define KPs_down_L 18    	//30			<2000rpm
#define KIs_down_L 3		//2
#define KPs_down_H 26    	//10		>=2000rpm
#define KIs_down_H 2


#define KPs_trans_L_down 20    	//40		<500rpm
#define KIs_trans_L_down 3		//5
#define KPs_trans_H_down 16		//8		>=500rpm
#define KIs_trans_H_down 2		//4

#define KPs_stop_0	8
#define KIs_stop_0	8
#define KPs_stop_1	8
#define KIs_stop_1	8
#define KPs_stop_2	8
#define KIs_stop_2	8
//case2 used for finding deadpoint 
#define KPs_deadpoint	20
#define KIs_deadpoint	15
//case2,cut thread
#define KPs_stop motor_para[0]//15			
#define KIs_stop motor_para[1]//10
//case3,step1
#define STOP_KPp_1 motor_para[2]//15
#define STOP_KPs_1 motor_para[3]//5
//case3,step2
#define STOP_KPp_2 motor_para[4]//8
#define STOP_KPs_2 motor_para[5]//30

#define PID_SPD_MAX_P 12000//9000 	
#define PID_SPD_MAX_N 12000//9000     //9000
//didt rangee
#define DERV 1000*motor_para[6] 	//9000

//chose the condition of using didt:,0 for use under 1000rpm,1 for never use
#define DIDT_SPEED_RANGE motor_para[7]	

//select the porportion of didt when cutting thread,the range is 0-4,as default is 4
#define DIDT_CUT_THREAD motor_para[8]
//#define DIDT_CUT_THREAD3 motor_para[9]	
//last step for motor.spd_obj == 0
#define ANGLE_P 6*ENCODER/360//6*ENCODER/360
#define ANGLE_N -(6*ENCODER/360)
#define LOCK_IQ 3000
#define LOCK_THETA 60


#define abs(z) ((z > 0)?(z):(-(z))) 
#define limit(a,b) a=(a>(b))?(b):a;a=(a<(-(b)))?(-(b)):a 

#endif

//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
