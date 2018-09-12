/*
--------------------------------------------------------------------------------------
      COPYRIGHT(C) 2018 Beijing xingdahao technology Co., Ltd.
                     ALL RIGHTS RESERVED 
  Project Number: sewing_machine_controller 
  File Name : bobbin_auto_change.c
  Description: bobbin auto change control 
  Version     Date     Author    Description
  0.0.1		2018.9.12  zla		porting form zoje vertical screen control program
--------------------------------------------------------------------------------------

*/
/*
	�Զ�����ӵ�е����������£�
	������أ�		
			[OUT]���̵��---����������X14��DSP3-B
			[IN]���̵��ԭ��---
			[IN]��о������---��⵱ǰ��о�Ƿ�Ϊ��
			[IN]���̶�����ť---�����˹�������о����һ��תһ��

	�������أ�
			[OUT]����۵��---����������X15��DSP3-A
			[IN]����۵��ԭ��---
			[OUT]��������---��������
			[OUT]�н�����---Ȼ��н�
*/
#ifndef BOBBIN_AUTO_CHANGE_H
#define BOBBIN_AUTO_CHANGE_H



#include "sfr62p.h"       //M16C/62P special function register definitions
#include "typedef.h"      //Data type define
#include "variables.h"    //External variables declaration
#include "common.h"       //Common constants definition
#include "delay.h"        //delay time definition
#include "iic_bus_eeprom.h"

/****************************************************************
					 ���ݽṹ����
			 
****************************************************************/
//��Ϊ������ʱû�п����Զ�������صĲ�������������Ȱ���������
//��ʱ�ŵ�ϵͳ������9��
typedef struct
{
	//�Զ�����ʹ�ܣ��н���������K151����
	UINT8 	k151_bobbin_case_enable;//1
	INT8 	k156_bobbin_case_arm_offset;//2
	INT8 	k157_bobbin_case_platform_offset;//3
	UINT16 	k158_bobbin_case_inout_delay;//4-5
	UINT16 	k159_bobbin_case_scrath_delay;//6-7
	UINT8 	k160_bobbin_case_current_level;//8
	UINT8 	k161_bobbin_case_stop_position;//9
	UINT8 	k162_bobbin_case_alarm_mode;//10
	UINT8 	k163_bobbin_case_restart_mode;//11
	UINT8 	k164_bobbin_case_workmode;//12
	INT8 	k192_bobbin_plateform_org_offset;//13

	//�Զ��������,�н���������ϵͳ������һ���У����ﶼ�ŵ���9��
	UINT8  bobbin_platform_speed;//14
	 INT8  bobbin_shake_distance;//15
	UINT8  bobbin_shake_time;	 //16
}SYSTEM_PARA9;

/****************************************************************
					 ���IO���Ŷ���
			 
****************************************************************/
//[IN]���̵��ԭ��
#define BOBBIN_CASE_PLATFORM_ORG		ADTCSM	//����5,p0_7
//[IN]���̶�����ť---�����˹�������о����һ��תһ��
#define BOBBIN_CASE_SWITCH				PORG	//����1,p2_2
//[IN]��о������---��⵱ǰ��о�Ƿ�Ϊ��
#define BOBBIN_CASE_EMPTY_CHECK         CORG    //����4,p2_4

//[IN]����۵��ԭ��
#define BOBBIN_CASE_ARM_ORG 			PSENS	//����2---5V,p2_3
//[OUT]��������---��������
#define BOBBIN_CASE_ARM_OUT  		    T_HALF  //����4,p4_5
//[OUT]�н�����---Ȼ��н�
#define BOBBIN_CASE_ARM_SCRATH          FL      //����,p4_1



/****************************************************************
					 ȫ�ֱ�������
					 
 *����Ϊextern��Ϊ���ⲿ�ܹ�����,������bobbin_auto_change.c��				 
****************************************************************/
extern UINT8 	bobbin_case_enable;//k151���Զ�����ʹ�ܣ�=1ʹ��
extern INT8  	bobbin_case_arm_offset;//k156����ͷ�Խ�λ����������
extern INT8   	bobbin_case_platform_offset;//k157������Խ�λ����������
extern UINT16 	bobbin_case_inout_delay;//k158��ǰ��ץ�����׵�λ��ʱ
extern UINT16 	bobbin_case_scrath_delay;//k159���н����׵�λ��ʱ
extern UINT8 	bobbin_case_current_level;//k160��ץ�۵������������λ,�Ѿ�δʹ�ã�������Ȼ����
extern UINT8 	bobbin_case_stop_position;//k161������ֹͣλ�ã�0-���̲࣬1-��ͷ��
extern UINT8 	bobbin_case_alarm_mode;//k162������ʽ��0-���߾������ֶ�����1-���߾���ʱ�Զ�����
extern UINT8 	bobbin_case_restart_mode;//k163��������췽ʽ��0-�ֶ�������1-�Զ�����
extern UINT8  	bobbin_case_workmode;//k164������о����ʽ��0-�Ż����̣�1-�����ɺ�
extern INT8 	bobbin_plateform_org_offset;//k192�����̵����λ����




/*
	bobbin_case_arm_position������۵�λ��
	0			�ϵ�û�ҹ�ԭ��
	50			λ���ڻ�ͷ
	100 		λ��������
*/
extern UINT8  	bobbin_case_arm_position;
extern UINT8  	bobbin_case_platform_position;
extern UINT16 	bobbin_case_dump_position;
extern UINT8  	bobbin_case_switch_counter;
extern UINT8  	bobbin_case_switch_flag;





/****************************************************************
					 ��������
				 
****************************************************************/
//IO��ȫ�ֱ����ĳ�ʼ��
void bobbin_init(void);
//��е�۵����ԭ��
void go_origin_bobbin_case_arm(UINT8 pos);
//��һ����о
UINT8 find_a_bobbin_case(UINT8 full);
//
void bobbin_case_motor_adjust(void);
//���⣬���������Ļ�������
UINT8 bobbin_case_workflow1(void);


UINT8 get_bobbin_case_arm_org_status(void);

//step_motor_drv.c���ṩ��DSP�ײ�֧�ֺ���
extern void movestep_cs3(UINT16 command,INT16 x_data,UINT8 timer_need);
extern UINT16 check_DSP3_input(void);
extern void inpress_down(UINT8 pos);








#endif




