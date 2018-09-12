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
	自动换梭拥有电气部件如下：
	梭盘相关：		
			[OUT]梭盘电机---开环步进，X14，DSP3-B
			[IN]梭盘电机原点---
			[IN]梭芯传感器---检测当前梭芯是否为空
			[IN]梭盘动作按钮---用于人工更换梭芯，按一下转一下

	换梭臂相关：
			[OUT]换梭臂电机---开环步进，X15，DSP3-A
			[IN]换梭臂电机原点---
			[OUT]伸缩气缸---伸出换梭臂
			[OUT]夹紧气缸---然后夹紧
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
					 数据结构定义
			 
****************************************************************/
//因为横屏暂时没有开发自动换梭相关的参数，因此这里先把设置数据
//暂时放到系统参数第9组
typedef struct
{
	//自动换梭使能，中捷竖屏上是K151参数
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

	//自动换梭相关,中捷竖屏存在系统参数第一组中，这里都放到第9组
	UINT8  bobbin_platform_speed;//14
	 INT8  bobbin_shake_distance;//15
	UINT8  bobbin_shake_time;	 //16
}SYSTEM_PARA9;

/****************************************************************
					 相关IO引脚定义
			 
****************************************************************/
//[IN]梭盘电机原点
#define BOBBIN_CASE_PLATFORM_ORG		ADTCSM	//输入5,p0_7
//[IN]梭盘动作按钮---用于人工更换梭芯，按一下转一下
#define BOBBIN_CASE_SWITCH				PORG	//输入1,p2_2
//[IN]梭芯传感器---检测当前梭芯是否为空
#define BOBBIN_CASE_EMPTY_CHECK         CORG    //输入4,p2_4

//[IN]换梭臂电机原点
#define BOBBIN_CASE_ARM_ORG 			PSENS	//输入2---5V,p2_3
//[OUT]伸缩气缸---伸出换梭臂
#define BOBBIN_CASE_ARM_OUT  		    T_HALF  //气阀4,p4_5
//[OUT]夹紧气缸---然后夹紧
#define BOBBIN_CASE_ARM_SCRATH          FL      //辅助,p4_1



/****************************************************************
					 全局变量声明
					 
 *定义为extern是为了外部能够引用,定义在bobbin_auto_change.c中				 
****************************************************************/
extern UINT8 	bobbin_case_enable;//k151，自动换梭使能，=1使能
extern INT8  	bobbin_case_arm_offset;//k156，机头对接位置修正补偿
extern INT8   	bobbin_case_platform_offset;//k157，换梭对接位置修正补偿
extern UINT16 	bobbin_case_inout_delay;//k158，前后抓紧气缸到位延时
extern UINT16 	bobbin_case_scrath_delay;//k159，夹紧气缸到位延时
extern UINT8 	bobbin_case_current_level;//k160，抓臂电机工作电流档位,已经未使用，但是仍然保留
extern UINT8 	bobbin_case_stop_position;//k161，换梭停止位置，0-梭盘侧，1-机头侧
extern UINT8 	bobbin_case_alarm_mode;//k162，换梭方式，0-底线警报后手动换梭，1-底线警报时自动换梭
extern UINT8 	bobbin_case_restart_mode;//k163，换梭起缝方式，0-手动启动，1-自动启动
extern UINT8  	bobbin_case_workmode;//k164，空梭芯处理方式，0-放回梭盘，1-放收纳盒
extern INT8 	bobbin_plateform_org_offset;//k192，梭盘电机零位补偿




/*
	bobbin_case_arm_position，换梭臂的位置
	0			上电没找过原点
	50			位置在机头
	100 		位置在梭盘
*/
extern UINT8  	bobbin_case_arm_position;
extern UINT8  	bobbin_case_platform_position;
extern UINT16 	bobbin_case_dump_position;
extern UINT8  	bobbin_case_switch_counter;
extern UINT8  	bobbin_case_switch_flag;





/****************************************************************
					 函数定义
				 
****************************************************************/
//IO和全局变量的初始化
void bobbin_init(void);
//机械臂电机找原点
void go_origin_bobbin_case_arm(UINT8 pos);
//找一个梭芯
UINT8 find_a_bobbin_case(UINT8 full);
//
void bobbin_case_motor_adjust(void);
//对外，启动完整的换梭流程
UINT8 bobbin_case_workflow1(void);


UINT8 get_bobbin_case_arm_org_status(void);

//step_motor_drv.c中提供的DSP底层支持函数
extern void movestep_cs3(UINT16 command,INT16 x_data,UINT8 timer_need);
extern UINT16 check_DSP3_input(void);
extern void inpress_down(UINT8 pos);








#endif




