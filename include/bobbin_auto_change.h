/*
--------------------------------------------------------------------------------------
      COPYRIGHT(C) 2018 Beijing xingdahao technology Co., Ltd.
                     ALL RIGHTS RESERVED 
  Project Number: sewing_machine_controller 
  File Name : bobbin_auto_change.c
  Description: bobbin auto change control 
  Version     Date     Author    Description
  0.0.1		2018.9.12  zla		porting form zoje vertical screen control program
  0.1.0		2018.9.14  zla		rearrange the code,change the the globle variables
                                to the static variables,only use the functions as
                                the communication mode to the application programe
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
#ifndef __BOBBIN_AUTO_CHANGE_H
#define __BOBBIN_AUTO_CHANGE_H

#include "typedef.h"      //Data type define


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
					 数据结构定义
			 
****************************************************************/
/*
	因为横屏暂时没有开发自动换梭相关的参数配置界面，因此这里先把
	设置数据暂时放到系统参数第9组
*/
typedef struct
{
	/*
		自动换梭使能，原竖屏K151
		1-打开
		其他-关闭 
	*/
	UINT8 	k151_bobbin_case_enable;//1

	/*
		机头对接位置修正补偿，原竖屏K156
		换梭臂电机到<机头位置>的微调值，根据实际机械安装情况进行调整
	*/
	INT8 	k156_bobbin_case_arm_offset;//2

	/*
		换梭对接位置修正补偿，原竖屏K157
		换梭臂电机到<梭盘位置>的微调值，根据实际机械安装情况进行调整
	*/
	INT8 	k157_bobbin_case_platform_offset;//3

	/*
		伸缩气缸到位延时，原竖屏K158,单位是ms
		加延时是为了保证动作完成，气缸有动作时间，系统中没有传感器获取
		位置信息所以需要延时等待
	*/
	UINT16 	k158_bobbin_case_inout_delay;//4-5

	/*
		换梭臂抓紧气缸延时，原竖屏K159,单位是ms
		加延时是为了保证动作完成，气缸有动作时间，系统中没有传感器获取
		位置信息所以需要延时等待
	*/
	UINT16 	k159_bobbin_case_scrath_delay;//6-7

	/*
		抓臂电机工作电流档位，原竖屏K160
		主控程序已经未使用，无需关注，电流档位设置在系统参数第1组的
		参数127、128
	*/
	UINT8 	k160_bobbin_case_current_level;//8

	/*
		换梭臂停止位置，原竖屏K161
		*0-梭盘侧，这边更好些，可以防止机械臂意外伸出撞到下轴
		1-机头侧
	*/
	UINT8 	k161_bobbin_case_stop_position;//9

	/*
		换梭方式，原竖屏K162
		0-底线警报后手动换梭
		 报错后，弹出ERROR，用户按下确定以后返回到READY中进行换梭，换梭
		 动作不需要用户再触发，这就是手动启动过程，在这个情况下K163无意义
		1-底线警报时自动换梭
		 面板一样报错，但是不等用户按确定，自动启动换梭流程，然后根据K163
		 的值判断，换梭后是报错确认进入ERROR状态（K163=0）（按下确定后到READY）
		 还是直接启动缝制（K163=1）
	*/
	UINT8 	k162_bobbin_case_alarm_mode;//10

	/*
		换梭起缝方式，原竖屏K163
		自动换梭以后启动方式，仅在K162=1时有意义
		0-手动启动
		1-自动启动
	*/
	UINT8 	k163_bobbin_case_restart_mode;//11

	/*
		空梭芯处理方式，原竖屏K164
		0-放回梭盘，可能会导致后续换梭取到空梭芯
		*1-放收纳盒（丢弃位置由变量bobbin_case_dump_position，即参数17指定）
	*/
	UINT8 	k164_bobbin_case_workmode;//12

	/*
		梭盘电机零位补偿，原竖屏K192
		梭盘电机原点补偿，根据实际机械安装情况进行调整
	*/
	INT8 	k192_bobbin_plateform_org_offset;//13

	/*
		梭盘移动速度，原竖屏K192系统参数第1组参数123，这里都统一整理到此
		实际上是每走一步的延时时间，单位是ms，因此值越大速度越慢
	*/
	UINT8  bobbin_platform_speed;//14

	/*
		梭盘抖动距离，原竖屏K192系统参数第1组参数124，这里都统一整理到此
		换梭臂从梭盘获取梭芯的时候，为了更好的抓取到梭芯，需要梭盘转一个
		小角度，这样方便机械臂抓取，抓取后，然后再转回来，就抓牢了
	*/
	 INT8  bobbin_shake_distance;//15

	/*
		梭盘抖动时间，原竖屏K192系统参数第1组参数125，这里都统一整理到此
		和梭盘抖动距离bobbin_shake_distance配合使用，单位是ms
	*/
	UINT8  bobbin_shake_time;	 //16
	
	/*
		空梭芯丢弃位置，单位是步，原中捷竖屏默认50，未开放此参数
		仅当K164=1时，即空梭芯处理方式是放收纳盒时有效
	*/
	UINT8  bobbin_case_dump_position;	 //17
}SYSTEM_PARA9;


/****************************************************************
					 全局变量声明
					 
 *定义为extern是为了外部能够引用,定义在bobbin_auto_change.c中				 
****************************************************************/
/*
	抓臂电机工作电流档位，原竖屏K160
	主控程序已经未使用，无需关注，电流档位设置在系统参数第1组的
	参数127、128
*/
extern UINT8 	bobbin_case_current_level;

//用于调试换梭功能的变量，借用检测模式里的输出检测,后续可能删除
extern UINT8 	bobbin_case_debug_cmd;


/****************************************************************
					 函数定义
				 
****************************************************************/
/*
	名称：IO和变量的初始化，包括从EEPROM中读取第9组参数
	参数：无
	返回值：无
	说明：开机时调用一次即可，
	最后修改日期：2018-9-14
*/
void bobbin_init(void);

/*
	名称：从EEPROM中读取第9组参数
	参数：无
	返回值：无
	说明：bobbin_init()将调用，当SALCK状态下修改系统参数第9组后，
	      也将调用此函数更新配置参数
	最后修改日期：2018-9-14
*/
void bobbin_get_configuration(void);

/*
	名称：查看自动换梭功能是否打开
	参数：无
	返回值：1-打开，其他-关闭
	说明：外部程序调用自动换梭模块相关函数前，需要先调用本函数查询
	      是否开启了自动换梭功能，只有开启了才能执行后续的操作
	最后修改日期：2018-9-14
*/
UINT8 bobbin_check_module_enable(void);

/*
	名称：检测READY状态下是否需要自动换梭动作标志
	参数：无
	返回值：1-需要，其他-不需要
	说明：针对底线警报后手动换梭（k162_bobbin_case_alarm_mode=0）的方式，
	     程序的处理机制是：发现底线警报时只报错，当用户在UI面板上按下
	     确认时，面板将发送改变状态指令（功能码0x81），在这里面把此标志
	     置1然后再进入READY状态，在REARY状态中响应换梭流程，然后等待用户
	     按下启动按钮DVA后再开始缝制，这个函数就是用于READY状态下检测是否
	     需要自动挡换梭
	最后修改日期：2018-9-14
*/
UINT8 bobbin_check_need_aciton_flag(void);

/*
	名称：清除READY状态下是否需要自动换梭动作标志
	参数：无
	返回值无
	说明：当READY状态中调用函数bobbin_check_need_aciton_flag()发现需要
	      自动换梭，便调用此函数清除标志，防止反复触发自动换梭流程
	最后修改日期：2018-9-14
*/
void bobbin_clear_need_aciton_flag(void);

/*
	名称：换梭臂电机到指定位置（机头或者梭盘位置）
	参数：pos，0-转到梭盘位置，1-转到机头位置
	返回值：无
	说明：类似于舵机的控制，只有两个地方。
	      机械臂定位盘角度220度，对应两个位置：机头对抓取位置 和 换梭盘对接位置
          挡片挡住时，机械臂逆时针转，从挡住一直到退出挡片 对应着位置是 换梭盘对接位置
          机械臂顺时针转，从挡住一直到退出挡片 对应着位置是 机头对抓位置
          挡片未挡住时，电机顺时针转，
          原点默认是 换梭盘对接位置
	      挡片是在电机轴上，有一级皮带传动，所以机械臂和挡片是旋转方向相反，
	      下面所说的方向是机械臂的方向
	最后修改日期：2018-9-14
*/
void go_origin_bobbin_case_arm(UINT8 pos);

/*
	名称：梭盘电机转到一个梭芯（空或者实）位置
	参数：full，0-找空梭芯位置，1-找有梭芯的位置，其他-转到下一个梭芯位置
	返回值：无
	说明：电机的旋转方向：负数-梭盘顺时针旋转 正数-梭盘逆时针旋转
	      原点传感器： 高电平 表示在挡片缺口里  低电平表示被挡上了。 传感器0528 NPN开漏
	      挡上时为高电平
	      正常找基准位置是：
	       1）电机顺时针转，找下一个挡片由挡到不挡的跳变
 		   2）找当前空位，电机逆时针转找到挡上的位置，然后再顺时针转到不挡边沿
	最后修改日期：2018-9-14
*/
UINT8 find_a_bobbin_case(UINT8 full);

/*
	名称：启动完整的自动换梭流程
	参数：无
	返回值：无
	说明：启动一个自动换梭流程，这里没有考虑到换梭模式和换梭后启动方式，
	      因此只能在除了RUN状态以外的地方使用，RUN状态下自动换梭要调用
	      函数：bobbin_auto_change_process_callback()
	最后修改日期：2018-9-14
*/
UINT8 bobbin_case_workflow1(void);


/****************************************************************
					 回调函数声明
				 
****************************************************************/
/*
	名称：梭盘动作按钮BOBBIN_CASE_SWITCH检测回调函数
	参数：times：指定检测的次数（时间）,当连续times次都检测到有效
	             电平时将置位标志位
	      polarity：指定有效的电平等级，0-低电平，1-高电平
	返回值：无
	说明：1ms定时器TA0中断函数中需要进行调用
	最后修改日期：2018-9-14
*/
void bobbin_check_switch_callback(UINT8 polarity, UINT16 times);
/*
	名称：根据梭盘动作按钮是否动作来控制梭盘电机移动到下一个位置
	参数：无
	返回值：无
	说明：在需要响应梭盘按钮动作的地方调用此函数即可，但是别忘了在
		  定时器TA0中断中调用函数bobbin_check_switch_callback()扫描
		  按钮是否按下
	最后修改日期：2018-9-14
*/
void bobbin_platform_move_to_next_callback(void);
/*
	名称：处理自动换梭流程外部调用者不需要考虑自动换梭的各种模式处理
	参数：无
	返回值：暂无意义，后续准备添加
	说明：在底线警报发现的地方调用本函数，本函数将根据设置的换梭方式和
		  换梭后的起缝方式来自动处理，外部调用者不需要深入处理
	注意：仅在运行状态下调用此回调函数，其他需要启动换梭流程的调用函数
	      bobbin_case_workflow1()
	最后修改日期：2018-9-14
*/
UINT8 bobbin_auto_change_process_callback(void);

#endif

