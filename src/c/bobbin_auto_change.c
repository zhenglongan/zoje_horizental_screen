

/*
--------------------------------------------------------------------------------------
      COPYRIGHT(C) 2018 Beijing xingdahao technology Co., Ltd.
                     ALL RIGHTS RESERVED 
  Project Number: sewing_machine_controller 
  File Name : bobbin_auto_change.c
  Description: bobbin auto change control 
  Version     Date     Author    Description
--------------------------------------------------------------------------------------

*/


#include "..\..\include\bobbin_auto_change.h"
#include "..\..\include\sfr62p.h"       //M16C/62P special function register definitions
#include "..\..\include\typedef.h"      //Data type define
#include "..\..\include\variables.h"    //External variables declaration
#include "..\..\include\common.h"       //Common constants definition
#include "..\..\include\delay.h"        //delay time definition
#include "..\..\include\iic_bus_eeprom.h"

/****************************************************************
					 全局变量定义
				 
****************************************************************/

/*
	抓臂电机工作电流档位，原竖屏K160
	主控程序已经未使用，无需关注，电流档位设置在系统参数第1组的
	参数127、128
*/
UINT8 	bobbin_case_current_level;
//用于调试换梭功能的变量，借用检测模式里的输出检测,后续可能删除
UINT8 	bobbin_case_debug_cmd;



/****************************************************************
				静态变量（模块内容变量）定义
				 
****************************************************************/
/*
	自动换梭使能，原竖屏K151
	1-打开
	其他-关闭 
*/
static UINT8 	bobbin_case_enable;

//因为横屏暂时没有开发自动换梭相关的参数，因此这里先把设置数据
//暂时放到系统参数第9组
static SYSTEM_PARA9 para9;
/*
	空梭芯丢弃位置，单位是步，原中捷竖屏默认50，未开放此参数
	仅当K164=1时，即空梭芯处理方式是放收纳盒时有效
*/
static UINT16 	bobbin_case_dump_position;//空梭芯丢弃位置,默认50
/*
	梭盘电机零位补偿，原竖屏K192
	梭盘电机原点补偿，根据实际机械安装情况进行调整
*/
static INT8 	bobbin_plateform_org_offset;//k192，梭盘电机零位补偿
/*
	机头对接位置修正补偿，原竖屏K156
	换梭臂电机到<机头位置>的微调值，根据实际机械安装情况进行调整
*/
static INT8  	bobbin_case_arm_offset;

/*
	换梭对接位置修正补偿，原竖屏K157
	换梭臂电机到<梭盘位置>的微调值，根据实际机械安装情况进行调整
*/
static INT8   	bobbin_case_platform_offset;

/*
	伸缩气缸到位延时，原竖屏K158,单位是ms
	加延时是为了保证动作完成，气缸有动作时间，系统中没有传感器获取
	位置信息所以需要延时等待
*/
static UINT16 	bobbin_case_inout_delay;

/*
	换梭臂抓紧气缸延时，原竖屏K159,单位是ms
	加延时是为了保证动作完成，气缸有动作时间，系统中没有传感器获取
	位置信息所以需要延时等待
*/
static UINT16 	bobbin_case_scrath_delay;
/*
	换梭臂停止位置，原竖屏K161
	*0-梭盘侧，这边更好些，可以防止机械臂意外伸出撞到下轴
	1-机头侧
*/
static UINT8 	bobbin_case_stop_position;
/*
	空梭芯处理方式，原竖屏K164
	0-放回梭盘，可能会导致后续换梭取到空梭芯
	*1-放收纳盒（丢弃位置由变量bobbin_case_dump_position，即参数17指定）
*/
static UINT8  	bobbin_case_workmode;

/*
	换梭起缝方式，原竖屏K163
	自动换梭以后启动方式，仅在K162=1时有意义
	0-手动启动
	1-自动启动
*/
static UINT8 	bobbin_case_restart_mode;

/*
	变量这两个变量用于在TA0中断中对梭盘按钮BOBBIN_CASE_SWITCH进行检测
	当连续100ms检测到电平为高时，将置位bobbin_case_switch_flag，只有
	将bobbin_case_switch_flag清零后，才会重新开始检测
*/
//extern UINT8  	bobbin_case_switch_counter;
static UINT8  	bobbin_case_switch_flag;
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
static UINT8 	bobbin_case_alarm_mode;

/*
	针对底线警报后手动换梭（k162_bobbin_case_alarm_mode=0）的方式，
	程序的处理机制是：发现底线警报时只报错，当用户在UI面板上按下
	确认时，面板将发送改变状态指令（功能码0x81），在这里面把此标志
	置1然后再进入READY状态，在REARY状态中响应换梭流程，然后等待用户
	按下启动按钮DVA后再开始缝制
*/
static UINT8  	bobbin_case_once_done_flag;//手动运行一次换梭动作



//============================================================================
//      静态函数（仅本文件内部有效）声明
//============================================================================
static void bobbin_case_motor_adjust(void);
static UINT8 get_bobbin_case_arm_org_status(void);


//============================================================================
//      模块需要使用的外部函数声明
//============================================================================
extern UINT16 string2int(UINT8 *src);
//step_motor_drv.c中提供的DSP底层支持函数
extern void movestep_cs3(UINT16 command,INT16 x_data,UINT8 timer_need);
extern UINT16 check_DSP3_input(void);
extern void inpress_down(UINT8 pos);


/*
	名称：IO和变量的初始化，包括从EEPROM中取出第9组参数信息
	参数：无
	返回值：无
	说明：开机时调用一次即可，
	最后修改日期：2018-9-14
*/
void bobbin_init(void)
{
//============================================================================
//      IO的方向设置已经在函数init_io()中处理了，这里不需要再额外处理
//============================================================================

//============================================================================
//		换梭相关变量初始化
//============================================================================

	bobbin_case_current_level = 5;

	bobbin_case_switch_flag = 0;
	
	bobbin_case_once_done_flag = 0;

	
	//读取系统参数第9组，这里暂时存着自动换梭相关参数配置等
	//相当于竖屏里从面板获取K系列相关参数
	bobbin_get_configuration();


	
}

/*
	名称：查看自动换梭功能是否打开
	参数：无
	返回值：1-打开，其他-关闭
	说明：外部程序调用自动换梭模块相关函数前，需要先调用本函数查询
	      是否开启了自动换梭功能，只有开启了才能执行后续的操作
	最后修改日期：2018-9-14
*/
UINT8 bobbin_check_module_enable(void)
{
	if(bobbin_case_enable==1)
		return 1;
	else
		return 0;
}

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
UINT8 bobbin_check_need_aciton_flag(void)
{
	return bobbin_case_once_done_flag;

}

/*
	名称：清除READY状态下是否需要自动换梭动作标志
	参数：无
	返回值无
	说明：当READY状态中调用函数bobbin_check_need_aciton_flag()发现需要
	      自动换梭，便调用此函数清除标志，防止反复触发自动换梭流程
	最后修改日期：2018-9-14
*/
void bobbin_clear_need_aciton_flag(void)
{
	bobbin_case_once_done_flag=0;
}


/*





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
void go_origin_bobbin_case_arm(UINT8 pos)
{
	UINT8 i,ret;
	UINT16 temp16;
	static bobbin_case_arm_position=0;
		
	if(bobbin_case_enable!=1)//未打开换梭功能时，将直接返回
		return;
	
	if( bobbin_case_arm_position == 0) //上电没找过原点
	{
		if( get_bobbin_case_arm_org_status() ==1)//光耦挡片没有挡上，在工作范围之外了
		{
			ret = find_a_bobbin_case(1);//先找个有旋梭的
			if(ret == 0)
			{
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
				   //if(sys.error == 0) //必须注释掉，否则如果K162=1，k163=0时，如果换梭失败，主控面板不能检测出来，直接忽略了	 
	      		   sys.error = ERROR_88;
				return;
			}
			//先顺时针找到挡片
			temp16 = 0;
			while( get_bobbin_case_arm_org_status() == 1)
			{
				temp16 = temp16 + 1;
				movestep_cs3(0x2000,1,1);   
				delay_ms(5);
				//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
				#if 1
	  	  		if(temp16 > 1600)// 0.45步距角
				#else
				if(temp16 > 800)// 0.45步距角
				#endif
		  	  	{
					sys.status = ERROR;
					StatusChangeLatch = ERROR;
					   //if(sys.error == 0) //必须注释掉，否则如果K162=1，k163=0时，如果换梭失败，主控面板不能检测出来，直接忽略了	 
	      			   sys.error = ERROR_89;     
	      			return;
				}
			}
			//假设是梭盘位置，抓一下看看
			BOBBIN_CASE_ARM_SCRATH = 0;  //机械手松开
			BOBBIN_CASE_ARM_OUT = 1;     //机械臂伸出去
			delay_ms(bobbin_case_inout_delay);
			//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
			#if 1
			movestep_cs3(0xa000,para9.bobbin_shake_distance,para9.bobbin_shake_time);
			#else
			movestep_cs3(0xa000,50,5);//shake_move1			
			#endif
			delay_ms(250);
			BOBBIN_CASE_ARM_SCRATH = 1;  //机械手夹紧
			//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
			#if 1
			movestep_cs3(0xa000,-para9.bobbin_shake_distance,para9.bobbin_shake_time);
			#else
			movestep_cs3(0xa000,-50,5);//shake_move2
			#endif
			delay_ms(200);
			BOBBIN_CASE_ARM_OUT = 0;     //机械臂收回
			delay_ms(300);
			if( BOBBIN_CASE_EMPTY_CHECK == 0)//抓过来了，位置正确
			{
				BOBBIN_CASE_ARM_OUT = 1;     //机械臂伸出去
				delay_ms(bobbin_case_inout_delay);
				BOBBIN_CASE_ARM_SCRATH = 0;  //机械手松开
				delay_ms(200);
				BOBBIN_CASE_ARM_OUT = 0;     //机械臂收回
				delay_ms(300);
			}
			else //没找到
			{
				temp16 = 0;
				for(i = 0;i<20;i++)
			    {
					movestep_cs3(0x2000,-1,1);   
					//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
					#if 1
					delay_ms(2);
					#else
					delay_us(600);
					#endif
				 }
				while( get_bobbin_case_arm_org_status() == 1)
				{
					temp16 = temp16 + 1;
					movestep_cs3(0x2000,-1,1);   
					delay_ms(5);
					//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
					#if 1
		  	  		if(temp16 > 1600)// 0.45步距角
					#else
					if(temp16 > 800)// 0.36步距角
					#endif
			  	  	{
						sys.status = ERROR;
						StatusChangeLatch = ERROR;
						   //if(sys.error == 0) //必须注释掉，否则如果K162=1，k163=0时，如果换梭失败，主控面板不能检测出来，直接忽略了	 
		      			   sys.error = ERROR_89;     
		      			return;
					}
				}
			}
			bobbin_case_arm_position = 100;
		}
		else //在挡边里边
		{
			if( pos == 0 ) //要找梭盘
			    bobbin_case_arm_position = 50;
			else
			    bobbin_case_arm_position = 100;
		}
	}
	
	if ( (pos == 0 )&&(bobbin_case_arm_position == 50) )//从 机头 往 梭盘 旋转 校正过程
	{
		temp16 = 0;
		while( get_bobbin_case_arm_org_status() == 1)//如果挡片没挡上
		{
			movestep_cs3(0x2000,-1,1);   //逆时针
			//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
			#if 1
			delay_ms(1);
			#else
			delay_us(300);
			#endif
			temp16 = temp16 + 1;
  	  		//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
			#if 1
		  	if(temp16 > 1600)// 0.45步距角
			#else
			if(temp16 > 800)// 0.36步距角
			#endif
	  	  	{
	  	  			sys.status = ERROR;
					StatusChangeLatch = ERROR;
					   //if(sys.error == 0) //必须注释掉，否则如果K162=1，k163=0时，如果换梭失败，主控面板不能检测出来，直接忽略了	 
	      			   sys.error = ERROR_89;     
	      			return;
	  	  	}

		}
		for(i=0;i<5;i++)//多走几步确认
		{
			movestep_cs3(0x2000,-1,1);   //逆时针
			delay_ms(1);
		}
	}
	if ( (pos == 1 )&&(bobbin_case_arm_position == 100) )//从 梭盘 向 机头 旋转 校正过程
	{
		temp16 = 0;
		while( get_bobbin_case_arm_org_status() == 1)
		{
			movestep_cs3(0x2000,1,1);  //顺时针
			//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
			#if 1
			delay_ms(1);
			#else
			delay_us(300);
			#endif
			temp16 = temp16 + 1;
			//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
			#if 1
  	  		if(temp16 > 1600)
			#else
			if(temp16 > 800)
			#endif
	  	  	{
	  	  			sys.status = ERROR;
					StatusChangeLatch = ERROR;
					   //if(sys.error == 0) //必须注释掉，否则如果K162=1，k163=0时，如果换梭失败，主控面板不能检测出来，直接忽略了	 
	      			   sys.error = ERROR_89;     
	      			return;
	  	  	}
	
		}
	}
	
	if( get_bobbin_case_arm_org_status() == 0)//如果挡片已经挡住了
	{	
		temp16 = 0;
		//先试试一段快走
		
		while(get_bobbin_case_arm_org_status() == 0)
	   	{
			if( pos == 0)//梭盘
				movestep_cs3(0x2000,-1,1);   //逆时针
			else
			    movestep_cs3(0x2000,1,1);  //顺时针
			//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
			#if 1
			delay_ms(1);
			#else
			delay_us(300);
			#endif			

			temp16 = temp16 + 1;
  	  		//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
			#if 1
  	  		if(temp16 > 1600)
			#else
			if(temp16 > 800)
			#endif
	  	  	{
	  	  			sys.status = ERROR;
					StatusChangeLatch = ERROR;
					   //if(sys.error == 0) //必须注释掉，否则如果K162=1，k163=0时，如果换梭失败，主控面板不能检测出来，直接忽略了	 
	      			   sys.error = ERROR_89;     
	      			return;
	  	  	}
	
		}
		for(i=0;i<15;i++)//多走几步确认退出来
		{
			if( pos == 0)//梭盘
				movestep_cs3(0x2000,-1,1);   //逆时针
			else
			    movestep_cs3(0x2000,1,1);  //顺时针
			//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
			#if 1
			delay_ms(1);
			#else
			delay_us(600);
			#endif
		}
	}
	
	temp16 = 0;
	while(get_bobbin_case_arm_org_status() == 1)//找到挡上的沿 =1表示没挡上（有一级非门）
	{
		if( pos == 0)//梭盘
			movestep_cs3(0x2000,1,1); 
		else
			movestep_cs3(0x2000,-1,1);
		delay_ms(8);
		
		temp16 = temp16 + 1;
  	  	//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
		#if 1
  	  		if(temp16 > 1600)
		#else
			if(temp16 > 800)
		#endif
	  	{
	  	  	sys.status = ERROR;
			StatusChangeLatch = ERROR;
			   //if(sys.error == 0) //必须注释掉，否则如果K162=1，k163=0时，如果换梭失败，主控面板不能检测出来，直接忽略了	 
	      	   sys.error = ERROR_89;     
	      	return;
	  	}
	}
	
	if( pos == 0)//梭盘
	{
		if( bobbin_case_platform_offset != 0 )
		{
			movestep_cs3(0x2000,bobbin_case_platform_offset,31); //换梭对接位置修正补偿
			delay_ms(70);
		}
		bobbin_case_arm_position = 100; //当前在梭盘
	}
	else
	{
		if( bobbin_case_arm_offset != 0 )
		{
			movestep_cs3(0x2000,bobbin_case_arm_offset,31);//机头对接位置修正补偿
			delay_ms(70);
		}
		bobbin_case_arm_position = 50; //当前在机头
	}
	
}

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
UINT8 find_a_bobbin_case(UINT8 full)
{
	UINT8 i,j;
	UINT16 temp16;
	if(bobbin_case_enable!=1)//未打开换梭功能时，将直接返回
		return 0;//返回失败
	
	//加个偏移位置
	if( bobbin_plateform_org_offset !=0 )//k192，梭盘电机零位补偿
	{
		movestep_cs3(0xa000,-bobbin_plateform_org_offset,30);
		delay_ms(100);
	}
	
	for( j=1; j<=8 ; j++)//一圈最多8个梭芯
	{
		//要找一个空梭芯，现在已经在缺口里，并且空位
		if( (full == 0)&&(BOBBIN_CASE_PLATFORM_ORG )&&(BOBBIN_CASE_EMPTY_CHECK == 0) )//要找空位发现当前就是空位
		{
				temp16 = 0;
				while(BOBBIN_CASE_PLATFORM_ORG != 0)
			   	{
					//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
					#if 1
					movestep_cs3(0xa000,1,1);//反向动作从缺口里退出来
					delay_ms(para9.bobbin_platform_speed);
					#else
					movestep_cs3(0xa000,8,5);//反向动作从缺口里退出来		
					delay_ms(1);			
					#endif					
					temp16 = temp16 + 1;
		  	  		if( temp16 > 20000)
			  	  	{
			  	  		sys.status = ERROR;
						StatusChangeLatch = ERROR;
						   //if(sys.error == 0) //必须注释掉，否则如果K162=1，k163=0时，如果换梭失败，主控面板不能检测出来，直接忽略了	 
			      		   sys.error = ERROR_89;     
			      		return 0;
			  	  	}
				}
		}
		else
		{
			if( BOBBIN_CASE_PLATFORM_ORG !=0 )//如果已经在缺口
			{
				temp16 = 0;
				while(BOBBIN_CASE_PLATFORM_ORG != 0)
			   	{
					//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
					#if 1
					movestep_cs3(0xa000,-1,1);//接着前进，电机走一步
					delay_ms(para9.bobbin_platform_speed);
					#else
					movestep_cs3(0xa000,-8,5);//接着前进，电机走一步	
					delay_ms(1);				
					#endif
					
					temp16 = temp16 + 1;
		  	  		if( temp16 > 20000)
			  	  	{
			  	  		sys.status = ERROR;
						StatusChangeLatch = ERROR;
						   //if(sys.error == 0) //必须注释掉，否则如果K162=1，k163=0时，如果换梭失败，主控面板不能检测出来，直接忽略了	 
			      		   sys.error = ERROR_89;     
			      		return 0;
			  	  	}

				}
				for( i=0; i<3; i++)
				{
					//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
					#if 1
					movestep_cs3(0xa000,-1,1);//电机走一步
					delay_ms(para9.bobbin_platform_speed);
					#else
					movestep_cs3(0xa000,-8,5);//电机走一步
					delay_ms(1);
					#endif
					
				}
			}
		}
		//到这基本上都是在挡上的状态，没进入到缺口里，找空位并且当前空位就回退，否则直接走到挡上为止。
		temp16 = 0;
		while(BOBBIN_CASE_PLATFORM_ORG == 0)//挡片挡上了，往缺口方向走
		{
					//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
					#if 1
					movestep_cs3(0xa000,-1,1);//电机顺时针走一步，找到不挡的跳变沿
					delay_ms(para9.bobbin_platform_speed);
					#else
					movestep_cs3(0xa000,-2,5);//电机顺时针走一步，找到不挡的跳变沿
					delay_ms(1);
					#endif
					temp16 = temp16 + 1;
		  	  		if( temp16 > 20000)
			  	  	{
			  	  		sys.status = ERROR;
						StatusChangeLatch = ERROR;
						   //if(sys.error == 0) //必须注释掉，否则如果K162=1，k163=0时，如果换梭失败，主控面板不能检测出来，直接忽略了	 
			      		   sys.error = ERROR_89;     
			      		return 0;
			  	  	}

					if ( BOBBIN_CASE_PLATFORM_ORG !=0 )
					    delay_ms(100);
		}
		//加个偏移位置
		if( bobbin_plateform_org_offset !=0 )
		{
			movestep_cs3(0xa000,bobbin_plateform_org_offset,30);
		}
		
			if( full == 1)//找个放梭芯的
			{
				if( BOBBIN_CASE_EMPTY_CHECK == 1)//有信号反馈
				{
					delay_ms(100);
					if( BOBBIN_CASE_EMPTY_CHECK == 1)
					{
						//bobbin_case_platform_position = (bobbin_case_platform_position + j)%9;
						//return bobbin_case_platform_position;
						return 1;
					}
				}
			}
			else if( full == 0)
			{
				if( BOBBIN_CASE_EMPTY_CHECK == 0)//空梭
				{
					delay_ms(100);
					if( BOBBIN_CASE_EMPTY_CHECK == 0)
					{
						//bobbin_case_platform_position = (bobbin_case_platform_position + j)%9;
						//return bobbin_case_platform_position;
						return 1;
					}
				}
			}
			else
			  break;
	}
	return 0;
}


/*
	名称：启动完整的自动换梭流程
	参数：无
	返回值：无
	说明：启动一个自动换梭流程，这里没有考虑到换梭模式和换梭后启动方式，
	      因此只能在除了RUN状态以外的地方使用，RUN状态下自动换梭要调用
	      函数：bobbin_auto_change_process_callback()
	最后修改日期：2018-9-14
*/
UINT8 bobbin_case_workflow1(void)
{
	UINT8 empty,full,j,i,k;
	
	if(bobbin_case_enable!=1)//未打开换梭功能时，将直接返回
		return 0;//返回失败
		
	//#if MACHINE_900_BOBBIN_DEBUG_MODE
	#if 0
	#else
	bobbin_case_motor_adjust();//调整主轴位置到80度，方便取出下轴空梭芯
	#endif
	
	for( i = 0; i<8 ; i++)
	{
		BOBBIN_CASE_ARM_SCRATH = 0;  //机械手松开
		BOBBIN_CASE_ARM_OUT = 0;     //机械臂收回
		go_origin_bobbin_case_arm(1);//先转到机头接抓位置
		delay_ms(100);
		BOBBIN_CASE_ARM_OUT = 1;     //机械臂伸出去
		delay_ms(bobbin_case_inout_delay);//前后抓紧气缸到位延时
		BOBBIN_CASE_ARM_SCRATH = 1;  //机械手夹紧
		delay_ms(bobbin_case_scrath_delay);//夹紧气缸到位延时
		BOBBIN_CASE_ARM_OUT = 0;     //机械臂收回
		delay_ms(500);
		if( bobbin_case_workmode == 0)//空梭芯放到梭盘
		{
			go_origin_bobbin_case_arm(0);//转到梭盘对应位置
			delay_ms(100);
			empty = find_a_bobbin_case(0);
			if( empty == 0 )
			{
				//报错退出
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
				//if(sys.error == 0) //必须注释掉，否则如果K162=1，k163=0时，如果换梭失败，主控面板不能检测出来，直接忽略了   
	      		   sys.error = ERROR_88;
				return 0;
			}
			BOBBIN_CASE_ARM_OUT = 1;     //机械臂伸出去
			delay_ms(bobbin_case_inout_delay);
			//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
			#if 1
			movestep_cs3(0xa000,para9.bobbin_shake_distance,para9.bobbin_shake_time);
			#else
			movestep_cs3(0xa000,90,31);//shake_move1
			#endif
			delay_ms(250);
			BOBBIN_CASE_ARM_SCRATH = 0;  //机械手松开
			delay_ms(300);
			//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
			#if 1
			movestep_cs3(0xa000,-para9.bobbin_shake_distance,para9.bobbin_shake_time);
			#else
			movestep_cs3(0xa000,-90,31);//shake_move2
			#endif
			delay_ms(200);
			BOBBIN_CASE_ARM_OUT = 0;     //机械臂收回
			delay_ms(200);
			if( BOBBIN_CASE_EMPTY_CHECK == 0)//如果发现当前位置没有信号，则有可能认为是没抓过来
			{
				if( i< 3)//试抓3次
				  continue;
				else     //直接从梭盘抓一个
				{
				}
			}
		}
		else //丢弃
		{
			go_origin_bobbin_case_arm(0);//转到收集位置
			delay_ms(50);
			for(k=0; k<bobbin_case_dump_position; k++)//试探着走50步看看
			{
				movestep_cs3(0x2000,-1,1);   //逆时针
				delay_ms(2);
			}
			BOBBIN_CASE_ARM_OUT = 1;     //机械臂伸出去
			delay_ms(bobbin_case_inout_delay);
			BOBBIN_CASE_ARM_SCRATH = 0;  //机械手松开
			delay_ms(200);
			BOBBIN_CASE_ARM_OUT = 0;     //机械臂收回
			delay_ms(200);
			go_origin_bobbin_case_arm(0);//转到收集位置
			delay_ms(100);
		}

			//放入梭壳后，确认传感器没有被晃动到外边
			k=0;
			while((BOBBIN_CASE_PLATFORM_ORG == 0)&&(k<20) )//挡片挡上了
			{
				movestep_cs3(0xa000,-2,5);//电机顺时针走一步，找到不挡的跳变沿
				delay_ms(1);
				k++;
			}
			for( j = 0; j<8 ; j++)
			{
				full = find_a_bobbin_case(1);//梭盘转到有梭芯的位置
				if( full == 0)
				{
					sys.status = ERROR;
					StatusChangeLatch = ERROR;
				//if(sys.error == 0) //必须注释掉，否则如果K162=1，k163=0时，如果换梭失败，主控面板不能检测出来，直接忽略了	 
		      		   sys.error = ERROR_88;
					return 0;
			    } 
				delay_ms(200);
				BOBBIN_CASE_ARM_OUT = 1;     //机械臂伸出去
				delay_ms(bobbin_case_inout_delay);
				//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
				#if 1
				movestep_cs3(0xa000,para9.bobbin_shake_distance,para9.bobbin_shake_time);
				#else
				movestep_cs3(0xa000,90,31);//shake_move1
				#endif
				delay_ms(250);
				BOBBIN_CASE_ARM_SCRATH = 1;  //机械手夹紧
				delay_ms(bobbin_case_scrath_delay);
				//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
				#if 1
				movestep_cs3(0xa000,-para9.bobbin_shake_distance,para9.bobbin_shake_time);
				#else
				movestep_cs3(0xa000,-90,31);//shake_move1
				#endif
				delay_ms(250);
				BOBBIN_CASE_ARM_OUT = 0;     //机械臂收回
				delay_ms(300);
				//核查一下
				if( BOBBIN_CASE_EMPTY_CHECK == 1)//有信号反馈,没抓出来
				{
					BOBBIN_CASE_ARM_SCRATH = 0;  //机械手松开
					go_origin_bobbin_case_arm(0);//转到梭盘对应位置
					delay_ms(200);
					if(j>=7)//如果已经找了8次了，都没能够把梭芯从梭盘上抓出，此时应该报错，2018-9-13
					{
						sys.status = ERROR;
						StatusChangeLatch = ERROR;
						//if(sys.error == 0) //必须注释掉，否则如果K162=1，k163=0时，如果换梭失败，主控面板不能检测出来，直接忽略了	 
			      		   sys.error = ERROR_89;
						return 0;
					}
				    continue;
				}
				go_origin_bobbin_case_arm(1);//转到机头对接位置
				delay_ms(100);
				BOBBIN_CASE_ARM_OUT = 1;     //机械臂伸出去
				delay_ms(1000);
				BOBBIN_CASE_ARM_SCRATH = 0;  //机械手松开
				delay_ms(200);
				BOBBIN_CASE_ARM_OUT = 0;     //机械臂收回
				delay_ms(500);
				if( bobbin_case_stop_position == 0)//换梭停止位置0-梭盘侧，1-机头侧
				    go_origin_bobbin_case_arm(0);//转到梭盘对应位置
				break;
			}
		break;
	  }	
	return 1;
}

/*
x32:5V
1-vcc 2-signal 3-gnd :0x01
4-vcc 5-signal 6-gnd :0x02
7-vcc 8-signal 9-gnd :0x04
24V
11-vcc 12-signal 13-gnd :0x08
14-vcc 15-signal 16-gnd :0x10
17-vcc 18-signal 19-gnd :0x20
*/
UINT8 get_bobbin_case_arm_org_status(void)
{
	//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
	#if 1
	if( BOBBIN_CASE_ARM_ORG == 1)
		return 1;
	else
		return 0;
	#else
	return (check_DSP3_input()&0x01);
	#endif
}

/*
	名称：从EEPROM中读取第9组参数
	参数：无
	返回值：无
	说明：bobbin_init()将调用，当SALCK状态下修改系统参数第9组后，
	      也将调用此函数更新配置参数
	最后修改日期：2018-9-14
*/
void bobbin_get_configuration(void)
{
	UINT16 index;
	index = 0;

	//读取系统参数第9组，这里暂时存着自动换梭相关参数配置等
	read_para_group(1600,svpara_disp_buf,205);

	//先读取到结构体para9中
	para9.k151_bobbin_case_enable = svpara_disp_buf[index++];
	para9.k156_bobbin_case_arm_offset = svpara_disp_buf[index++];
	para9.k157_bobbin_case_platform_offset = svpara_disp_buf[index++];
	para9.k158_bobbin_case_inout_delay  = string2int(&svpara_disp_buf[index]);	index +=2;
	para9.k159_bobbin_case_scrath_delay  = string2int(&svpara_disp_buf[index]);	index +=2;
	para9.k160_bobbin_case_current_level = svpara_disp_buf[index++];
	para9.k161_bobbin_case_stop_position = svpara_disp_buf[index++];
	para9.k162_bobbin_case_alarm_mode = svpara_disp_buf[index++];
	para9.k163_bobbin_case_restart_mode = svpara_disp_buf[index++];
	para9.k164_bobbin_case_workmode = svpara_disp_buf[index++];
	para9.k192_bobbin_plateform_org_offset = svpara_disp_buf[index++];
	para9.bobbin_platform_speed = svpara_disp_buf[index++];
	para9.bobbin_shake_distance = svpara_disp_buf[index++];
	para9.bobbin_shake_time = svpara_disp_buf[index++];
	para9.bobbin_case_dump_position = svpara_disp_buf[index++];//17

	//启用了自动换梭功能
	if(para9.k151_bobbin_case_enable==1)
	{
		//然后从para9转存到对应的全局变量中
		bobbin_case_enable=para9.k151_bobbin_case_enable;
		bobbin_case_arm_offset=para9.k156_bobbin_case_arm_offset;
		bobbin_case_platform_offset=para9.k157_bobbin_case_platform_offset;
		bobbin_case_inout_delay=para9.k158_bobbin_case_inout_delay;
		bobbin_case_scrath_delay=para9.k159_bobbin_case_scrath_delay;
		bobbin_case_current_level=para9.k160_bobbin_case_current_level;
		bobbin_case_stop_position=para9.k161_bobbin_case_stop_position;
		bobbin_case_alarm_mode=para9.k162_bobbin_case_alarm_mode;
		bobbin_case_restart_mode=para9.k163_bobbin_case_restart_mode;
		bobbin_case_workmode=para9.k164_bobbin_case_workmode;
		bobbin_plateform_org_offset=para9.k192_bobbin_plateform_org_offset;
		bobbin_case_dump_position=para9.bobbin_case_dump_position;
	}
	else
	{
		bobbin_case_enable=0;
		bobbin_case_arm_offset=0;
		bobbin_case_platform_offset=0;
		bobbin_case_inout_delay=0;
		bobbin_case_scrath_delay=0;
		bobbin_case_current_level=0;
		bobbin_case_stop_position=0;
		bobbin_case_alarm_mode=0;
		bobbin_case_restart_mode=0;
		bobbin_case_workmode=0;
		bobbin_plateform_org_offset=0;
		bobbin_case_dump_position=50;
	}

}


/****************************************************************
					 回调函数定义
				 
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
void  bobbin_check_switch_callback(UINT8 polarity, UINT16 times)
{
	static UINT16 bobbin_case_switch_counter=0;
	
	//只有自动换梭功能打开才有效
	if(bobbin_case_enable!=1)//未打开换梭功能时，将直接返回
		return;//返回失败

	if( BOBBIN_CASE_SWITCH == polarity)//检测到指定的有效电平
	{
		if( bobbin_case_switch_flag == 0)
		{
			bobbin_case_switch_counter++;
			if( bobbin_case_switch_counter > times )//超过指定次数
			{
				bobbin_case_switch_flag = 1;
			}
		}
		else
		    bobbin_case_switch_counter = 0;
	}
	else
	{
		bobbin_case_switch_counter = 0;
		bobbin_case_switch_flag = 0;
	}
}
/*
	名称：根据梭盘动作按钮是否动作来控制梭盘电机移动到下一个位置
	参数：无
	返回值：无
	说明：在需要响应梭盘动作的地方调用此函数即可，但是别忘了在
		  定时器TA0中断中调用函数bobbin_check_switch_callback()
		  扫描按钮是否按下
	最后修改日期：2018-9-14
*/
void bobbin_platform_move_to_next_callback(void)
{
	if(bobbin_case_enable!=1)//未打开换梭功能时，将直接返回
		return;//返回失败
	if( bobbin_case_switch_flag ==1 )
	{
		bobbin_case_switch_flag = 0;
		find_a_bobbin_case(5);//移动梭盘电机到下一个位置
	}
}


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
//UINT8 bobbin_auto_change_process_callback(UINT16 *error_code)
UINT8 bobbin_auto_change_process_callback(void)
{
	UINT8 temp8;
	temp8 = 0;
	if( bobbin_case_alarm_mode == 1)//先换梭再报警
	{
		sys.status =  ERROR;//报错
		sys.error = ERROR_81;
		//error_code = ERROR_81;
		if( bobbin_case_restart_mode == 1)//换梭起缝方式0-手动启动，1-自动启动
		{
			//用于告知面板，底线不足,这个时候即使用户按下了确认将状态切换到READY也无所谓
			//因为后面又把状态恢复到了RUN
			delay_ms(1000);
			sys.status =  RUN;//面板显示的底线不足自动消失，继续缝制
			sys.error = 0;
			StatusChangeLatch = READY;
		}
		
		temp8 = bobbin_case_workflow1();//启动换梭流程
		if(temp8==0)//换梭失败了
		{
			delay_ms(500);
			//换梭失败，返回，相关的错误状态已经在函数bobbin_case_workflow1()中被设置了
			return 1;
		}
		bobbin_case_once_done_flag = 0;
		if(bobbin_case_restart_mode == 1)
		{
			delay_ms(20);
		}
		
	}
	else//如果手动换梭，直接跳转到这里，等待拿下面板确认，然后跳转到READY中换梭，然后等待DVA按下后重新启动
	{
		sys.status =  ERROR;
		sys.error = ERROR_81;
		StatusChangeLatch = ERROR;
		bobbin_case_once_done_flag = 1;//表示切换到READY状态后需要启动换梭
		return 1;
	}
	return 0;//正常返回
}




//============================================================================
//      静态函数（仅本文件内部有效）定义
//============================================================================
/*
	主轴停车到特定位置：80度，这样方便
*/
static void bobbin_case_motor_adjust(void)
{	 
	UINT8 temp8;	
	INT16 temp16;	
	while(motor.stop_flag == 0)    
  	{
    	rec_com(); 
  	}

	temp16 = motor.angle_adjusted;
	if( (temp16 >256)||(temp16 <200) )
	{
		//inpress_down(inpress_high_hole);//后续增加这个高度的处理，原来的程序没有这个变量
		inpress_down(inpress_high_base);
		motor.dir = 0;
		motor.spd_obj = 100;
		while(1)
	   	{
	    		rec_com(); 
		    	if(motor.spd_ref == motor.spd_obj)
		    	{
			    	break;
		    	}
	   	}
		motor.stop_angle = 228;//80d
		motor.spd_obj = 0;  
	    while(motor.stop_flag == 0)    
		{
		  rec_com(); 
	     
		}
		delay_ms(280);
		inpress_up();
		delay_ms(80);
	}
}































