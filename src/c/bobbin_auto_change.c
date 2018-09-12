

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

/****************************************************************
					 全局变量定义
				 
****************************************************************/
UINT8 bobbin_case_enable;//k151，自动换梭使能
UINT8  bobbin_case_switch_counter,bobbin_case_switch_flag,bobbin_case_workmode;
INT8   bobbin_case_arm_offset,bobbin_case_platform_offset;
UINT8  bobbin_case_arm_position,bobbin_case_platform_position;
UINT16 bobbin_case_dump_position;
UINT16 bobbin_case_inout_delay,bobbin_case_scrath_delay;
INT8 bobbin_plateform_org_offset;
UINT8 bobbin_case_stop_position,bobbin_case_alarm_mode,bobbin_case_restart_mode;
UINT8  bobbin_case_current_level;


//因为横屏暂时没有开发自动换梭相关的参数，因此这里先把设置数据
//暂时放到系统参数第9组
static SYSTEM_PARA9 para9;

//============================================================================
//      静态函数（仅本文件内部有效）声明
//============================================================================
static void bobbin_get_configuration(void);
extern UINT16 string2int(UINT8 *src);



//IO和全局变量的初始化
//开机时调用一次即可
void bobbin_init(void)
{
//============================================================================
//      IO的方向设置已经在函数init_io()中处理了，这里不需要再额外处理
//============================================================================

//============================================================================
//		换梭相关变量初始化
//============================================================================

	/*
		bobbin_case_arm_position，换梭臂的位置
		0			上电没找过原点
		50			位置在机头
		100 		位置在梭盘
	*/
	bobbin_case_arm_position = 0;
	bobbin_case_current_level = 5;
	//读取系统参数第9组，这里暂时存着自动换梭相关参数配置等
	//相当于竖屏里从面板获取K系列相关参数
	bobbin_get_configuration();


	
}

//找机械臂原点
//001x
/*
机械臂定位盘角度220度，对应两个位置：机头对抓取位置 和 换梭盘对接位置
挡片挡住时，机械臂逆时针转，从挡住一直到退出挡片 对应着位置是 换梭盘对接位置
            机械臂顺时针转，从挡住一直到退出挡片 对应着位置是 机头对抓位置
挡片未挡住时，电机顺时针转，
原点默认是 换梭盘对接位置

bobbin_case_arm_position
0           上电没找过原点
50          位置在机头
100         位置在梭盘
pos 
0-------向梭盘转
1-------向机头转
挡片是在电机轴上，有一级皮带传动，所以机械臂和挡片是旋转方向相反，下面所说的方向是机械臂的方向
*/
void go_origin_bobbin_case_arm(UINT8 pos)
{
	UINT8 i,ret;
	UINT16 temp16;
	
	if( bobbin_case_arm_position == 0) //上电没找过原点
	{
		if( get_bobbin_case_arm_org_status() ==1)//光耦挡片没有挡上，在工作范围之外了
		{
			ret = find_a_bobbin_case(1);//先找个有旋梭的
			if(ret == 0)
			{
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
				if(sys.error == 0)    
	      		   sys.error = ERROR_51;
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
					if(sys.error == 0)    
	      			   sys.error = ERROR_49;     
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
						if(sys.error == 0)    
		      			   sys.error = ERROR_49;     
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
					if(sys.error == 0)    
	      			   sys.error = ERROR_49;     
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
					if(sys.error == 0)    
	      			   sys.error = ERROR_49;     
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
					if(sys.error == 0)    
	      			   sys.error = ERROR_49;     
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
			if(sys.error == 0)    
	      	   sys.error = ERROR_49;     
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

//找一个梭盘位置
/*
 电机的旋转方向：负数-梭盘顺时针旋转 正数-梭盘逆时针旋转
 原点传感器： 高电平 表示在挡片缺口里  低电平表示被挡上了。 传感器0528 NPN开漏  挡上时为高电平
 正常找基准位置是： 
  1）电机顺时针转，找下一个挡片由挡到不挡的跳变
  2）找当前空位，电机逆时针转找到挡上的位置，然后再顺时针转到不挡边沿
  full=0 表示找空位；非0表示要找有梭芯的位置
*/
UINT8 find_a_bobbin_case(UINT8 full)
{
	UINT8 i,j;
	UINT16 temp16;
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
						if(sys.error == 0)    
			      		   sys.error = ERROR_50;     
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
						if(sys.error == 0)    
			      		   sys.error = ERROR_50;     
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
						if(sys.error == 0)    
			      		   sys.error = ERROR_50;     
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
停车到特定位置
*/
void bobbin_case_motor_adjust(void)
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
//先 从机头取出 再 选一个放入
UINT8 bobbin_case_workflow1(void)
{
	UINT8 empty,full,j,i,k;
	//#if MACHINE_900_BOBBIN_DEBUG_MODE
	#if 0
	#else
	bobbin_case_motor_adjust();
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
				if(sys.error == 0)    
	      		   sys.error = ERROR_51;
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
				full = find_a_bobbin_case(1);
				if( full == 0)
				{
					sys.status = ERROR;
					StatusChangeLatch = ERROR;
					if(sys.error == 0)    
		      		   sys.error = ERROR_51;
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
				if( BOBBIN_CASE_EMPTY_CHECK == 1)//有信号反馈,没找出来
				{
					BOBBIN_CASE_ARM_SCRATH = 0;  //机械手松开
					go_origin_bobbin_case_arm(0);//转到梭盘对应位置
					delay_ms(200);
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




//============================================================================
//      静态函数（仅本文件内部有效）定义
//============================================================================
static void bobbin_get_configuration(void)
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
	
}

































