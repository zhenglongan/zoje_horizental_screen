

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
					 ȫ�ֱ�������
				 
****************************************************************/
UINT8 bobbin_case_enable;//k151���Զ�����ʹ��
UINT8  bobbin_case_switch_counter,bobbin_case_switch_flag,bobbin_case_workmode;
INT8   bobbin_case_arm_offset,bobbin_case_platform_offset;
UINT8  bobbin_case_arm_position,bobbin_case_platform_position;
UINT16 bobbin_case_dump_position;
UINT16 bobbin_case_inout_delay,bobbin_case_scrath_delay;
INT8 bobbin_plateform_org_offset;
UINT8 bobbin_case_stop_position,bobbin_case_alarm_mode,bobbin_case_restart_mode;
UINT8  bobbin_case_current_level;


//��Ϊ������ʱû�п����Զ�������صĲ�������������Ȱ���������
//��ʱ�ŵ�ϵͳ������9��
static SYSTEM_PARA9 para9;

//============================================================================
//      ��̬�����������ļ��ڲ���Ч������
//============================================================================
static void bobbin_get_configuration(void);
extern UINT16 string2int(UINT8 *src);



//IO��ȫ�ֱ����ĳ�ʼ��
//����ʱ����һ�μ���
void bobbin_init(void)
{
//============================================================================
//      IO�ķ��������Ѿ��ں���init_io()�д����ˣ����ﲻ��Ҫ�ٶ��⴦��
//============================================================================

//============================================================================
//		������ر�����ʼ��
//============================================================================

	/*
		bobbin_case_arm_position������۵�λ��
		0			�ϵ�û�ҹ�ԭ��
		50			λ���ڻ�ͷ
		100 		λ��������
	*/
	bobbin_case_arm_position = 0;
	bobbin_case_current_level = 5;
	//��ȡϵͳ������9�飬������ʱ�����Զ�������ز������õ�
	//�൱�������������ȡKϵ����ز���
	bobbin_get_configuration();


	
}

//�һ�е��ԭ��
//001x
/*
��е�۶�λ�̽Ƕ�220�ȣ���Ӧ����λ�ã���ͷ��ץȡλ�� �� �����̶Խ�λ��
��Ƭ��סʱ����е����ʱ��ת���ӵ�סһֱ���˳���Ƭ ��Ӧ��λ���� �����̶Խ�λ��
            ��е��˳ʱ��ת���ӵ�סһֱ���˳���Ƭ ��Ӧ��λ���� ��ͷ��ץλ��
��Ƭδ��סʱ�����˳ʱ��ת��
ԭ��Ĭ���� �����̶Խ�λ��

bobbin_case_arm_position
0           �ϵ�û�ҹ�ԭ��
50          λ���ڻ�ͷ
100         λ��������
pos 
0-------������ת
1-------���ͷת
��Ƭ���ڵ�����ϣ���һ��Ƥ�����������Ի�е�ۺ͵�Ƭ����ת�����෴��������˵�ķ����ǻ�е�۵ķ���
*/
void go_origin_bobbin_case_arm(UINT8 pos)
{
	UINT8 i,ret;
	UINT16 temp16;
	
	if( bobbin_case_arm_position == 0) //�ϵ�û�ҹ�ԭ��
	{
		if( get_bobbin_case_arm_org_status() ==1)//���Ƭû�е��ϣ��ڹ�����Χ֮����
		{
			ret = find_a_bobbin_case(1);//���Ҹ��������
			if(ret == 0)
			{
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
				if(sys.error == 0)    
	      		   sys.error = ERROR_51;
				return;
			}
			//��˳ʱ���ҵ���Ƭ
			temp16 = 0;
			while( get_bobbin_case_arm_org_status() == 1)
			{
				temp16 = temp16 + 1;
				movestep_cs3(0x2000,1,1);   
				delay_ms(5);
				//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
				#if 1
	  	  		if(temp16 > 1600)// 0.45�����
				#else
				if(temp16 > 800)// 0.45�����
				#endif
		  	  	{
					sys.status = ERROR;
					StatusChangeLatch = ERROR;
					if(sys.error == 0)    
	      			   sys.error = ERROR_49;     
	      			return;
				}
			}
			//����������λ�ã�ץһ�¿���
			BOBBIN_CASE_ARM_SCRATH = 0;  //��е���ɿ�
			BOBBIN_CASE_ARM_OUT = 1;     //��е�����ȥ
			delay_ms(bobbin_case_inout_delay);
			//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
			#if 1
			movestep_cs3(0xa000,para9.bobbin_shake_distance,para9.bobbin_shake_time);
			#else
			movestep_cs3(0xa000,50,5);//shake_move1			
			#endif
			delay_ms(250);
			BOBBIN_CASE_ARM_SCRATH = 1;  //��е�ּн�
			//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
			#if 1
			movestep_cs3(0xa000,-para9.bobbin_shake_distance,para9.bobbin_shake_time);
			#else
			movestep_cs3(0xa000,-50,5);//shake_move2
			#endif
			delay_ms(200);
			BOBBIN_CASE_ARM_OUT = 0;     //��е���ջ�
			delay_ms(300);
			if( BOBBIN_CASE_EMPTY_CHECK == 0)//ץ�����ˣ�λ����ȷ
			{
				BOBBIN_CASE_ARM_OUT = 1;     //��е�����ȥ
				delay_ms(bobbin_case_inout_delay);
				BOBBIN_CASE_ARM_SCRATH = 0;  //��е���ɿ�
				delay_ms(200);
				BOBBIN_CASE_ARM_OUT = 0;     //��е���ջ�
				delay_ms(300);
			}
			else //û�ҵ�
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
		  	  		if(temp16 > 1600)// 0.45�����
					#else
					if(temp16 > 800)// 0.36�����
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
		else //�ڵ������
		{
			if( pos == 0 ) //Ҫ������
			    bobbin_case_arm_position = 50;
			else
			    bobbin_case_arm_position = 100;
		}
	}
	
	if ( (pos == 0 )&&(bobbin_case_arm_position == 50) )//�� ��ͷ �� ���� ��ת У������
	{
		temp16 = 0;
		while( get_bobbin_case_arm_org_status() == 1)//�����Ƭû����
		{
			movestep_cs3(0x2000,-1,1);   //��ʱ��
			//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
			#if 1
			delay_ms(1);
			#else
			delay_us(300);
			#endif
			temp16 = temp16 + 1;
  	  		//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
			#if 1
		  	if(temp16 > 1600)// 0.45�����
			#else
			if(temp16 > 800)// 0.36�����
			#endif
	  	  	{
	  	  			sys.status = ERROR;
					StatusChangeLatch = ERROR;
					if(sys.error == 0)    
	      			   sys.error = ERROR_49;     
	      			return;
	  	  	}

		}
		for(i=0;i<5;i++)//���߼���ȷ��
		{
			movestep_cs3(0x2000,-1,1);   //��ʱ��
			delay_ms(1);
		}
	}
	if ( (pos == 1 )&&(bobbin_case_arm_position == 100) )//�� ���� �� ��ͷ ��ת У������
	{
		temp16 = 0;
		while( get_bobbin_case_arm_org_status() == 1)
		{
			movestep_cs3(0x2000,1,1);  //˳ʱ��
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
	
	if( get_bobbin_case_arm_org_status() == 0)//�����Ƭ�Ѿ���ס��
	{	
		temp16 = 0;
		//������һ�ο���
		
		while(get_bobbin_case_arm_org_status() == 0)
	   	{
			if( pos == 0)//����
				movestep_cs3(0x2000,-1,1);   //��ʱ��
			else
			    movestep_cs3(0x2000,1,1);  //˳ʱ��
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
		for(i=0;i<15;i++)//���߼���ȷ���˳���
		{
			if( pos == 0)//����
				movestep_cs3(0x2000,-1,1);   //��ʱ��
			else
			    movestep_cs3(0x2000,1,1);  //˳ʱ��
			//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
			#if 1
			delay_ms(1);
			#else
			delay_us(600);
			#endif
		}
	}
	
	temp16 = 0;
	while(get_bobbin_case_arm_org_status() == 1)//�ҵ����ϵ��� =1��ʾû���ϣ���һ�����ţ�
	{
		if( pos == 0)//����
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
	
	if( pos == 0)//����
	{
		if( bobbin_case_platform_offset != 0 )
		{
			movestep_cs3(0x2000,bobbin_case_platform_offset,31); //����Խ�λ����������
			delay_ms(70);
		}
		bobbin_case_arm_position = 100; //��ǰ������
	}
	else
	{
		if( bobbin_case_arm_offset != 0 )
		{
			movestep_cs3(0x2000,bobbin_case_arm_offset,31);//��ͷ�Խ�λ����������
			delay_ms(70);
		}
		bobbin_case_arm_position = 50; //��ǰ�ڻ�ͷ
	}
	
}

//��һ������λ��
/*
 �������ת���򣺸���-����˳ʱ����ת ����-������ʱ����ת
 ԭ�㴫������ �ߵ�ƽ ��ʾ�ڵ�Ƭȱ����  �͵�ƽ��ʾ�������ˡ� ������0528 NPN��©  ����ʱΪ�ߵ�ƽ
 �����һ�׼λ���ǣ� 
  1�����˳ʱ��ת������һ����Ƭ�ɵ�������������
  2���ҵ�ǰ��λ�������ʱ��ת�ҵ����ϵ�λ�ã�Ȼ����˳ʱ��ת����������
  full=0 ��ʾ�ҿ�λ����0��ʾҪ������о��λ��
*/
UINT8 find_a_bobbin_case(UINT8 full)
{
	UINT8 i,j;
	UINT16 temp16;
	//�Ӹ�ƫ��λ��
	if( bobbin_plateform_org_offset !=0 )//k192�����̵����λ����
	{
		movestep_cs3(0xa000,-bobbin_plateform_org_offset,30);
		delay_ms(100);
	}
	
	for( j=1; j<=8 ; j++)//һȦ���8����о
	{
		//Ҫ��һ������о�������Ѿ���ȱ������ҿ�λ
		if( (full == 0)&&(BOBBIN_CASE_PLATFORM_ORG )&&(BOBBIN_CASE_EMPTY_CHECK == 0) )//Ҫ�ҿ�λ���ֵ�ǰ���ǿ�λ
		{
				temp16 = 0;
				while(BOBBIN_CASE_PLATFORM_ORG != 0)
			   	{
					//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
					#if 1
					movestep_cs3(0xa000,1,1);//��������ȱ�����˳���
					delay_ms(para9.bobbin_platform_speed);
					#else
					movestep_cs3(0xa000,8,5);//��������ȱ�����˳���		
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
			if( BOBBIN_CASE_PLATFORM_ORG !=0 )//����Ѿ���ȱ��
			{
				temp16 = 0;
				while(BOBBIN_CASE_PLATFORM_ORG != 0)
			   	{
					//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
					#if 1
					movestep_cs3(0xa000,-1,1);//����ǰ���������һ��
					delay_ms(para9.bobbin_platform_speed);
					#else
					movestep_cs3(0xa000,-8,5);//����ǰ���������һ��	
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
					movestep_cs3(0xa000,-1,1);//�����һ��
					delay_ms(para9.bobbin_platform_speed);
					#else
					movestep_cs3(0xa000,-8,5);//�����һ��
					delay_ms(1);
					#endif
					
				}
			}
		}
		//��������϶����ڵ��ϵ�״̬��û���뵽ȱ����ҿ�λ���ҵ�ǰ��λ�ͻ��ˣ�����ֱ���ߵ�����Ϊֹ��
		temp16 = 0;
		while(BOBBIN_CASE_PLATFORM_ORG == 0)//��Ƭ�����ˣ���ȱ�ڷ�����
		{
					//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
					#if 1
					movestep_cs3(0xa000,-1,1);//���˳ʱ����һ�����ҵ�������������
					delay_ms(para9.bobbin_platform_speed);
					#else
					movestep_cs3(0xa000,-2,5);//���˳ʱ����һ�����ҵ�������������
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
		//�Ӹ�ƫ��λ��
		if( bobbin_plateform_org_offset !=0 )
		{
			movestep_cs3(0xa000,bobbin_plateform_org_offset,30);
		}
		
			if( full == 1)//�Ҹ�����о��
			{
				if( BOBBIN_CASE_EMPTY_CHECK == 1)//���źŷ���
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
				if( BOBBIN_CASE_EMPTY_CHECK == 0)//����
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
ͣ�����ض�λ��
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
		//inpress_down(inpress_high_hole);//������������߶ȵĴ���ԭ���ĳ���û���������
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
//�� �ӻ�ͷȡ�� �� ѡһ������
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
		BOBBIN_CASE_ARM_SCRATH = 0;  //��е���ɿ�
		BOBBIN_CASE_ARM_OUT = 0;     //��е���ջ�
		go_origin_bobbin_case_arm(1);//��ת����ͷ��ץλ��
		delay_ms(100);
		BOBBIN_CASE_ARM_OUT = 1;     //��е�����ȥ
		delay_ms(bobbin_case_inout_delay);//ǰ��ץ�����׵�λ��ʱ
		BOBBIN_CASE_ARM_SCRATH = 1;  //��е�ּн�
		delay_ms(bobbin_case_scrath_delay);//�н����׵�λ��ʱ
		BOBBIN_CASE_ARM_OUT = 0;     //��е���ջ�
		delay_ms(500);
		if( bobbin_case_workmode == 0)//����о�ŵ�����
		{
			go_origin_bobbin_case_arm(0);//ת�����̶�Ӧλ��
			delay_ms(100);
			empty = find_a_bobbin_case(0);
			if( empty == 0 )
			{
				//�����˳�
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
				if(sys.error == 0)    
	      		   sys.error = ERROR_51;
				return 0;
			}
			BOBBIN_CASE_ARM_OUT = 1;     //��е�����ȥ
			delay_ms(bobbin_case_inout_delay);
			//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
			#if 1
			movestep_cs3(0xa000,para9.bobbin_shake_distance,para9.bobbin_shake_time);
			#else
			movestep_cs3(0xa000,90,31);//shake_move1
			#endif
			delay_ms(250);
			BOBBIN_CASE_ARM_SCRATH = 0;  //��е���ɿ�
			delay_ms(300);
			//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
			#if 1
			movestep_cs3(0xa000,-para9.bobbin_shake_distance,para9.bobbin_shake_time);
			#else
			movestep_cs3(0xa000,-90,31);//shake_move2
			#endif
			delay_ms(200);
			BOBBIN_CASE_ARM_OUT = 0;     //��е���ջ�
			delay_ms(200);
			if( BOBBIN_CASE_EMPTY_CHECK == 0)//������ֵ�ǰλ��û���źţ����п�����Ϊ��ûץ����
			{
				if( i< 3)//��ץ3��
				  continue;
				else     //ֱ�Ӵ�����ץһ��
				{
				}
			}
		}
		else //����
		{
			go_origin_bobbin_case_arm(0);//ת���ռ�λ��
			delay_ms(50);
			for(k=0; k<bobbin_case_dump_position; k++)//��̽����50������
			{
				movestep_cs3(0x2000,-1,1);   //��ʱ��
				delay_ms(2);
			}
			BOBBIN_CASE_ARM_OUT = 1;     //��е�����ȥ
			delay_ms(bobbin_case_inout_delay);
			BOBBIN_CASE_ARM_SCRATH = 0;  //��е���ɿ�
			delay_ms(200);
			BOBBIN_CASE_ARM_OUT = 0;     //��е���ջ�
			delay_ms(200);
			go_origin_bobbin_case_arm(0);//ת���ռ�λ��
			delay_ms(100);
		}

			//������Ǻ�ȷ�ϴ�����û�б��ζ������
			k=0;
			while((BOBBIN_CASE_PLATFORM_ORG == 0)&&(k<20) )//��Ƭ������
			{
				movestep_cs3(0xa000,-2,5);//���˳ʱ����һ�����ҵ�������������
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
				BOBBIN_CASE_ARM_OUT = 1;     //��е�����ȥ
				delay_ms(bobbin_case_inout_delay);
				//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
				#if 1
				movestep_cs3(0xa000,para9.bobbin_shake_distance,para9.bobbin_shake_time);
				#else
				movestep_cs3(0xa000,90,31);//shake_move1
				#endif
				delay_ms(250);
				BOBBIN_CASE_ARM_SCRATH = 1;  //��е�ּн�
				delay_ms(bobbin_case_scrath_delay);
				//#if COMPILE_MACHINE_TYPE == MACHINE_CONFIG_NUMBER39
				#if 1
				movestep_cs3(0xa000,-para9.bobbin_shake_distance,para9.bobbin_shake_time);
				#else
				movestep_cs3(0xa000,-90,31);//shake_move1
				#endif
				delay_ms(250);
				BOBBIN_CASE_ARM_OUT = 0;     //��е���ջ�
				delay_ms(300);
				//�˲�һ��
				if( BOBBIN_CASE_EMPTY_CHECK == 1)//���źŷ���,û�ҳ���
				{
					BOBBIN_CASE_ARM_SCRATH = 0;  //��е���ɿ�
					go_origin_bobbin_case_arm(0);//ת�����̶�Ӧλ��
					delay_ms(200);
				    continue;
				}
				go_origin_bobbin_case_arm(1);//ת����ͷ�Խ�λ��
				delay_ms(100);
				BOBBIN_CASE_ARM_OUT = 1;     //��е�����ȥ
				delay_ms(1000);
				BOBBIN_CASE_ARM_SCRATH = 0;  //��е���ɿ�
				delay_ms(200);
				BOBBIN_CASE_ARM_OUT = 0;     //��е���ջ�
				delay_ms(500);
				if( bobbin_case_stop_position == 0)//����ֹͣλ��0-���̲࣬1-��ͷ��
				    go_origin_bobbin_case_arm(0);//ת�����̶�Ӧλ��
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
//      ��̬�����������ļ��ڲ���Ч������
//============================================================================
static void bobbin_get_configuration(void)
{
	UINT16 index;
	index = 0;

	//��ȡϵͳ������9�飬������ʱ�����Զ�������ز������õ�
	read_para_group(1600,svpara_disp_buf,205);

	//�ȶ�ȡ���ṹ��para9��
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
	

	//Ȼ���para9ת�浽��Ӧ��ȫ�ֱ�����
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

































