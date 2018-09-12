//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : common.h
//  Description: common constants definition 
//  Version    Date     Author    Description

//  ...
//--------------------------------------------------------------------------------------
#ifndef COMMON_H
#define COMMON_H
#define MACHINE_TYPE   			0x10
#define DAHAO_TYPE     			447

//--------------------------------------------------------------------------------------
// software version number    
//--------------------------------------------------------------------------------------
#define VERSION_NUM_1   		07    
#define VERSION_NUM_2   		1     
#define VERSION_YEAR    		9     
#define VERSION_MONTH   		07    
#define VERSION_DAY     		21    
    
#define MainFatherVersion		05
#define MainChildVersion		07 
#define MainSVNVersion			4051//404//403//402//401//2083//2082//208//2078//2077//2076//2075//2074//2073//2072//2063//2062//2061//20527//20526

#define ENABLE_LED_ALARM_FUNCTION   0 // led报警功能

#define MULTIPULE_IO_ENABLE         1 // 开放多功能IO编程功能
#define USE_ENCODER_Z_PULSE			1 // 中压脚用编码器的Z信号

#define ROTATE_CUTTER_ENABLE		0 // 旋转切刀功能
#define BOBBIN_CHANGER_ENABLE       0 // 自动换梭
#define BOBBIN_THREAD_DETECT        0 
#define BOBBIN_CHANGER_POWERON      0 //自动供电

#define USE_SC013K_PALTFORM         1
#define DEBUG_PARA_OUTPUT		 	0
#define IIC_FUNCTION_ENABLE         1
#define INSERPOINT_ENABLE           1  //插补算?

#define Rot_Trans_Ratio			    5//旋转切刀减速器传动比
#define cur_laser_control			1//旋转切刀激光刀切换  0：激光刀 1：旋转切刀
#define ERROR_OUTPUT_DEBUG      	0
#define BOTE_AGING_MAINMOTOR    	1
#define NOPMOVE_STOP_ENABLE     	1 //空送急停
#define UART1_DEBUG_OUT         	0

#define NEED_TAKE_UP_ORIGIN_PHASE   1 //
#define OPEN_LAST_5_STITCH_SPEED    1 //


#define FIRST_STITCH_NOT_ACTION     1 //第一针不抬起功能
#define RESOLUTION 				10
#define QUICKMOVE_JUDGEMENT     127


#define COM_MONITOR_FUN         	0
#define FOLLOW_INPRESS_FUN_ENABLE   1

#define ONE_STITCH_STOP         	1	 //一针停车
#define DEBUG_DLG               	0
#define DA0_OUTPUT_IMMEDIATELY  	0    //主界面张力值变化立即输出
#define DA0_TEST_FUN_ENABLE     	0    //测试电子线张力
#define DEBUG_MAIN_MOTOR        	0    //借用参数调主轴
#define CHECK_BOARD             	0    //板级加密
#define INSTALLMENT          		1    //分期付款方式
#define TIAOMA               		1    //条码识别器
#define DA1_OUTPUT_ANGLE	    	0    //调试角度输出
#define DA1_OUTPUT_SPEED        	0    //调试速度输出


#define STEPPER_WAITTING_PROTOCOL   1    //延时等待新协议
#define DEBUG_TEST_NOPMOVE      	0    //测试空送效果
#define FUZHUYAJIAO          		1    //小压脚及大压脚极性 0:常闭，全伺服工作方式；1：常开，10080方式
#define JITOUBAN                	1    //0:无机头板  1：SC0413


#define ENABLE_JUMP_NOPMOVE     	1    //空送可以快速移动非一针一针动作
#define ENABLE_SEND_STEPPER_SPEED 	0

#define INPRESS_UP_DELAY        	20      //中压脚抬起延时，10080为20毫秒
#define HIGH_LIMIT_SPEED_10080  	3000 
#define SERVO_MOTOR             	0
#define STEPPER_MOTOR           	1
#define TIME_PARAMETER_1        	1

//配置是否使用SC0716步进板，如果用这个板，表示DSP2作为XY电机，DSP1-A作为中压脚
#define USE_SC0716_BOARD			1

//--------------------------------------------------------------------------------------
// condition compile
//--------------------------------------------------------------------------------------
#define AT_SOLENOID  			1             // 1---tension+45
#define DEBUG                   0
#define TA0_PRIO_4				0
//--------------------------------------------------------------------------------------
// frequency definition
//--------------------------------------------------------------------------------------
#define FX   					24000000
#define PWM_FREQ 				10000
#define	REN_ANGLE 				240
//--------------------------------------------------------------------------------------
// system status definition
//--------------------------------------------------------------------------------------
#define RESERVE     0     // 00
#define FREE        1     // 01
#define READY       2     // 02
#define RUN         3     // 03
#define ERROR       4     // 04
#define PREWIND     5     // 05
#define WIND        6     // 06
#define INPRESS     7     // 07
#define POWEROFF    8     // 08
#define SINGLE      9     // 09
#define MANUAL      10    // 10
#define SETOUT      11    // 11
#define EMERSTOP    12    // 12
#define PREEDIT     13    // 13
#define EDIT        14    // 14
#define NOEDIT      15    // 15
#define FINISH      16    // 16
#define NEEDLE      17    // 17
#define WAITOFF     18    // 18
#define TRIM        19    // 19
#define SLACK       20    // 20
#define CHECKI03    21    // 21
#define CHECKI04    22    // 22
#define CHECKI05    23    // 23
#define CHECKI06    24    // 24
#define CHECKI07    25    // 25
#define CHECKI08    26    // 26
#define CHECKI10    27    // 27
#define CHECKI11    28    // 28
#define MULTI_IO    29    // 29
#define COORSHIFT   30    // 30
#define CALSEW      31    // 31
#define DOWNLOAD    32
#define CONTINUE    33    // 组合花样连续缝
#define DOWNLOAD_DRV1 35  //DSP1升级
#define DOWNLOAD_DRV2 36  //DSP2升级
#define DOWNLOAD_DRV3 38  //DSP3升级
#define DOWNLOAD_DRV4 39  //DSP4升级
#define DOWNLOAD_SPFL 40  //IO程序升级
//--------------------------------------------------------------------------------------
// system error number definition
//--------------------------------------------------------------------------------------
#define OK 			0    // no error
#define ERROR_01	1    // pedal is not in normal position 
#define ERROR_02 	2    // emergency break
#define ERROR_03 	3    
#define ERROR_04 	4    // 300V undervoltage
#define ERROR_05 	5    // 300V overvoltage
#define ERROR_06 	6
#define ERROR_07 	7    // IPM overcurrent or overvoltage
#define ERROR_08 	8    // 24V overvoltage 
#define ERROR_09 	9    // 24V undervoltage
#define ERROR_10 	10
#define ERROR_11 	11
#define ERROR_12 	12
#define ERROR_13 	13   // no motor encoder connect
#define ERROR_14 	14   // motor is not normal 
#define ERROR_15 	15   // out of sewing range
#define ERROR_16	16   // needle is not in normal position 
#define ERROR_17 	17   // thread breakage detection
#define ERROR_18 	18   // cut knife is not in normal position	
#define ERROR_19 	19   // pause button is not in normal position	
#define ERROR_20 	20   // machine overturn  	 	                    
#define ERROR_21 	21
#define ERROR_22 	22
#define ERROR_23 	23   // catcher is not in normal position  	 	           
#define ERROR_24 	24   // panel is not matching   	 	              
#define ERROR_25 	25   // X origin sensor is not normal  	 	        
#define ERROR_26 	26   // Y origin sensor is not normal  	 	        
#define ERROR_27 	27   // press origin sensor is not normal  	 	    
#define ERROR_28 	28   // catch thread origin sensor is not normal  
#define ERROR_29 	29   // inpress origin sensor is not normal  	 	  
#define ERROR_30 	30   // stepping motor driver communication is not normal
#define ERROR_31 	31   // stepping motor overcurrent
#define ERROR_32 	32   // stepping motor driver power is not normal
#define ERROR_33 	33
#define ERROR_34 	34
#define ERROR_35 	35
#define ERROR_36 	36
#define ERROR_37 	37
#define ERROR_38 	38
#define ERROR_39 	39
#define ERROR_40 	40
#define ERROR_41 	41
#define ERROR_42 	42
#define ERROR_43 	43
#define ERROR_44 	44
//===============================
#define ERROR_45 	45
#define ERROR_46 	46
#define ERROR_47 	47
#define ERROR_48 	48
#define ERROR_49	49
#define ERROR_50	50
#define ERROR_51	51
#define ERROR_52	52
#define ERROR_53    53
#define ERROR_54    54 // x step out of tolerance
#define ERROR_55    55 // y step out of tolerance
#define ERROR_56    56
#define ERROR_57    57
#define ERROR_58    58
#define ERROR_59    59
#define ERROR_60    60

#define ERROR_61    61
#define ERROR_62    62
#define ERROR_63    63
#define ERROR_64    64
#define ERROR_65    65
#define ERROR_66    66
#define ERROR_67    67
#define ERROR_68    68
#define ERROR_69    69
#define ERROR_70    70

#define ERROR_71    71
#define ERROR_72    72
#define ERROR_73    73
#define ERROR_74    74
#define ERROR_75    75
#define ERROR_76    76
#define ERROR_77    77
#define ERROR_78    78
#define ERROR_79    79
#define ERROR_81    81
#define ERROR_82    82
#define ERROR_83    83
#define ERROR_85    85
#define ERROR_86    86
#define ERROR_88    88
#define ERROR_89    89
#define ERROR_90    90
#define ERROR_91    91 //未识别模板报错
#define ERROR_92    92 //步进曲线
#define ERROR_93    93 //中压脚电机过流
#define ERROR_94    94 //剪线电机过流
#define ERROR_95    95 //中压脚电机异常
#define ERROR_96    96 //剪线电机异常
#define ERROR_97    97 //读卡模块异常
#define ERROR_98    98 //90V电压过载
#define ERROR_99    99
#define ERROR_100    100
#define ERROR_101    101
#define ERROR_102    102
#define ERROR_103    103
#define ERROR_104    104
#define ERROR_105    105
//--------------------------------------------------------------------------------------
// inpresser origin definition 
//--------------------------------------------------------------------------------------
#define IN_ORIGIN 220
//--------------------------------------------------------------------------------------
// definition
//--------------------------------------------------------------------------------------
#define UP         0
#define DOWN       1
#define OUT        0
#define IN         1
#define OFF        0
#define ON         1
#define MECHANICAL 0
#define ELECTRICAL 1
//--------------------------------------------------------------------------------------
// pin definition
//--------------------------------------------------------------------------------------
#define UZKIN       p0_0
#define DV          p0_1
#define U24V        p0_2
#define FL1         p0_3   
#define POWER_FAULT p0_5
#define SNT_OVL     p0_5
#define SENSOR8     p0_6        //INPUT8---SC0419
#define ADTCSM      p0_7        //INPUT 5

#define SNT4_ON     p1_0
#define POWER_OFF   p1_0
#define SNT_H       p1_1

#define DVSM        p1_3
#define ADTC        p1_4
#define EXTSM       p1_5
#define EXTEND      p1_6     	 //OUTPUT1   

#define DVA         p1_7         //START
#define DVB         p1_2         //PRESS


#define XORG        p2_0         //
#define YORG        p2_1         // 
#define PORG        p2_2         //INPUT1 
#define PSENS       p2_3		 //INPUT2
#define CORG        p2_4		 //INPUT4
#define CSENS       p2_5		 //INPUT3
#define IORG        p2_6         //YORG 
#define SFSW        p2_7

#define TH_BRK      p3_0
#define PAUSE       p3_1

#define T_OC        p3_2
#define L_AIR       p3_3 
#define R_AIR       p3_4
#define LM_AIR      p3_5
#define FW   		p3_7		

#define OUTPUT_ON   p4_0
#define FL		   	p4_1
#define FA          p4_2  
#define T_CLK       p4_3        //OUTPUT6
#define T_DIR       p4_4		//OUTPUT5
#define T_HALF      p4_5		//OUTPUT4


#define T_OC1    	p4_6
#define FR_ON  	    p4_7      	//OUTPUT2

#define CE          p5_0
#define ALARM_LED   p5_1
#define PWR_LED     p5_2
#define SUM         p5_3
#define FK_OFF      p5_4   			//OUTPUT3 


#define EPM         p5_5
#define ONE_WIRE    p5_6
#define T_CLK_EXTEND    p5_7

#define RDSET		p6_0   
#define RST_PN      p6_1
#define RXD0        p6_2
#define TXD0        p6_3
#define BUSY        p6_4
#define SCLK        p6_5
#define RXD1        p6_6
#define TXD1        p6_7

#define SDA         p7_0
#define SCL         p7_1
#define V           p7_2
#define V_          p7_3
#define W           p7_4
#define W_          p7_5
#define IB          p7_6
#define IA          p7_7

#define U           p8_0
#define U_          p8_1
#define ISM         p8_2
#define BLDC_ON     p8_3
#define PWR_ON      p8_4
#define TORG        p8_6
#define RST         p8_6

#define SNT5_ON      p8_6

#define AC_OVDT     p8_7

#define DA0         p9_3
#define DA1         p9_4
#define SPI_CLK     p9_5
#define SPI_OUT     p9_6
#define SPI_IN      p9_7

//如果使用SC0716步进板，XY电机改到了DSP2，为了方便处理，直接更改片选
#if USE_SC0716_BOARD
#define SPISTE1     p10_1
#define SPISTE2     p10_0
#else
#define SPISTE1     p10_0
#define SPISTE2     p10_1

#endif
//这里可能需要处理下
#if MULTIPULE_IO_ENABLE
#define SPISTE3     p0_4
#define SPISTE4     p3_6
#else
#define SPISTE3     p10_2
#define SPISTE4     p10_3
#endif
#define SENSOR6     p10_4      //INPUT 6---SC0419
#define SENSOR7     p10_5      //INPUT 7---SC0419
#define TSENS       p10_6
#define T_HALF_EXTEND  p10_5
#define T_DIR_EXTEND   p10_6
#define DV2          p10_7


//zoje
#define COOL_AIR       				R_AIR   //ZOJE
#define PEN_SIGNAL     				T_HALF  //ZOJE 气阀4

#define LASER_SIGNAL   				FL	  	//ZOJE 气阀6	

#define FILL_OIL  		  			FR_ON   //气阀2
#define OIL_EMPTY_DETECT  			CSENS   //输入3
#define BLOW_AIR      	  			R_AIR   
#define AIR_FW	       	  			FK_OFF  //气阀3

#define LASER_POWER_PIN             FL      //辅助
#define LASER_POWER_ON          	T_DIR   //气阀5
#define LASET_MIRROR_COVER          T_CLK   //气阀6
#define LASER_FUN_PIN               FK_OFF  //气阀3     
#define LASER_HEADER_PROTECT        PORG    //输入1
#define LASER_INDICATIOR_LED        p1_6    //EXTEND
#define LASER_LED_ON                1


//换梭相关，这里的定义后续应该要删掉，先保留吧
#if BOBBIN_CHANGER_ENABLE
	#define BOBBIN_CHANGE_START      T_CLK  //气阀6
	#define BOBBIN_CHANGE_ERROR      ADTCSM //INPUT5
	#define BOBBIN_CHANGE_WORKINGNOW CSENS  //INPUT3    
#endif

#if BOBBIN_THREAD_DETECT	
	#define BOBBIN_EMPTY_DETECTOR1	 PORG   //INPUT1
	#define BOBBIN_EMPTY_DETECTOR2   PSENS  //INPUT2
	#define BOBBIN_EMPTY_SOLENOID    T_DIR  //气阀5     
#endif	

#define YELLOW_LED			  T_HALF   //气阀4	
#define GREEN_LED			  T_DIR    //气阀5 
#define RED_LED				  T_CLK    //气阀6

//--------------------------------------------------------------------------------------
// communication definition
//--------------------------------------------------------------------------------------
#define BAUD 					57600
#define BAUD_4800 				4800
#define BAUD_9600 				9600
#define BAUD_19200 				19200
#define BAUD_38400 				38400
#define BAUD_115200				115200
#define BAUD_RATE 				(FX/BAUD-8)>>4
#define BAUD_RATE_4800  		(FX/BAUD_4800-8)>>4
#define BAUD_RATE_9600  		(FX/BAUD_9600-8)>>4
#define BAUD_RATE_19200  		(FX/BAUD_19200-8)>>4
#define BAUD_RATE_38400  		(FX/BAUD_38400-8)>>4
#define BAUD_RATE_115200		(FX/BAUD_115200-8)>>4

#define BAUD_IIC 				400000
#define BAUD_RATE_IIC 			(FX/BAUD_IIC-1)>>1
//--------------------------------------------------------------------------------------
// interrupt priority definition
//--------------------------------------------------------------------------------------
#define TB1_IPL           		0x01
#define TB3_IPL           		0x01
#define TB4_IPL           		0x01


#define TA0_IPL           		0x03
#define UART_TRANSMIT_IPL 		0x04
#define UART_RECEIVE_IPL  		0x04


#define UART1_TRANSMIT_IPL   	0x06
#define UART1_RECEIVE_IPL_7  	0x07

#define TB2_IPL           		0x05
#define TB0_IPL           		0x06
#define INT0_IPL          		0x06
#define	INT3_IPL		  		0x06
#define INT1_IPL          		0x07
#define INT2_IPL          		0x07
#define SPI_IPL           		0x07


#define WIPE_START_TIME   		10
#define WIPE_END_TIME 	  		30
#define INPRESS_DELAY_TIME  	30

#define MAXSPEED1				32
#define MAXSPEED0				32
#define MINSPEED				2
#define MOTORSTUCKTIME			1000
#define WIPEANDINPRESSTIME		150
#define QUICKMOVETIME			30
#define RotateOriginCheck  		1
#define MIN_X_DISTANCE			100
#define	fabsm(z)				((z >= 0) ? (z) : -(z))

//--------------------------------------------------------------------------------------
// stepmotor confirm data 
//--------------------------------------------------------------------------------------
#define X_CLEAN           		0x0004
#define Y_CLEAN           		0x0004
#define X_INQUIRE         		0x0002
#define Y_INQUIRE         		0x0002
#define X_JUDGE           		0x5555
#define Y_JUDGE           		0x5555


#define DAACTIONTIME      		1000
#define CUTACTIONTIME     		600//1000
#define FWACTIONTIME      		1000


#define ONE_WIRE    	  		p5_6
#define DEADPOINT_SPD 		    120//150  0923

#define FOOTHALF_UP   			FA=1  //FA
#define FOOTHALF_DOWN 			FA=0
#define FOOTHALF_IN	  			FL=0  //FL
#define FOOTHALF_OUT  			FL=1


#define CODE_SCALE 				1024
#define DEGREE_0    			0
#define DEGREE_53   			151
#define DEGREE_180  			512

#define OVC_DSP1           				0xD1A1  // dsp1's motor overcurrent
#define OVC_DSP2           				0xD2A2  // dsp2's motor overcurrent
#define OVD_DSP1           				0xD5A5  // 12 dsp1's motor         
#define OVD_DSP2           				0xD6A6  // 75 dsp2's motor       
#define STEPPER_IGNORE_STOP             0xD0A0  //指令未执行
#define SPI_RX_ERR_CHK                  0xC03F  //spi数据包校验错误
#define SPI_RX_ERR_ILLG                 0xC03E  //spi非法命和非法配置字
#define SPI_UNCONNECT					0xFFFF

		
#define INPRESS_UP_POSITION             80
#define TOTAL_STITCH_COUNTER         	6000
#define HALF_STITCH_COUNTER          	TOTAL_STITCH_COUNTER/2
#define HALF_WRITE_OFFSET            	HALF_STITCH_COUNTER*4
#define DEGREE_60                       240

#define OPEN_LOOP_TIME                  1
#define CLOSE_LOOP_TIME                 0

#define FOLLOW_INPRESS_HIGH 		    1
#define FOLLOW_INPRESS_LOW  		    0


#define ENABLE_X_ORIGIN_FUN       0
#define AUTO_FIND_START_POINT     1

#define FOURTH_GENERATION    	  0 //第四代电控系统开关： 1四代、0五代
#define FIFTH_GENERATION    	  1

#define AIR_CUTTER				  1
#define SOLENOID_CUTTER           0
#define STEPPER_MOTER_CUTTER      2

#define AIR_INPRESSER			  0
#define STEPPER_INPRESSER         1
#define SOLENOID_INPRESSER        2
#define FOLLOW_UP_INPRESSER       3

#define AIR_WIPPER				  0
#define SOLENOID_WIPPER           1
#define MOTOR_DIRECTION_POS       0
#define MOTOR_DIRECTION_NEG       1

#define X_SENSOR_AT_LEFT          0
#define X_SENSOR_AT_RIGHT         1

#define DSP1  1
#define DSP2  2
#define DSP3  3
#define DSP4  4

#endif
//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
