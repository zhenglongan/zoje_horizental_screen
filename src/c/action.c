/*
--------------------------------------------------------------------------------------
        COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
                     ALL RIGHTS RESERVED 
  Project Number: sewing_machine_controller 
  File Name : action.c
  Description:
  Version    Date     Author    Description
------------------------------------------------------------------------------------
*/
//--------------------------------------------------------------------------------------
// 	Includes
//--------------------------------------------------------------------------------------
#include "..\..\include\sfr62p.h"         // M16C/62P special function register definitions
#include "..\..\include\typedef.h"        // Data type define
#include "..\..\include\common.h"         // Common constants definition
#include "..\..\include\variables.h"      // External variables declaration
#include "..\..\include\stepmotor.h"      // stepper motor function
#include "..\..\include\delay.h"          // delay time definition    
#include "..\..\include\motor.h"          // constants definition
#include "..\..\include\math.h"           // math library definition
#include "..\..\include\communication.h"  // Communication function
#include "..\..\include\solenoid.h"       // solenoid driver definition
#include "..\..\include\system.h"       // solenoid driver definition
//--------------------------------------------------------------------------------------
//  Internal function define
//--------------------------------------------------------------------------------------
void go_origin_x(void);
void go_origin_y(void);

void foot_up(void);
void foot_half_up(void);
void footer_both_up(void);
void foot_down(void);
void foot_half_down(void);
void footer_both_down(void);
void inpress_up(void);
void inpress_down(UINT8 pos);
void go_origin_allmotor(void);
void go_commandpoint(INT16 commandpointcoorx,INT16 commandpointcoory);
void go_origin_xy(void);
void go_origin_zx(void);

void find_dead_center(void);
void find_up_position(void);
void find_dead_point(void);
void find_zero(void);
void go_startpoint(void);
void check_data(UINT8 control_flag);
void calculate_angle(void);
void process_data(void);
void process_edit_data(void);
void go_beginpoint(UINT8 FirstNopmoveFlag);
void conprocess_data(void);
void conprocess_edit_data(void);
void single_next(void);
void single_back(void);
void back_startpoint(void);
void single_end(void);
void single_start(void);
void single_stop(void);
void move_next(void);
void move_back(void);
void move_startpoint(void);
void move_edit_startpoint(void);
void course_next(void);
void course_back(void);
void course_stop(void);
void shift_12(void);
void shift_01(void);
void shift_03(void);
void shift_04(void);
void shift_06(void);
void shift_07(void);
void shift_09(void);
void shift_10(void);
void remove_12(void);
void remove_01(void);
void remove_03(void);
void remove_04(void);
void remove_06(void);
void remove_07(void);
void remove_09(void);
void remove_10(void);
void remove_stop(void);
void go_setoutpoint(void);
void turnoff_led(void);
void turnoff_buz(void);
void turnoff_ledbuz(void);
void turnon_led(void);
void turnon_buz(void);
void flash_led(void);
void flash_buz(void);
void emergency(void);
void para_confirm(void);
void sewing_stop(void);
void pause_stop(void);
UINT8 detect_position(void);
void reset_panel(void);
void initial_mainmotor(void);

void move_xy(void);
void zpl_process(void);
void stretch_foot_in(void);
void footer_both_down(void);
void inpress_to(INT16 a);
void do_pat_point_add_one(void);
void do_pat_point_sub_one(void);
void coor_com_fun(void);
UINT8 scan_pause_func(UINT8 *pause_flag_tmp,UINT8 system_staus_tmp);
UINT8 check_footer_status(void);
void process_making_pen_signal(UINT8 flag);
void go_origin_yj(void);
void go_origin_qd(void);

//角度与码盘线数转换表
const INT16 angle_tab[ ]=
{
//   0    1    2    3    4    5    6    7    8    9  
	0,   3,   6,   9,  11,  14,  17,  20,  23,  26, // 0-9   
//  10   11   12   13   14   15   16   17   18   19  
	28,  31,  34,  37,  40,  43,  46,  48,  51,  54, // 10-19  	      	
//  20   21   22   23   24   25   26   27   28   29  
	57,  60,  63,  65,  68,  71,  74,  77,  80,  82, // 20-29 	  
//  30   31   32   33   34   35   36   37   38   39  
	85,  88,  91,  94,  97, 100, 102, 105, 108, 111, // 30-39 	  
//  40   41   42   43   44   45   46   47   48   49  
	14, 117, 119, 122, 125, 128, 131, 134, 137, 139, // 40-49 	  	
//  50   51   52   53   54   55   56   57   58   59  
	142, 145, 148, 151, 154, 156, 159, 162, 165, 168, // 50-59 	  		
//  60   61   62   63   64   65   66   67   68   69  
	171, 174, 176, 179, 182, 185, 188, 191, 193, 196, // 60-69 
//  70   71   72   73   74   75   76   77   78   79  
	 199, 202, 205, 208, 210, 213, 216, 219, 222, 225, // 70-79 	 
//  80   81   82   83   84   85   86   87   88   89  
	 228, 230, 233, 236, 239, 242, 245, 247, 250, 253, // 80-89 	 
//  90   91   92   93   94   95   96   97   98   99  
	 256, 259, 262, 265, 267, 270, 273, 276, 279, 282, // 90-99 	 
// 100  101  102  103  104  105  106  107  108  109  
	 284, 287, 290, 293, 296, 299, 302, 304, 307, 310, // 100-109 	 
// 110  111  112  113  114  115  116  117  118  119  
	 313, 316, 319, 321, 324, 327, 330, 333, 336, 338, // 110-119
// 120  121  122  123  124  125  126  127  128  129  
	 341, 344, 347, 350, 353, 356, 358, 361, 364, 367, // 120-129
// 130  131  132  133  134  135  136  137  138  139  
	 370, 373, 375, 378, 381, 384, 387, 390, 393, 395, // 130-139
// 140  141  142  143  144  145  146  147  148  149  
	 398, 401, 404, 407, 410, 412, 415, 418, 421, 424, // 140-149
// 150  151  152  153  154  155  156  157  158  159  
	 427, 430, 432, 435, 438, 441, 444, 447, 449, 452, // 150-159
// 160  161  162  163  164  165  166  167  168  169  
	 455, 458, 461, 464, 466, 469, 472, 475, 478, 481, // 160-169
// 170  171  172  173  174  175  176  177  178  179  
	 484, 486, 489, 492, 495, 498, 501, 503, 506, 509, // 170-179
// 180  181  182  183  184  185  186  187  188  189  
	512, 515, 518, 521, 523, 526, 529, 532, 535, 538, // 180-189
// 190  191  192  193  194  195  196  197  198  199  
	 540, 543, 546, 549, 552, 555, 558, 560, 563, 566, // 190-199	 	 	 	 	 	 	  	  	 	 
// 200  201  202  203  204  205  206  207  208  209  
	 569, 572, 575, 577, 580, 583, 586, 589, 592, 594, // 200-209	 	 	 	 	 	 	  	  	 	 	 
// 210  211  212  213  214  215  216  217  218  219  
	 597, 600, 603, 606, 609, 612, 614, 617, 620, 623, // 210-219	 	 	 	 	 	 	  	  	 	 	 	 
// 220  221  222  223  224  225  226  227  228  229  
	 626, 629, 631, 634, 637, 640, 643, 646, 649, 651, // 220-229	 	 	 	 	 	 	  	  	 	 	 	 
// 230  231  232  233  234  235  236  237  238  239  
	 654, 657, 660, 663, 666, 668, 671, 674, 677, 680, // 230-239	 	 	 	 	 	 	  	  	 	 	 	 
// 240  241  242  243  244  245  246  247  248  249  
	 683, 686, 688, 691, 694, 697, 700, 703, 705, 708, // 240-249	 	 	 	 	 	 	  	  	 	 	 	 
// 250  251  252  253  254  255  256  257  258  259  
	 711, 714, 717, 720, 722, 725, 728, 731, 734, 737, // 250-259	 	 	 	 	 	 	  	  	 	 	 	 
// 260  261  262  263  264  265  266  267  268  269  
	 740, 742, 745, 748, 751, 754, 757, 759, 762, 765, // 260-269	 	 	 	 	 	 	  	  	 	 	 	 
// 270  271  272  273  274  275  276  277  278  279  
	 768, 771, 774, 777, 779, 782, 785, 788, 791, 794, // 270-279	 	 	 	 	 	 	  	  	 	 	 	 
// 280  281  282  283  284  285  286  287  288  289  
	 796, 799, 802, 805, 808, 811, 814, 816, 819, 822, // 280-289
// 290  291  292  293  294  295  296  297  298  299  
	 825, 828, 831, 833, 836, 839, 842, 845, 848, 850, // 290-299	 	 	 	 	 	 	  	  	 	 	 	 
// 300  301  302  303  304  305  306  307  308  309  
	 853, 856, 859, 862, 865, 868, 870, 873, 876, 879, // 300-309	 	 	 	 	 	 	  	  	 	 	 	 	 	 	 	 	 	 	 	 	  	  	 	 	 	 	 	 	 	 	 	 	 
// 310  311  312  313  314  315  316  317  318  319  
	 882, 885, 887, 890, 893, 896, 899, 902, 905, 907, // 310-319	 	 	 	 	 	 	  	  	 	 	 	 	 	 	 	 	 	 	 	 	  	  	 	 	 	 	 	 	 	 	 	 	 
// 320  321  322  323  324  325  326  327  328  329  
	 910, 913, 916, 919, 922, 924, 927, 930, 933, 936, // 320-329	 	 	 	 	 	 	  	  	 	 	 	 	 	 	 	 	 	 	 	 	  	  	 	 	 	 	 	 	 	 	 	 	 
// 330  331  332  333  334  335  336  337  338  339  
	 939, 942, 944, 947, 950, 953, 956, 959, 961, 964, // 330-339	 	 	 	 	 	 	  	  	 	 	 	 	 	 	 	 	 	 	 	 	  	  	 	 	 	 	 	 	 	 	 	 	 
// 340  341  342  343  344  345  346  347  348  349  
	 967, 970, 973, 976, 978, 981, 984, 987, 990, 993, // 340-349	 	 	 	 	 	 	  	  	 	 	 	 	 	 	 	 	 	 	 	 	  	  	 	 	 	 	 	 	 	 	 	 	 
// 350  351  352  353  354  355  356  357  358  359  
	 996, 998,1001,1004,1007,1010,1013,1015,1018,1021, // 350-359	
// 360   361  
	1024,1027,	 	  	  	 	 	 	 	 	 	 	 	 	 	 	 	  	  	 	 	 	 	 	 	 	 	 	 	 	  	 	 	 	 	 	  	  	 	 	 	 	 	 	 	 	 	 	 	 	  	  	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 
};

const UINT16 inpress_spdlimit_speed_tab[]=
{
	// 	0.1		0.2		0.3		0.4		0.5		0.6		0.7		0.8		0.9		1.0
		3000,	3000,	3000,	3000,	3000,	3000,	3000,	3000,	3000,	3000,
	// 	1.1		1.2		1.3		1.4		1.5		1.6		1.7		1.8		1.9		2.0
		3000,	3000,	3000,	3000,	3000,	3000,	3000,	3000,	3000,	3000,
	// 	2.1		2.2		2.3		2.4		2.5		2.6		2.7		2.8		2.9		3.0
		3000,	3000,	3000,	3000,	3000,	3000,	2900,	2800,	2700,	2600,
	// 	3.1		3.2		3.3		3.4		3.5		3.6		3.7		3.8		3.9		4.0  
		2600,	2500,	2500,	2400,	2300,	2300,	2300,	2300,	2300,	2200,
	// 	4.1		4.2		4.3		4.4		4.5		4.6		4.7		4.8		4.9		5.0
		2200,	2200,	2100,	2100,	2100,	2000,	2000,	2000,	2000,	2000,
	// 	5.1		5.2		5.3		5.4		5.5		5.6		5.7		5.8		5.9		6.0
	    2000,	2000,	2000,	1900,	1900,	1900,	1800,	1800,	1800,	1800,
	// 	6.1		6.2		6.3		6.4		6.5		6.6		6.7		6.8		6.9		7.0
	    1800,	1700,	1700,	1700,	1600,	1600,	1600,	1500,	1500,	1500,
	// 	7.1		7.2		7.3		7.4		7.5		7.6		7.7		7.8		7.9		8.0
		1500,	1500,	1400,	1400,	1400,	1400,	1400,	1300,	1300,	1300
};
//=======================================================================================================================================
//中压脚随动动作
const UINT16 inpress_follow_down_angle_tab[]={//k174
	    // 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		   60, 60, 60, 55, 40, 35, 35, 35, 35, 35, 35, 35, 30, 25, 20, 20, 20, 20, 20, 20, 20, 20, 20, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10
	};
	const UINT8 inpress_follow_down_speed_tab[]={//k175
		// 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		   63, 63, 63, 50, 50, 41, 34, 29, 26, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 13, 13, 12, 12, 11, 11, 11, 10, 10, 9,  9,  9,  9,
	};

	const UINT16 inpress_follow_up_angle_tab[]={
		// 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		  250,250,250,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,230,220,210,210,200,200,200,200,200,200
	};
	const UINT8 inpress_follow_up_speed_tab[]={
		// 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		   63, 63, 63, 50, 50, 41, 34, 29, 26, 23, 22, 21, 20, 18, 17, 16, 15, 14, 13, 12, 11, 11, 11, 10, 10, 10, 10, 10, 9,  8,  8,  8,  8
	};
//===========================================================================	

	const UINT16 inpress_follow_down_angle_tab2[]={//k174
	    // 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		  340,340,340,340,340,340,340,340,340,340,340,340,340,340,340,340,340,340,340,340,340,340,340,340,340,340,340,340,340,340,340,340,330
	};
	const UINT8 inpress_follow_down_speed_tab2[]={//k175
		// 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		   63, 63, 60, 60, 60, 48, 40, 35, 30, 26, 23, 21, 19, 18, 18, 18, 17, 16, 15, 14, 14, 14, 13, 10, 10, 10, 10, 10, 9, 9,  9,  9,  9,
	};

	const UINT16 inpress_follow_up_angle_tab2[]={
		// 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		  200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,200,180,180,180,180,180,160,160,160,160,160
	};
	const UINT8 inpress_follow_up_speed_tab2[]={
		// 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		   63, 63, 60, 60, 45, 36, 30, 25, 22, 20, 20, 18, 16, 15, 14, 13, 12, 11, 11, 10, 10, 9,  9,  9,  9,  9,  9,  9,  9,  8,  8,  8,  8
	};
//===========================================================================	
	const UINT16 inpress_follow_down_angle_tab3[]={//k174
	    // 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		  340,340,340,340,340,340,340,340,340,340,340, 40, 40, 40, 30, 30, 20, 20, 20, 20, 20, 30, 30, 40, 35, 30, 25, 20, 30, 30, 30, 30, 20
	};
	const UINT8 inpress_follow_down_speed_tab3[]={//k175
		// 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		   63, 63, 60, 60, 60, 48, 40, 35, 30, 26, 23, 20, 18, 17, 17, 17, 17, 16, 15, 14, 12, 11, 11, 10, 10, 10, 10, 10, 9, 9,  9,  9,  9,
	};

	const UINT16 inpress_follow_up_angle_tab3[]={
		// 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		  200,200,200,200,200,200,200,200,200,200,200,260,260,250,240,240,250,250,250,250,245,250,250,260,240,240,240,240,240,230,200,180,160
	};
	const UINT8 inpress_follow_up_speed_tab3[]={
		// 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		   63, 63, 60, 60, 45, 36, 30, 25, 22, 20, 20, 16, 16, 15, 15, 14, 13, 12, 11, 11, 11, 10, 10, 10, 10,  9,  9,  8,  8,  8,  8,  8,  8
	};

//===========================================================================	
	const UINT16 inpress_follow_down_angle_tab4[]={//k174
	    // 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		  340,340, 80, 80, 70, 70, 60, 60, 50, 50, 30, 30, 30, 30, 30, 30, 20, 20, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10
	};
	const UINT8 inpress_follow_down_speed_tab4[]={//k175
		// 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		   63, 63, 60, 60, 56, 43, 36, 31, 28, 25, 23, 21, 19, 17, 16, 15, 15, 14, 13, 13, 12, 11, 11, 10, 10, 10,  9,  9, 8,  8,  8,  8,  8,
	};

	const UINT16 inpress_follow_up_angle_tab4[]={
		// 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		  200,200,220,220,215,210,200,200,195,195,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175,175
	};
	const UINT8 inpress_follow_up_speed_tab4[]={
		// 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		   63, 63, 60, 60, 58, 46, 38, 32, 27, 24, 22, 20, 18, 16, 15, 15, 14, 13, 12, 11, 11, 10, 10,  9,  9,  9,  9,  8,  8,  8,  8,  8,  8
	};
	
	//===========================================================================	
	const UINT16 inpress_follow_down_angle_tab5[]={//k174
	    // 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		  340,340, 40, 40, 40, 40, 40, 40, 40, 340,340,340,340,340,320,320,340,320,340,340,340,340,340,340,340,340,340,340,340,340,340,340,340
	};
	const UINT8 inpress_follow_down_speed_tab5[]={//k175
		// 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		   63, 63, 30, 30, 30, 30, 30, 28, 24, 28, 28, 25, 24, 23, 23, 21, 18, 20, 16, 15, 14, 13, 12, 12, 12, 10, 10, 10, 10, 9,  8,  8,  8,
	};

	const UINT16 inpress_follow_up_angle_tab5[]={
		// 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		  200,200,250,230,230,230,230,230,230,230,220,180,200,190,190,190,180,190,180,185,185,190,200,180,200,200,180,175,175,175,175,175,175
	};
	const UINT8 inpress_follow_up_speed_tab5[]={
		// 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		   63, 63, 50, 50, 50, 30, 30, 25, 25, 19, 19, 23, 19, 18, 15, 14, 14, 12, 11, 11, 10, 10, 9,  9,  8,  8,  8,  8,  7,  7,  7,  7,  7
	};
	
	//===========================================================================	
	const UINT16 inpress_follow_down_angle_tab6[]={//k174
	    // 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		  340,340, 70, 70, 70, 60, 45, 41, 10, 10, 10, 340,340,340,340,340,340,340,340,340,340,340,330,330,330,330,310,310,310,310,310,310,310
	};
	const UINT8 inpress_follow_down_speed_tab6[]={//k175
		// 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		   63, 63, 40, 40, 40, 36, 34, 32, 30, 28, 26, 26, 24, 22, 21, 19, 18, 17, 16, 15, 14, 13, 12, 11, 11, 11, 11, 11, 11, 10, 9,  8,  8,
	};

	const UINT16 inpress_follow_up_angle_tab6[]={
		// 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		  200,200,240,240,245,255,250,240,240,240, 240,190,190,190,190,190,185,185,185,185,185,185,185,175,175,175,155,155,155,155,155,155,155
	};
	const UINT8 inpress_follow_up_speed_tab6[]={
		// 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		   63, 63, 50, 50, 40, 30, 26, 20, 19, 18, 16, 18, 17, 15, 14, 14, 13, 12, 11, 11, 10, 9,  9,  9,  9,  9,  9,  9,  9,  8,  7,  7,  7
	};
	//===========================================================================	
	const UINT16 inpress_follow_down_angle_tab7[]={//k174
	    // 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		  340,340, 70, 70, 70, 60, 45, 41, 10, 10, 10, 340,340,340,340,340,10, 10, 10, 10, 10, 10, 10,330,330,330,310,310,310,310,310,310,310
	};
	const UINT8 inpress_follow_down_speed_tab7[]={//k175
		// 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		   63, 63, 40, 40, 40, 36, 34, 32, 30, 28, 26, 26, 25, 22, 21, 21, 16, 15, 14, 13, 11, 13, 12, 11, 11, 11, 11, 11, 11, 10, 9,  8,  8,
	};

	const UINT16 inpress_follow_up_angle_tab7[]={
		// 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		  200,200,240,240,245,255,250,240,240,240, 240,230,230,210,220,220,220,220,220,220,220,220,220,175,175,175,155,155,155,155,155,155,155
	};
	const UINT8 inpress_follow_up_speed_tab7[]={
		// 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		   63, 63, 50, 50, 40, 30, 26, 20, 19, 18, 16, 16, 15, 16, 14, 13, 14, 13, 13, 12, 12, 11, 11,  9,  9,  9,  9,  9,  9,  8,  7,  7,  7
	};
	
	//===========================================================================	
	const UINT16 inpress_follow_down_angle_tab8[]={//k174
	    // 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		  340,340, 45, 45, 49, 48, 45, 46, 46, 46, 42, 43, 49, 48, 55, 51, 47, 52, 46, 51, 54, 47, 50, 53, 56, 45, 48, 51, 55, 60, 60, 64, 70
	};
	const UINT8 inpress_follow_down_speed_tab8[]={//k175
		// 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		   63, 63, 63, 50, 50, 41, 34, 29, 26, 23, 22, 21, 20, 18, 16, 16, 15, 14, 13, 12, 12, 11, 11, 10, 9,  9,  8,  8,  8,  7,  7,  7,  6,
	};

	const UINT16 inpress_follow_up_angle_tab8[]={
		// 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		  200,200,330,330,330,330,330,330,330,330, 330,330,330,330,330,330,330,330,330,330,330,330,330,330,330,330,330,330,330,330,330,330,330
	};
	const UINT8 inpress_follow_up_speed_tab8[]={
		// 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
		   63, 63, 63, 42, 33, 26, 21, 18, 16, 14, 12, 11, 11, 10, 10, 9,  8,  8,  7,  7,  7,  6,  6,  6,  6,  5,  5,  5,  5,  5,  5,  5,  5
	};
//======================================================================================================================================	
const UINT8 MoveTime_Speed_10080_ND80[] =
{
//	0	1	2	3	4	5	6	7	8	9	10	11	12	13	14	15	16	17	18	19	20	21	22	23	24	25  26  27  28  29 30 31 32
    59, 59, 59, 59, 59,	59,	57,	56,	53,	52,	48,	45,	42,	39,	38,	37,	35,	31,	30,	29,	27,	26,	25,	24, 23, 22, 21, 20, 19 ,18,17,16,15
};
const UINT16 MoveStartAngle_10080_y[] =
{
//	0	1	2	3	4	5	6	7	8	9   10	11	12	13	14	15	16	17	18	19	20	21	22	23	24	25  26  27  28  29  30  31  32
// 300,300,250,230,210,210,210,210,190,180,179,160,160,155,140,140,130,130,130,130,100, 95, 90, 57, 57, 50, 20, 20, 20, 67, 57, 57, 57 
   300,300,280,260,240,190,170,170,150,140,139,130,130,125,110,110,100,100,170,170,140,135,130, 97, 77, 70, 80, 120,120,167,157,157,157
};
 const UINT8 MoveTime_Speed_10080_y[] =
{
//	0	1	2	3	4	5	6	7	8	9	10	11	12	13	14	15	16	17	18	19	20	21	22	23	24	25  26  27  28  29  30  31  32
//	52, 52, 52, 52, 52, 52,	52, 52,	47, 37,	32, 28,	25, 21,	20, 20,	18, 15,	15, 14,	18, 17,	16, 15, 19, 18, 18, 18, 18, 17, 16, 15, 14 
    63, 63, 63, 63, 63, 63,	63, 63,	63, 58,	51, 45,	42, 36,	31, 28,	24, 19,	16, 15,	19, 16,	13, 12, 14, 13, 13, 13, 13, 12, 11, 15, 14 
};

const UINT16 MoveStartAngle_10080_x[] =
{
//	0	1	2	3	4	5	6	7	8	9	10	11	12	13	14	15	16	17	18	19	20	21	22	23	24	25  26  27  28  29  30  31 32
// 300,300,250,230,220,220,220,220,220,220,220,220,220,220,220,220,170,170,170,170,190,185,180,137,127,127,127,127,127,117,107,107,107
   300,300,260,240,230,200,200,180,160,160,140,140,140,150,150,150,130,120,100,100, 90, 85,110, 87, 77, 77, 67, 57, 57, 37, 37, 37, 37
};
const UINT8 MoveTime_Speed_10080_x[] =
{
//  0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16  17  18  19  20  21	22	23	24	25  26  27  28  29  30  31  32
//  52, 52, 50, 48, 46, 44, 42, 40, 37, 28, 25, 23, 22, 21, 21, 21, 21, 19, 19, 18, 17, 15,	13, 12, 11, 10, 10, 10, 10, 10, 10 ,10, 10	
	63, 63, 63, 63, 63, 63, 63, 63, 62, 53, 50, 46, 39, 36, 33, 31, 31, 29, 29, 27, 26, 24,	21, 20, 19, 18, 18, 18, 14, 14, 12 ,12, 12
};

const UINT16 spdlimit_10080_345_tab[]=                         
{
	
	// 	0.1		0.2		0.3		0.4		0.5		0.6		0.7		0.8		0.9		1.0
		3200,   3200,   3200,   3200,   3200,   3200,   3200,   3200,   3200,   3200,
	// 	1.1		1.2		1.3		1.4		1.5		1.6		1.7		1.8		1.9		2.0
		3200,   3200,   3200,   3200,   3200,   3200,   3200,   3200,   3200,   3200,
	// 	2.1		2.2		2.3		2.4		2.5		2.6		2.7		2.8		2.9		3.0
		3200,   3200,   3200,   3200,   3200,   3200,   3200,   3200,   3200,   3200,
	// 	3.1		3.2		3.3		3.4		3.5		3.6		3.7		3.8		3.9		4.0  
		3100,	2800,	2600,	2400,	2300,	2200,	2200,	2200,	2200,	2200,
	// 	4.1		4.2		4.3		4.4		4.5		4.6		4.7		4.8		4.9		5.0
		2200,	2100,	2100,	2000,	2000,	2000,	2000,	2000,	1900,	1800,
	// 	5.1		5.2		5.3		5.4		5.5		5.6		5.7		5.8		5.9		6.0
	    1800,	1800,	1800,	1900,	1800,	1800,	1800,	1700,	1700,	1600,
	// 	6.1		6.2		6.3		6.4		6.5		6.6		6.7		6.8		6.9		7.0
	    1600,	1600,	1600,	1600,	1600,	1600,	1600,	1500,	1500,	1500,
	// 	7.1		7.2		7.3		7.4		7.5		7.6		7.7		7.8		7.9		8.0
		1500,	1500,	1400,	1400,	1400,	1400,	1400,	1300,	1300,	1300,
	// 	8.1		8.2		8.3		8.4		8.5		8.6		8.7		8.8		8.9		9.0
		1300,	1200,	1200,	1200,	1200,	1200,	1100,	1100,	1100,	1100,
	// 	9.1		9.2		9.3		9.4		9.5		9.6		9.7		9.8		9.9		10.0
		1100,	1100,	1100,	1100,	1100,	1100,	1000,	1000,	1000,	1000,
	// 	10.1	10.2	10.3	10.4	10.5	10.6	10.7	10.8	10.9	11.0
		1000,	1000,	1000,	1000,	1000,	1000,	1000,	1000,	1000,	1000,
	// 	11.1	11.2	11.3	11.4	11.5	11.6	11.7	11.8	11.9	12.0
		1000,	900,	900,	900,	900,	900,	900,	900,	900,	900,
	// 	12.1	12.2	12.3	12.4	12.5	12.6	12.7
		800,	800,	800,	800,	800,	800,	800
};
//=========================================================================
const UINT8 MoveTime_Limtit[] = 
{
//	0   1	2   3	4	5	6	7	8	9   10  11	12  13	14  15	16  17	18  19	20  21	22  23  24  25  26  27 28 29 30 31 32
	63, 63, 63, 63, 63, 63,	63, 63,	63, 63,	60, 54,	50, 46,	42, 40,	37, 35,	33, 31,	30, 28,	27, 26, 25, 24, 23, 22,21,20,20,19,18
};

INT16 ChangeX(PATTERN_DATA *pp)
{
	INT16 sx;
	UINT8 ux;
	UINT16 deltaX;
	
#if SUPPORT_0_5MM_FORMAT
   
	deltaX = ((pp->para) >>4)&0x07 ;
	ux =  pp-> xstep;
	deltaX = (deltaX <<7)+(UINT16)(ux&0x7f);
	deltaX =  deltaX<<1;
	if( (pp->para) & 0x80)
		deltaX += 1;
#else
	deltaX = (pp->para >>4)&0x0f ;
	ux =  pp-> xstep;
	deltaX = (deltaX <<7)+(UINT16)(ux&0x7f);
#endif
	if(ux >= 0x80)
		sx = - (INT16)deltaX;
	else
		sx = deltaX;
	return sx;
}

INT16 ChangeY(PATTERN_DATA *pp)
{
	INT16 sx;
	UINT8 ux;
	UINT16 deltaY;
#if SUPPORT_0_5MM_FORMAT
	deltaY = pp->para&0x07 ;
	ux =  pp-> ystep;
	deltaY = (deltaY <<7)+(UINT16)(ux&0x7f);
	deltaY = deltaY <<1;
	if( pp->para &0x08)
		deltaY += 1;
#else	
	deltaY = pp->para&0x0f ;
	ux =  pp-> ystep;
	deltaY = (deltaY <<7)+(UINT16)(ux&0x7f);
#endif	
	if(ux >= 0x80)
		sx = - (INT16)deltaY;
	else
		sx = deltaY;
	return sx;
}
/*	
	XORG = 1  the sensor is actived
*/
void go_origin_x(void)
{	
	UINT8 i;
	UINT16 temp16;
	UINT16 speed_time,run_time;
	UINT16 j=1;	                                                 
   
   if( para.x_origin_mode == AUTO_FIND_START_POINT)
   {
	   delay_ms(1);			
  	   allx_step = 0;	
	   return;
   }
	pause_flag = 0;
	speed_time = 300;
	
	if( XORG == para.x_sensor_open_level )		//not working
	{	
		temp16 = 0;
		run_time = 12000;
		j= 0;
        while(XORG == para.x_sensor_open_level )
        {
	        if( j < 40)
				movestepx_time = OPEN_LOOP_TIME;
			else
            	movestepx_time = CLOSE_LOOP_TIME;
				
	    	if( x_sensor_pos == X_SENSOR_AT_LEFT ) 
				movestep_x(1); 
			else
				movestep_x(-1); 
				
			if( j< 40 )
			{
			   run_time -= speed_time;
			   j ++ ;
			   delay_us(run_time);
			}
			else
			   delay_us(300);
				
  	  	    temp16 = temp16 + 1;
  	  	    if(temp16 > 30000)
  	  	    {
  	  		   	sys.status = ERROR;
		    	StatusChangeLatch = ERROR;
				if( sys.error == 0)
      	    		sys.error = ERROR_25;
      	    	return;
  	  	    }			
			if (PAUSE == PAUSE_ON)
			{
				delay_ms(10);                           
				if(PAUSE == PAUSE_ON)	
				{
					pause_flag = 1;
					return;
				}	
			}
			if(sys.status == ERROR)
			{
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
				return;
			}
			rec_com();		
        }
		
		if( j >= 40)
		{
			movestepx_time = CLOSE_LOOP_TIME;
		}
		else
		{
			movestepx_time = OPEN_LOOP_TIME;		
		}
		
		for(i=5;i>0;i--) 
	    {    
	    	if( x_sensor_pos == X_SENSOR_AT_LEFT ) 
				movestep_x(1);  
			else
				movestep_x(-1);  
			run_time += speed_time;	  
  	    	delay_us(run_time);								
	    }			
			
		delay_ms(20);
		while(XORG != para.x_sensor_open_level)
		{
			 movestepx_time = OPEN_LOOP_TIME;
			 if( x_sensor_pos == X_SENSOR_AT_LEFT )//退回去
			 	 movestep_x(-1);
			 else
    		 	 movestep_x(1);
			 delay_ms(12);
			 if(sys.status == ERROR)
			{
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
				return;
			}
		}
		
		for(i=0;i<5;i++)
		{
			movestepx_time = OPEN_LOOP_TIME;
			if( x_sensor_pos == X_SENSOR_AT_LEFT )
	    		movestep_x(-1);
			else
				movestep_x(1);
			delay_ms(12);
        }
    }
 	else //already in cover position
	{
		temp16 = 0;
		j = 0;
		run_time = 12000;
  	    while( XORG != para.x_sensor_open_level )
        {
	        if( j < 40 )
			{
				movestepx_time = OPEN_LOOP_TIME;
			}
			else
			{
	           	movestepx_time = CLOSE_LOOP_TIME;
			}
			if( x_sensor_pos == X_SENSOR_AT_LEFT )
		    	movestep_x(-1); 
			else
				movestep_x(1); 

			if( j< 40 )
			{
			    run_time -= speed_time;
			    j ++ ;
				delay_us(run_time);
			}
			else
				delay_us(300);
				
  	  	    temp16 = temp16 + 1;
  	  	    if( temp16 > 30000 )
  	  	    {
  	  		    sys.status = ERROR;
			    StatusChangeLatch = ERROR;
				if( sys.error == 0 )
      		    	sys.error = ERROR_25; 
      		    return;
  	  	    }	
				if (PAUSE == PAUSE_ON)
				{
					delay_ms(10);                           
					if(PAUSE == PAUSE_ON)	
					{
						pause_flag = 1;
						return;
					}	
				}	
			if(sys.status == ERROR)
			{
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
				return;
			}
			rec_com();
       }
       if( j>=40 )
	   {
		    movestepx_time = CLOSE_LOOP_TIME;
	    }
		else
		{
		    movestepx_time = OPEN_LOOP_TIME;
		}
		for(i=5;i>0;i--)
		{
			if( x_sensor_pos == X_SENSOR_AT_LEFT )
				movestep_x(-1);
			else
				movestep_x(1);
			run_time += (speed_time*8-1600); 
  	    	delay_us(run_time);
	    }
		delay_ms(20);
	}
	
	delay_ms(12);
	temp16 = 0;
	for(i=0;i<3;i++)
	{
	   	while(XORG == para.x_sensor_open_level)
       	{
	        movestepx_time = OPEN_LOOP_TIME;
			if( x_sensor_pos == X_SENSOR_AT_LEFT )	
		   	    movestep_x(1);  	  
			else
				movestep_x(-1); 
			
  	  	    delay_ms(12);
  	  	    temp16 = temp16 + 1;
  	  	    if( temp16 > 30000)
  	  	    {
  	  		   	sys.status = ERROR;
			    StatusChangeLatch = ERROR;
				if(sys.error == 0)
      		       sys.error = ERROR_25;      	            
      		    return;
  	  	    }
				if (PAUSE == PAUSE_ON)
				{
					delay_ms(10);                           
					if(PAUSE == PAUSE_ON)	
					{
						pause_flag = 1;
						return;
					}	
				}
			rec_com();	
        }
		delay_ms(20);
	}

  	delay_ms(1);	
	allx_step = 0;	
	if( x_origin_offset !=0) 
 		go_commandpoint(x_origin_offset,ally_step);		
  	allx_step = 0;	
}
//--------------------------------------------------------------------------------------
//  Name:		go_origin_y 
//  Parameters:	None
//  Returns:	None
//  Description: y motor go origin
//--------------------------------------------------------------------------------------
void go_origin_y(void)
{	
	UINT8 i;
	UINT16 temp16;
	UINT16 speed_time,run_time;
	UINT16 j=1;
	
	speed_time = 292;
	j = 1;
	pause_flag = 0;
	if(YORG == para.y_sensor_open_level)   
	{	
		temp16 = 0;
		j = 1;
		run_time = 12000;
		while(YORG == para.y_sensor_open_level)
		{ 
	        if(j<40)
			{
				movestepy_time = OPEN_LOOP_TIME;			
			}
			else
			{
	        	movestepy_time = CLOSE_LOOP_TIME;
			}
			movestep_y(-1);
			
			if(run_time <850)  
			   run_time = 850;	
				    
			delay_us(run_time);
			if( j<= 40 )
			{
				run_time -= speed_time;
				j++;
			}
	    	temp16 = temp16 + 1;
  	  		if(temp16 > 18000)
  	  		{
  	  			sys.status = ERROR;
				StatusChangeLatch = ERROR;
				if(sys.error == 0)
      				sys.error = ERROR_26;     	// Y origin sensor is not normal  	 	            
      			return;
  	  		}
			if (PAUSE == PAUSE_ON)
			{
				delay_ms(10);                           
				if(PAUSE == PAUSE_ON)	
				{
					pause_flag = 1;
					return;
				}	
			}
			if(sys.status == ERROR)
			{
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
				return;
			}
			rec_com();
	  	}
	 
		if(j>=40)
		{
			movestepy_time = CLOSE_LOOP_TIME;
			
		}
		else
		{
			movestepy_time = OPEN_LOOP_TIME;
			
		}
			
		for(i=5;i>0;i--) 
	    {   
		    	 movestep_y(-1);  	  
	  	     	 run_time += (speed_time*8-1600); 
	  	     	 delay_us(run_time);
	    }
		delay_ms(20);
		while( YORG != para.y_sensor_open_level)
		{
			movestepy_time = OPEN_LOOP_TIME;
				 
		    movestep_y(1);
			delay_ms(12);
			if(sys.status == ERROR)
			{
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
				return;
			}
		}
		
		for( i=0;i<5;i++)
		{
				 movestepy_time = OPEN_LOOP_TIME;
				 
		    	 movestep_y(1);
				 delay_ms(12);
		}
  	}
  	else            						// sensor is not covered
  	{
  		temp16 = 0;
		j = 1;
		run_time = 12000;
  		while(YORG != para.y_sensor_open_level)
    	{
	        if(j<40)
			{
				movestepy_time = OPEN_LOOP_TIME;
				
			}
			else
			{
	        	movestepy_time = CLOSE_LOOP_TIME;
				
			}
			movestep_y(1);
			
			if(run_time < 850)  
			   run_time = 850;	
			  	  
			delay_us(run_time);
			if( j<= 40 )
			{
				run_time -= speed_time;
				j++;
			}
  	  		temp16 = temp16 + 1;
  	  		if(temp16 > 18000)
  	  		{
  	  			sys.status = ERROR;
				StatusChangeLatch = ERROR;
				if(sys.error == 0)
      				sys.error = ERROR_26;     	// Y origin sensor is not normal  	 	            
      			return;
  	  		}
			if (PAUSE == PAUSE_ON)
			{
				delay_ms(10);                           
				if(PAUSE == PAUSE_ON)	
				{
					pause_flag = 1;
					return;
				}	
			}
			if(sys.status == ERROR)
			{
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
				return;
			}
			rec_com();	
    	}
	    if(j>=40)
		{
			movestepy_time = CLOSE_LOOP_TIME;
			
		}
		else
		{
			movestepy_time = OPEN_LOOP_TIME;
			
		}
		for(i=5;i>0;i--) 
	    { 
			movestep_y(1);  	  
  	  		run_time += (speed_time*8-1600); 
  	     	delay_us(run_time);
		}
		
		delay_ms(20);
	}
    temp16 = 0;
	delay_ms(12);
	for(i=0;i<3;i++)
	{
		while(YORG == para.y_sensor_open_level)
    	{
	        movestepy_time = OPEN_LOOP_TIME;
			
			movestep_y(-1);  	  
  	  		delay_ms(12);

  	  		temp16 = temp16 + 1;
  	  		if(temp16 > 8000)
  	  		{
  	  			sys.status = ERROR;
				StatusChangeLatch = ERROR;
				if(sys.error == 0)
      			   sys.error = ERROR_26;     		 	            
      			return;
  	  		}
			if (PAUSE == PAUSE_ON)
			{
				delay_ms(10);                           
				if(PAUSE == PAUSE_ON)	
				{
					pause_flag = 1;
					return;
				}	
			}
			if(sys.status == ERROR)
			{
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
				return;
			}
			rec_com();	
    	}    
		delay_ms(20);  
  	}
  	delay_ms(1);		
	if( y_origin_offset !=0 )
 		go_commandpoint(allx_step,y_origin_offset);
	ally_step = 0;
}

void go_origin_xy_both(void)
{	
	UINT8 i,moving_flag;
	UINT16 temp16;
	UINT16 speed_time,run_time;
	UINT8 y_cnt,x_cnt,x_over_cnt,y_over_cnt;
	
	speed_time = 292;//(300- (UINT16)(50/(11-go_origin_speed))); 
	
	y_cnt = 1;
	x_cnt = 1;
	run_time = 12000;
	x_over_cnt = 0;
	y_over_cnt = 0;
	pause_flag = 0;
	//step 1
	for( temp16 = 0;temp16 <25000; temp16++)
	{
		moving_flag = 1;
		if( (YORG != para.y_sensor_open_level)||(y_over_cnt<5) )
		{
			if( YORG == para.y_sensor_open_level)
				y_over_cnt ++;
			moving_flag = 0;
		    if(y_cnt < 40)
			{
				movestepy_time = OPEN_LOOP_TIME;
				
			}
			else
			{
	        	movestepy_time = CLOSE_LOOP_TIME;
				
			}
			movestep_y(1);
			delay_us(400);
			if( y_cnt < 40 )
			    y_cnt++;
		}
		if( (XORG == para.x_sensor_open_level)||(x_over_cnt<5) )
		{
			if( XORG != para.x_sensor_open_level)
			    x_over_cnt++;
			moving_flag = 0;
			if( x_cnt < 40)
			{
				movestepx_time = OPEN_LOOP_TIME;
				
			}
			else
			{
		        movestepx_time = CLOSE_LOOP_TIME;
				
			}
			if( x_sensor_pos == X_SENSOR_AT_LEFT)
				movestep_x(1);
			else
		    	movestep_x(-1);  
			if( x_cnt < 40)
				x_cnt++;			
		}
		
		if( temp16 <= 40 )
			run_time -= speed_time;
		if( run_time < 400)  
		    run_time = 400;	
		delay_us(run_time);
		if (PAUSE == PAUSE_ON)
		{
			delay_ms(10);                           
			if(PAUSE == PAUSE_ON)	
			{
				pause_flag = 1;
				return;
			}	
		}
		rec_com();
		if(sys.status == ERROR)
		{
			sys.status = ERROR;
			StatusChangeLatch = ERROR;
			return;
		}
		if( moving_flag == 1)
		    break;
	}
	//delay_ms(12);
	//go_origin_y();	
	//delay_ms(12);
	//go_origin_x();
	//=================================================
	temp16 = 0;
	delay_ms(12);

	for(i=0;i<3;i++)
	{
		while(YORG == para.y_sensor_open_level)
    	{
	        movestepy_time = CLOSE_LOOP_TIME;
			
			movestep_y(-1);  	  
  	  		delay_ms(5);

  	  		temp16 = temp16 + 1;
  	  		if(temp16 > 8000)
  	  		{
  	  			sys.status = ERROR;
				StatusChangeLatch = ERROR;
				if(sys.error == 0)
      			   sys.error = ERROR_26;     		 	            
      			return;
  	  		}
			rec_com();	
			if(sys.status == ERROR)
			{
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
				return;
			}
    	}    
		delay_ms(20);  
  	}
  	delay_ms(12);		
	ally_step = 0;
	//==============================================
	//         xorg=0    xorg =1
	//left       1       -1
	//right      -1       1
	if( para.x_origin_mode == AUTO_FIND_START_POINT)
   {
	   delay_ms(1);			
  	   allx_step = 0;	
	   return;
   }
	i=0;
	while( (XORG != para.x_sensor_open_level)||(i<5) )
	{
		movestepx_time = CLOSE_LOOP_TIME;
		
		if( x_sensor_pos == X_SENSOR_AT_LEFT)
			movestep_x(-1);
		else
		    movestep_x(1);  
		delay_ms(2);
		i++;
		if(sys.status == ERROR)
		{
			sys.status = ERROR;
			StatusChangeLatch = ERROR;
			return;
		}
	}
	temp16 = 0;
	delay_ms(5);
	for(i=0;i<3;i++)
	{
		while(XORG == para.x_sensor_open_level)
        {
	            movestepx_time = CLOSE_LOOP_TIME;
				
				if( x_sensor_pos == X_SENSOR_AT_LEFT)
					movestep_x(1);
				else
		    		movestep_x(-1); 
		  	  	delay_ms(5);
  	  	    	temp16 = temp16 + 1;
  	  	    	if(temp16 > 20000)
  	  	    	{
  	  		    	sys.status = ERROR;
			    	StatusChangeLatch = ERROR;
					if(sys.error == 0)
      		    	   sys.error = ERROR_25;     	 	            
      		    	return;
  	  	    	}
				if(sys.status == ERROR)
				{
					sys.status = ERROR;
					StatusChangeLatch = ERROR;
					return;
				}
				
  	  	}
		delay_ms(20);
    }
 	allx_step = 0;
	ally_step = 0;
	if( (x_origin_offset !=0) || (y_origin_offset !=0) )
		 go_commandpoint(x_origin_offset,y_origin_offset);
	allx_step = 0;
	ally_step = 0;	 
	
}
#if ROTATE_CUTTER_ENABLE
void go_origin_qd(void)
{	
	UINT8 i;
	UINT16 temp16,j ;
    j = 0;

	if(PSENS != 0)           // sensor is not covered   
	{	
		temp16 = 0;
    	while(PSENS != 0)   
    	{
    		movestep_qd(-1,0);
			if(j < 2)
			    delay_ms(3);//	
			else if(j <4 )
				delay_us(2000);
		    else
				delay_us(1500);//1500
			if( j < 4 )
				j++;
			temp16 = temp16 + 1;			
			if( sys.status == ERROR)
		    {
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
				return;
			}
			if( temp16 > 60000)
  		  	{
  		  		sys.status = ERROR;
				StatusChangeLatch = ERROR;
    	  		sys.error = ERROR_85; 	 	            
    	  		return;
  		  	}	
			rec_com();	
    	}
  	}
  	else                    // sensor is covered
  	{
  		temp16 = 0;
		j = 0;
  		while(PSENS == 0)    
    	{
    		movestep_qd(1,0);
		    if(j < 2)
			    delay_ms(3);	
			else if(j <4 )
				delay_us(2000);
		    else
				delay_us(1500);
			if( j < 4 )
				j++;
			
  		  	temp16 = temp16 + 1;			
			if(sys.status == ERROR)
			{
				  sys.status = ERROR;
				  StatusChangeLatch = ERROR;
				  return;
			}
  		  	if(temp16 > 60000)
  		  	{
  		  		sys.status = ERROR;
				StatusChangeLatch = ERROR;
    	  		sys.error = ERROR_85;      	 	            
    	  		return;
  		  	}	
			rec_com();	
    	}
    	for(i=2;i>0;i--)     			 // continue running
		{
			movestep_qd(1,0);
  	  		delay_ms(2);
		}
		delay_ms(2);
		temp16 = 0;
		j = 0;
		while(PSENS == 1)                   
   		{
      		movestep_qd(-1,0);
			delay_ms(2);			
	  		temp16 = temp16 + 1;			
			if( sys.status == ERROR)
			{
			    sys.status = ERROR;
				StatusChangeLatch = ERROR;
				return;
			}
  	  		if(temp16 > 60000)
  	  		{
  	  			sys.status = ERROR;
				StatusChangeLatch = ERROR;
      			sys.error = ERROR_85;    	            
      			return;
  	  		}	
			rec_com();	
    	}
  	}

}
#endif
//--------------------------------------------------------------------------------------
//  Name:		go_origin_allmotor
//  Parameters:	None
//  Returns:	None
//  Description: all motor go origin 
//--------------------------------------------------------------------------------------
void go_origin_allmotor(void)
{	
	UINT8 temp8,opl_flag,i;
	PATTERN_DATA *TempStart_pointTemp;	
	pause_flag = 0;	 
	PEN_SIGNAL = 0;
	LASER_SIGNAL = 0;
	SUM = 0;
    temp8 = detect_position();	
    if(temp8 == OUT)    
    {
		find_dead_center();
	}
	if( first_power_on_flag == 0)
	{
		FA = 1;	
		delay_ms(50);
		first_power_on_flag = 1;
	}
	go_origin_zx(); 
	FA = 0;	
	if(sys.status == ERROR)
    {
		origin_com = 0;
		predit_shift = 0;
		return;
    }
	if(sys.status != FINISH)
	{
		if( u37 == 0)
		{
			footer_both_down();
			delay_ms(120); 
		}
	}
	if( cut_mode == STEPPER_MOTER_CUTTER )
		go_origin_yj();
	#if ROTATE_CUTTER_ENABLE
		go_origin_qd();	
	#endif	
	if(sys.status == ERROR)
    {
		origin_com = 0;
		predit_shift = 0;
		return;
    }
	manual_operation_flag = 0;
    if( (making_pen_actoin_flag ==1)&&((x_bios_offset != 0)||(y_bios_offset != 0)) )
	{
		if( making_pen_offset_done == 1)
		{
			making_pen_nopmove_flag =1;
			go_commandpoint(allx_step - x_bios_offset ,ally_step - y_bios_offset);
			making_pen_offset_done = 0;
			making_pen_nopmove_flag =0;
		}
		marking_finish_flag = 1;
		making_pen_actoin_flag = 0;
		laser_cutter_aciton_flag = 0;
	}
	last_pattern_number = 0;
	serail_number =0;
	opl_flag = opl_origin_flag ;
	go_origin_xy();
	if( pause_flag == 1 )
    {
		origin_com = 0;
		predit_shift = 0;
		return;
    }
	SNT_H = 0;
	if( finish_nopmove_pause_flag == 1)//快速找起缝点的过程中出现急停
	{
		origin_com = 0;
		predit_shift = 0;
		return;
	}

	if(sys.status == ERROR)
    {
			origin_com = 0;
			predit_shift = 0;
			return;
     }

		if( origin_adjust_flag ==1 )
		{
				if((x_bios!=0)||(y_bios!=0))                
				{
				    go_commandpoint(x_bios,y_bios);
					allx_step = 0;
					ally_step = 0;
				}
		}
		delay_ms(10);
 		
		allx_step = 0;
		ally_step = 0;
		
	    not_in_origin_flag = 0;
		opl_origin_flag = 1;
		origin_com = 0;
		pat_point = (PATTERN_DATA *)(pat_buf);
		sta_point = pat_point;
		SewTestStitchCounter = 0;
		
		pat_buff_total_counter = 0;
	    pat_writting_flag = 0;
	    pat_buff_write_offset = 0;
	
	    
		LastPatternHaveSOP = 0;
		already_in_origin = 1;
		nopmove_flag = 0;
	    making_pen_status =0;
		move_flag = 0;
		cut_flag = 0;
		stop_flag = 0;
		origin2_lastmove_flag = 0;
		SewingStopFlag = 0;
		RotateFlag = 0;
		FootRotateFlag = 0;
		end_flag = 0;
	
		last_inpress_position = inpress_high_base;
		inpress_first_flag = 0;     
		fun_default_flag = 0;

		if( special_machine_type ==0)
		{
			T_DIR =0;
			T_CLK = 0;
			T_HALF =0;           
			FK_OFF =0;
			FR_ON =0;
			FA = 0;
		}

		manual_cut_flag = 0;
		course_next_flag = 0;
		base_tension = tension_release_value;
		sewing_tenion = base_tension;
		temp_tension_last = 255;
		
		if( sys.status != PREEDIT && ready_go_setout_com ==0)
		{
			if(origin_footer_status == 0)
			  footer_both_down();
			else if(u37 != 2)
			  footer_both_up();
		}
	if( para.second_start_switch == 1)
	{
		second_start_counter = 1;
	}
	if( (sewingcontrol_flag == 2)&&(sewingcontrol_stitchs != 0) )
		need_backward_sewing = 1; 
	if( sewingcontrol_flag == 1)
		need_action_two = 1;
	if( ready_go_setout_com ==0)
		predit_shift = 0;	
	return_from_setout = 1;
	need_action_once = 0;
	nop_move_pause_flag = 0;
	making_pen_actoin_flag = 0;
	
}
//--------------------------------------------------------------------------------------
//  Name:		go_origin_xy
//  Parameters:	None
//  Returns:	None
//  Description: x motor and y motor go origin 
//--------------------------------------------------------------------------------------
void go_origin_xy(void)
{
	already_in_origin = 0;
	if(opl_origin_flag == 0 ) 	    
	 {
	     switch( u240)
		 {
			case 0://标准
			case 4://XY轴同步
			case 1://反转
			    if( para.x_origin_mode == AUTO_FIND_START_POINT)
			    {
	   				go_origin_y();
		      		if( pause_flag == 1 )
	  	         		return;
			    }
				else
					go_origin_xy_both();
				if( pause_flag == 1 )
	  	         	return;
			break;
			case 2://Y轴到X轴
				go_origin_y();
				if( pause_flag == 1 )
	  	         	return;
	  	      	go_origin_x();
				if( pause_flag == 1 )
	  	         	return;
			break;
			case 3://X轴到Y轴
				go_origin_x();
				if( pause_flag == 1 )
	  	         	return;
				go_origin_y();
				if( pause_flag == 1 )
	  	         	return;
			break; 
		 }
		 opl_origin_flag = 1;		 
		 finish_nopmove_pause_flag = 0;
		 last_pattern_number = 0;
		 serail_number = 0;
	 }
	 else                     
	 {
		  //1.0 如果说出现空送急停了，不论是平时空送还是回起缝点空送，都要从发生停车的位置返回到（0,0）
		  if( (nop_move_pause_flag ==1)||(finish_nopmove_pause_flag==1) )
		  {
		     allx_step = last_allx_step + read_step_x;//坐标值更新到与当前位置对应起来，否则Y向就出现撞出台板
			 ally_step = last_ally_step + read_step_y;
			 nop_move_pause_flag = 0;
			 finish_nopmove_pause_flag = 0;
		  }
		  if (special_go_allmotor_flag == 0)//空送急停后回读坐标出错，不知道当前位置，就不能通过直接回到（0,0）方式回原点
		  {
			 switch( u240)
			 {
				case 0://标准
				case 4://XY轴同步
				    go_commandpoint(0,0);          //坐标都正常的情况下，回原点就可以先高速回到（0,0）位置，然后再调传感器找原点
				break;
				case 1://反转
					go_origin_xy_both();
				break;
				case 2://Y轴到X轴
					go_commandpoint(allx_step,0);
			      	if( (sys.status == ERROR)||(nop_move_pause_flag == 1) )
					{
		  	         	opl_origin_flag = 0;
						finish_nopmove_pause_flag = 1;
						return;
					}					
		  	      	go_commandpoint(0,0);
					if( (sys.status == ERROR)||(nop_move_pause_flag == 1) )
					{
		  	         	opl_origin_flag = 0;
						finish_nopmove_pause_flag = 1;
						return;
					}
				break;
				case 3://X轴到Y轴
					go_commandpoint(0,ally_step);
					if( (sys.status == ERROR)||(nop_move_pause_flag == 1) )
					{
		  	         	opl_origin_flag = 0;
						finish_nopmove_pause_flag = 1;
						return;
					}
					go_commandpoint(0,0);	
					if( (sys.status == ERROR)||(nop_move_pause_flag == 1) )
					{
		  	         	opl_origin_flag = 0;
						finish_nopmove_pause_flag = 1;
						return;
					}
				break;
			 }
			 if( nop_move_pause_flag ==1)   //回起缝点过程中发现按急停了
			 {
				 finish_nopmove_pause_flag = 1;
				 nop_move_pause_flag = 0;
			 }
			 else
		 	 	delay_ms(50);
		  }
		  if(( finish_nopmove_pause_flag ==0)&&(u39 !=4))//没有在找原点空送到（0,0）阶段出现急停，则再执行传感器找原点操作
		  {
		 	  #if NEED_TAKE_UP_ORIGIN_PHASE
			  footer_both_up(); 
			  #endif
			  if( y_origin_offset ==0 )
			  {
 				  go_origin_y();
		 	  	  delay_ms(50);
			  }
			  if( x_origin_offset ==0 )
		 	  	  go_origin_x();
          }
	      opl_origin_flag = 1;
		  special_go_allmotor_flag = 0;
	 }
	 if( finish_nopmove_pause_flag ==0 )
	 {
		allx_step = 0;
		ally_step = 0;
		end_flag = 0;
		already_in_origin = 1;
	 }
}

//--------------------------------------------------------------------------------------
//  Name:		foot_up
//  Parameters:	None
//  Returns:	None
//  Description: foot up
//--------------------------------------------------------------------------------------
void foot_up(void)
{
	if(foot_flag == 0)
	{
		LM_AIR = 1;
		delay_ms(20);
		T_DIR_EXTEND = 1;
	}
	foot_flag = 1;
	foot_com = 2;
	
	if(u104 == 1)
	{	      	
		if( footer_working_mode ==1)
			delay_ms(150);						      				
		inpress_up(); 											
	}						      			
	if( sys.status != PREEDIT)
		already_auto_find_start_point = 0;						      			
}

void foot_half_up(void)
{
	if(foot_half_flag == 0)
	{
		FL = 1;
		//FL_ON= 0;
	}
	foot_half_flag = 1;      
	foot_half_com = 1;      
}

/*

*/
void footer_both_up(void)
{
	if(footer_working_mode != 0)         
	{
		if(LRfooter_up_mode == 2)
		{
			foot_half_up();
			if( footer_working_mode ==1)
				delay_ms(100);
			foot_up();
		}
		else	
		{
	   	    foot_up();
			if(footer_working_mode ==1)
			   delay_ms(100);
		    foot_half_up();
		}
		FootNumber=0;
	}
	else
	   foot_up();
}
//--------------------------------------------------------------------------------------
//  Name:		foot_down
//  Parameters:	None
//  Returns:	None
//  Description: foot down
//--------------------------------------------------------------------------------------
void foot_down(void)
{
	INT16 tmp16;
	if(foot_flag == 1)
	{
		T_DIR_EXTEND = 0;
		delay_ms(250);
		LM_AIR = 0;
	}
	delay_ms(10);
  	foot_flag = 0; 
  	foot_com = 2; 
	if(u104 == 1)
	{	      	
		if(footer_working_mode ==1)
			delay_ms(50);
		inpress_down(inpress_high_base); 
    }
}
//--------------------------------------------------------------------------------------
//  Name:		footer_procedure
//  Parameters:	None
//  Returns:	None
//  Description: 
//  step number:    1          2         3       4
//  mode 1:     foot_down  half_down  half_up  foot_up
//  mode 2:     half_down  foot_down  foot_up  half_up
//--------------------------------------------------------------------------------------

void footer_procedure(void)
{
	INT16 tmp16;
	if( footer_working_mode == 0 )//only one footer
	{
		if( foot_flag == 1)
      	    foot_down();
		else 
		    foot_up();	
    }
    else
    {
		FootNumber++;
		if(FootNumber >= 5)
			FootNumber = 1;
			
		if( LRfooter_down_mode == 0 )//action at same time
		{
			if( (foot_flag ==1) && (foot_half_flag ==1) )
			{
				foot_down();
				foot_half_down();
			}
			else
			{
				foot_up();
				foot_half_up();
			}
		}
		else
		{	
				switch(FootNumber)
		   		{
					case 1:
						if(LRfooter_down_mode == 1 )//first footer then half
							foot_down();
						else
							foot_half_down();
					break;
					case 2:
						if(LRfooter_down_mode == 1 )
							foot_half_down();
						else
							foot_down();
					break;
					case 3:
						if(LRfooter_up_mode == 0 )//both up
						{
							if(LRfooter_down_mode == 1 )
							{
								foot_half_up();
								foot_up();
							}
							else
							{
								foot_up();
								foot_half_up();
							}
						    FootNumber = 0; 	
						} 
						else if(LRfooter_up_mode == 1 )
							foot_up();
						else
							{
								foot_half_up();
								FOOTHALF_UP;
							}
					break;
					case 4:
					    if(LRfooter_up_mode == 1 )
							foot_half_up();
						else
							foot_up();
					break;
				}
			
		}
	}
}

void foot_half_down(void)
{
	if(foot_half_flag == 1)
	{
			//FL_ON = 1;
			FL = 0;
			delay_ms(10);
	 }
  	foot_half_flag = 0;      	
	foot_half_com = 0;        
}

void footer_both_down(void)
{
	if(footer_working_mode != 0)
	{
		if(LRfooter_down_mode == 1)
		{
				foot_down();
				if( footer_working_mode ==1 )
					delay_ms(100);
				foot_half_down();
		}
		else	
		{
				foot_half_down();
				if(footer_working_mode ==1)
					delay_ms(100);
				foot_down();
		}
		FootNumber=2;
	}
	else
		foot_down();
}
//--------------------------------------------------------------------------------------
//  Name:		inpress_up
//  Parameters:	None
//  Returns:	None
//  Description: inpress up
//--------------------------------------------------------------------------------------
void inpress_up(void)
{
	if( already_up_flag == 1)
	{
		already_up_flag = 0;
		return;
	}
	if(inpress_type == AIR_INPRESSER) 
	{
		R_AIR = 0;
		delay_ms(delay_of_inpress_up);
		inpress_flag = 1;     
	 	return ;
	}
	#if FOLLOW_INPRESS_FUN_ENABLE
	if( inpress_follow_high_flag == FOLLOW_INPRESS_HIGH )
	{
		movestep_zx(-inpress_follow_range,inpress_follow_down_speed);
		inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
		delay_ms(80);
	}
	#endif
	inpress_to(inpress_origin);
	delay_ms(80);
	//if(delay_of_inpress_up != 0 )
	//   delay_ms(delay_of_inpress_up);
	if( blow_air_counter != 0)
		delay_ms(wiper_end_time);
	FA = 0;				
	delay_ms(30);
	inpress_flag = 1;     
	inpress_com = 2;      
	MotorPositionSet = 0;	
	if(delay_of_inpress_up!=0)
	   delay_ms(delay_of_inpress_up);

}
//--------------------------------------------------------------------------------------
//  Name:		inpress_down
//  Parameters:	None
//  Returns:	None
//  Description: inpress down
//--------------------------------------------------------------------------------------
void inpress_down(UINT8 pos)
{
	if(inpress_type == AIR_INPRESSER)   
	{
		R_AIR = 1;
		delay_ms(100);
		inpress_flag = 0;   
		return ;
	}
	if(pos > 80)
       pos = 80;
    FA = 1;
	delay_ms(inpress_down_delay + 100);
	inpress_to(pos);  
	delay_ms(80);
	inpress_flag = 0; 
	inpress_com = 0;   

}

//--------------------------------------------------------------------------------------
//  Name:		find_dead_center
//  Parameters:	None
//  Returns:	None
//  Description:  motor find dead center
//--------------------------------------------------------------------------------------
void find_dead_center(void)
{	 
	INT16 temp16;	
	while(motor.stop_flag == 0)    
  	{
    	rec_com();             				// communication with panel	
  	}
	find_deadpoint_flag = 1;
	if(u42 ==0 )//u236
	{
	   temp16 = motor.angle_adjusted;		
       if( (temp16 > u236)&&(temp16 < DEGREE_180) )
		   	motor.dir = 1;
	   else
	        motor.dir = 0;
	    motor.spd_obj = DEADPOINT_SPD;	
	    while(1)
	    {
	    	rec_com(); 
		   	if(motor.spd_ref == motor.spd_obj)
		   	{
		    	break;
		   	}
	    }
	    motor.stop_angle = u236;
		if(motor.stop_angle >= CODE_SCALE)
		{
			motor.stop_angle = motor.stop_angle - CODE_SCALE;
		}          	  
	    motor.spd_obj = 0;  
	    while(motor.stop_flag == 0)    
	    {
	    	rec_com(); 
	    }       
	}
	else //u42=1
 	{
		temp16 = motor.angle_adjusted;		
       if( (temp16 > dead_point_degree)&&(temp16 < DEGREE_180) )
		   	motor.dir = 1;
	   else
	        motor.dir = 0;
	    motor.spd_obj = DEADPOINT_SPD;	
	    while(1)
	    {
	    	rec_com(); 
		   	if(motor.spd_ref == motor.spd_obj)
		   	{
		    	break;
		   	}
	    }
	    motor.stop_angle = dead_point_degree;
		if(motor.stop_angle >= CODE_SCALE)
		{
			motor.stop_angle = motor.stop_angle - CODE_SCALE;
		}  
    	motor.spd_obj = 0;            
    	while(motor.stop_flag == 0)    
    	{
      		rec_com();             		
    	}
  	}
	find_deadpoint_flag = 0;
  	delay_ms(20);
	need_down_flag = 1;
}

void keep_running_for_dead_center(void)
{	 
	INT16 temp16;	
	while(motor.stop_flag == 0)    
  	{
    	rec_com();             				// communication with panel	
  	}
	find_deadpoint_flag = 1;
	if(u42 ==0 )//u236
	{
	   temp16 = motor.angle_adjusted;		
        motor.dir = 0;
	    motor.spd_obj = 30;	
	    while(1)
	    {
	    	rec_com(); 
		   	if(motor.spd_ref == motor.spd_obj)
		   	{
		    	break;
		   	}
	    }
	    motor.stop_angle = u236;
		if(motor.stop_angle >= CODE_SCALE)
		{
			motor.stop_angle = motor.stop_angle - CODE_SCALE;
		}          	  
	    motor.spd_obj = 0;  
	    while(motor.stop_flag == 0)    
	    {
	    	rec_com(); 
	    }       
	}
	else //u42=1
 	{
		temp16 = motor.angle_adjusted;		
        motor.dir = 0;
	    motor.spd_obj = 30;	
	    while(1)
	    {
	    	rec_com(); 
		   	if(motor.spd_ref == motor.spd_obj)
		   	{
		    	break;
		   	}
	    }
	    motor.stop_angle = dead_point_degree;
		if(motor.stop_angle >= CODE_SCALE)
		{
			motor.stop_angle = motor.stop_angle - CODE_SCALE;
		}  
    	motor.spd_obj = 0;            
    	while(motor.stop_flag == 0)    
    	{
      		rec_com();             		
    	}
  	}
	find_deadpoint_flag = 0;
  	delay_ms(20);
	need_down_flag = 1;
}

void needle_down(void)
{
	while(motor.stop_flag == 0)    
  	{
    	rec_com();             				
  	}
	motor.dir = 0;
  	motor.spd_obj = 30;	 
  	while(1)
  	{
  		rec_com();             				
    	if(motor.spd_ref == motor.spd_obj)
    	{
	    	break;
    	}
  	}
	motor.stop_angle = DEGREE_180;
  	motor.spd_obj = 0;            
  	while(motor.stop_flag == 0)    
  	{
    	rec_com();  
  	}
	need_down_flag = 0;
}

void find_dead_point(void)
{	 
	INT16 temp16;	
	while(motor.stop_flag == 0)    
  	{
    	rec_com();             			
  	}
	find_deadpoint_flag = 1;
	temp16 = motor.angle_adjusted;	
  	if( (temp16>dead_point_degree)&&(temp16 <= DEGREE_180))
	{	
    	motor.dir = 1;
    	motor.spd_obj = DEADPOINT_SPD;	
    	while(1)
    	{
    		rec_com();             			
	    	if(motor.spd_ref == motor.spd_obj)
	    	{
		    	break;
	    	}
    	}
		motor.stop_angle = dead_point_degree ;  
		if(motor.stop_angle >= CODE_SCALE)
		{
			motor.stop_angle = motor.stop_angle - CODE_SCALE;
		}
    	motor.spd_obj = 0;  
    	while(motor.stop_flag == 0)    
    	{
    		rec_com();             		
    	}       
  	}
 	else
	{
	  	//--------------------------------------------------------------------------------------
	  	//  clockwise to up position
	  	//--------------------------------------------------------------------------------------
		motor.dir = 0;
		motor.spd_obj = DEADPOINT_SPD; 
		while(1)
		{
		  	rec_com();             			
		   	if(motor.spd_ref == motor.spd_obj)
		    {
			    break;
		    }
		}
		motor.stop_angle = dead_point_degree ;      
		if(motor.stop_angle >= CODE_SCALE)
		{
			motor.stop_angle = motor.stop_angle - CODE_SCALE;
		} 
		motor.spd_obj = 0;            
		while(motor.stop_flag == 0)    
		{
		    rec_com();             			
		} 
	}
	find_deadpoint_flag = 0;
 
}


void find_up_position(void)
{	 
	UINT8 temp8;	
	INT16 temp16;	
	
	while(motor.stop_flag == 0)    
  	{
    	rec_com();             			
  	}
	temp8 = detect_position();
	if(temp8 == OUT)
	{	
		find_deadpoint_flag = 1;
		temp16 = motor.angle_adjusted;		
	    if( (temp16 > u236)&&(temp16 < DEGREE_180) )
			   	motor.dir = 1;
		else
				motor.dir = 0;
	  	motor.spd_obj = DEADPOINT_SPD; 
	  	while(1)
	  	{
	  		rec_com();             			
	    	if(motor.spd_ref == motor.spd_obj)
	    	{
		    	break;
	    	}
	  	}
		motor.stop_angle = u236 ;     
		if(motor.stop_angle >= CODE_SCALE)
		{
			motor.stop_angle = motor.stop_angle - CODE_SCALE;
		}  
	  	motor.spd_obj = 0;            
	  	while(motor.stop_flag == 0)    
	  	{
	    	rec_com();             			
	  	}
		find_deadpoint_flag = 0;
	}
  	delay_ms(20);
    
}
void find_zero(void)
{
	UINT8 temp8;	
	INT16 temp16;	

	while(motor.stop_flag == 0)    
  	{
    	rec_com();             					
  	}
	
	temp8 = detect_position();
	if(temp8 == OUT)
	{	
		find_deadpoint_flag = 1;
	
		temp16 = motor.angle_adjusted;		
	    if( (temp16 > dead_point_degree)&&(temp16 < DEGREE_180) )
			   	motor.dir = 1;
		else
		        motor.dir = 0;
		motor.spd_obj = DEADPOINT_SPD;	
		while(1)
		{
		    	rec_com(); 
			   	if(motor.spd_ref == motor.spd_obj)
			   	{
			    	break;
			   	}
		}
		motor.stop_angle = dead_point_degree;
		if(motor.stop_angle >= CODE_SCALE)
		{
			motor.stop_angle = motor.stop_angle - CODE_SCALE;
		}          	  
		motor.spd_obj = 0;  
	    while(motor.stop_flag == 0)    
	    {
		    rec_com(); 
	    }
		find_deadpoint_flag = 0;
	}
  	delay_ms(20);
    
}

//--------------------------------------------------------------------------------------
//  Name:		go_startpoint
//  Parameters:	None
//  Returns:	None
//  Description: x motor and y motor go startpoint
//--------------------------------------------------------------------------------------
void go_startpoint(void)
{
	PATTERN_DATA *TempPatpoint;
	StopStatusFlag = 0;
	pat_point = (PATTERN_DATA *)(pat_buf);

	process_data();
	if(nopmove_flag == 1)
    {	  
		end_flag = 0;
		do_pat_point_sub_one();
		while(nopmove_flag == 1)
		{
			go_beginpoint(0); 
			if( nop_move_pause_flag == 1)
			{
				aging_com = 0;
				pause_flag = 0;
				return;
			}

			if(OutOfRange_flag == 1)
			{
				break;
			}
			process_data();
			if(nopmove_flag == 1)
			{
				delay_ms(QUICKMOVETIME);
				do_pat_point_sub_one();
			}
			else if(FootRotateFlag == 1||cut_flag==1||origin2_lastmove_flag == 1 || stop_flag == 1 || move_flag == 1||RotateFlag ==1||SewingStopFlag == 1||inpress_high_flag == 1)
			{
				do_pat_point_sub_one();
				break;
			}
			else if(end_flag == 1)
			{
				do_pat_point_sub_one();
				end_flag = 0;
				break;
			}
		}
	}
	else
	{
		do_pat_point_sub_one();
	}
	if(stop_flag == 1)	
	{
		StopStatusFlag = 1;
	} 	 
	if(origin2_lastmove_flag == 0)
	{
		TempPatpoint = pat_point;
		process_data();
		if(move_flag == 1)
		{
			SewTestStitchCounter = 1;
			move_flag = 0;
		}
		else if(end_flag == 1)
		{
			SewTestStitchCounter = 0;
			end_flag = 0;
		}
		pat_point = TempPatpoint;	
		LastPatternHaveSOP = 0;
	}
	else if(origin2_lastmove_flag == 1)
	{
		origin2_point = pat_point;
		SewTestStitchCounter = 0;
		LastPatternHaveSOP = 1;
	}

	TempStart_point = pat_point;

}

const UINT16 X2_tab[]=
{
//   0    1    2    3    4    5    6    7    8    9  
	 0,   1,   4,   9,  16,  25,  36,  49,  64,  81,
//  10   11   12   13   14   15   16   17   18   19  
	100, 121, 144, 169, 196, 225, 256, 289, 324, 361,
//  20   21   22   23   24   25   26   27   28   29  
	400, 441, 484, 529, 576, 625, 676, 729, 784, 841,
//  30   31   32   33   34   35   36   37   38   39  
	900, 961, 1024,1089,1156,1225,1296,1369,1444,1521,
//  40   41   42   43   44   45   46   47   48   49  
	1600,1681,1764,1849,1936,2025,2116,2209,2304,2401,
//  50   51   52   53   54   55   56   57   58   59  
	2500,2601,2704,2809,2916,3025,3136,3249,3364,3481,
//  60   61   62   63   64   65   66   67   68   69  
	3600,3721,3844,3969,4096,4225,4356,4489,4624,4761,
//  70   71   72   73   74   75   76   77   78   79  
	4900,5041,5184,5329,5476,5625,5776,5929,6084,6241,
//  80   81   82   83   84   85   86   87   88   89  
	6400,6561,6724,6889,7056,7225,7396,7569,7744,7921,
//  90   91   92   93   94   95   96   97   98   99  
	8100,8281,8464,8649,8836,9025,9216,9409,9604,9801,
// 100   101   102   103   104   105   106   107   108   109  
   10000,10201,10404,10609,10816,11025,11236,11449,11664,11881,
// 110   111   112   113   114   115   116   117   118   119  
   12100,12321,12544,12769,12996,13225,13456,13689,13924,14161,
// 120   121   122   123   124   125   126   127   128  129
   14400,14641,14884,15129,15376,15625,15876,16129,16384,16641,
// 130   131   132   133   134   135   136   137   138  139      
   16900,17161,17424,17689,17956,18225,18496,18769,19044,19321,
// 140   141   142   143   144   145   146   147   148  149     
   19600,19881,20164,20449,20736,21025,21316,21609,21904,22201,
// 150   151   152   153   154   155   156   157   158  159     
   22500,22801,23104,23409,23716,24025,24336,24649,24964,25281,
// 160   161   162   163   164   165   166   167   168  169   
   25600,25921,26244,26569,26896,27225,27556,27889,28224,28561,
// 170   171   172   173   174   175   176   177   178  179
   28900,29241,29584,29929,30276,30625,30976,31329,31684,32041,
// 180   181   182   183   184   185   186   187   188  189   
   32400,32761,33124,33489,33856,34225,34596,34969,35344,35721,
// 190   191   192   193   194   195   196   197   198  199   
   36100,36481,36864,37249,37636,38025,38416,38809,39204,39601,
// 200   201   202   203   204   205   206   207   208   209 
   40000,40401,40804,41209,41616,42025,42436,42849,43264,43681,
// 210   211   212   213   214   215   216   217   218   219    
   44100,44521,44944,45369,45796,46225,46656,47089,47524,47961,
// 220   221   222   223   224   225   226   227   228   229      
   48400,48841,49284,49729,50176,50625,51076,51529,51984,52441,
// 230   231   232   233   234   235   236   237   238   239      
   52900,53361,53824,54289,54756,55225,55696,56169,56644,57121,
// 240   241   242   243   244   245   246   247   248   249      
   57600,58081,58564,59049,59536,60025,60516,61009,61504,62001,
// 250   251   252   253   254   255    
   62500,63001,63504,64009,64516,65025
};
//--------------------------------------------------------------------------------------
//  Name:		check_data
//  Parameters:	None
//  Returns:	None
//  Description: check 3 data and set speed
//--------------------------------------------------------------------------------------

void check_data(UINT8 control_flag)
{
	UINT8 temp8,max8;
	static UINT8 Dec_Counter;
	INT16 x_temp,y_temp;
	INT16 i;
	UINT16 max16,temp16,setup_speed,inpress_tmp_speed;
	UINT16 length,k,m;
	UINT16 temp_speed;
	INT16 speed_margin;
	INT16 inpress_temp_delta;
	INT16 inpress_temp_new;
	INT16 inpress_temp_old;
	INT16 inpress_temp_change;
	UINT32 tmp1,tmp2;
	UINT8 c_flag;
	UINT16 Corner_deceleration_speed;
	PATTERN_DATA *Temp_point;
	UINT16 last_stitch_length;
	
	inpress_temp_delta = 0;
	inpress_temp_new = 0;
	inpress_temp_old = inpress_high;
	inpress_temp_change = 0;
	
	x_temp = 0;
	y_temp = 0;
	max16 = 0;
	slow_flag = 0;
	max8 = 0;
	c_flag = 0;
	
	Temp_point = pat_point;

	for(i=0;i<slowdown_stitchs;i++) 
	{		
		switch(Temp_point->func &0xFF)		
	    {
	    	//-----------------------------------------------------------------------	
	      	// fuction code
	      	//-----------------------------------------------------------------------	
			case 0x05:	
				if(slow_flag == 0) 
					{
						slow_flag = 1; 
						Dec_Counter = i;
					}
				break;
			case 0x1d://1d 02 xx 00
			      switch( (Temp_point)->para)
				  {
					 case 3://marking pen
						 if(slow_flag == 0) 
						{
							slow_flag = 1; 
							Dec_Counter = i;
						}
					 break;
					 case 2:
						 inpress_temp_delta = ((INT16)(Temp_point)->xstep);
						 inpress_temp_new = inpress_high_base + inpress_temp_delta;
			    		 if(inpress_temp_new > 80)
							inpress_temp_new = 80;
						 if(inpress_temp_new < 0)
							inpress_temp_new = 0;
			    		 inpress_temp_change = inpress_temp_new - inpress_temp_old;
			    		 inpress_temp_old = inpress_temp_new;
						 break;
						 
					 case 6://1d 06 xx 00 拐角降速
						if( start_to_speed_down == 0 )//先看是否已经启动了拐角降速
						{
						   if( Temp_point->xstep == 0 )//针数设置错误
							   break;
								
						   if( i == 4 )
						   {
							   start_to_speed_down = 1;
							   speed_down_counter = 0;
							   speed_down_stitchs = 9;							   
							   //0 1 2 3 4 5 6 7 8 
							   //0 0 0 0 1 0 0 0 0  1针降速
							   //0 0 0 1 2 1 0 0 0  2针降速
							   //0 0 1 2 3 2 1 0 0  3针降速
							   //0 1 2 3 4 3 2 1 0  4针降速
							   //1 2 3 4 5 4 3 2 1  5针降速
							   for( k=0; k<10; k++)
							        ratio_array [k] = motor.spd_obj;	//默认维持原速	
							   		 
							  ratio_array[4] = (UINT16)para.Corner_deceleration_speed*100;//默认中间一针降到设定转速						  
						  	  
							  switch(Temp_point->xstep)//看拐点要降几针
							  {
								  case 1:							  		
								  break;
								  case 2://2针降速
										ratio_array[3] = (UINT16)para.Corner_deceleration_speed1*100;//升速第一针
										ratio_array[5] = (UINT16)para.Corner_deceleration_speed1*100;
								  break;
								  case 3://3针降速
										ratio_array[2] = (UINT16)para.Corner_deceleration_speed2*100; 
										ratio_array[3] = (UINT16)para.Corner_deceleration_speed1*100;
										
										ratio_array[5] = (UINT16)para.Corner_deceleration_speed1*100;
										ratio_array[6] = (UINT16)para.Corner_deceleration_speed2*100;							
								  break;
								  case 4://4针降速
						
								  		ratio_array[1] = (UINT16)para.Corner_deceleration_speed3*100;
								     	ratio_array[2] = (UINT16)para.Corner_deceleration_speed2*100;
										ratio_array[3] = (UINT16)para.Corner_deceleration_speed1*100;
																	
										ratio_array[5] = (UINT16)para.Corner_deceleration_speed1*100;
										ratio_array[6] = (UINT16)para.Corner_deceleration_speed2*100;
										ratio_array[7] = (UINT16)para.Corner_deceleration_speed3*100;
								  break;
								  case 5://5针降速
									
								  		ratio_array[0] = (UINT16)para.Corner_deceleration_speed4*100;
										ratio_array[1] = (UINT16)para.Corner_deceleration_speed3*100;
								     	ratio_array[2] = (UINT16)para.Corner_deceleration_speed2*100;
										ratio_array[3] = (UINT16)para.Corner_deceleration_speed1*100;
																		
										ratio_array[5] = (UINT16)para.Corner_deceleration_speed1*100;
										ratio_array[6] = (UINT16)para.Corner_deceleration_speed2*100;
										ratio_array[7] = (UINT16)para.Corner_deceleration_speed3*100;
										ratio_array[8] = (UINT16)para.Corner_deceleration_speed4*100;
								  break;
							  }//end switch
							  /*
							   if(  Temp_point->xstep >=3 )
							        Corner_deceleration_speed = para.Corner_deceleration_speed;
							   else if( Temp_point->xstep == 2 )
							   	    Corner_deceleration_speed = 14;
							   else if(  Temp_point->xstep ==1 )
							   	    Corner_deceleration_speed = 22;
							   setup_speed = Corner_deceleration_speed*100;
							   
							   if( motor.spd_obj < setup_speed)//实际运行的转速比降速还低,就不用提前降速处理了
							        break;
							   start_to_speed_down = 1;
							   speed_down_counter = 0;
							   speed_margin =  motor.spd_obj - setup_speed;//转速差
							   speed_down_stitchs = 9;						   
			  
							   //0 1 2 3 4 5 6 7 8 
							   //0 0 0 0 1 0 0 0 0  1针降速
							   //0 0 0 1 2 1 0 0 0  2针降速
							   //0 0 1 2 3 2 1 0 0  3针降速
							   //0 1 2 3 4 3 2 1 0  4针降速
							   //1 2 3 4 5 4 3 2 1  5针降速
							   

							   for( k=0;k<10;k++)
							       ratio_array [k] = motor.spd_obj;	//默认维持原速	
							   		 
							  ratio_array[4] = setup_speed;//默认中间一针降到设定转速
						  
						  	  
							  switch(Temp_point->xstep)//看拐点要降几针
							  {
								  case 1:							  		
								  break;
								  case 2://2针降速
								  		speed_margin = speed_margin/2;
										ratio_array[3] = motor.spd_obj - speed_margin;
										ratio_array[5] = setup_speed + speed_margin ;
								  break;
								  case 3://3针降速	
								  		speed_margin = speed_margin/3;						   		
										ratio_array[2] = motor.spd_obj - speed_margin; 
										ratio_array[3] = motor.spd_obj - 2*speed_margin;
										ratio_array[5] = setup_speed + speed_margin ;
										ratio_array[6] = setup_speed + 2*speed_margin ;							
								  break;
								  case 4://4针降速
								  		speed_margin = speed_margin/4;	
								  		ratio_array[1] = motor.spd_obj - speed_margin;
								     	ratio_array[2] = motor.spd_obj - 2*speed_margin;
										ratio_array[3] = motor.spd_obj - 3*speed_margin;
																	
										ratio_array[5] = setup_speed + speed_margin ;
										ratio_array[6] = setup_speed + 2*speed_margin ;
										ratio_array[7] = setup_speed + 3*speed_margin ;
								  break;
								  case 5://5针降速
								  		speed_margin = speed_margin/5;	
									
								  		ratio_array[0] = motor.spd_obj - speed_margin;
										ratio_array[1] = motor.spd_obj - 2*speed_margin;
								     	ratio_array[2] = motor.spd_obj - 3*speed_margin;
										ratio_array[3] = motor.spd_obj - 4*speed_margin;
																		
										ratio_array[5] = setup_speed + speed_margin ;
										ratio_array[6] = setup_speed + 2*speed_margin ;
										ratio_array[7] = setup_speed + 3*speed_margin ;
										ratio_array[8] = setup_speed + 4*speed_margin ;
								  break;
							  }//end switch
							  */
						  }
					  }//end if
					 break;					 
					 
				  }//case 1d
				 break; 				 
				 
			case 0x02://02 00 00 00
					if( slow_flag == 0 ) 
					{
						slow_flag = 1; 
						Dec_Counter = i;
						c_flag = 1;
					}
					break;
			case 0x04://04 00 00 00 
			case 0x14://14 00 00 00
			case 0x06://06 00 00 00
			case 0x07://07 00 00 00	 
			case 0x1f://1f 00 00 00
					if( slow_flag == 0 ) 
					{
						slow_flag = 1; 
						Dec_Counter = i;
						if( finish_cut_flag ==1)
						    c_flag = 1;
					} 
				break;  
	   		case 0x1b: 	
      		case 0x03:   
				 x_temp = ChangeX(Temp_point);
				 y_temp = ChangeY(Temp_point);   	 
				 if(slow_flag == 0) 
				 {
					  slow_flag = 1; 
					  Dec_Counter = i;
					  if( before_nopmove_cut_flag ==1 )
					      c_flag = 1;
				 }
			    break;    					
    		//-----------------------------------------------------------------------	
      		// normal stitch
      		//-----------------------------------------------------------------------
			case 0x61:  //
    		case 0x41: 	//
      		case 0x21:  //
      		case 0x01:  //
				x_temp = ChangeX((Temp_point));
				y_temp = ChangeY((Temp_point));
    		    break;    					
    		//-----------------------------------------------------------------------	
      		// default
      		//-----------------------------------------------------------------------		   		  	  				
    		default:   
				break;
	    }
		
		Temp_point++;		
		if(super_pattern_flag == 1)
		{
			if( Temp_point >= (PATTERN_DATA *)(pat_buf)+TOTAL_STITCH_COUNTER )			
			    Temp_point = (PATTERN_DATA *)(pat_buf);				
		}
		
		if( x_temp >= 0)
		  	temp16 = X2_tab[x_temp];
		else
		  	temp16 = X2_tab[-x_temp];
		if( y_temp >= 0)
		  	temp16 += X2_tab[y_temp];
		else
		    temp16 += X2_tab[-y_temp];
		 
		for( k=0;k<256;k++)//Z^2=>z (Z^2<127^2)
		{
			if( temp16 <= X2_tab[k])
			   break;
		}
		temp16 = k;
		last_stitch_length = temp16;
		
		if(i == 0) 		//current stitch
		   caculate_stitch_step = k ;
	
    	if(temp16 > max16)
    	{
      		max16 = temp16;	
    	}
		//-----------------------------------------------------------------------	
    	// check max inpress change length
    	//-----------------------------------------------------------------------	    
    	temp8 = fabsm((INT8)inpress_temp_change);
    	if(temp8 > max8)
    	{
      		max8 = temp8;	
    	}	   
   		if(slow_flag == 1)
		   break;
  	}
   
 	//inpress	
  	if(fabsm((INT8)inpress_real_delta_runing)> max8)
  	{
    	max8 = fabsm((INT8)inpress_real_delta_runing);	
    }
 	
  	last_speed = motor.spd_obj;     
	
	if( max16 > 127)
	    max16 = 126;		      
	if( max16 == 0)
	  	max16 = 1;
	length = max16/10;
  
		
  	if(slow_flag == 1)               
  	{
  		length = 13; 
		switch(Dec_Counter)
				{
					#if OPEN_LAST_5_STITCH_SPEED
					case 0:
						if(motor.spd_obj >= (UINT16)para.last_1_speed *100)
							{
								temp_speed = para.last_1_speed*100;
							}
							else
							{
								temp_speed = last_speed;
							}
						break;
					case 1:
						if(motor.spd_obj >= (UINT16)para.last_2_speed *100)
							{
								temp_speed = (UINT16)para.last_2_speed *100;
							}
							else
							{
								temp_speed = last_speed;
							}
					
						break;
					case 2:
						
							if(motor.spd_obj >= (UINT16)para.last_3_speed *100)
							{
								temp_speed = (UINT16)para.last_3_speed *100;
							}
							else
							{
								temp_speed = last_speed;
							}
						
						break;
					case 3:
						if(motor.spd_obj >= (UINT16)para.last_4_speed *100)
						{
							temp_speed = (UINT16)para.last_4_speed *100;
						}
						else
						{
							temp_speed = last_speed;
						}
						break;
					case 4:
					
						if(motor.spd_obj >= (UINT16)para.last_5_speed *100)
						{
							temp_speed = (UINT16)para.last_5_speed *100;
						}
						else
						{
							temp_speed = last_speed;
						}
			
						break;
						
					#else
					case 0:
						if( sewingcontrol_tail_flag != 0)
						{
							if(motor.spd_obj >= 10*tail_sewing_speed)
							{
								temp_speed = 10*tail_sewing_speed;
							}
							else
							{
								temp_speed = last_speed;
							}
						}
						else
						{
							if(motor.spd_obj >= 10*u211)
							{
								temp_speed = 10*u211;
							}
							else
							{
								temp_speed = last_speed;
							}
						}
						break;
					case 1:
						if( sewingcontrol_tail_flag != 0)
						{
							if(motor.spd_obj >= 10*tail_sewing_speed)
							{
								temp_speed = 10*tail_sewing_speed;
							}
							else
							{
								temp_speed = last_speed;
							}	
						}
						else
						{
							if(motor.spd_obj >= 400)
							{
								temp_speed = 400;
							}
							else
							{
								temp_speed = last_speed;
							}
						}
						break;
					case 2:
						if( sewingcontrol_tail_flag != 0)
						{
							if(motor.spd_obj >= 10*tail_sewing_speed)
							{
								temp_speed = 10*tail_sewing_speed;
							}
							else
							{
								temp_speed = last_speed;
							}
						}
						else
						{
							if(motor.spd_obj >= 1000)
							{
								temp_speed = 1000;
							}
							else
							{
								temp_speed = last_speed;
							}
						}
						break;
					case 3:
						if(motor.spd_obj >= 1600)
						{
							temp_speed = 1600;
						}
						else
						{
							temp_speed = last_speed;
						}
						break;
					case 4:
					
						if(motor.spd_obj >= 2200)
						{
							temp_speed = 2200;
						}
						else
						{
							temp_speed = last_speed;
						}
			
						break;
					#endif	
		    	
			}
  	}      
  	else
  	{
            temp_speed = spdlimit_10080_345_tab[max16-1] ;
				
			if( ( inpress_type == FOLLOW_UP_INPRESSER )&&( temp_speed > inpress_follow_speed ) )//随动中压脚降速；
				  temp_speed = inpress_follow_speed;
			if( sew_speed >  PatternSpeedLimited*100) 
	  			sew_speed =  PatternSpeedLimited*100;
			if( temp_speed > PatternSpeedLimited*100) 
	  			temp_speed = PatternSpeedLimited*100;	
			
			//-----------------------------------------------------------------------	
		  	// speed increase or speed decrease
		  	//-----------------------------------------------------------------------    
		  	if(sew_speed > temp_speed)    
		  	{							
			  	if(last_speed > temp_speed)
			  	{
			  		speed_margin = last_speed - temp_speed;
			  		if(speed_margin > 1500)
			  		{
		  				temp_speed = last_speed - 1500;
			  		} 
		  		}
			  	else      
			  	{
			  		speed_margin = temp_speed - last_speed;
			  		if(speed_margin > 1500)
			  		{
		  				temp_speed = last_speed + 1500;
			  		}
		  		}
		  	}
		  	else                             							
		  	{															
			  	if(last_speed > sew_speed)    							
			  	{
			  		speed_margin = last_speed - sew_speed;
			  		if(speed_margin > 1500)
			  		{
		  				temp_speed = last_speed - 1500;
		  			}
		  			else
		  			{
		  				temp_speed = sew_speed;
		  			}
		  		}
		  		else 
		  		{
			  		speed_margin = sew_speed - last_speed;
			  		if(speed_margin > 1500)
			  		{
		  				temp_speed = last_speed + 1500;  			
		  			}
		  			else
		  			{
		  				temp_speed = sew_speed;
		  			}
		  		}
		  	}                               
  	}  
	/*
	   拐点降速的处理
	   有设定的具体降速值,用几针降到指定转速,然后再升速回去	   
	*/
    if( (start_to_speed_down == 1 )&&( (length!=13)||(tail_sewing_flag==1) ) )//尾部加固或者是不提前降速情况下的拐点降速处理
	{
		if( speed_down_stitchs >0)
		{
			if( control_flag == 1)
				speed_down_stitchs--;
			setup_speed = ratio_array[speed_down_counter];//(ratio_array[speed_down_counter]+50)/100*100;	
			if( setup_speed < 200)
		   		setup_speed = 200;
			if(temp_speed > setup_speed)
	    	{
	      	   temp_speed = setup_speed;	
	    	} 			
			if( control_flag == 1)
				speed_down_counter++;
		}
		else
		{
		    start_to_speed_down = 0;
		}
	}
		
	if( para.slow_start_mode == 1)
	{
			if(stitch_counter < 18)
	  		{
				switch(stitch_counter)
	  	  		{
			  	  	case 1:
						temp16 = 200;
						break;  
					case 2:
						temp16 = 200;
						break;		  	      
			      	case 3: 
						temp16 = 400;
						break;  	  	      
			      	case 4: 
						temp16 = 600;
						break;      		      
			      	case 5: 
						temp16 = 800;
						break;	      	      	            	
			      	case 6: 
						temp16 = 1000;
						break;    
					case 7: 
						temp16 = 1200; 
						break;
					case 8: 
						temp16 = 1400;   
						break;
					case 9: 
						temp16 = 1500; 
						break;
					case 10: 
						temp16 = 1700;   
						break;
					case 11: 
						temp16 = 2000;  
						break;
					case 12: 
						temp16 = 2100;   
						break;	
					case 13: 
						temp16 = 2200;   
						break;
					case 14: 
						temp16 = 2300;  
						break;	 		  	 		  		 		  		      
					case 15: 
						temp16 = 2400;   
						break;
					case 16: 
						temp16 = 2500;   
						break;
					case 17: 
						temp16 = 2600;
  						break;
	  	  		}
			if(temp_speed > temp16)
	    	{
	      		temp_speed = temp16;	
	    	} 	
		}
	}
	else
	{
			if(stitch_counter < 6)//23
	  		{
				switch(stitch_counter)
	  	  		{
			  	  	case 1:
						temp16 = u10 * 100;//300
						if( temp16 > 1500)
						    temp16 = 1500;
						break;  	  	      
			      	case 2: 
						temp16 = u11 * 100;//500
						if( temp16 > 3000)
						    temp16 = 3000;
						break;  	  	      
			      	case 3: 
						temp16 = u12 * 100;//700
						if( temp16 > 3000)
						    temp16 = 3000;
						break;      		      
			      	case 4: 
						temp16 = u13 * 100;//900
						if( temp16 > 3000)
						    temp16 = 3000;
						break;	      	      	            	
			      	case 5: 
						temp16 = u14 * 100;//1100  
						if( temp16 > 3000)
						    temp16 = 3000;
						break;    
					
			      	default:                      
						break;        			
	  	  		}

			if(RotateFlag == 1)
			   temp_speed = temp16;	
	     	else if(temp_speed > temp16)
	    	{
	      		temp_speed = temp16;	
	    	} 	
		}
	}	
  	//-----------------------------------------------------------------------	
  	// set object speed
  	//-----------------------------------------------------------------------

	if( (length!=13)&&( ((para.slow_start_mode == 0)&&(stitch_counter>5))||((para.slow_start_mode == 1)&&(stitch_counter>17)) )&&( start_to_speed_down == 0) )
	{
		 if( (max16 <= HighSpeedStitchLength )&&(inpress_type != FOLLOW_UP_INPRESSER) )
		 {
			 temp_speed = sew_speed;
		 }
	}
	
	before_down_speed = temp_speed;     
	      
	if( length!=13 )
	{
		if(  u229 >= 1 )
		{
			movestepx_time = MoveTime_Speed_10080_ND80[temp_speed/100];
			if(u229 == 1 )
			   tmp1 = 345 - u231 * 6;//default
			else
			   tmp1 = 345 - u232 * 6;
			// n =60000/(t*360/temp16)   
			tmp2 = 3*movestepx_time;
			tmp1 = (500*tmp1)/tmp2;
		
			tmp1 = ((tmp1 +50)/100)*100;
			temp_speed =(UINT16)tmp1;
		}	
		if(temp_speed > before_down_speed)
		   temp_speed = before_down_speed;
	}
	if( control_flag == 1)
	{
		motor.spd_obj = temp_speed;	
		spd_monitor = temp_speed;	
	}

#if FOLLOW_INPRESS_FUN_ENABLE
    if( motor.spd_obj> temp_speed)
	    inpress_tmp_speed = (motor.spd_obj - temp_speed)/2+temp_speed;
	else
		inpress_tmp_speed = temp_speed;
    
	temp_speed = inpress_tmp_speed / 100;
	
	if( temp_speed > 30)
	    temp_speed = 30;
		
	if( para.zx_curver == 0 )
	{
		inpress_follow_down_angle = inpress_follow_down_angle_tab2[temp_speed];
		inpress_follow_down_speed = inpress_follow_down_speed_tab2[temp_speed];
		inpress_follow_up_angle   = inpress_follow_up_angle_tab2[temp_speed];
		inpress_follow_up_speed   = inpress_follow_up_speed_tab2[temp_speed];
		
	}	
	else if( para.zx_curver == 1 )
	{
	    inpress_follow_down_angle = inpress_follow_down_angle_tab3[temp_speed];
		inpress_follow_down_speed = inpress_follow_down_speed_tab3[temp_speed];
		inpress_follow_up_angle   = inpress_follow_up_angle_tab3[temp_speed];
		inpress_follow_up_speed   = inpress_follow_up_speed_tab3[temp_speed];
	}
	else if( para.zx_curver == 2 )
	{
		inpress_follow_down_angle = inpress_follow_down_angle_tab4[temp_speed];
		inpress_follow_down_speed = inpress_follow_down_speed_tab4[temp_speed];
		inpress_follow_up_angle   = inpress_follow_up_angle_tab4[temp_speed];
		inpress_follow_up_speed   = inpress_follow_up_speed_tab4[temp_speed];
	}
	else if( para.zx_curver == 3 )
	{
	    inpress_follow_down_angle = inpress_follow_down_angle_tab[temp_speed];
		inpress_follow_down_speed = inpress_follow_down_speed_tab[temp_speed];
		inpress_follow_up_angle   = inpress_follow_up_angle_tab[temp_speed];
		inpress_follow_up_speed   = inpress_follow_up_speed_tab[temp_speed];
	}
	else if( para.zx_curver == 4 )
	{
	    inpress_follow_down_angle = inpress_follow_down_angle_tab5[temp_speed];
		inpress_follow_down_speed = inpress_follow_down_speed_tab5[temp_speed];
		inpress_follow_up_angle   = inpress_follow_up_angle_tab5[temp_speed];
		inpress_follow_up_speed   = inpress_follow_up_speed_tab5[temp_speed];
	}
	else if( para.zx_curver == 5 )
	{
	    inpress_follow_down_angle = inpress_follow_down_angle_tab6[temp_speed];
		inpress_follow_down_speed = inpress_follow_down_speed_tab6[temp_speed];
		inpress_follow_up_angle   = inpress_follow_up_angle_tab6[temp_speed];
		inpress_follow_up_speed   = inpress_follow_up_speed_tab6[temp_speed];
	}
	else if( para.zx_curver == 6 )
	{
	    inpress_follow_down_angle = inpress_follow_down_angle_tab8[temp_speed];
		inpress_follow_down_speed = inpress_follow_down_speed_tab8[temp_speed];
		inpress_follow_up_angle   = inpress_follow_up_angle_tab8[temp_speed];
		inpress_follow_up_speed   = inpress_follow_up_speed_tab8[temp_speed];
		inpress_follow_range = 3;
	}
	else
	{
	    inpress_follow_down_angle = inpress_follow_down_angle_tab7[temp_speed];
		inpress_follow_down_speed = inpress_follow_down_speed_tab7[temp_speed];
		inpress_follow_up_angle   = inpress_follow_up_angle_tab7[temp_speed];
		inpress_follow_up_speed   = inpress_follow_up_speed_tab7[temp_speed];
	}
	
	if( follow_inpresser_angle_adj != 0)//随动角度微调
	{
		inpress_follow_down_angle += follow_inpresser_angle_adj;
		if( inpress_follow_down_angle > 340 )
			inpress_follow_down_angle -= 340;
				
		inpress_follow_up_angle += follow_inpresser_angle_adj;
		
		if( inpress_follow_up_angle > 340 )
			inpress_follow_up_angle -= 340;
	}

#endif
	
}


//--------------------------------------------------------------------------------------
//  Name:		calculate_angle
//  Parameters:	None
//  Returns:	None
//  Description: calculate stepper motor move angle
//--------------------------------------------------------------------------------------
void calculate_angle(void)
{
	UINT16 temp16;
	UINT16 length;
	INT16 temp_angle,temp_anglex,temp_angley;
	INT16 temp_speed;
	UINT16 temp16x;
	UINT16 temp16y;
    INT16 x_temp,y_temp;

    temp_angle = 0;
	//temp_speed = motor.spd_obj;
	temp_speed = spd_monitor;
	if( u229 >= 1)
	  	temp_speed = before_down_speed; 	
			
  	temp_speed = temp_speed/100;
	
   	temp16x = fabsm(xstep_cou);
	temp16y = fabsm(ystep_cou);
	
	movestepx_time = MoveTime_Speed_10080_x[temp_speed];
	temp_anglex    = MoveStartAngle_10080_x[temp_speed];
	
	movestepy_time = MoveTime_Speed_10080_y[temp_speed];
	temp_angley    = MoveStartAngle_10080_y[temp_speed];
	
	movestepx_time += mode1_x_time;
	if( movestepx_time >= MoveTime_Limtit[temp_speed] )
		movestepx_time =  MoveTime_Limtit[temp_speed]-1;
	if( movestepx_time < 6 )
		movestepx_time = 6;
	
	movestepy_time += mode1_y_time;
	if( movestepy_time >= MoveTime_Limtit[temp_speed] )
		movestepy_time =  MoveTime_Limtit[temp_speed]-1;
	if( movestepy_time < 6 )
		movestepy_time = 6;	

	temp_anglex += mode1_x_angle;
	if( temp_anglex < 10)
	    temp_anglex = 10;
	if( temp_anglex > 350)
	    temp_anglex = 350;

	temp_angley += mode1_y_angle; 
	if( temp_angley < 10)
	    temp_angley = 10;
	if( temp_angley > 350)
	    temp_angley = 350;		   
	
	if( (temp16x == 0 )&&(temp16y > 0) )
	{
	     temp_anglex = temp_angley;
		 movestepx_time = movestepy_time;
	}
	else if( (temp16y == 0 )&&(temp16x > 0) )
	{
	   	 temp_angley = temp_anglex;
		 movestepy_time = movestepx_time;
	}
	   		 
	if( temp_anglex > temp_angley)
	    temp_angle = temp_angley;
	else
	    temp_angle = temp_anglex;	
	   
	movestepx_angle = angle_tab[temp_anglex];
    movestepy_angle = angle_tab[temp_angley];
	   
	if( temp_angle >350)
	    temp_angle =350;
		   
   	movestep_angle = angle_tab[temp_angle];
	
}
//--------------------------------------------------------------------------------------
//  Name:		process_data
//  Parameters:	None
//  Returns:	None
//  Description: process pattern data
//--------------------------------------------------------------------------------------
				
void process_data(void)
{
	UINT8 break_flag;
	INT16 temp16;
	xstep_cou = 0 ;
    ystep_cou = 0 ;	
	break_flag = 0;
	nopmove_flag = 0;
	move_flag = 0;
	cut_flag = 0;
	stop_flag = 0;
	origin2_lastmove_flag = 0;
	SewingStopFlag = 0;
	RotateFlag = 0;
	FootRotateFlag = 0;
	StopStatusFlag = 0;
	end_flag = 0;
	atum_flag = 0;
	hevi_flag = 0;
	fun_flag = 0;	
	fun_code = 0;
	inpress_high_flag = 0;
	while(1)
	{
    	switch(pat_point->func)
    	{
			case 0x1f://1f 00 00 00
					end_flag = 1;  
					break_flag = 1; 
				break;
			case 0x02://02 00 00 00
					cut_flag = 1;   
					break_flag = 1; 
				break;  
			case 0x04://04 00 00 00  USTP
					stop_flag = 1;  
					stop_number = 1;										
					break_flag = 1; 
				break;
			case 0x0d:
					fun_code = 4;
					fun_flag = 1;
					T_HALF = (pat_point->xstep == 1)? 1 : 0;
					fun_default_flag =1;
				break;   
			case 0x0c:
					fun_code = 3;
					fun_flag = 1;
					FK_OFF = (pat_point->xstep == 1)? 1 : 0;
					fun_default_flag =1;
				break;  
			case 0x0b:
					fun_code = 2;
					fun_flag = 1;
					FR_ON = (pat_point->xstep == 1)? 1 : 0;
					fun_default_flag =1;
				break;
			case 0x05:
					fun_code = 1;
					fun_flag = 1;
					T_DIR_EXTEND = (pat_point->xstep == 1)? 1 : 0;
					fun_default_flag =1;
				break;

			case 0x14://14 00 00 00 DSTP
					stop_flag = 1;  
					stop_number = 2;											
					break_flag = 1; 
				break;	
			case 0x06://06 00 00 00 
					origin2_lastmove_flag = 1;
					break_flag = 1;
					LastPatternHaveSOP = 1;
					origin2_point = pat_point ;
				break;	
			case 0x07://07 00 00 00	
					RotateFlag = 1;
					break_flag = 1;
				break;
			case 0x1d:
					switch(pat_point->para)
					{
						case 0://FUN5
							fun_code = 5;
							fun_flag = 1;
							fun_default_flag =1;
							T_DIR = (pat_point->xstep == 1)? 1 : 0;
						break;
						case 1://FUN6
							fun_code = 6;
							fun_flag = 1;
							fun_default_flag =1;
							T_CLK = (pat_point->xstep == 1)? 1 : 0;
						break;
						case 2://FUN7
							 inpress_delta = ((INT16)(pat_point)->xstep);
							 inpress_high = inpress_high_base + inpress_delta;						 
							 
				    		 if(inpress_high > 80)
								inpress_high = 80;
							 if(inpress_high < 0)
								inpress_high = 0;
				    		 inpress_high_flag = 1; 
							 break_flag = 1;
						break;
						case 3://FUN8
							FootRotateFlag = 1;
							making_pen_status = pat_point->xstep;
							break_flag = 1;
						break;
						case 9://
						if( k03 == ELECTRICAL )
						{
							temp16 = (INT16)pat_point->ystep<<8;
							temp16 = temp16 | (INT16)pat_point->xstep;
							sewing_tenion = temp16 + base_tension;
							if( sewing_tenion >255)
							    sewing_tenion = 255;
							temp_tension = sewing_tenion;   
							if( motor.spd_obj >0 )    
	      			 			at_solenoid();	
						}
						break;
					}	
					//break_flag = 1;	
					break;
			case 0x1c:
				switch(pat_point->para)
				{
					case 0://1c 00 00 00 
						nop_move_k = 9-pat_point->xstep;
						lastmove_flag = 1;
						nopmove_flag = 1; 
					break;
					case 1://1c 01 00 00
						atum_flag = 1;
						atum_data = pat_point->xstep;
					break;
					case 2://1c 02 00 00 
						hevi_flag = 1;
						hevi_data = pat_point->xstep;
					break;
					case 3://1c 03 00 00 
						SewingStopValue = pat_point->xstep;
						SewingStopFlag = 1;
					break;
					case 5://1c 05 00 00 //旋转机头角度控制
						
						temp16 = (UINT8)pat_point->ystep;
						temp16 = temp16 <<8;
					  	temp16 = temp16 + (UINT8)pat_point->xstep;
					  	rotated_abs_angle =temp16;
						rotated_function_flag = 1;
					break;
				}	
				break_flag = 1;
		    break;				
	   		case 0x1b: lastmove_flag = 1;		
      		case 0x03:       	 
				xstep_cou = ChangeX(pat_point);
				ystep_cou = ChangeY(pat_point);
    		    nopmove_flag = 1; 
    		    break_flag = 1;
    		    break;    					
    		//-----------------------------------------------------------------------	
      		// normal stitch
      		//-----------------------------------------------------------------------
			case 0x61:  //
    		case 0x41: 	//
      		case 0x21:  //
      		case 0x01:  //
				xstep_cou = ChangeX(pat_point);
				ystep_cou = ChangeY(pat_point);
    		    move_flag = 1; 
    		    break_flag = 1;
				if (pat_point->func == 0x61)
				    PatternSpeedLimited = u217;
				else if (pat_point->func == 0x41)
				    PatternSpeedLimited = u219;
				else if (pat_point->func == 0x21)
				    PatternSpeedLimited = u220;
				else if (pat_point->func == 0x01)
				    PatternSpeedLimited = u218;    
    		    break;    					
    		default:   
				break;
    	}
		do_pat_point_add_one();  
    	if(break_flag == 1)
    	{
    		break;
    	}	 
  	}                           
}
//--------------------------------------------------------------------------------------
//  Name:		process_edit_data
//  Parameters:	None
//  Returns:	None
//  Description: process pattern data
//--------------------------------------------------------------------------------------
void process_edit_data(void)
{
	UINT8 break_flag;
	INT16 temp16;
	xstep_cou = 0 ;
    ystep_cou = 0 ;	
	break_flag = 0;	
	nopmove_flag = 0;
	move_flag = 0;
	cut_flag = 0;
	stop_flag = 0;
	origin2_lastmove_flag = 0;
	SewingStopFlag = 0;
	RotateFlag = 0;
	FootRotateFlag = 0;
	end_flag = 0;
	atum_flag = 0;
	hevi_flag = 0;
	fun_flag = 0;	
	inpress_high_flag = 0; 
	stop_number = 0;
	while(1)
	{
    		first_stitch_flag = 0;
			switch(pat_point->func)
    		{
	    		//-----------------------------------------------------------------------	
	      		// fuction code
	      		//-----------------------------------------------------------------------
				case 0x1f://1f 00 00 00
						end_flag = 1; 
						break_flag = 1; 
					break;
				case 0x02://02 00 00 00 
						cut_flag = 1;  
						break_flag = 1; 
					break;  
				case 0x04://04 00 00 00
						stop_flag = 1;  
						stop_number = 1;
						break_flag = 1; 
					break;
			    
			    case 0x0d:
						fun_code = 4;
						fun_flag = 1;
						T_HALF = (pat_point->xstep == 1)? 1 : 0;
						fun_default_flag =1;
					break;   
			    case 0x0c:
						fun_code = 3;
						fun_flag = 1;
						FK_OFF = (pat_point->xstep == 1)? 1 : 0;
						fun_default_flag =1;
					break;  
			    case 0x0b:
						fun_code = 2;
						fun_flag = 1;
						FR_ON = (pat_point->xstep == 1)? 1 : 0;
						fun_default_flag =1;
					break;  
				case 0x05:
						fun_code = 1;
						fun_flag = 1;
						T_DIR_EXTEND = (pat_point->xstep == 1)? 1 : 0;
						fun_default_flag =1;
						break;
				
				case 0x14://14 00 00 00 
						stop_flag = 1;  
						stop_number = 2;
						break_flag = 1; 
					break;	
				case 0x06://06 00 00 00 
						origin2_lastmove_flag = 1;
						break_flag = 1;
					break;	
				case 0x07://07 00 00 00	 
						RotateFlag = 1;
						break_flag = 1;
					break;
				case 0x1d:
					switch(pat_point->para)
					{
						case 0://FUN5
							fun_code = 5;
							fun_flag = 1;
							T_DIR = (pat_point->xstep == 1)? 1 : 0;
							fun_default_flag =1;
						break;
						case 1://FUN6
							fun_code = 6;
							fun_flag = 1;
							T_CLK = (pat_point->xstep == 1)? 1 : 0; 
							fun_default_flag =1;
						break;
						case 2://FUN7
							 inpress_delta = ((INT16)(pat_point)->xstep);
							 inpress_high = inpress_high_base + inpress_delta;
				    		 if(inpress_high > 80)
								inpress_high = 80;
							 if(inpress_high < 0)
								inpress_high = 0;
				    		 inpress_high_flag = 1; 
							break_flag = 1;
						break;
						case 3://1d 03 00 00 
							FootRotateFlag = 1;
							making_pen_status = pat_point->xstep;
							break_flag = 1;
						break;
						case 6:
						    break_flag = 1;
						break;
						case 9://
						if( k03 == ELECTRICAL )
						{
							temp16 = (INT16)pat_point->ystep<<8;
							temp16 = temp16 | (INT16)pat_point->xstep;
							sewing_tenion = temp16 + base_tension;
							if( sewing_tenion >255)
							    sewing_tenion = 255;
							temp_tension = sewing_tenion;       
							if( motor.spd_obj >0 )
	      			 			at_solenoid();	
						}	
						break;
					}	
					
					break;
				case 0x1c:
					switch(pat_point->para)
					{
						case 0://1c 00 00 00 
							nop_move_k = 9-pat_point->xstep;
							lastmove_flag = 1;
							nopmove_flag = 1; 
						break;
						case 1://1c 01 00 00
							atum_flag = 1;
							atum_data = pat_point->xstep;
						break;
						case 2://1c 02 00 00 
							hevi_flag = 1;
							hevi_data = pat_point->xstep;
						break;
						case 3://1c 03 00 00 
							SewingStopValue = pat_point->xstep;
							SewingStopFlag = 1;
						break;
					}
					break_flag = 1;
			    break;				
		   		case 0x1b: lastmove_flag = 1;	//	
		   		/* no break */
	      		case 0x03:       	 
					xstep_cou = ChangeX(pat_point);
					ystep_cou = ChangeY(pat_point);
	    		    nopmove_flag = 1; 
	    		    break_flag = 1;
	    		    break;    					
	    		//-----------------------------------------------------------------------	
	      		// normal stitch
	      		//-----------------------------------------------------------------------
				case 0x61:  //(H)
	    		case 0x41: 	//(MD1)
	      		case 0x21:  //(MD2)
	      		case 0x01:  //(L)
					xstep_cou = ChangeX(pat_point);
					ystep_cou = ChangeY(pat_point);
					if( (xstep_cou != 0)||(ystep_cou != 0) )
					{
	    		    	move_flag = 1; 
	    		    	break_flag = 1;
					}
	    		    break;    					
	    		//-----------------------------------------------------------------------	
	      		// default
	      		//-----------------------------------------------------------------------		   		  	  				
	    		default:   
					break;
	    	}
//		pat_point++;
        do_pat_point_add_one();
    	if(break_flag == 1)
    	{
    		break;
    	}	 
  	}                           
}

UINT32 Calculate_QuickMove_Time(UINT16 temp16_x,UINT16 temp16_y)
{
	UINT32 temp32,temp16_max,delta,kk,bb;
	UINT32 sw_value;
	
	if( sys.status == READY )
	    sw_value = delay_of_go_setout % 10;
	else if( (sys.status == SETOUT)||(sys.status == FINISH))
		sw_value = delay_of_go_setout % 10;	
	else
		sw_value = delay_of_nop_move % 10;
	
		if( making_pen_actoin_flag == 1)//记号笔空送速度
		    sw_value = marking_speed-1;
	
	if( temp16_x > temp16_y )
		temp16_max = temp16_x;
	else
		temp16_max = temp16_y;
			
	if( temp16_max <= 255)
		return 95;
		
	 
	if( temp16_max <=1300 )//65mm 1300
	   	kk = 125;
	else if( temp16_max <=1600 )//80mm 1600
	   	kk = 135;
	else if( temp16_max <=2300 )//110mm 2300
	   	kk = 145;
	else if( temp16_max <=2400 )//150mm 2400
	   	kk = 150;
	else if( temp16_max <=3000 )//150mm 3000
	   	kk = 160;
	else if( temp16_max <=4200 )//210mm 4200
		kk = 170;
	else if( temp16_max <=5000 )//250mm 5000
		kk = 173;
	else if( temp16_max <=6000 )//300mm 6000
		kk = 175;
	else if( temp16_max <=7000 )//350mm 7000
		kk = 185;
	else if( temp16_max <=8000 )//400mm 8000
		kk = 190;
	else if( temp16_max <=9000 )//450mm 9000
		kk = 200;
	else if( temp16_max <=10000 )//500mm 10000
		kk = 205;
	else if( temp16_max <=11000 )//550mm 11000
		kk = 208;
	else if( temp16_max <=12000 )//600mm 12000
		kk = 210;
	else 
		kk = 215;	 
	
	bb = sw_value;
	if(bb < 1)
	   bb = 1;
	temp32 =(UINT32)temp16_max *10/kk + bb*50+150;

	return temp32;

}
/**
    试缝过程中，从当前位置回退到空送开始位置
*/
#if ENABLE_JUMP_NOPMOVE
void back_endpoint(void)
{	
	UINT16 temp16_x,temp16_y,temp16_max;	
	UINT16 i;	
	INT16 tempx_step,tempy_step,tmpx,tmpy;
	UINT16 quick_time;	
	PATTERN_DATA *TempPatpoint;
	INT16 allx_temp_step,ally_temp_step;
	
	move_flag = 0;
	lastmove_flag = 0;	
	tempx_step = 0;
	tempy_step = 0;
	stop_flag = 0;
	origin2_lastmove_flag = 0;
	quick_time = 60;
	
	TempPatpoint = pat_point;
	allx_temp_step = allx_step;
	ally_temp_step = ally_step;
	bakeup_total_counter = pat_buff_total_counter;
    //进入的时候已经有一针空送
	if(nopmove_flag == 1)
	{
	    tempx_step = tempx_step + xstep_cou;
	    tempy_step = tempy_step + ystep_cou;      
	    allx_step = allx_step - xstep_cou;
	    ally_step = ally_step - ystep_cou;   
	}
	while(1)
	{				
	  	conprocess_data();  //先减再判断    
	    if(nopmove_flag == 1)
	    {
    		tempx_step = tempx_step + xstep_cou;//统计空送坐标
      		tempy_step = tempy_step + ystep_cou;      
      		allx_step = allx_step - xstep_cou;
      		ally_step = ally_step - ystep_cou;                             
      		nopmove_flag = 0;
			SewTestStitchCounter--; //空送也要统计试缝针数信息
			if(lastmove_flag == 1)
      		{ 
			   break;
			}
    	}
    	else
    	{
			if(start_flag == 1)
			   single_flag = 0;
			else
		 	   do_pat_point_add_one();  	
    		lastmove_flag = 0;
			origin2_lastmove_flag = 0;
    		break;
    	}	
    	rec_com();    // communication with panel                                       	        
  	}     
    temp16_y = fabsm(tempy_step);
	temp16_x = fabsm(tempx_step);
	
	if(temp16_x > temp16_y)
  	{
  		temp16_max = temp16_x;
  	}
  	else
  	{
  		temp16_max = temp16_y;
  	}
	

    if(temp16_max >0)
	{
		quick_time = Calculate_QuickMove_Time(temp16_x,temp16_y);
	}
	//--------------------------------------------------------------------------------------
  	// y go start point
  	//--------------------------------------------------------------------------------------
	if(temp16_y >0)
	{
    	y_quickmove(quick_time,-tempy_step);
	}
  	delay_ms(1);
  	//--------------------------------------------------------------------------------------
  	// x go start point
  	//--------------------------------------------------------------------------------------
  	if( temp16_x > 0)
  	{
		 x_quickmove(quick_time,-tempx_step);
  	}
  	//--------------------------------------------------------------------------------------
  	// delay
  	//-------------------------------------------------------------------------------------- 

		delay_ms(quick_time);
		for(i=0;i<quick_time;i++)
		{
			delay_ms(1);
			if( check_motion_done() )
			{
			   delay_ms(50);
			   break;
			}
		}
	  
}
#endif
//--------------------------------------------------------------------------------------
//  Name:		go_beginpoint
//  Parameters:	None
//  Returns:	None
//  Description: x motor and y motor go beginpoint
//--------------------------------------------------------------------------------------
void go_beginpoint(UINT8 FirstNopmoveFlag)
{	
	UINT16 temp16_x,temp16_y,temp16_max;	
	UINT32 i;
	INT16 allx_temp_step,ally_temp_step;	
	INT16 tempx_step,tempy_step,add_x,add_y; 
	UINT16 quick_time ,j,tmpx,tmpy;	
	PATTERN_DATA *TempPatpoint;
	UINT8 fast_flag,temp8;
	
	fast_flag =0;
	move_flag = 0;
	nopmove_flag = 0;
	lastmove_flag = 0;	
	tempx_step = 0;
	tempy_step = 0;
	quick_time = 60;
	//保留空送前的坐标、指针、总针数
	TempPatpoint = pat_point;
	allx_temp_step = allx_step;
	ally_temp_step = ally_step;
	bakeup_total_counter = pat_buff_total_counter;
	add_x = 0;
	add_y = 0;
	
	if(making_pen_actoin_flag == 1)
	{
		if( making_pen_status == 1 )
			PEN_SIGNAL = 0;
		else if( making_pen_status == 4 )
			LASER_SIGNAL = 0;
	}
	temp8 = detect_position();	
	if(temp8 == OUT)     
	{	
		find_dead_center();		
	}	
	go_origin_zx();
	if( (sys.error != 0)&&(stay_flag==0) )
	{
		sys.status = ERROR;
		StatusChangeLatch = ERROR;
		return;
	} 
	//--------------------------------------------------------------------------------------
  	//  process pattern data
  	//--------------------------------------------------------------------------------------
	while(1)
	{				
  		process_data();           
    	if(nopmove_flag == 1)
    	{	  
    		tempx_step = tempx_step + xstep_cou;
      		tempy_step = tempy_step + ystep_cou;      
      		allx_step = allx_step + xstep_cou;
      		ally_step = ally_step + ystep_cou; 
			              
			if(allx_step < -RESOLUTION*u213 || allx_step > RESOLUTION*u214 || ally_step < -RESOLUTION*u216 || ally_step > RESOLUTION*u215)
			{
				allx_step = allx_temp_step;
				ally_step = ally_temp_step;
				pat_point = TempPatpoint;
				OutOfRange_flag = 1;
				return;
			}
			  
      		if(lastmove_flag == 1)
      		{  
				//目标位置记录下来
				target_pat_point = pat_point;
    			target_allx_step = allx_step;
				target_ally_step = ally_step;
				target_total_counter = pat_buff_total_counter;
				
				
				if( (sewingcontrol_flag == 2)&&(need_backward_sewing == 1) &&(making_pen_actoin_flag == 0) )
				{
					process_data();
					if(origin2_lastmove_flag == 1) 
					{
						origin2_lastmove_flag = 0;
						process_data();
						do_pat_point_sub_one();
					}
					else
					   do_pat_point_sub_one();
					if( move_flag == 1)
					{
					    if( sewingcontrol_stitchs !=0 )
						   need_action_once = 1;
						if( sewingcontrol_stitchs > 0 )
						{
							i = 0;
							need_action_once = 1;
							while(i<sewingcontrol_stitchs)
							{
								move_flag = 0;
								process_data();
								if( move_flag == 1 )
								{
									  i++;
									  tempx_step = tempx_step + xstep_cou;
						      		  tempy_step = tempy_step + ystep_cou;      
									  allx_step = allx_step + xstep_cou;
		      						  ally_step = ally_step + ystep_cou;   
									  add_x += xstep_cou;
									  add_y += ystep_cou;            
									  move_flag = 0;
								}
								//if( (end_flag==1)||(nopmove_flag==1) )
								//  break;
							}
						
						}
					}
				}
			
				 	  
    	  		nopmove_flag = 0;
    	  		lastmove_flag = 0;
				process_data();  
				do_pat_point_sub_one();    
				if(end_flag ==1)
				  fast_flag =1;
    	  		break;   	    	    	
      		}       
      		nopmove_flag = 0;
    	}
    	else
    	{
			do_pat_point_sub_one();   	    	
    		break;   	
    	}	
    	rec_com();                                       	        
  	}     
	 if( (fast_flag == 1) &&(LastPatternHaveSOP == 1) )        
	 {
		 allx_step = allx_temp_step;
		 ally_step = ally_temp_step;
		 return;
	 }
  	//--------------------------------------------------------------------------------------
  	// y go start point
  	//--------------------------------------------------------------------------------------
	temp16_y = fabsm(tempy_step);
	temp16_x = fabsm(tempx_step);
	
	if(temp16_x > temp16_y)
  	{
  		temp16_max = temp16_x;
  	}
  	else
  	{
  		temp16_max = temp16_y;
  	}
	
	if(temp16_max > 0)
	{
		quick_time = Calculate_QuickMove_Time(temp16_x,temp16_y);
	}
	
  	//--------------------------------------------------------------------------------------
  	// y go start point
  	//--------------------------------------------------------------------------------------
  	if(temp16_y > 0)
  	{	
		y_quickmove(quick_time,tempy_step);	
 	}
  	delay_ms(1);
  	//--------------------------------------------------------------------------------------
  	// x go start point
  	//--------------------------------------------------------------------------------------
  	if(temp16_x > 0)
  	{
		x_quickmove(quick_time,tempx_step);
  	}
  	//--------------------------------------------------------------------------------------
  	// delay
  	//-------------------------------------------------------------------------------------- 

	  if( quick_time < 98)
	      quick_time = 98;	 
	  for(i=0;i<quick_time - 98;i++)//58
	  {
	     delay_ms(1);
		 #if NOPMOVE_STOP_ENABLE
		 //if(sys.status == RUN)
		 {
			 if(PAUSE == PAUSE_ON)   
		     {
			     delay_ms(2);
			     if(PAUSE == PAUSE_ON)
			     {				
					 nop_move_pause_flag = 1; 					 
				     nop_move_emergency(temp16_x,temp16_y); //STOP MOVING
					 for(j=0;j<2000;j++)
					 {
						delay_ms(1);
						if( check_motion_done() )
						   break;
				  	 }
				     if( j == 2000)
					 {
					 	   sys.error = ERROR_70;//ERROR_15;
						   special_go_allmotor_flag = 1;
						   sys.status = ERROR;
						   return;
					 }
					 tmpx = 0;
					 tmpy = 0;
					 if( temp16_x > QUICKMOVE_JUDGEMENT )
					     tmpx = get_x_distance();	//回读的值以0.1mm为单位，并且全是正值				 
					 else if( temp16_x != 0)
					     tmpx = temp16_x;
					 else
					 	 tempx_step = 0;
						 
					 delay_ms(10);
					 
					 if( temp16_y > QUICKMOVE_JUDGEMENT  )
					     tmpy = get_y_distance();
					 else if( temp16_y != 0 )
					     tmpy = temp16_y;					 
					 else
						 tempy_step = 0;
										 
					 if( tmpx == STEPPER_IGNORE_STOP )
					     tmpx = temp16_x;					 
					 if( tmpy == STEPPER_IGNORE_STOP )
					     tmpy = temp16_y;
					   
					 read_step_x = tmpx;
					 read_step_y = tmpy;
					 
					 last_pattern_point = TempPatpoint;
					 last_allx_step = allx_temp_step;
					 last_ally_step = ally_temp_step;
					 
					 //**************************
					  if( (tmpx == 0)&&(temp16_x !=0) )//回读值为0,但是发的指令不为0,认为是没走
					  {
						  if(tmpy !=0)
						    read_step_x = temp16_x;
					  }
					  if( (tmpy == 0)&&(temp16_y !=0) )
					  {
						  if(tmpx !=0)
						    read_step_y = temp16_y;
					  }
					 					 
					 if( tempx_step >0 )//向正方向走（机针从左往右）
					 {
					  	 nop_move_remainx = tempx_step - read_step_x;
						 if( nop_move_remainx < 0)
						 {
						     sys.error = ERROR_15;
							 special_go_allmotor_flag = 1;
						 }
					 }
					 else if ( tempx_step < 0)
					 {
					     nop_move_remainx = tempx_step + read_step_x;
						 if( nop_move_remainx > 0)
						 {
						    sys.error = ERROR_15;
							special_go_allmotor_flag = 1;
						  }
						 read_step_x = -read_step_x;
					 }
					 else
					 {
						 nop_move_remainx = 0;
						 read_step_x = 0;
					 }
					      
					 if( tempy_step >0 )	 
					 {
					     nop_move_remainy = tempy_step - read_step_y;
						 if( nop_move_remainy <0 )
						 {
						   sys.error = ERROR_15;
						   special_go_allmotor_flag = 1;
						 }
					 }
					 else if( tempy_step <0 )
					 {
					     nop_move_remainy = tempy_step + read_step_y;
						 if( nop_move_remainy >0 )
						 {
						   sys.error = ERROR_15;
						   special_go_allmotor_flag = 1;
						 }
						 read_step_y = -read_step_y;
					 }
					 else
					 {
						 nop_move_remainy = 0;
						 read_step_y = 0;
					 }
					 //**************************
					 if( (sewingcontrol_flag == 2)&&(need_backward_sewing == 1) )
					 {
						 /*
						 tempx_step -= add_x;
						 tempy_step -= add_y;
						 if( sewingcontrol_stitchs >0 )//空送急停后，再启动不加固了。
						    need_action_once = 0;
						 */
						 if (tempx_step != 0)
						   nop_move_remainx -= add_x;
						 if (tempy_step != 0)
						   nop_move_remainy -= add_y;
						 if( sewingcontrol_stitchs >0 )//空送急停后，再启动不加固了。
						    need_action_once = 0;
					 }
					 break;
		  	     }
			 }
		 }
		#endif
	  }
	  delay_ms(100);
 	  
	  if( nop_move_pause_flag == 0)
		{
			for(i=0;i<quick_time;i++)
			{
				delay_ms(1);
				if( check_motion_done() )
				{
				   //delay_ms(50);
				   break;
				}
			}
		}
	brkdt_flag = 0;
	thbrk_count = 0;
	thbrk_flag = 0;
}

//--------------------------------------------------------------------------------------
//  Name:		conprocess_data
//  Parameters:	None
//  Returns:	None
//  Description: process pattern data
//--------------------------------------------------------------------------------------
void conprocess_data(void)
{
	UINT8 break_flag;
	INT16 temp16;
	xstep_cou = 0 ;
    ystep_cou = 0 ;	
	break_flag = 0;	
	start_flag = 0;
	nopmove_flag = 0;
	move_flag = 0;
	cut_flag = 0;
	stop_flag = 0;
	origin2_lastmove_flag = 0;
	SewingStopFlag = 0;
	RotateFlag = 0;
	FootRotateFlag = 0;
	end_flag = 0;
	atum_flag = 0;
	hevi_flag = 0;
	fun_flag = 0;	
	StopStatusFlag = 0;
	
	inpress_high_flag = 0; 
	while(1)
	{
		if( (pat_point == sta_point)&&(super_pattern_flag != 1) )
		{
			start_flag = 1;
			break;
		}
		do_pat_point_sub_one();
		if( start_flag == 1)
			break;
   		switch(pat_point->func)
    	{
	    		//-----------------------------------------------------------------------	
	      		// fuction code
	      		//-----------------------------------------------------------------------
				case 0x1f://1f 00 00 00
						end_flag = 1;   
						break_flag = 1; 
					break;
				case 0x02://02 00 00 00
						cut_flag = 1;   
						break_flag = 1; 
					break;  
				case 0x04://04 00 00 00 
						stop_flag = 1;  
						stop_number = 1;										
						break_flag = 1; 
					break;
				case 0x0d:
						fun_code = 4;
						fun_flag = 1;
						T_HALF = (pat_point->xstep == 1)? 1 : 0;
						fun_default_flag =1;
				break; 
				case 0x0c:
						fun_code = 3;
						fun_flag = 1;
						FK_OFF = (pat_point->xstep == 1)? 1 : 0;
						fun_default_flag =1;
				break;  			
				case 0x0b:
						fun_code = 2;
						fun_flag = 1;
						FR_ON = (pat_point->xstep == 1)? 1 : 0;
						fun_default_flag =1;
				break;
				case 0x05:
						fun_code = 1;
						fun_flag = 1;
						T_DIR_EXTEND = (pat_point->xstep == 1)? 1 : 0;
						fun_default_flag =1;
				break;
				case 0x14://14 00 00 00 
						stop_flag = 1;  
						stop_number = 2;											
						break_flag = 1; 
					break;	
				case 0x06://06 00 00 00 
						origin2_lastmove_flag = 1;
						break_flag = 1;
					break;	
				case 0x07://07 00 00 00	
						RotateFlag = 1;
						break_flag = 1;
					break;
			case 0x1d:
					switch(pat_point->para)
					{
						case 0://FUN5
							fun_code = 5;
							fun_flag = 1;
							T_DIR = (pat_point->xstep == 1)? 1 : 0;
							fun_default_flag =1;
						break;
						case 1://FUN6
							fun_code = 6;
							fun_flag = 1;
							T_CLK = (pat_point->xstep == 1)? 1 : 0;
							fun_default_flag =1;
						break;
						case 2://FUN7
							//fun_code = 7;
							//fun_flag = 1;
							 inpress_delta = ((INT16)(pat_point)->xstep);
							 inpress_high = inpress_high_base + inpress_delta;
				    		 if(inpress_high > 80)
								inpress_high = 80;
							 if(inpress_high < 0)
								inpress_high = 0;
				    		 inpress_high_flag = 1; 
							break_flag = 1;
						break;
						case 3://1d 03 00 00 
							FootRotateFlag = 1;
							making_pen_status = pat_point->xstep;
							break_flag = 1;
						break;
						case 6:
						    break_flag = 1;
						break;
						case 9://
						if( k03 == ELECTRICAL )
						{
							temp16 = (INT16)pat_point->ystep<<8;
							temp16 = temp16 | (INT16)pat_point->xstep;
							sewing_tenion = temp16 + base_tension;
							if( sewing_tenion >255)
							    sewing_tenion = 255;
							temp_tension = sewing_tenion; 
							if( motor.spd_obj >0 )       
	      			 			at_solenoid();	
						}
						break;
					}	
					//break_flag = 1;	
					break;
			case 0x1c:
					switch(pat_point->para)
					{
						case 0://1c 00 00 00 
							nop_move_k = 9-pat_point->xstep;
							lastmove_flag = 1;
							nopmove_flag = 1; 
						break;
						case 1://1c 01 00 00
							atum_flag = 1;
							atum_data = pat_point->xstep;
						break;
						case 2://1c 02 00 00 
							hevi_flag = 1;
							hevi_data = pat_point->xstep;
						break;
						case 3://1c 03 00 00 
							SewingStopValue = pat_point->xstep;
							SewingStopFlag = 1;
						break;
					}
					break_flag = 1;	
			    break;				
	   		case 0x1b: lastmove_flag = 1;	//	
	   		case 0x03:       	 
				xstep_cou = ChangeX(pat_point);
				ystep_cou = ChangeY(pat_point);
    		    nopmove_flag = 1; 
    		    break_flag = 1;
    		    break;    					
    		//-----------------------------------------------------------------------	
      		// normal stitch
      		//-----------------------------------------------------------------------
			case 0x61:  //
    		case 0x41: 	//
      		case 0x21:  //
      		case 0x01:  //
				xstep_cou = ChangeX(pat_point);
				ystep_cou = ChangeY(pat_point);
    		    move_flag = 1; 
    		    break_flag = 1;
				if (pat_point->func == 0x61)
				  PatternSpeedLimited = u217;
				else if (pat_point->func == 0x41)
				  PatternSpeedLimited = u219;
				else if (pat_point->func == 0x21)
				  PatternSpeedLimited = u220;
				else if (pat_point->func == 0x01)
				  PatternSpeedLimited = u218;
    		    break;    					
    		//-----------------------------------------------------------------------	
      		// default
      		//-----------------------------------------------------------------------		   		  	  				
    		default:   
				break;
    	}     
    	if(break_flag == 1)
    	{
    		break;
    	}	 
  	}                           
}
//--------------------------------------------------------------------------------------
//  Name:		conprocess_edit_data
//  Parameters:	None
//  Returns:	None
//  Description: process pattern data
//--------------------------------------------------------------------------------------
void conprocess_edit_data(void)
{
	UINT8 break_flag;
	INT16 temp16;
	xstep_cou = 0 ;
    ystep_cou = 0 ;		
	break_flag = 0;	
	start_flag = 0;
	first_stitch_flag = 0;
	nopmove_flag = 0;
	move_flag = 0;
	cut_flag = 0;
	stop_flag = 0;
	origin2_lastmove_flag = 0;
	SewingStopFlag = 0;
	RotateFlag = 0;
	FootRotateFlag = 0;
	end_flag = 0;
	atum_flag = 0;
	hevi_flag = 0;
	fun_flag = 0;
	inpress_high_flag = 0; 	
	while(1)
	{
		if( (pat_point == (PATTERN_DATA *)(pat_buf))&&(super_pattern_flag != 1) )
		{
			break_flag = 1;
			first_stitch_flag = 1;
			break;
		}
		//else
		{	
			//pat_point--;
			do_pat_point_sub_one();
			switch(pat_point->func)
      		{
				
				case 0x1f://1f 00 00 00
						end_flag = 1;   
						break_flag = 1; 
					break;
				case 0x02://02 00 00 00 
						cut_flag = 1;  
						break_flag = 1; 
					break;  
				case 0x04://04 00 00 00 
						stop_flag = 1;  
						stop_number = 1;
						break_flag = 1; 
					break;
				case 0x0d:
					fun_code = 4;
					fun_flag = 1;
					//FR_ON = (pat_point->xstep == 1)? 1 : 0;
					T_HALF = (pat_point->xstep == 1)? 1 : 0;
					fun_default_flag =1;
				break;   //2012-8-6  ADD FOR SC039
				case 0x0c:
						fun_code = 3;
						fun_flag = 1;
						FK_OFF = (pat_point->xstep == 1)? 1 : 0;
						fun_default_flag =1;
					break;   //2012-8-6  ADD FOR SC039
			
				case 0x0b:
						fun_code = 2;
						fun_flag = 1;
						FR_ON = (pat_point->xstep == 1)? 1 : 0;
						fun_default_flag =1;
					break;
				case 0x05:
						fun_code = 1;
						fun_flag = 1;
						T_DIR_EXTEND = (pat_point->xstep == 1)? 1 : 0;
						fun_default_flag =1;
					break;
				
				case 0x14://14 00 00 00 
						stop_flag = 1;  
						stop_number = 2;
						break_flag = 1; 
					break;	
				case 0x06://06 00 00 00 
						origin2_lastmove_flag = 1;
						break_flag = 1;
					break;	
				case 0x07://07 00 00 00	
						RotateFlag = 1;
						break_flag = 1;
					break;
				case 0x1d:
					switch(pat_point->para)
					{
						case 0://FUN5
							fun_code = 5;
							fun_flag = 1;
							T_DIR = (pat_point->xstep == 1)? 1 : 0; 
							fun_default_flag =1;
						break;
						case 1://FUN6
							fun_code = 6;
							fun_flag = 1;
							T_CLK = (pat_point->xstep == 1)? 1 : 0;
							fun_default_flag =1;
						break;
						case 2://FUN7
							//fun_code = 7;
							//fun_flag = 1;
							inpress_delta = ((INT16)(pat_point)->xstep);
							 inpress_high = inpress_high_base + inpress_delta;
				    		 if(inpress_high > 80)
								inpress_high = 80;
							 if(inpress_high < 0)
								inpress_high = 0;
				    		 inpress_high_flag = 1; 
							break_flag = 1;
						break;
						case 3://1d 03 00 00 
							FootRotateFlag = 1;
							making_pen_status = pat_point->xstep;
							break_flag = 1;
						break;
						case 6:
						    break_flag = 1;
						break;
						case 9://
						if( k03 == ELECTRICAL )
						{
							temp16 = (INT16)pat_point->ystep<<8;
							temp16 = temp16 | (INT16)pat_point->xstep;
							sewing_tenion = temp16 + base_tension;
							if( sewing_tenion >255)
							    sewing_tenion = 255;
							temp_tension = sewing_tenion;  
							if( motor.spd_obj >0 )      
	      			 			at_solenoid();	
						}	
						break;
					}	
					//break_flag = 1;	
					break;
				case 0x1c:
					switch(pat_point->para)
					{
						case 0://1c 00 00 00 
							nop_move_k = 9-pat_point->xstep;
							lastmove_flag = 1;
							nopmove_flag = 1; 
						break;
						case 1://1c 01 00 00
							atum_flag = 1;
							atum_data = pat_point->xstep;
						break;
						case 2://1c 02 00 00 
							hevi_flag = 1;
							hevi_data = pat_point->xstep;
						break;
						case 3://1c 03 00 00 
							SewingStopValue = pat_point->xstep;
							SewingStopFlag = 1;
						break;
					}	
					break_flag = 1;	
			    break;				
		   		case 0x1b: lastmove_flag = 1;	//	
	      		case 0x03:       	 
					xstep_cou = ChangeX(pat_point);
				    ystep_cou = ChangeY(pat_point);
	    		    nopmove_flag = 1; 
	    		    break_flag = 1;
	    		    break;    					
	    		//-----------------------------------------------------------------------	
	      		// normal stitch
	      		//-----------------------------------------------------------------------
				case 0x61:  //(H)
	    		case 0x41: 	//(MD1)
	      		case 0x21:  //(MD2)
	      		case 0x01:  //(L)
					xstep_cou = ChangeX(pat_point);
				    ystep_cou = ChangeY(pat_point);
	    		    move_flag = 1; 
	    		    break_flag = 1;
					if (pat_point->func == 0x61)
					  PatternSpeedLimited = u217;
					else if (pat_point->func == 0x41)
					  PatternSpeedLimited = u219;
					else if (pat_point->func == 0x21)
					  PatternSpeedLimited = u220;
					else if (pat_point->func == 0x01)
					  PatternSpeedLimited = u218; 
	    		    break;    					
	    		//-----------------------------------------------------------------------	
	      		// default
	      		//-----------------------------------------------------------------------		   		  	  				
	    		default:   
					break;
      		}
			
    	}     
    	if(break_flag == 1)
    	{
    		break;
    	}	 
  	}                           
}

//--------------------------------------------------------------------------------------
//  Name:		single_next
//  Parameters:	None
//  Returns:	None
//  Description: single next step
//--------------------------------------------------------------------------------------
void single_next(void)
{
	UINT8 i;
	if(check_footer_status())
	{	
			if(already_in_origin ==0) 
			{
				single_flag = 1; 	
	    		single_reply = 0x51; 
				return;
			}
			process_data(); 
			
			if(end_flag == 1)
	    	{   
	    		single_flag = 0; 	
	    		single_reply = 0x51;
	    		end_flag = 0;  
				do_pat_point_sub_one();
	    		return;
	    	}
			if( (inpress_high_flag ==1)&&(inpress_follow_flag == 1) )
			{
					last_inpress_position = inpress_high;
					inpress_to(inpress_high);
					inpress_high_flag = 0;  
			}

			if( (move_flag == 1)||(nopmove_flag == 1)||(FootRotateFlag == 1))
	    	{	
				single_flag = 1;
	      		single_reply = 0x01;
	    	}
			if((cut_flag ==1)||(origin2_lastmove_flag == 1)||(	SewingStopFlag == 1)||( RotateFlag	== 1)||(stop_flag == 1))
			{
				single_flag = 0; 	
	    		single_reply = 0x51; 
				cut_flag = 0;
				stop_flag = 0;
				origin2_lastmove_flag = 0;
				SewingStopFlag = 0;
				RotateFlag = 0;
			} 
		}
		else
		{
			single_flag = 0; 	
	    	single_reply = 0x51;
		}
}


//--------------------------------------------------------------------------------------
//  Name:		single_edit_continue_next
//  Parameters:	None
//  Returns:	None
//  Description: continue next step
//--------------------------------------------------------------------------------------
void single_edit_continue_next(void)
{
	UINT8 i;
	if(foot_flag == 0)
	{	
		if(sys.error == OK)
		{
			process_edit_data();
		}
		else if(sys.error == ERROR_15)
		{
			single_flag = 0; 	
  			single_reply = 0x51;     // stepper motor is not move
			return;
		}
		if( inpress_follow_flag == 1)
		{
			if(inpress_high_flag ==1)
			{
				last_inpress_position = inpress_high;
				inpress_to(inpress_high);
				inpress_high_flag = 0;  
			}
		}

		if(end_flag == 1)
    	{   
    		single_flag = 0; 	
    		single_reply = 0x51;  
			end_flag = 0;
			do_pat_point_sub_one();
    		return;
    	}        
		if( FootRotateFlag == 1)
	   {
		   process_making_pen_signal(0);
		   FootRotateFlag = 0;
		   predit_shift = 0;
		   single_flag = 0; 	
    	   single_reply = 0x51; 
		}
    	if( (move_flag == 1) || (nopmove_flag == 1) )
    	{	
	    	allx_step = allx_step + xstep_cou;
  			ally_step = ally_step + ystep_cou;
			
			if(allx_step < -RESOLUTION*u213 || allx_step > RESOLUTION*u214 || ally_step < -RESOLUTION*u216 || ally_step > RESOLUTION*u215)
			{
				do_pat_point_sub_one();
				single_flag = 0; 	
  				single_reply = 0x51;
				nopmove_flag = 0;
				move_flag = 0;
				sys.error = ERROR_15;
				StatusChangeLatch = ERROR;
				allx_step = allx_step - xstep_cou;
  				ally_step = ally_step - ystep_cou;
				return;
			}
			else
			{
				//single_flag = 6;
	      		single_reply = 0x02;
				allx_step = allx_step - xstep_cou;
  				ally_step = ally_step - ystep_cou;
			}
    	}
		if((cut_flag ==1)||(origin2_lastmove_flag == 1)||(	SewingStopFlag == 1)||( RotateFlag	== 1)||(stop_flag == 1))
		{
				cut_flag = 0;
				stop_flag = 0;
				origin2_lastmove_flag = 0;
				SewingStopFlag = 0;
				RotateFlag = 0;
		}	
  	}
  	else
  	{
  		single_flag = 0; 	
  		single_reply = 0x51;     // stepper motor is not move
  	}	
}
//--------------------------------------------------------------------------------------
//  Name:		single_back
//  Parameters:	None
//  Returns:	None
//  Description: single back step
//--------------------------------------------------------------------------------------
void single_back(void)
{	
	PATTERN_DATA *TempStart_pointTemp;

	UINT8 i;
    if(check_footer_status())
	{	
			if(already_in_origin ==0) 
			{
				single_flag = 2; 	
	    		single_reply = 0x51; 
				return;
			}
		  	conprocess_data();
			if( (inpress_high_flag ==1)&&(inpress_follow_flag == 1) )
				{
					TempStart_pointTemp = pat_point;
				
					TempStart_pointTemp--;
					while(TempStart_pointTemp > (PATTERN_DATA *)(pat_buf))
					{
						if( (TempStart_pointTemp->func == 0x1d) && (TempStart_pointTemp->para == 2) )
						{
							inpress_delta = ((INT16)((TempStart_pointTemp)->xstep));
							inpress_high = inpress_high_base + inpress_delta;
							if(inpress_high > 80)
								inpress_high = 80;
							if(inpress_high < 0)
								inpress_high = 0;
						    break;
						}
				
					TempStart_pointTemp--;
					}
					if(TempStart_pointTemp == (PATTERN_DATA *)(pat_buf))
					    inpress_high = inpress_high_base;
					
				
					last_inpress_position = inpress_high;
				    inpress_to(inpress_high);
				    inpress_high_flag = 0;  
			}
	    	if(start_flag == 1)
	    	{	 
	    		single_flag = 0; 	
	  	  		single_reply = 0x51;
	  	  		return;
	    	}
			
	    	if((move_flag == 1)||(nopmove_flag == 1)||(FootRotateFlag == 1))
	    	{	
		    	single_flag = 2;
	      		single_reply = 0x81;
	    	}
			if((cut_flag ==1)||(origin2_lastmove_flag == 1)||(	SewingStopFlag == 1)||( RotateFlag	== 1)||(stop_flag == 1))
			{
				single_flag = 0; 	
	    		single_reply = 0x51; 
				cut_flag = 0;
				stop_flag = 0;
				origin2_lastmove_flag = 0;
				SewingStopFlag = 0;
				RotateFlag = 0;
			} 
		}
		else
		{
			single_flag = 0; 	
	  		single_reply = 0x51;
		}
}
//--------------------------------------------------------------------------------------
//  Name:		single_edit_continue_back
//  Parameters:	None
//  Returns:	None
//  Description: continue back step
//--------------------------------------------------------------------------------------
void single_edit_continue_back(void)
{	
	PATTERN_DATA *TempStart_pointTemp;

	UINT8 i;
	if(foot_flag == 0)
	{	
	  	if(already_in_origin ==0) 
		{
				single_flag = 2; 	
	    		single_reply = 0x51; 
				return;
		}
	  	conprocess_data();
 		
		if(start_flag ==1)
    	{
   			single_flag = 0; 	
  	  		single_reply = 0x51;  
  	  		return;
    	}
		if( inpress_follow_flag == 1)
		{
			if(inpress_high_flag ==1)
			{
		        TempStart_pointTemp = pat_point;			
				TempStart_pointTemp--;
				while(TempStart_pointTemp > (PATTERN_DATA *)(pat_buf))
				{
					if( (TempStart_pointTemp->func == 0x1d) && (TempStart_pointTemp->para == 2) )
					{
						inpress_delta = ((INT16)((TempStart_pointTemp)->xstep));
						inpress_high = inpress_high_base + inpress_delta;
						if(inpress_high > 80)
							inpress_high = 80;
						if(inpress_high < 0)
							inpress_high = 0;
					    break;
					}
		
				TempStart_pointTemp--;
				}
				if(TempStart_pointTemp == (PATTERN_DATA *)(pat_buf))
				    inpress_high = inpress_high_base;

				last_inpress_position = inpress_high;
			    inpress_to(inpress_high);
			    inpress_high_flag = 0;  
			}	
		}
		if( FootRotateFlag == 1)
		{
			process_making_pen_signal(1);
			FootRotateFlag = 0;
			single_flag = 0; 	
  			single_reply = 0x51;
		}
    	if(nopmove_flag == 1 || move_flag == 1)
    	{	
	    	//single_flag = 7;
      		single_reply = 0x82;
    	}
		if((cut_flag ==1)||(origin2_lastmove_flag == 1)||(	SewingStopFlag == 1)||( RotateFlag	== 1)||(stop_flag == 1))
		{
			cut_flag = 0;
			stop_flag = 0;
			origin2_lastmove_flag = 0;
			SewingStopFlag = 0;
			RotateFlag = 0;
		}
  	}
  	else    
  	{
  		single_flag = 0; 	
  		single_reply = 0x51;
  	}	
}

//--------------------------------------------------------------------------------------
//  Name:		back_startpoint
//  Parameters:	None
//  Returns:	None
//  Description: back to startpoint
//--------------------------------------------------------------------------------------
void back_startpoint(void)
{
	if( (foot_flag == 0)||((foot_flag==1)&&(u202==1)) )
	{	
	  	single_flag = 3; 
	  	single_reply = 0x31;
  	}
  	else    
  	{
  		single_flag = 0; 
  		single_reply = 0x51;
  	}
}

//--------------------------------------------------------------------------------------
//  Name:		single_end
//  Parameters:	None
//  Returns:	None
//  Description: move to endpoint
//--------------------------------------------------------------------------------------
void single_end(void)
{
    if(check_footer_status())
	{
	  	single_flag = 6; 
	  	single_reply = 0x02;
  	}
  	else    
  	{
  		single_flag = 0; 
  		single_reply = 0x51;
  	}
}
//--------------------------------------------------------------------------------------
//  Name:		single_start
//  Parameters:	None
//  Returns:	None
//  Description: move to startpoint
//--------------------------------------------------------------------------------------
void single_start(void)
{
    if(check_footer_status())
	{		
		single_flag = 7; 
		single_reply = 0x82;
  	}
  	else    
  	{
  		single_flag = 0; 
  		single_reply = 0x51;
  	}
}
//--------------------------------------------------------------------------------------
//  Name:		single_stop
//  Parameters:	None
//  Returns:	None
//  Description: move stop
//--------------------------------------------------------------------------------------
void single_stop(void)
{
	if(foot_flag == 0)
	{				  
	  	single_flag = 8; 
	  	single_reply = 0x52;
  	}
  	else    
  	{
  		single_flag = 0; 
  		single_reply = 0x51;
  	}
}
//--------------------------------------------------------------------------------------
//  Name:		move_next
//  Parameters:	None
//  Returns:	None
//  Description: move next
//--------------------------------------------------------------------------------------
void move_next(void)
{
	UINT16 temp_xy_max,temp_x_length,temp_y_length;     
	                   
	allx_step = allx_step + xstep_cou;
  	ally_step = ally_step + ystep_cou;
	
	temp_x_length = fabsm(xstep_cou);
	temp_y_length = fabsm(ystep_cou);

	movestepx_time = 0;
	movestepy_time = 0;
	if(temp_x_length > 0)
	{
		
		movestepx_time = temp_x_length <<TIME_PARAMETER_1;				   
		movestep_x(xstep_cou);
		delay_us(400);
	}
	if(temp_y_length > 0)
	{
		
		movestepy_time = temp_y_length <<TIME_PARAMETER_1;				      
		movestep_y(ystep_cou);
	}
	if( movestepy_time > movestepx_time )
		movestepxy_time = movestepy_time;
	else
	    movestepxy_time = movestepx_time;

	nopmove_flag = 0;	
  	move_flag = 0; 
	delay_ms(movestepxy_time +1);
  	single_flag = 0; 
	xstep_cou = 0;	
	ystep_cou = 0;
}
//--------------------------------------------------------------------------------------
//  Name:		move_continue_next
//  Parameters:	None
//  Returns:	None
//  Description: move_continue_next
//--------------------------------------------------------------------------------------
void move_continue_next(void)
{
	UINT16 temp_xy_max,temp_x_length,temp_y_length;
		
	allx_step = allx_step + xstep_cou;
  	ally_step = ally_step + ystep_cou;
	
	temp_x_length = fabsm(xstep_cou);
	temp_y_length = fabsm(ystep_cou);
	
	movestepx_time = 0;
	movestepy_time = 0;
	if(temp_x_length > 0)
	{
		
		movestepx_time = temp_x_length <<TIME_PARAMETER_1;				   	   
		movestep_x(xstep_cou);
		delay_us(400);
	}
	if(temp_y_length > 0)
	{
		
		movestepy_time = temp_y_length <<TIME_PARAMETER_1;				   	   
		movestep_y(ystep_cou);
	}
	if( movestepx_time > movestepy_time)
	    movestepxy_time = movestepx_time;
	else
	    movestepxy_time = movestepy_time;
		
  	move_flag = 0; 
	nopmove_flag = 0;	
	xstep_cou = 0;	
	ystep_cou = 0;
}

//--------------------------------------------------------------------------------------
//  Name:		move_back
//  Parameters:	None
//  Returns:	None
//  Description: move back
//--------------------------------------------------------------------------------------
void move_back(void)
{
	UINT16 temp_xy_max,temp_x_length,temp_y_length;    
	
	allx_step = allx_step - xstep_cou;
  	ally_step = ally_step - ystep_cou;
	
	temp_x_length = fabsm(xstep_cou);
	temp_y_length = fabsm(ystep_cou);
	
	movestepx_time = 0;
	movestepy_time = 0;
	if(temp_x_length > 0)
	{
		
		movestepx_time = temp_x_length <<TIME_PARAMETER_1;				   	   
		movestep_x(-xstep_cou);
		delay_us(400);
	}
	if(temp_y_length > 0)
	{
		
		movestepy_time = temp_y_length <<TIME_PARAMETER_1;				   	   
		movestep_y(-ystep_cou);
	}
	if( movestepy_time > movestepx_time )
		movestepxy_time = movestepy_time;
	else
	    movestepxy_time = movestepx_time;

	nopmove_flag = 0;
  	move_flag = 0; 
	delay_ms(movestepxy_time+1);
  	single_flag = 0; 
	xstep_cou = 0;	
	ystep_cou = 0;
}
//--------------------------------------------------------------------------------------
//  Name:		move_continue_back
//  Parameters:	None
//  Returns:	None
//  Description:move_continue_back
//--------------------------------------------------------------------------------------
void move_continue_back(void)
{
	UINT16 temp_xy_max,temp_x_length,temp_y_length;
	
	allx_step = allx_step - xstep_cou;
  	ally_step = ally_step - ystep_cou;
	
	temp_x_length = fabsm(xstep_cou);
	temp_y_length = fabsm(ystep_cou);
	
	movestepx_time = 0;
	movestepy_time = 0;
	if(temp_x_length > 0)
	{
		
		movestepx_time = temp_x_length << TIME_PARAMETER_1;
		movestep_x(-xstep_cou);
		delay_us(400);
	}
	if(temp_y_length > 0)
	{
		
		movestepy_time = temp_y_length <<TIME_PARAMETER_1;				   	   
		movestep_y(-ystep_cou);
	}
	if( movestepx_time  > movestepy_time)
	    movestepxy_time = movestepx_time;
	else
	    movestepxy_time = movestepy_time;
		
	nopmove_flag = 0;
  	move_flag = 0; 
	xstep_cou = 0;	
	ystep_cou = 0;
	
}
//--------------------------------------------------------------------------------------
//  Name:		move_startpoint
//  Parameters:	None
//  Returns:	None
//  Description: move startpoint
//--------------------------------------------------------------------------------------
void move_startpoint(void)
{
		SewTestStitchCounter = 1;
		if(u48 == 0)
		{
			go_setoutpoint();
		}
		else if(u48 == 1)
		{
			if(start_flag == 0)
			{
				while(single_flag != 0)
				{	
					course_back();
					if(single_flag == 0)
					{
						break;
					}
				}
			}
		}
		else if(u48 == 2)
		{
			go_origin_allmotor(); 			
			go_startpoint();        
		} 

	single_flag = 0; 
}
//--------------------------------------------------------------------------------------
//  Name:		move_startpoint
//  Parameters:	None
//  Returns:	None
//  Description: move startpoint
//--------------------------------------------------------------------------------------
void move_edit_startpoint(void)
{
	first_stitch_flag = 0;
	while(first_stitch_flag == 0)
	{
		conprocess_edit_data();
		if(first_stitch_flag == 1)
		{
			break;
		}
	}
	go_origin_allmotor();          
	footer_both_down();     
	single_flag = 0; 
}
//--------------------------------------------------------------------------------------
//  Name:		course_next
//  Parameters:	None
//  Returns:	None
//  Description: course next
//--------------------------------------------------------------------------------------
void course_next(void)
{		
	UINT16 i,temp_xy_speed;
	UINT16 temp_xy_max,temp_x_length,temp_y_length;       

	movestepxy_time = 0;
	process_data();
	
	if( inpress_follow_flag == 1)
	{
		if(nopmove_flag == 1)
		{
			if(inpress_flag == 0)  
			{
				inpress_up();
				delay_ms(100);
			}	      	
		}
		if(move_flag == 1)
		{
			if(inpress_flag == 1)  
			{
				inpress_down(last_inpress_position);        	
				delay_ms(100);
			}
		}
		if(inpress_high_flag ==1)
		{
			last_inpress_position = inpress_high;
			inpress_to(inpress_high);
			inpress_high_flag = 0;  
			delay_ms(200);
		}
	}
	if(end_flag == 1)
  	{   
  		single_flag = 0; 	    	
  		end_flag = 0;
		do_pat_point_sub_one();
  		return;
  	}    
	if((nopmove_flag == 1)||(move_flag == 1))
	{
		SewingTestEndFlag = 1;
		SewTestStitchCounter++;
		allx_step = allx_step + xstep_cou;
    	ally_step = ally_step + ystep_cou;
		if(allx_step < -RESOLUTION*u213 || allx_step > RESOLUTION*u214 || ally_step < -RESOLUTION*u216 || ally_step > RESOLUTION*u215)
		{
			allx_step = allx_step - xstep_cou;
    		ally_step = ally_step - ystep_cou;
			//pat_point--;
			do_pat_point_sub_one();
			move_flag = 0;
			nopmove_flag = 0; 
			single_flag = 0; 
			sys.error = ERROR_15;
			StatusChangeLatch = ERROR;
		}
		else
		{
			temp_x_length = fabsm(xstep_cou);
	        temp_y_length = fabsm(ystep_cou);
	
	        if(temp_x_length > temp_y_length)       
	           temp_xy_max = temp_x_length;
	        else
	           temp_xy_max = temp_y_length;
	        if(temp_xy_max == 0)
	           temp_xy_max = 1;
			   
			if( temp_xy_max <11 )
			{
				movestepxy_time = temp_xy_max <<TIME_PARAMETER_1;
			}
			else
			{   
	           temp_xy_speed = spdlimit_10080_345_tab[temp_xy_max - 1];    
           	   temp_xy_speed = temp_xy_speed/100;
	           movestepxy_time = MoveTime_Speed_10080_ND80[temp_xy_speed] + single_move_speed;
			}  
			movestepx_time = 0;
			movestepy_time = 0;
			
			if(temp_x_length > 0)
	        {
			
		        movestepx_time = movestepxy_time;       
				if(movestepx_time > (temp_x_length <<TIME_PARAMETER_1) )
				{
				   movestepx_time = temp_x_length <<TIME_PARAMETER_1;				   
				}
	        	movestep_x(xstep_cou);
		        delay_us(400);
	        }
	        if(temp_y_length > 0)
	        {
				
		        movestepy_time = movestepxy_time;       
				if(movestepy_time > (temp_y_length <<TIME_PARAMETER_1) )
				{
				   movestepy_time = temp_y_length <<TIME_PARAMETER_1;
				   
				}
		        movestep_y(ystep_cou);
	        }
			if( movestepx_time  > movestepy_time)
			    movestepxy_time = movestepx_time;
			else
			    movestepxy_time = movestepy_time;
		}
	    nopmove_flag = 0; 
		move_flag = 0;
		SewingTestEndFlag = 0;
		if( nop_move_pause_flag ==1 )
		    single_flag = 0;
		predit_shift = 0;
		for(i=0;i<movestepxy_time+1;i++)
	        {
		        delay_ms(1);
		        //if(scan_pause_func(&pause_flag,READY))
		        	//return;
	        }
	}
	
	if(FootRotateFlag == 1)
	{
		process_making_pen_signal(0);
		FootRotateFlag = 0;
	}
	if((cut_flag ==1)||(origin2_lastmove_flag == 1)||(	SewingStopFlag == 1)||( RotateFlag	== 1)||(stop_flag == 1))
	{
		cut_flag = 0;
		origin2_lastmove_flag = 0;
		SewingStopFlag = 0;
		RotateFlag = 0;
		stop_flag = 0;
	} 	

}

//--------------------------------------------------------------------------------------
//  Name:		course_back
//  Parameters:	None
//  Returns:	None
//  Description: course back
//--------------------------------------------------------------------------------------
void course_back(void)
{
	UINT16 temp_xy_max,temp_x_length,temp_y_length;   
	UINT16 temp_xy_speed,i;
	PATTERN_DATA *TempStart_pointTemp;
	movestepxy_time = 0;
	conprocess_data();
  	if(start_flag == 1)
  	{
  		start_flag = 0;
  		single_flag = 0; 	
    	return;
  	}
	
	if( inpress_follow_flag == 1)
	{	
		if(nopmove_flag == 1)
		{
			if(inpress_flag == 0)  
			{
				inpress_up();
				delay_ms(100);
			}	      	
		}
		if(move_flag == 1)
		{
			if(inpress_flag == 1)  
			{
				inpress_down(last_inpress_position);        	
				delay_ms(100);
			}
		}
	
		if(inpress_high_flag ==1)
		{
		  pat_buff_total_counter_temp = pat_buff_total_counter;
		  TempStart_pointTemp = pat_point;
		  TempStart_pointTemp--;
	  
	  while(pat_buff_total_counter_temp > 0)
		{
			if( (TempStart_pointTemp->func == 0x1d) && (TempStart_pointTemp->para == 2) )
			{
				inpress_delta = ((INT16)((TempStart_pointTemp)->xstep));
				inpress_high = inpress_high_base + inpress_delta;
				if(inpress_high > 80)
					inpress_high = 80;
				if(inpress_high < 0)
					inpress_high = 0;
			    break;
			}		
		 TempStart_pointTemp--;
		}
		if(pat_buff_total_counter_temp == 0)
		    inpress_high = inpress_high_base;

		last_inpress_position = inpress_high;
	    inpress_to(inpress_high);
	    inpress_high_flag = 0;
		delay_ms(200);
	}
	}
	
	if(move_flag == 1)
	{
		SewingTestEndFlag = 1;
		allx_step = allx_step - xstep_cou;
    	ally_step = ally_step - ystep_cou;
		temp_x_length = fabsm(xstep_cou);
	    temp_y_length = fabsm(ystep_cou);
	
	    if(temp_x_length > temp_y_length)       
	       temp_xy_max = temp_x_length;
	    else
	       temp_xy_max = temp_y_length;
		if(temp_xy_max == 0)
	       temp_xy_max = 1;
		   
		if( temp_xy_max <11 )
		{
			movestepxy_time = temp_xy_max <<TIME_PARAMETER_1;
		}
		else
		{
	    	temp_xy_speed = spdlimit_10080_345_tab[temp_xy_max - 1];    
        	temp_xy_speed = temp_xy_speed/100;
	    	movestepxy_time = MoveTime_Speed_10080_ND80[temp_xy_speed] + single_move_speed;
		}	   
		movestepx_time = 0;
		movestepy_time = 0;
		
	    if(temp_x_length > 0)
	    {
			
		    movestepx_time = movestepxy_time;     
			if(movestepx_time > (temp_x_length <<TIME_PARAMETER_1) )
			{
			   movestepx_time = temp_x_length <<TIME_PARAMETER_1;
			}
	        movestep_x(-xstep_cou);
		    delay_us(400);
	    }
	    if(temp_y_length > 0)
	    {
			
		    movestepy_time = movestepxy_time;
			if(movestepy_time > (temp_y_length <<TIME_PARAMETER_1) )
			{
			   movestepy_time = temp_y_length <<TIME_PARAMETER_1;
			}
		    movestep_y(-ystep_cou);
	    }					 
		if( movestepx_time  > movestepy_time)
		    movestepxy_time = movestepx_time;
		else
		    movestepxy_time = movestepy_time;
	    nopmove_flag = 0; 
		move_flag = 0; 
		if( nop_move_pause_flag ==1 )
		    single_flag = 0;
		predit_shift = 0;
		SewTestStitchCounter--;

		for(i=0;i<movestepxy_time;i++)//2011-8-2
	        {
		        delay_ms(1);
		        //if(scan_pause_func(&pause_flag,READY))
		        	//return;
	        }
		
		SewingTestEndFlag = 0;
	}    
	if( FootRotateFlag == 1)
	{ 
	    process_making_pen_signal(1);
		FootRotateFlag = 0;
	}       

	if((cut_flag ==1)||(origin2_lastmove_flag == 1)||(	SewingStopFlag == 1)||( RotateFlag	== 1)||(stop_flag == 1))
	{
		cut_flag = 0;
		origin2_lastmove_flag = 0;
		SewingStopFlag = 0;
		RotateFlag = 0;
		stop_flag = 0;
	} 
}

//--------------------------------------------------------------------------------------
//  Name:		course_stop
//  Parameters:	None
//  Returns:	None
//  Description: course stop
//--------------------------------------------------------------------------------------
void course_stop(void)
{

	single_flag = 0; 
}
//--------------------------------------------------------------------------------------
//  Name:		shift_12
//  Parameters:	None
//  Returns:	None
//  Description: shift step to 12,
//--------------------------------------------------------------------------------------
void shift_12(void)
{
	if(foot_flag == 0 || (foot_flag == 1 && u202 == 1))  
	{	
		ally_step = ally_step - 1;
		if(ally_step < -RESOLUTION*u216 || ally_step > RESOLUTION*u215)
		{
			remove_stop();
			ally_step = ally_step + 1;
		}
		else
		{
			
		   	movestepy_time = OPEN_LOOP_TIME;
			movestep_y(-1);
			delay_ms(12);
    	}   
  	}
  	shift_flag = 0; 
  	shift_reply = 0;
}
//--------------------------------------------------------------------------------------
//  Name:		shift_01
//  Parameters:	None
//  Returns:	None
//  Description: shift step to 01，
//--------------------------------------------------------------------------------------
void shift_01(void)
{
	if(foot_flag == 0 || (foot_flag == 1 && u202 == 1))    //2011-11-21
	{	
		ally_step = ally_step - 1;
        allx_step = allx_step - 1;
		 
		if(allx_step < -RESOLUTION*u213 || allx_step > RESOLUTION*u214 || ally_step < -RESOLUTION*u216 || ally_step > RESOLUTION*u215)
		{
			remove_stop();
		    ally_step = ally_step + 1;
      		allx_step = allx_step + 1; 
		}
		else
		{
			
			movestepy_time = OPEN_LOOP_TIME;
		    movestep_y(-1);
      		delay_us(400);
      		movestepx_time = OPEN_LOOP_TIME;
			movestep_x(-1);
			delay_ms(12);
    	}
  	}
  	shift_flag = 0; 
	shift_reply = 0; 
}
//--------------------------------------------------------------------------------------
//  Name:		shift_03
//  Parameters:	None
//  Returns:	None
//  Description: shift step to 03，
//--------------------------------------------------------------------------------------
void shift_03(void)
{
	if(foot_flag == 0 || (foot_flag == 1 && u202 == 1))    
	{	
		   allx_step = allx_step - 1;

		if(allx_step < -RESOLUTION*u213 || allx_step > RESOLUTION*u214)
		{
			remove_stop();
			allx_step = allx_step + 1;
		}
		else
		{
			
			movestepx_time = OPEN_LOOP_TIME;
			movestep_x(-1);
			delay_ms(12);
    	}
  	}
  	shift_flag = 0; 
	shift_reply = 0;   
}
//--------------------------------------------------------------------------------------
//  Name:		shift_04
//  Parameters:	None
//  Returns:	None
//  Description: shift step to 04，
//--------------------------------------------------------------------------------------
void shift_04(void)
{
	if(foot_flag == 0 || (foot_flag == 1 && u202 == 1))    //2011-11-21
	{	
		ally_step = ally_step + 1;
      	allx_step = allx_step - 1; 
		
		if(allx_step < -RESOLUTION*u213 || allx_step > RESOLUTION*u214 || ally_step < -RESOLUTION*u216 || ally_step > RESOLUTION*u215)
		{
			remove_stop();
			ally_step = ally_step - 1;
      		allx_step = allx_step + 1; 
		}
		else
		{
			
			movestepy_time = OPEN_LOOP_TIME;
			movestep_y(1);
      		delay_us(400);
      		movestepx_time = OPEN_LOOP_TIME;
			movestep_x(-1);
			delay_ms(12);
    	}
  	}
  	shift_flag = 0; 
	shift_reply = 0; 
}
//--------------------------------------------------------------------------------------
//  Name:		shift_06
//  Parameters:	None
//  Returns:	None
//  Description: shift step to 06，
//--------------------------------------------------------------------------------------
void shift_06(void)
{
	if(foot_flag == 0 || (foot_flag == 1 && u202 == 1))    //2011-11-21
	{	
		   ally_step = ally_step + 1;
		if(ally_step < -RESOLUTION*u216 || ally_step > RESOLUTION*u215)
		{
			remove_stop();
			ally_step = ally_step - 1;
		}
		else
		{
			
			movestepy_time = OPEN_LOOP_TIME;
			movestep_y(1);
			delay_ms(12);
    	}
  	}
  	shift_flag = 0; 
	shift_reply = 0;
}
//--------------------------------------------------------------------------------------
//  Name:		shift_07
//  Parameters:	None
//  Returns:	None
//  Description: shift step to 07，
//--------------------------------------------------------------------------------------
void shift_07(void)
{
	if(foot_flag == 0 || (foot_flag == 1 && u202 == 1))    //2011-11-21
	{	
		ally_step = ally_step + 1;
      	allx_step = allx_step + 1;
		if(allx_step < -RESOLUTION*u213 || allx_step > RESOLUTION*u214 || ally_step < -RESOLUTION*u216 || ally_step > RESOLUTION*u215)
		{
			remove_stop();
			ally_step = ally_step - 1;
      		allx_step = allx_step - 1;
		}
		else
		{
			
			movestepy_time = OPEN_LOOP_TIME;
			movestep_y(1);
      		delay_us(400);
      		movestepx_time = OPEN_LOOP_TIME;
			movestep_x(1);
			delay_ms(12);
    	}
  	}
  	shift_flag = 0; 
	shift_reply = 0; 
}
//--------------------------------------------------------------------------------------
//  Name:		shift_09
//  Parameters:	None
//  Returns:	None
//  Description: shift step to 09，
//--------------------------------------------------------------------------------------
void shift_09(void)
{
	if(foot_flag == 0 || (foot_flag == 1 && u202 == 1))    //2011-11-21
	{
		   allx_step = allx_step + 1;
		
		if(allx_step < -RESOLUTION*u213 || allx_step > RESOLUTION*u214)
		{
			remove_stop();
			allx_step = allx_step - 1;
		}
		else
		{	
			
	    	movestepx_time = OPEN_LOOP_TIME;
			movestep_x(1);
			delay_ms(12);
    	}
  	}
  	shift_flag = 0; 
	shift_reply = 0;  
}
//--------------------------------------------------------------------------------------
//  Name:		shift_10
//  Parameters:	None
//  Returns:	None
//  Description: shift step to 10，
//--------------------------------------------------------------------------------------
void shift_10(void)
{
	if(foot_flag == 0 || (foot_flag == 1 && u202 == 1))    //2011-11-21
	{
		ally_step = ally_step - 1;
      	allx_step = allx_step + 1; 
		if(allx_step < -RESOLUTION*u213 || allx_step > RESOLUTION*u214 || ally_step < -RESOLUTION*u216 || ally_step > RESOLUTION*u215)
		{
			remove_stop();
			ally_step = ally_step + 1;
      		allx_step = allx_step - 1; 
		}
		else
		{
			
			movestepy_time = OPEN_LOOP_TIME;
			movestep_y(-1);
      		delay_us(400);
      		movestepx_time = OPEN_LOOP_TIME;
			movestep_x(1);
			delay_ms(12);
    	}
  	} 
  	shift_flag = 0; 
	shift_reply = 0;
}
//--------------------------------------------------------------------------------------
//  Name:		remove_12
//  Parameters:	None
//  Returns:	None
//  Description: remove step to 12 
//--------------------------------------------------------------------------------------
void remove_12(void)
{
	if(foot_flag == 0 || (foot_flag == 1 && u202 == 1))    
	{	
		ally_step = ally_step - 1;
		if(ally_step < -RESOLUTION*u216 || ally_step > RESOLUTION*u215)
		{
			remove_stop();
			ally_step = ally_step + 1;
		}
		else
		{
			
			movestepy_time = CLOSE_LOOP_TIME;
			movestep_y(-1);
			//delay_us(1020);
		}
	}
    else
    {
    	remove_stop();
    }	
}
//--------------------------------------------------------------------------------------
//  Name:		remove_01
//  Parameters:	None
//  Returns:	None
//  Description: remove step to 01 
//--------------------------------------------------------------------------------------
void remove_01(void)
{
	if(foot_flag == 0 || (foot_flag == 1 && u202 == 1))  
	{	
		ally_step = ally_step - 1;
		allx_step = allx_step - 1;
	
		if(allx_step < -RESOLUTION*u213 || allx_step > RESOLUTION*u214 || ally_step < -RESOLUTION*u216 || ally_step > RESOLUTION*u215)
		{
			remove_stop();
			ally_step = ally_step + 1;
		    allx_step = allx_step + 1;
		}
		else
		{
			
			movestepy_time = CLOSE_LOOP_TIME;
			movestep_y(-1);
		    delay_us(400);
		    movestepx_time = CLOSE_LOOP_TIME;
			movestep_x(-1);
			//delay_us(1020);
		}
	}
    else
    {
    	remove_stop();
    }	
}
//--------------------------------------------------------------------------------------
//  Name:		remove_03
//  Parameters:	None
//  Returns:	None
//  Description: remove step to 03 
//--------------------------------------------------------------------------------------
void remove_03(void)
{
	if(foot_flag == 0 || (foot_flag == 1 && u202 == 1))
	{	
		
	    allx_step = allx_step - 1;
		if(allx_step < -RESOLUTION*u213 || allx_step > RESOLUTION*u214)
		{
			remove_stop();
			allx_step = allx_step + 1;
			return;
		}
		else
		{
			
			movestepx_time = CLOSE_LOOP_TIME;
			movestep_x(-1);
			//delay_us(1020);
		}
	}
    else
    {
    	remove_stop();
    }
}
//--------------------------------------------------------------------------------------
//  Name:		remove_04
//  Parameters:	None
//  Returns:	None
//  Description: remove step to 04
//--------------------------------------------------------------------------------------
void remove_04(void)
{
	if(foot_flag == 0 || (foot_flag == 1 && u202 == 1))    //2011-11-21
	{	
		
		ally_step = ally_step + 1;
		allx_step = allx_step - 1;
	
		if(allx_step < -RESOLUTION*u213 || allx_step > RESOLUTION*u214 || ally_step < -RESOLUTION*u216 || ally_step > RESOLUTION*u215)
		{
			remove_stop();
			ally_step = ally_step - 1;
		    allx_step = allx_step + 1;
		}
		else
		{
			
			movestepy_time = CLOSE_LOOP_TIME;
			movestep_y(1);
		    delay_us(400);
		    movestepx_time = CLOSE_LOOP_TIME;
			movestep_x(-1);
			//delay_us(1020);
		}
	}
    else
    {
    	remove_stop();
    }
}
//--------------------------------------------------------------------------------------
//  Name:		remove_06
//  Parameters:	None
//  Returns:	None
//  Description: remove step to 06 
//--------------------------------------------------------------------------------------
void remove_06(void)
{
	if(foot_flag == 0 || (foot_flag == 1 && u202 == 1)) 
	{	
		   ally_step = ally_step + 1;
		if(ally_step < -RESOLUTION*u216 || ally_step > RESOLUTION*u215)
		{
			remove_stop();
			   ally_step = ally_step - 1;
		}
		else
		{
			
			movestepy_time = CLOSE_LOOP_TIME;
			movestep_y(1);
			//delay_us(1020);
		}
	}
    else
    {
    	remove_stop();
    }	
}
//--------------------------------------------------------------------------------------
//  Name:		remove_07
//  Parameters:	None
//  Returns:	None
//  Description: remove step to 07 
//--------------------------------------------------------------------------------------
void remove_07(void)
{
	if(foot_flag == 0 || (foot_flag == 1 && u202 == 1))    
	{	
		ally_step = ally_step + 1;
		allx_step = allx_step + 1;
		
		if(allx_step < -RESOLUTION*u213 || allx_step > RESOLUTION*u214 || ally_step < -RESOLUTION*u216 || ally_step > RESOLUTION*u215)
		{
			remove_stop();
			ally_step = ally_step - 1;
		    allx_step = allx_step - 1;
		}
		else
		{
			
			movestepy_time = CLOSE_LOOP_TIME;
			movestep_y(1);
		    delay_us(400);
		    movestepx_time = CLOSE_LOOP_TIME;
			movestep_x(1);
			//delay_us(1020);
		}
	}
    else
    {
    	remove_stop();
    }	
}
//--------------------------------------------------------------------------------------
//  Name:		remove_09
//  Parameters:	None
//  Returns:	None
//  Description: remove step to 09 
//--------------------------------------------------------------------------------------
void remove_09(void)
{
	if(foot_flag == 0 || (foot_flag == 1 && u202 == 1))    
	{
		    allx_step = allx_step + 1;
		if(allx_step < -RESOLUTION*u213 || allx_step > RESOLUTION*u214)
		{
			remove_stop();
			   allx_step = allx_step - 1;
		}
		else
		{
			
			movestepx_time = CLOSE_LOOP_TIME;
			movestep_x(1);
			//delay_us(1020);
		}
	}
    else
    {
    	remove_stop();
    }
}
//--------------------------------------------------------------------------------------
//  Name:		remove_10
//  Parameters:	None
//  Returns:	None
//  Description: remove step to 10 
//--------------------------------------------------------------------------------------
void remove_10(void)
{
	if(foot_flag == 0 || (foot_flag == 1 && u202 == 1))    
	{
		ally_step = ally_step - 1;
		allx_step = allx_step + 1;
		if(allx_step < -RESOLUTION*u213 || allx_step > RESOLUTION*u214 || ally_step < -RESOLUTION*u216 || ally_step > RESOLUTION*u215)
		{
			remove_stop();
			ally_step = ally_step + 1;
		    allx_step = allx_step - 1;
		}
		else
		{
			
			movestepy_time = CLOSE_LOOP_TIME;
			movestep_y(-1);
		    delay_us(400);
		    movestepx_time = CLOSE_LOOP_TIME;
			movestep_x(1);
			//delay_us(1020);
		}
	}
    else
    {
    	remove_stop();
    }
}
//--------------------------------------------------------------------------------------
//  Name:		remove_stop
//  Parameters:	None
//  Returns:	None
//  Description: remove step stop
//--------------------------------------------------------------------------------------
void remove_stop(void)
{
  	DelayCounter = 0;
	shift_flag = 0; 
	shift_reply = 0;
}

//--------------------------------------------------------------------------------------
//  Name:		go_setoutpoint
//  Parameters:	None
//  Returns:	None
//  Description: Go setout point from the last positon;
//--------------------------------------------------------------------------------------
void go_setoutpoint(void)
{
	UINT16 temp16_x,temp16_y,temp16_max;		
	UINT32 i;	
	INT16 temp16_xo,temp16_yo;	
	UINT16 quick_time = 60;	
	if(inpress_flag ==0)
	{
		inpress_up();            
	}
	
	stay_end_flag = 0;	
	
	if(super_pattern_flag !=1 || ready_go_setout_com  ==1)
	{
	
		pat_point = (PATTERN_DATA *)(pat_buf);	
		pat_buff_total_counter = 0;
	
		move_flag = 0;
		lastmove_flag = 0;
		nopmove_flag = 0;
		origin2_lastmove_flag = 0;
		end_flag = 0;
		sox_step = 0;
		soy_step = 0;
	
		inpress_high = inpress_origin; 
		//--------------------------------------------------------------------------------------
	  	//  process pattern data
	  	//--------------------------------------------------------------------------------------
		while(1)
		{				
		  	process_data();            
		    if( nopmove_flag == 1)
		    {	      
	      		sox_step = sox_step + xstep_cou;
		      	soy_step = soy_step + ystep_cou; 
				nopmove_flag = 0;
	    	}
	    	else
	    	{
				if(origin2_lastmove_flag == 0 && move_flag == 1)
				{
	    			do_pat_point_sub_one();
					move_flag = 0;
					SewTestStitchCounter = 1; 
					break;
				}
				else if(origin2_lastmove_flag == 1)
				{
					do_pat_point_sub_one();
					SewTestStitchCounter = 0;
					break;
				}
				if(end_flag == 1 || move_flag == 1 || RotateFlag == 1 || inpress_high_flag == 1)
				{
					end_flag = 0;
					move_flag = 0;
					RotateFlag = 0;
					inpress_high_flag = 0;
					do_pat_point_sub_one();
					break;
				}
	    	}	
	    	rec_com();                                        	        
	  	}

	   	//--------------------------------------------------------------------------------------
	  	// calculate displacement
	  	//-------------------------------------------------------------------------------------- 
	  	temp16_xo = sox_step - allx_step; 
	  	temp16_yo = soy_step - ally_step;
	
		temp16_y = fabsm(temp16_yo);
		temp16_x = fabsm(temp16_xo);
	
		if(temp16_x > temp16_y)
	  	{
	  		temp16_max = temp16_x;
	  	}
	  	else
	  	{
	  		temp16_max = temp16_y;
	  	}


		if(temp16_max > 0)
		{
			quick_time = Calculate_QuickMove_Time(temp16_x,temp16_y);
		}

	  	//--------------------------------------------------------------------------------------
	  	// y go start point
	  	//--------------------------------------------------------------------------------------
	  	if( temp16_y > 0)
	  	{
		 	 y_quickmove(quick_time,temp16_yo);
	  	}
	  	delay_ms(1);
	  	//--------------------------------------------------------------------------------------
	  	// x go start point
	  	//--------------------------------------------------------------------------------------
	  	if(temp16_x > 0)
	  	{
			 x_quickmove(quick_time,temp16_xo);	
	  	}
		
	  	//--------------------------------------------------------------------------------------
	  	// delay
	  	//-------------------------------------------------------------------------------------- 
		delay_ms(quick_time);
		for(i=0;i<quick_time;i++)
		{
				delay_ms(1);
				if( check_motion_done() )
				{
				   delay_ms(50);
				   break;
				}
		}

	
	  	//--------------------------------------------------------------------------------------
	  	// change all step counter
	  	//-------------------------------------------------------------------------------------- 
	  	allx_step = sox_step; 
	  	ally_step = soy_step;	
	
	}
	TempStart_point = pat_point;

		
}
//--------------------------------------------------------------------------------------
//  Name:		turnoff_led
//  Parameters:	None
//  Returns:	None
//  Description: turn off led  
//--------------------------------------------------------------------------------------
void turnoff_led(void)
{
	if(alarmled_flag == 1)         			 
  	{	
		if( para.platform_type == FOURTH_GENERATION)
    		ALARM_LED = 0; 
    	alarmled_flag = 0; 
  	}	
}
//--------------------------------------------------------------------------------------
//  Name:		turnoff_buz
//  Parameters:	None
//  Returns:	None
//  Description: turn off buzzer 
//--------------------------------------------------------------------------------------
void turnoff_buz(void)
{
  	if(alarmbuz_flag == 1)         			 
  	{	
    	SUM = 0;              // turn off ALARM BUZZER   
    	alarmbuz_flag = 0;    // switch flag		     		   
  	}
}
//--------------------------------------------------------------------------------------
//  Name:		turnoff_ledbuz
//  Parameters:	None
//  Returns:	None
//  Description: turn off led and buzzer 
//--------------------------------------------------------------------------------------
void turnoff_ledbuz(void)
{
	turnoff_led();
  	turnoff_buz();
}
//--------------------------------------------------------------------------------------
//  Name:		turnon_led
//  Parameters:	None
//  Returns:	None
//  Description: turn on led  
//--------------------------------------------------------------------------------------
void turnon_led(void)
{
  	if(alarmled_flag == 0)
  	{	         		   
		if( para.platform_type == FOURTH_GENERATION)    		   
    		ALARM_LED = 1; 
    	alarmled_flag = 1; 
  	}
}
//--------------------------------------------------------------------------------------
//  Name:		turnon_buz
//  Parameters:	None
//  Returns:	None
//  Description: turn on buzzer 
//--------------------------------------------------------------------------------------
void turnon_buz(void)
{
  	if(alarmbuz_flag == 0)
  	{	         		   
    	SUM = 1;              // turn on ALARM BUZZER  
    	alarmbuz_flag = 1;    // switch flag		     		   
  	}
}
//--------------------------------------------------------------------------------------
//  Name:		flash_led
//  Parameters:	None
//  Returns:	None
//  Description: flash led  
//--------------------------------------------------------------------------------------
void flash_led(void)
{	          	  
  	if(sound_count <= 25000)
  	{	
  		turnon_led();
  	}         		 
  	else 
  	{
  		turnoff_led();
  	}		
}
//--------------------------------------------------------------------------------------
//  Name:		flash_buz
//  Parameters:	None
//  Returns:	None
//  Description: flash buzzer  
//--------------------------------------------------------------------------------------
void flash_buz(void)
{	          	  
  	if(sound_count <= 25000)
  	{	
  		turnon_buz();
  	}         		 
  	else 
  	{
  		turnoff_buz();
  	}
}
//--------------------------------------------------------------------------------------
//  Name:		emergency
//  Parameters:	None
//  Returns:	None
//  Description: emergency measure  
//--------------------------------------------------------------------------------------
void emergency(void)
{	          	     
  	//--------------------------------------------------------------------------------------
  	// turn off all output
  	//-------------------------------------------------------------------------------------- 
  	if(motor.spd > 10)
  	{
   		motor.spd_obj = 0;
		motor.iq = 0;
   		U=1;U_=1;V=1;V_=1;W=1;W_=1;
   		prcr = 0x02;
   		inv03 = 0;
   		prcr = 0x00;
   		U_=0;V_=0;W_=0;
  	}
  	else
  	{
   		U=1;U_=1;V=1;V_=1;W=1;W_=1;
   		prcr = 0x02;
   		inv03 = 0;
   		prcr = 0x00;
   		OUTPUT_ON = 1;
		
		motor.iq = 0;
		motor.spd_obj = 0;
  	}   	    

}
//--------------------------------------------------------------------------------------
//  Name:		para_confirm
//  Parameters:	None
//  Returns:	None
//  Description: parameter confirm  
//--------------------------------------------------------------------------------------
void para_confirm(void)
{	  
		
	 if(inpress_high > 80)
		inpress_high = 80;
	 if(inpress_high < 0)
		inpress_high = 0;
	    
	if(sew_speed > 100*MAXSPEED0 || sew_speed < 200)
	{
	  	sew_speed = 1600;
	}

 	u02 = 2;
  	u03 = 6;
  	u04 = 10;
  	u05 = 15;
  	u06 = 20;  
  	//--------------------------------------------------------------------------------------
  	//  1st to 5th stitch speed confirm without thread clamp
  	//--------------------------------------------------------------------------------------  	
  	if(u10 > 15 || u10 < 2)
  	{
  		u10 = 15;
  	}
	if(u11 > MAXSPEED0 || u11 < 2)
	  	{
	  		u11 = MAXSPEED0;
	  	}
	if(u12 > MAXSPEED0 || u12 < 2)
	  	{
	  		u12 = MAXSPEED0;
	  	}
	if(u13 > MAXSPEED0 || u13 < 2)
	  	{
	  		u13 = MAXSPEED0;
	  	}
	if(u14 > MAXSPEED0 || u14 < 2)
	  	{
	  		u14 = MAXSPEED0;
	  	}
}
//--------------------------------------------------------------------------------------
//  Name:		sewing_stop
//  Parameters:	None
//  Returns:	None
//  Description: sewing stop 
//--------------------------------------------------------------------------------------
void sewing_stop(void)
{          	     
	motor.stop_angle = u236 ; 
	if(motor.stop_angle >= CODE_SCALE)
	{
		motor.stop_angle = motor.stop_angle - CODE_SCALE;
	}
  	motor.spd_obj = 0;  	    		
}
void sewing_stop_mid(void)
{          	     
	motor.stop_angle = (240*CODE_SCALE/360) ; //240du = 682/1024
	if(motor.stop_angle >= CODE_SCALE)
	{
		motor.stop_angle = motor.stop_angle - CODE_SCALE;
	}
  	motor.spd_obj = 0;  	    		
}
//--------------------------------------------------------------------------------------
//  Name:		pause_stop
//  Parameters:	None
//  Returns:	None
//  Description: pause stop 
//--------------------------------------------------------------------------------------
void pause_stop(void)
{	
	INT16 temp16;
	
	zpl_pass = 0;
	
#if ONE_STITCH_STOP
	jiting_flag = 1;  
	process_flag = 0;    	                       	
	if(making_pen_actoin_flag == 0)  
	{ 	                       	
		sewing_stop();  
		while(motor.stop_flag == 0)
		{
			rec_com();
		}
		//delay_ms(100);
		//temp16 = detect_position();	
		//if(temp16 == OUT)     
		//{
			//find_dead_center();
		//}
	   //	delay_ms(20);
		delay_ms(50); 
		manual_cut_flag = 1;
		if( ((u97 == 0)&&(stay_flag == 1))||( (LTR_trim_option == 0)&&(stay_flag == 0) ))
		{	
			delay_ms(50); 	
			motor.dir = 0;
		    motor.spd_obj = 10*u211; 
			while(1)
		    {
		    	rec_com(); 
			   	if(motor.spd_ref == motor.spd_obj)
			   	{
			    	break;
			   	}
		    }
			trim_action();
			SNT_H = 0; 
			if( u206 == 1)
				delay_ms(140+delay_of_wipper_down);
			manual_cut_flag = 0;
		}		
		inpress_up();   
		delay_ms(50);
		if(u208 == 1)
		{
		   footer_both_up();
		} 
	}
	if(making_pen_actoin_flag == 1 )
	{
			while(rec1_total_counter>0)
				delay_ms(20);
			tb4s = 0;
			laser_cutter_aciton_flag = 0;
			tra1_ind_w = 0;
			tra1_ind_r = 0;
			LASER_SIGNAL = 0;
	}
	inpress_high = inpress_position;
	//--------------------------------------------------------------------------------------
	//  switch system status 
	//--------------------------------------------------------------------------------------  	     
	sys.status = ERROR;
	StatusChangeLatch = ERROR;
	if( sys.error == OK)
	{
	    if( stay_flag == 1)
			sys.error = ERROR_02;
		else
		   	sys.error = ERROR_17; 
		if( OutOfRange_flag == 1)
			sys.error = ERROR_15;
	}
	status_now = READY;
#else
	if(motor.spd_obj >= 50 && motor.spd_obj <= 1000)
	{
		if( motor.spd_obj > 10*u211)
			motor.spd_obj = 10*u211;

		if(u97 == 0)
		{	
			trim_action();
			SNT_H = 0; 
			if( u206 == 1)
			delay_ms(140+delay_of_wipper_down);
			manual_cut_flag = 0;
		}
		else if(u97 == 1)
		{
			temp16 = motor.angle_adjusted;
	      	while(temp16 <= tension_start_angle)
	        {  
				flag_start_waitcom = 1;
				if(flag_wait_com == 1)
				{     
	        		rec_com(); 
					counter_wait_com = 0;
					flag_wait_com = 0;
				}                  
				temp16 = motor.angle_adjusted;                         
	        }
			flag_start_waitcom = 0;      
			process_flag = 0;    	                       	
	      	sewing_stop();  
				 
			while(motor.stop_flag == 0)
			{
				rec_com();
			}
				   	     	
	      	delay_ms(20);
				
			//if(u207 == 1)  //????
			//{
			//	find_dead_point();
			//}
			delay_ms(50); 
			manual_cut_flag = 1;
		}
		//--------------------------------------------------------------------------------------
		//  inpresser up
		//--------------------------------------------------------------------------------------
		inpress_up();   
		delay_ms(50);
		if(u208 == 1)
		{
		    footer_both_up();
		} 
		//--------------------------------------------------------------------------------------
		//  switch system status 
		//--------------------------------------------------------------------------------------  	     
		sys.status = ERROR;
		StatusChangeLatch = ERROR;
		if(sys.error == OK)
		{
	    	if( stay_flag == 1)
				sys.error = ERROR_02;
			else
			   	sys.error = ERROR_17; 
			if( OutOfRange_flag == 1)
				sys.error = ERROR_15;
		}
		status_now = READY;
	}	 
	else if(motor.spd_obj > 800 && motor.spd_obj < 1800)
	{
		motor.spd_obj = 800;      
	}
	else if( (motor.spd_obj >= 1800)&&(motor.spd_obj < 2500) )
	{
		motor.spd_obj = 1700;      
	}
	else if(motor.spd_obj >= 2500)
	{
		motor.spd_obj = 2400;      
	}	
#endif
    if( making_pen_actoin_flag == 1)
	{
		if( making_pen_status == 1 )
			PEN_SIGNAL = 0;
		else if( making_pen_status == 4 )
			LASER_SIGNAL = 0;
	}
		
}
//--------------------------------------------------------------------------------------
//  Name:		detect_position
//  Parameters:	None
//  Returns:	UINT8
//  Description: detect needle position
//--------------------------------------------------------------------------------------
UINT8 detect_position(void)
{	   
	INT16 temp16,comp1;	
  			
  	temp16 = motor.angle_adjusted;
	if(u42 == 0)
	{	
	  	if(temp16 > (INT16)u236+12 || temp16 < (INT16)u236-12)    
	  	{
      		return OUT;
	  	}
	  	else
	  	{		    			   	
	    	return IN;    			
	  	}
	}
	else
	{

		temp16 = ( motor.angle_adjusted + 52) % CODE_SCALE;
		comp1 =  dead_point_degree+52 ;//[0-150]
		if( (temp16 > comp1+12) ||( comp1 >temp16 +12) )
			return OUT;
		else
		    return IN;
	}	 
}


//--------------------------------------------------------------------------------------
//  Name:		reset_panel
//  Parameters:	None
//  Returns:	None
//  Description: reset_panel
//--------------------------------------------------------------------------------------
void reset_panel(void)
{	     
	delay_us(50000);
	delay_us(50000);
	RST_PN = 0;		      // reset control panel
}
//--------------------------------------------------------------------------------------
//  Name:		initial_mainmotor
//  Parameters:	None
//  Returns:	None
//  Description: initial main motor
//--------------------------------------------------------------------------------------
void initial_mainmotor(void)
{	     
	if(motorconfig_flag == 0)
  	{
	  	if(sys.error == 0)
	    {
    		motor_start();

    		while(motor.stop_flag == 0)    
      		{
        		rec_com();             // communication with panel	
      		}    
			
      		delay_ms(200);  
			if(FindZeroFlag == 1)
			{
				if(u42 ==0)
				  	find_up_position();
				else
					find_zero();
				motorconfig_flag = 1;
			}
			else if(FindZeroFlag == 0)					
			{
				delay_ms(200);
				if(FindZeroFlag == 1)
				{
					if(u42 ==0)
					  	find_up_position();
					else
						find_zero();
					motorconfig_flag = 1;
				}
				else if(FindZeroFlag == 0)
				{
					delay_ms(200);
					if(FindZeroFlag == 1)//2011-5-15
					{
						if(u42 ==0)
						  	find_up_position();
						else
							find_zero();
						motorconfig_flag = 1;
					}
					else if(FindZeroFlag == 0)
					{
						delay_ms(200);
						if(FindZeroFlag == 1)//2011-5-15
						{
							if(u42 ==0)
							  	find_up_position();
							else
								find_zero();
							motorconfig_flag = 1;
						}
						else if(FindZeroFlag == 0)
						{
							delay_ms(200);
							if(FindZeroFlag == 1)//2011-5-15
							{
								if(u42 ==0)
								  	find_up_position();
								else
									find_zero();
								motorconfig_flag = 1;
							}
							else if(FindZeroFlag == 0)
							{
								delay_ms(200);
								if(FindZeroFlag == 1)//2011-5-15
								{
									if(u42 ==0)
									  	find_up_position();
									else
										find_zero();
									motorconfig_flag = 1;
								}
								else
								{
									EncoderZ_flag = 0;
								}
							}
						}
					}

				}
			}
			
    	}
    	else
    	{ 
    		sys.status = ERROR;
			StatusChangeLatch = ERROR;
    		return;
    	}	  
  	}
}

//--------------------------------------------------------------------------------------
//  Name:		go_edit1_commandpoint
//  Parameters:	None
//  Returns:	None
//  Description: go edit command point
//--------------------------------------------------------------------------------------
void go_commandpoint(INT16 commandpointcoorx,INT16 commandpointcoory)
{
	UINT16 temp16_x,temp16_y,temp16_max;		
	UINT32 i;
	INT16 temp16_xo,temp16_yo;

	UINT8  quick_move_flag,temp8;		
	INT16 allx_temp_step,ally_temp_step;	
	INT16 tempx_step,tempy_step,add_x,add_y; 
	UINT16 quick_time ,j,tmpx,tmpy;	
	PATTERN_DATA *TempPatpoint;
  	//--------------------------------------------------------------------------------------
  	// calculate displacement
  	//-------------------------------------------------------------------------------------- 
	quick_time = 60;	
	quick_move_flag = 0; 
	TempPatpoint = pat_point;
	allx_temp_step = allx_step;
	ally_temp_step = ally_step;
	bakeup_total_counter = pat_buff_total_counter;
	if(inpress_flag == 0)
		inpress_up();
  	temp16_xo = commandpointcoorx - allx_step; 
  	temp16_yo = commandpointcoory - ally_step;   
	tempx_step = temp16_xo;
    tempy_step = temp16_yo; 
	
	temp16_y = fabsm(temp16_yo);
	temp16_x = fabsm(temp16_xo);
	
	
	if(temp16_x > temp16_y)
  	{
  		temp16_max = temp16_x;
  	}
  	else
  	{
  		temp16_max = temp16_y;
  	}

	if(temp16_max > 0)
	{
		quick_time = Calculate_QuickMove_Time(temp16_x,temp16_y);
		quick_move_flag = 1;
	}	
	
	temp8 = detect_position();	
	if(temp8 == OUT)     
	{
		find_dead_center();
	}
  	//--------------------------------------------------------------------------------------
  	// y go start point
  	//--------------------------------------------------------------------------------------
  	if( temp16_y > 0)
  	{
		y_quickmove(quick_time,temp16_yo);
 	}
  	delay_ms(1);
  	//--------------------------------------------------------------------------------------
  	// x go start point
  	//--------------------------------------------------------------------------------------
  	if(temp16_x > 0)
  	{
	     x_quickmove(quick_time,temp16_xo);	
	}
	if( quick_time < 98)
	    quick_time = 98;	 
	for(i=0;i<quick_time - 98;i++)//58
	{
	     delay_ms(1);
		 #if NOPMOVE_STOP_ENABLE
		 //if(sys.status == RUN)
		 {
			 if((PAUSE == PAUSE_ON)&&(making_pen_nopmove_flag ==0))  
		     {
			     delay_ms(2);
			     if(PAUSE == PAUSE_ON)
			     {				
					 nop_move_pause_flag = 1; 					 
				     nop_move_emergency(temp16_x,temp16_y); //STOP MOVING
					 for(j=0;j<2000;j++)
					 {
						delay_ms(1);
	
						if( check_motion_done() )
						   break;
				  	 }
				     if( j == 2000)
					 {
					 	   sys.error = ERROR_70;//ERROR_15;
						   special_go_allmotor_flag = 1;
						   sys.status = ERROR;
						   return;
					 }
					 tmpx = 0;
					 tmpy = 0;
					 if( temp16_x > QUICKMOVE_JUDGEMENT )
					     tmpx = get_x_distance();					 
					 else if( temp16_x != 0)
					     tmpx = temp16_x;
					 else
					 	 tempx_step = 0;
						 
					 delay_ms(10);
					 
					 if( temp16_y > QUICKMOVE_JUDGEMENT  )
					     tmpy = get_y_distance();
					 else if( temp16_y != 0 )
					     tmpy = temp16_y;					 
					 else
						 tempy_step = 0;
										 
					 if( tmpx == STEPPER_IGNORE_STOP )
					     tmpx = temp16_x;					 
					 if( tmpy == STEPPER_IGNORE_STOP )
					     tmpy = temp16_y;
					   
					 read_step_x = tmpx;
					 read_step_y = tmpy;
					 
					 last_pattern_point = TempPatpoint;
					 last_allx_step = allx_temp_step;
					 last_ally_step = ally_temp_step;
					 
					 //**************************
					  if( (tmpx == 0)&&(temp16_x !=0) )
					  {
						  if(tmpy !=0)
						    read_step_x = temp16_x;
					  }
					  if( (tmpy == 0)&&(temp16_y !=0) )
					  {
						  if(tmpx !=0)
						    read_step_y = temp16_y;
					  }
					 					 
					 if( tempx_step >0 )
					 {
					  	 nop_move_remainx = tempx_step - read_step_x;
						 if( nop_move_remainx < 0)
						 {
						     sys.error = ERROR_15;
							 special_go_allmotor_flag = 1;
						 }
					 }
					 else if ( tempx_step < 0)
					 {
					     nop_move_remainx = tempx_step + read_step_x;
						 if( nop_move_remainx > 0)
						 {
						    sys.error = ERROR_15;
							special_go_allmotor_flag = 1;
						  }
						 read_step_x = -read_step_x;
					 }
					 else
					 {
						 nop_move_remainx = 0;
						 read_step_x = 0;
					 }
					      
					 if( tempy_step >0 )	 
					 {
					     nop_move_remainy = tempy_step - read_step_y;
						 if( nop_move_remainy <0 )
						 {
						   sys.error = ERROR_15;
						   special_go_allmotor_flag = 1;
						 }
					 }
					 else if( tempy_step <0 )
					 {
					     nop_move_remainy = tempy_step + read_step_y;
						 if( nop_move_remainy >0 )
						 {
						   sys.error = ERROR_15;
						   special_go_allmotor_flag = 1;
						 }
						 read_step_y = -read_step_y;
					 }
					 else
					 {
						 nop_move_remainy = 0;
						 read_step_y = 0;
					 }
					 //**************************
					 if( (sewingcontrol_flag == 2)&&(need_backward_sewing == 1) )
					 {
						 /*
						 tempx_step -= add_x;
						 tempy_step -= add_y;
						 if( sewingcontrol_stitchs >0 )//空送急停后，再启动不加固了。
						    need_action_once = 0;
						 */
						 if (tempx_step != 0)
						   nop_move_remainx -= add_x;
						 if (tempy_step != 0)
						   nop_move_remainy -= add_y;
						 if( sewingcontrol_stitchs >0 )//空送急停后，再启动不加固了。
						    need_action_once = 0;
					 }
					 break;
		  	     }
			 }
		 }
		#endif
	  }
	  delay_ms(200);
  	//--------------------------------------------------------------------------------------
  	// delay
  	//-------------------------------------------------------------------------------------- 
		if( nop_move_pause_flag == 0 )
		{
			for(i=0;i<quick_time;i++)
			{
				delay_ms(1);
				if( check_motion_done() )
				{
				   delay_ms(50);
				   break;
				}
			}
		}
  	//--------------------------------------------------------------------------------------
  	// change all step counter
  	//--------------------------------------------------------------------------------------    
  	allx_step = commandpointcoorx; 
  	ally_step = commandpointcoory;
	if(quick_move_flag)
		delay_ms(nop_move_k*nop_move_delay);
  	delay_ms(1);   

}
//--------------------------------------------------------------------------------------
//  Name:		move_xy
//  Parameters:	None
//  Returns:	None
//  Description: move xy motor
//--------------------------------------------------------------------------------------
void move_xy(void)
{	     
	INT16 temp16;
		
	allx_step = allx_step + xstep_cou;                        
  	ally_step = ally_step + ystep_cou;
	
	if( movestepy_angle > movestepx_angle )
	{
			if(fabsm(xstep_cou) > 0)
			{
				if( movex_delay_flag ==1 )			
				{
				   while(  movex_delay_counter >0)
				  	    ;
				}
				
				movex_delay_flag = 0;
				temp16 = motor.angle_adjusted;
			  	while( (temp16 <= movestepx_angle)&&(motor.spd_obj >0) )
			  	{
					temp16 = motor.angle_adjusted;
					if(sys.status == ERROR)
					   return;
					flag_start_waitcom = 1;
					if(flag_wait_com == 1)
					{     
		        		rec_com(); 
						counter_wait_com = 0;
						flag_wait_com = 0;
					}                  
			  	}
				flag_start_waitcom = 0;
				
				movestep_x(xstep_cou);
				test_flag = 1;				
				movex_delay_counter = movestepx_time + 1;
				movex_delay_flag = 1;		
				
				delay_us(400);
			}
			if(fabsm(ystep_cou) > 0)
			{
		
				if( movey_delay_flag ==1)			
				{
				   while( movey_delay_counter >0)
				  	    ;
				}
		
				movey_delay_flag = 0;
				temp16 = motor.angle_adjusted;
				while( (temp16 <= movestepy_angle) &&(motor.spd_obj>0) )
			  	{
					temp16 = motor.angle_adjusted;
					if(sys.status == ERROR)
						return;
					flag_start_waitcom = 1;
					if(flag_wait_com == 1)
					{     
		        		rec_com(); 
						counter_wait_com = 0;
						flag_wait_com = 0;
					}                  
			  	}
				flag_start_waitcom = 0;
				
				movestep_y(ystep_cou); 
				test_flag = 21;
				movey_delay_counter = movestepy_time + 1;
				movey_delay_flag = 1;
				
			}
		}
		else//y<=x
		{
			
			if(fabsm(ystep_cou) > 0)
			{
				if( movey_delay_flag ==1)	
				{
				   while(  movey_delay_counter >0)
				  	    ;
				}			
				movey_delay_flag = 0;
				temp16 = motor.angle_adjusted;
				while( (temp16 <= movestepy_angle) &&(motor.spd_obj >0) )
			  	{
					temp16 = motor.angle_adjusted;
					if(sys.status == ERROR)
						return;
					flag_start_waitcom = 1;
					if(flag_wait_com == 1)
					{     
		        		rec_com(); 
						counter_wait_com = 0;
						flag_wait_com = 0;
					}                  
			  	}
				flag_start_waitcom = 0;
				
				movestep_y(ystep_cou); 
				test_flag= 21;			
				movey_delay_counter = movestepy_time + 1;
				movey_delay_flag = 1;				
				delay_us(400);
			}
			if(fabsm(xstep_cou) > 0)
			{
				if( movex_delay_flag ==1)	
				{
				   while(  movex_delay_counter >0)
				  	    ;
				}
			
				movex_delay_flag = 0;
				temp16 = motor.angle_adjusted;
			  	while( (temp16 <= movestepx_angle) &&( motor.spd_obj>0) )
			  	{
					temp16 = motor.angle_adjusted;
					if(sys.status == ERROR)
					   return;
					flag_start_waitcom = 1;
					if(flag_wait_com == 1)
					{     
		        		rec_com(); 
						counter_wait_com = 0;
						flag_wait_com = 0;
					}                  
			  	}
				flag_start_waitcom = 0;
				
				movestep_x(xstep_cou);
				test_flag = 1;
				
				movex_delay_counter = movestepx_time + 1;
				movex_delay_flag = 1;
			}
		}

	rec_com();
  	move_flag = 0; 
	  
}

//---------------------------------------------------------------------------------------
//  Name:		zpl_process
//  Parameters:	None
//  Returns:	None
//  Description: zpl process
//--------------------------------------------------------------------------------------
void zpl_process(void)
{	     
	if(pause_flag ==0 && thbrk_flag == 0 && software_key_pause ==0)
  	{	 
	    if(process_flag == 1)
	    {	
      		if( making_pen_actoin_flag == 0)
			    check_data(1);   
			process_data();        		
			if( making_pen_actoin_flag == 0)	    
      		    calculate_angle();	   		 
			SewTestStitchCounter++;	
    	}
    	stay_flag = 0;           			// no emergency break
    	brkdt_flag = 0;          			// no thread breakage detection  	    
    	if((TH_BRK == 1)&&( making_pen_actoin_flag == 0))
    	{
  	  		thbrk_count =thbrk_count + 1;
    	}
		else if(TH_BRK == 0)
			thbrk_count = 0;
			
		zpl_pass = 0;
  	}
  	else
  	{
	  	if( (pause_flag == 1)||(software_key_pause == 1) )
	  	{	
  	  		stay_flag = 1;         			// emergency break
  		}                        
  		if(thbrk_flag == 1)      
  		{                        
  	  		brkdt_flag = 1;        			// thread breakage detection  	  	  	  	  	  	  	  	
  		}	
  	} 
}

//--------------------------------------------------------------------------------------
//  Name:		trim
//  Parameters:	None
//  Returns:	None
//  Description: control machines of trim stauts
//--------------------------------------------------------------------------------------
void trim(void)
{	
	inpress_down(0); 
	delay_ms(250);	
	motor.dir = 0;
	motor.spd_obj = 10*u211; 	 
	trim_action();
	inpress_up();
	delay_ms(300);
}

//--------------------------------------------------------------------------------------
//  Name:		go_origin_zx
//  Parameters:	None
//  Returns:	None
//  Description: presser motor go origin
//--------------------------------------------------------------------------------------
void go_origin_zx(void)
{	
	UINT8 i;
	UINT16 temp16,j ;
    j = 0;
	
	if(inpress_type == AIR_INPRESSER)   
	{
	   inpress_up();	
	   return ;
	}

	if(get_IORG_statu() != 0)           // sensor is not covered   
	{	
		temp16 = 0;
		if( sys.status == ERROR)
		{
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
				return;
		}
    	while(get_IORG_statu() != 0)   
    	{
    		delay_us(500);
			movestep_zx(-1,1);
			if(j < 2)
			    delay_ms(2);	
			else if(j <4 )
				delay_us(1000);
		    else
				delay_us(500);
			if( j < 4 )
				j++;
			temp16 = temp16 + 1;			
			if( sys.status == ERROR)
		    {
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
				return;
			}
			if( temp16 > 1600)
  		  	{
  		  		sys.status = ERROR;
				StatusChangeLatch = ERROR;
    	  		sys.error = ERROR_29; 	 	            
    	  		return;
  		  	}	
			rec_com();	
    	}
  	}
  	else                    // sensor is covered
  	{
  		temp16 = 0;
		j = 0;
		if( sys.status == ERROR)
		{
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
				return;
		}
  		while(get_IORG_statu() == 0)    
    	{
			delay_us(500);
    		movestep_zx(1,1);
		    if(j < 2)
			    delay_ms(2);	
			else if(j <4 )
				delay_us(1000);
		    else
				delay_us(500);
			if( j < 4 )
				j++;
			
  		  	temp16 = temp16 + 1;			
			if(sys.status == ERROR)
			{
				  sys.status = ERROR;
				  StatusChangeLatch = ERROR;
				  return;
			}
  		  	if(temp16 > 1600)
  		  	{
  		  		sys.status = ERROR;
				StatusChangeLatch = ERROR;
    	  		sys.error = ERROR_29;      	 	            
    	  		return;
  		  	}	
			rec_com();	
    	}
    	for(i=2;i>0;i--)     			 // continue running
		{
			movestep_zx(1,1);
  	  		delay_ms(2);
		}
		delay_ms(1);
		temp16 = 0;
		j = 0;
		while(get_IORG_statu() == 1)                   
   		{
      		delay_us(500);
			movestep_zx(-1,1);
			delay_us(500);
			delay_ms(1);			
	  		temp16 = temp16 + 1;			
			if( sys.status == ERROR)
			{
			    sys.status = ERROR;
				StatusChangeLatch = ERROR;
				return;
			}
  	  		if(temp16 > 1600)
  	  		{
  	  			sys.status = ERROR;
				StatusChangeLatch = ERROR;
      			sys.error = ERROR_29;    	            
      			return;
  	  		}	
			rec_com();	
    	}
  	}
    for(i=0;i<3;i++)     // running ensure IORG=1 
	{
		movestep_zx(-1,1);
  	  	delay_ms(2);
	}
  	allin_step = 0;
	inpress_flag = 1;     
	inpress_com = 0;      
	MotorPositionSet = 0;
    inpress_position = inpress_origin;
	steper_footer_position = 0;
	inpress_mod_remain = 0;
	delay_ms(10);
	FA = 0;
}

void go_origin_yj(void)
{	
	UINT8 i;
	UINT16 temp16,j ;
    j = 0;
	if( cut_mode != STEPPER_MOTER_CUTTER )
		return;
	if(get_CORG_statu() != 0)           // sensor is not covered   
	{	
		temp16 = 0;
    	while(get_CORG_statu() != 0)   
    	{
    		delay_us(1000);
			movestep_yj(-1,1);
			if(j < 2)
			    delay_ms(2);//	
			else if(j <4 )
				delay_us(1000);
		    else
				delay_us(500);//1500
			if( j < 4 )
				j++;
			temp16 = temp16 + 1;			
			if( sys.status == ERROR)
		    {
				sys.status = ERROR;
				StatusChangeLatch = ERROR;
				return;
			}
			if( temp16 > 1600)
  		  	{
  		  		sys.status = ERROR;
				StatusChangeLatch = ERROR;
    	  		sys.error = ERROR_85; 	 	            
    	  		return;
  		  	}	
			rec_com();	
    	}
		for(i=3;i>0;i--)     			
		{
			movestep_yj(-1,1);
  	  		delay_ms(2);
		}
  	}    
	//else // sensor is covered
  	{
  		temp16 = 0;
		j = 0;
  		while(get_CORG_statu() == 0)    //0--对应Z高电平
    	{
    		delay_us(1000);
			movestep_yj(1,1);
		    if(j < 2)
			    delay_ms(2);	
			else if(j <4 )
				delay_us(1000);
		    else
				delay_us(500);
			if( j < 4 )
				j++;
			
  		  	temp16 = temp16 + 1;			
			if(sys.status == ERROR)
			{
				  sys.status = ERROR;
				  StatusChangeLatch = ERROR;
				  return;
			}
  		  	if(temp16 > 1600)
  		  	{
  		  		sys.status = ERROR;
				StatusChangeLatch = ERROR;
    	  		sys.error = ERROR_85;      	 	            
    	  		return;
  		  	}	
			rec_com();	
    	}
    	for(i=2;i>0;i--)     			 // continue running
		{
			movestep_yj(1,1);
  	  		delay_ms(2);
		}
		delay_ms(2);
		temp16 = 0;
		j = 0;

		while(get_CORG_statu() == 1)   //等低电平到高电平                
   		{
      		delay_us(1000);
			movestep_yj(-1,1);
			delay_ms(3);			
	  		temp16 = temp16 + 1;			
			if( sys.status == ERROR)
			{
			    sys.status = ERROR;
				StatusChangeLatch = ERROR;
				return;
			}
  	  		if(temp16 > 1600)
  	  		{
  	  			sys.status = ERROR;
				StatusChangeLatch = ERROR;
      			sys.error = ERROR_85;    	            
      			return;
  	  		}	
			rec_com();	
    	}
  	}

}
//--------------------------------------------------------------------------------------
//  Name:		inpress_to
//  Parameters:	None
//  Returns:	None
//  Description: inpresser go to x high use 110ms
//--------------------------------------------------------------------------------------
void inpress_to(INT16 a)
{
	INT16 step;
	UINT16 temp;
	INT16 now_position;
	INT16 obj_position;
	
	if(inpress_type == AIR_INPRESSER)   
	  return ;

	//--------------------------------------------------------------------------------------
    //  inpresser up or down 
    //--------------------------------------------------------------------------------------  
	now_position = inpress_position;
	obj_position = a;
	if(obj_position != now_position)
	{	
		//step = (obj_position - now_position)*7/10;
		step = (obj_position - now_position);
		temp = fabsm(step);	
		if( temp == 0)
		{
			inpress_position = obj_position;  
			return;
		}
		//printf_uart("step=%d,",temp);
	    if( temp < 20)                  	   //50rpm
		{	
		    movestep_zx(step,30);       
		}
 	    else if((temp >= 20) && (temp < 40))   //100rpm
		{	    
			movestep_zx(step,30);        
		}  	
		else if((temp >= 40) && (temp < 60))   //150rpm
		{	    
			movestep_zx(step,30);     
		} 
		else if((temp >= 60) && (temp < 80))   //200rpm
		{	    
			movestep_zx(step,30);
	    }
	    else 
		{
			if(temp<=127)
			movestep_zx(step,63);			  //196*0.45 =>63ms  257rpm
			else
		   		zx_quickmove(80,step);         
		}
		inpress_position = obj_position;  
	
	} 	
	
		
}
//--------------------------------------------------------------------------------------
//  Name:		inpress_to_forsingle
//  Parameters:	None
//  Returns:	None
//  Description: inpresser go to x high use 110ms
//--------------------------------------------------------------------------------------
void inpress_to_forsingle(INT16 a)
{
	INT16 step;
	UINT16 temp;
	INT16 now_position;
	INT16 obj_position;
	
	if(inpress_type == AIR_INPRESSER)   
	  return ;
	
	if(a > 80)
      a = 80; 
	
	now_position = inpress_position;
	obj_position = a;
	inpress_real_delta_runing = now_position - obj_position;
	if(obj_position != now_position)
	{		
		step = (obj_position - now_position);
		temp = fabsm(step);
		if( temp == 0)
		{
			inpress_position = obj_position;  
			return;
		}	
		if(temp < 20)                  		   // move 0.5mm ~ 7.0mm 
		{	    
		   movestep_zx(step,32);       
		}
 	    else if((temp >= 20) && (temp < 40))   // move more than 7.0mm 
		{	    
			movestep_zx(step,64);             
		}  	
		else if((temp >= 40) && (temp < 60))   // move more than 7.0mm 
		{	    
			movestep_zx(step,80);             
		} 
		else if((temp >= 60) && (temp < 80))  // move Δ from 4.8mm to 5.8mm
		{	    
			movestep_zx(step,96);
	    }
	    else 
		{
		   movestep_zx(step,160);
		}
        inpress_position = obj_position; 
	
	} 	
	
	
}

void do_pat_point_add_one(void)
{
	pat_point++;
	pat_buff_total_counter ++;
	if(super_pattern_flag == 1)
	{
		if( pat_point >= (PATTERN_DATA *)(pat_buf)+TOTAL_STITCH_COUNTER )
		{
		    pat_point = (PATTERN_DATA *)(pat_buf);
			//pat_buff_total_counter += TOTAL_STITCH_COUNTER;
		}
	}
	
		total_counter_temp = pat_buff_total_counter%6000;
	
}
void do_pat_point_sub_one(void)
{
	pat_point--;
	if(pat_buff_total_counter >0)
	   pat_buff_total_counter --;
	if(super_pattern_flag == 1)
	{
		if( pat_point < (PATTERN_DATA *)(pat_buf) ) 
		{
		    if( pat_buff_total_counter >0) 
			   pat_point = (PATTERN_DATA *)(pat_buf)+TOTAL_STITCH_COUNTER-1;
			else
			{
			   pat_point = (PATTERN_DATA *)(pat_buf);
			   start_flag = 1;
			}
		}
	}
}

void coor_com_fun(void)
{
	if (u39 == 0 )//stay at current point
	   return;
	
	predit_shift = 1;
  	go_commandpoint(comx_step,comy_step);
	predit_shift = 0;
	coor_com = 0;	
}

UINT8 scan_pause_func(UINT8 *pause_flag_tmp,UINT8 system_staus_tmp)
{
	if(*pause_flag_tmp)
	{
		 status_now = system_staus_tmp;
		 single_flag = 0;
		 sys.status = ERROR;
		 StatusChangeLatch = ERROR;
		 sys.error = ERROR_02;
         *pause_flag_tmp = 0;
		 return 1;
	}
	else
		return 0;
}

void special_sewing(UINT8 flag,UINT8 cnt)
{
	UINT8 i,j,last_stitch_down;
	INT16 temp16_max,temp16;
	UINT8 action_flag0,action_flag1,action_flag2,action_flag3,action_flag4,action_flag5;
	INT16 inpress_up_angle,inpress_down_angle,max_angle;

	i = cnt;
	while( i > 0 )
	{
		while(motor.angle_adjusted < DEGREE_60) //等到60度
		{
			rec_com();
			if(sys.status == ERROR)
				return;
		}
		  		
		if( flag == 0 )    
		    process_data();
		else               		     
		{
		    conprocess_data();
			if( start_flag == 1)
			    break;
		}   
		
		if( move_flag == 1)
		{
				 #if FOLLOW_INPRESS_FUN_ENABLE
		 		 
				 check_data(1);							   
				 calculate_angle();
				 
				 action_flag0 = 1;
				 action_flag1 = 1;
				 action_flag2 = 1;
				 action_flag3 = 1;
				 action_flag4 = 1;
				 action_flag5 = 1;
				 temp16 = motor.angle_adjusted;
				 inpress_up_angle   = angle_tab[inpress_follow_up_angle];
				 inpress_down_angle = angle_tab[inpress_follow_down_angle];
				 
				 if( (inpress_down_angle > inpress_up_angle )&&(last_stitch_down ==1) )
				      action_flag2 = 0;
				 else if( (inpress_down_angle > inpress_up_angle )&&(last_stitch_down ==0))
				 {
					  inpress_down_angle = 5;
				 }	
				 while( temp16 < 994 )
				 {
					 temp16 = motor.angle_adjusted;
					 if( (thread_holding_switch == 1)&&(i ==cnt)&&(stitch_counter==1)&&(action_flag5 == 1) )//起缝夹线器的处理程序
					 {
					      	if(temp16 > thread_holding_start_angle) 
					        { 
					        	action_flag5 = 0;
								fw_action_counter = 0;
								fw_action_flag = 1;
								FW = 1;
							}
					 }	
					 //4
					 if( (temp16 > inpress_down_angle ) &&( action_flag2 == 1) )//中压脚下降角度
					 {
						action_flag2 = 0;
						if( inpress_follow_high_flag == FOLLOW_INPRESS_HIGH )
						{
							test_flag =1;
							movestep_zx(-inpress_follow_range,inpress_follow_down_speed);
							inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
						}
					 }
					 //5
					 if( (temp16 > inpress_up_angle ) &&( action_flag3 == 1) )
					 {
						  action_flag3 = 0;
						  #if FIRST_STITCH_NOT_ACTION 
						  if( (inpress_follow_high_flag == FOLLOW_INPRESS_LOW )&&(stitch_counter >inpress_lower_stitchs) )
						  #else
						  if( inpress_follow_high_flag == FOLLOW_INPRESS_LOW )
							#endif
						  {
								test_flag =21;
								movestep_zx(inpress_follow_range,inpress_follow_up_speed);//先升后降
								inpress_follow_high_flag = FOLLOW_INPRESS_HIGH;		
								//========================================
								stitch_counter++;
								pat_point++;
								check_data(0);//看看下针转速，但不改变实际转速
								inpress_down_angle = angle_tab[inpress_follow_down_angle];
								pat_point--;
								stitch_counter--;
								if( inpress_down_angle > inpress_up_angle)//默认情况下是下降小于起始的
								{
									action_flag2 = 1;//允许再次下降
									last_stitch_down = 1;
								}
								else
									last_stitch_down = 0;
								//========================================
						  }
				     }
					 //6
					if( (temp16 > movestep_angle ) &&( action_flag4 == 1) )
					{
						 action_flag4 = 0;
						 if(fabsm(ystep_cou) > 0)
					     {
							 if( flag == 0)
								 movestep_y(ystep_cou);
							 else //if(flag != 3)
						         movestep_y(-ystep_cou);
							 if(fabsm(xstep_cou) > 0)
							    delay_us(400);
					     }
						 if(fabsm(xstep_cou) > 0)
						 {
							 if( flag == 0)
							     movestep_x(xstep_cou);
							 else //if(flag != 3)
								 movestep_x(-xstep_cou);
						 }
					}
					rec_com();
				 }
				 
				 
			#else	 
				 
				 //check_data();				   
				 calculate_angle(); 
				 
				 if( (thread_holding_switch == 1)&&(i ==cnt)&&(stitch_counter==1) )//起缝夹线器的处理程序
				 {
						temp16 = motor.angle_adjusted;
				      	while(temp16 <= thread_holding_start_angle)    //180d
				        { 
				        	rec_com();
							temp16 = motor.angle_adjusted;
						}
						fw_action_counter = 0;
						fw_action_flag = 1;
						FW = 1;
				 }		
				 	 
				 //================================================================
				 temp16 = motor.angle_adjusted;
			     while( temp16 <= movestep_angle ) 
		    	 { 
				   rec_com();   
				   if(sys.status == POWEROFF)   
				      return;
				   temp16 = motor.angle_adjusted;        	                                                    
		  	     }
		   
				 if(fabsm(ystep_cou) > 0)
			     {
					 
				      if( flag == 0)
					     movestep_y(ystep_cou); 
					  else
					     movestep_y(-ystep_cou); 
				      delay_us(400);
			     }
				 if(fabsm(xstep_cou) > 0)
				 {
					 
				 	  if( flag == 0)
					      movestep_x(xstep_cou);
					  else
					      movestep_x(-xstep_cou);
				 }
			#endif	 
				 if( flag == 0)
				 {
					 allx_step = allx_step + xstep_cou;                        
	  				 ally_step = ally_step + ystep_cou;	    
				 }
				 else
				 {
					 allx_step = allx_step - xstep_cou;                        
	  				 ally_step = ally_step - ystep_cou;	    
				 }
				 
				 move_flag = 0;
				 if( (thread_holding_switch == 1)&&(i ==cnt)&&(stitch_counter==1) )
				 {
					while(motor.angle_adjusted <= thread_holding_end_angle) 
					      rec_com();
					 FW = 0;
				 }	
				 i--;
				 if( (i == 0)&&(tail_sewing_speed_flag ==1) )
				      motor.spd_obj = u211*10;

				 while(motor.angle_adjusted >= 16)
			     {
			    	   rec_com(); 
			    	   if(sys.status == ERROR)
			    			  break;
			     }  
				 //stitch_counter++;  
		   }
		   else if( tail_sewing_flag == 0)
		   {
			   
			    if( flag == 0 )    
				    do_pat_point_sub_one();
				else               		     
				{
				    do_pat_point_add_one();  
				}   
				break;
		   }
	}
	if( thread_holding_switch == 1 )
		FW = 0;

}
	
void SewingReverse(void)
{
	INT8 tmp;
	
	//-----------------------------------------------------------------------------------
	//adjust speed
	//---------------------------------------------------------------------------------
	if(MotorSpeedRigister >= 2 && MotorSpeedRigister <= MAXSPEED0)
	{
			sew_speed = MotorSpeedRigister*100;
	}
	else if(MotorSpeedRigister == 0)
	{
			MotorSpeedRigister = sew_speed/100;
	}

	if(sewingcontrol_stitchs > 0 )//V型
	{
		   motor.spd_obj  = u10 * 100;  
		   spd_monitor = motor.spd_obj;
		   special_sewing( 1 ,sewingcontrol_stitchs);
	}
	else if(sewingcontrol_stitchs< 0)//N型
	{
		tmp = fabs(sewingcontrol_stitchs);
		motor.spd_obj  = u10 * 100; 
		spd_monitor = motor.spd_obj;
		special_sewing( 0 ,tmp);
		while(motor.angle_adjusted >= 16)
	    {
	    	rec_com(); 
		}
		special_sewing( 1 ,tmp);
	}
	
}
void zoom_in_one_stitch(void)
{

	INT8 tmp_x,tmp_y,remain_x,remain_y,i,tmp,flag;
	INT16 temp16;
	UINT8 action_flag0,action_flag1,action_flag2,action_flag3,action_flag4;
	INT16 inpress_up_angle,inpress_down_angle,max_angle;
	//1
	remain_x = 0;
	remain_y = 0;
	for(i=0;i<3;i++)
	{
		if( i== 2)
		{
			tmp_x = xstep_cou - remain_x;
			tmp_y = ystep_cou - remain_y;
		}
		else
		{
			tmp_x = xstep_cou/3;
			tmp_y = ystep_cou/3;
			remain_x +=tmp_x;
			remain_y +=tmp_y;
		}
		if( thread_holding_switch == 1)
	    {
				if( (i ==0)&&(stitch_counter==1) )
				{
					temp16 = motor.angle_adjusted;
			      	while(temp16 <= thread_holding_start_angle)    //180d
			        { 
			        	rec_com();
						temp16 = motor.angle_adjusted;
					}
					FW = 1;
				}
				else
				{
					FW = 0;
				}
	    }
		#if FOLLOW_INPRESS_FUN_ENABLE
				 action_flag0 = 1;
				 action_flag1 = 1;
				 action_flag2 = 1;
				 action_flag3 = 1;
				 action_flag4 = 1;
				 
				 temp16 = motor.angle_adjusted;
				 inpress_up_angle   = angle_tab[inpress_follow_up_angle];
				 inpress_down_angle = angle_tab[inpress_follow_down_angle];
				 
				 if( inpress_down_angle > inpress_up_angle )
				    action_flag2 = 0;
				 
				 while( temp16 < 994 )
				 {
					 temp16 = motor.angle_adjusted;
					 
					 if( (temp16 > inpress_down_angle ) &&( action_flag2 == 1) )//中压脚下降角度
					 {
						action_flag2 = 0;
					
						if( inpress_follow_high_flag == FOLLOW_INPRESS_HIGH )
						{
						   test_flag = 1;
						   movestep_zx(-inpress_follow_range,inpress_follow_down_speed);//先升后降
						   inpress_follow_high_flag = FOLLOW_INPRESS_LOW;
						}
					 }
					 //3
					 if( (temp16 > inpress_up_angle ) &&( action_flag3 == 1) )
					 {
							action_flag3 = 0;
							if( inpress_follow_high_flag == FOLLOW_INPRESS_LOW )
						    {
								test_flag = 21;
								movestep_zx(inpress_follow_range,inpress_follow_up_speed);
								inpress_follow_high_flag = FOLLOW_INPRESS_HIGH;
								//========================================
								stitch_counter++;
								pat_point++;
								check_data(0);//看看下针转速，但不改变实际转速
								inpress_down_angle = inpress_follow_down_angle<<2;
								pat_point--;
								stitch_counter--;
								if( inpress_down_angle > inpress_up_angle)//默认情况下是下降小于起始的
								{
									action_flag2 = 1;//允许再次下降
								}
								//========================================
							}
				     }
					 //4
					if( (temp16 > movestep_angle ) &&( action_flag4 == 1) )
					{
						 action_flag4 = 0;
						 if(fabsm(tmp_y) > 0)
						{
					        movestep_y(tmp_y); 
					        delay_us(400);
					    }
					    if(fabsm(tmp_x) > 0)
						{
						     movestep_x(tmp_x);
						}
					}
					
					rec_com();
				 }
    	#else
		
		
		temp16 = motor.angle_adjusted;
		while( temp16 <= movestep_angle ) //300
		{ 
			rec_com();   
	    	temp16 = motor.angle_adjusted;        	                                                    
		}
    	if(fabsm(tmp_y) > 0)
		{
			
	        movestep_y(tmp_y); 
	        delay_us(400);
	    }
	    if(fabsm(tmp_x) > 0)
		{
			
		     movestep_x(tmp_x);
		}
		
		#endif
		
		allx_step = allx_step + tmp_x;                        
  		ally_step = ally_step + tmp_y;	    
		if( i<3 )
		{
			 while(motor.angle_adjusted >= 16)
		     {
		    	rec_com(); 
		     }  	
		}
	}
	if( thread_holding_switch == 1)
	    FW = 0;
	if(stay_flag == 1)
	{
		   sewing_stop();
	       while(motor.stop_flag == 0)
	       {
	      	rec_com();
	       }
		   delay_ms(50);
	       if(inpress_flag == 0)  
			{
				inpress_up();
				delay_ms(100);
			}
	}


}

void tail_sewing(void)
{
	PATTERN_DATA *tmp_point;
    spd_monitor = motor.spd_obj;
    if( sewingcontrol_tail_flag == 2)//后几针加固
	{
		tmp_point = pat_point;
		if( sewingcontrol_tail_stitches > 0)
		{
			tail_sewing_flag = 1;
			special_sewing(1,sewingcontrol_tail_stitches);
			while(motor.angle_adjusted >= 16)
		  		rec_com(); 
			tail_sewing_speed_flag = 1;	
			special_sewing(0,sewingcontrol_tail_stitches);
			tail_sewing_speed_flag = 0;	
			while(motor.angle_adjusted >= 16)
		  		rec_com();
		}
		pat_point = tmp_point;
	}
	tail_sewing_flag = 0;
	if( (sewingcontrol_flag ==2)&&(sewingcontrol_stitchs !=0) )
	    need_backward_sewing = 1;
    if( sewingcontrol_flag == 1)
	    need_action_two = 1;

    if( stay_flag == 1)
	{
    	 sewing_stop();
	     while(motor.stop_flag == 0)
	     {
	     	rec_com();
	     }
		 delay_ms(50);
	     if(inpress_flag == 0)  
		 {
			inpress_up();
			delay_ms(100);
		 }
	}
		
}

//--------------------------------------------------------------------------------------
UINT8 check_footer_status(void)
{	
	if( u202 == 1)
	    return 1;
	if(footer_working_mode == 0)
	{
		if( foot_flag == 0)
		  return 1;
	}
	else
	{
		if( return_from_setout != 0) //not run yet!
		{
			if( (foot_flag == 0)&&(foot_half_flag == 0) )
			  	return 1;
		}
		else
		{
			if( foot_flag == 0)
			    return 1;
		}
	}
	return 0;
}

#if BOBBIN_THREAD_DETECT
UINT8 detect_bottom_thread_status(void)
{
	UINT8 ret,i;
	ret = 0;
	i = 0;
	BOBBIN_EMPTY_SOLENOID = 1;
	delay_ms(200);
	if( BOBBIN_EMPTY_DETECTOR2 )//到达最深位置,底线空
	    ret = 1;
	BOBBIN_EMPTY_SOLENOID = 0;
	i = 0;
	while( BOBBIN_EMPTY_DETECTOR1 == 0)
	{
		delay_ms(100);
		i++;
		if( i >20 )
		    break;
	}
	return ret;
}
#endif

#if BOBBIN_CHANGER_ENABLE
void bobbin_change_process(void)
{
	UINT16 cnt1s,cnt13s;
	//turnon_buz();
	bobbin_change_in_progress = 1;
	BOBBIN_CHANGE_START = 1;
	delay_ms(200);
	BOBBIN_CHANGE_START = 0;
	cnt1s = 0;
	cnt13s = 0;
	while( BOBBIN_CHANGE_WORKINGNOW == 1)//动作禁止信号由高低电平变成低电平，转换完成恢复高电平
	{
		rec_com();
		delay_ms(1);
		cnt1s ++;
		if( cnt1s >= 1000)
		{
			cnt1s = 0;
			cnt13s ++;
			if( cnt13s >=13)
			{
				cnt13s = 0;
				sys.status = ERROR;
			    sys.error = ERROR_88;//请更换自动换梭装置的送料盘
				//break;
			}
		}
		while( BOBBIN_CHANGE_ERROR ==1)//正常为高电平，出现异常时为低电平
		{
			rec_com();
			sys.status = ERROR;
			sys.error = ERROR_89; //自动换梭装置异常
		}
	}
	//turnoff_buz();
	bobbin_change_in_progress = 0;
}
#endif

void process_making_pen_signal(UINT8 flag)
{
	UINT8 i;
	INT16 tmpx,tmpy;
	
	if( flag == 0) //正向过程
	{
		if(( making_pen_status == 1 )||( making_pen_status == 4 ))//记号笔开始
			{
				if( making_pen_status == 1 )
				{
					x_bios_offset = x_pen_offset;
					y_bios_offset = y_pen_offset;
				}
				else
				{
					x_bios_offset = x_laser_offset;
					y_bios_offset = y_laser_offset;
				}
				if( marking_finish_flag ==1)//之前是结束状态，防止用户连续多次的输入记号笔开始信号，只偏移一次
				{
					if( (x_bios_offset != 0)||(y_bios_offset != 0) )//
					{
						tmpx = x_bios_offset + allx_step;
						tmpy = y_bios_offset + ally_step;
						if(tmpx < -RESOLUTION*u213 || tmpx > RESOLUTION*u214 || tmpy < -RESOLUTION*u216 || tmpy > RESOLUTION*u215)
						{
							sys.error = ERROR_15;
							StatusChangeLatch = ERROR;
						}
						else
						{
							making_pen_nopmove_flag =1;
							go_commandpoint( tmpx , tmpy );
							making_pen_offset_done = 1;
							making_pen_nopmove_flag =0;
						}
					}
					delay_ms(200);
					marking_finish_flag = 0;
				}
				if( sys.error == 0)
				{
					if( making_pen_status == 1 )
						PEN_SIGNAL = 1;
					else if( making_pen_status == 4 )
						LASER_SIGNAL = 1;
					if( para.laser_function_enable == 1)
					{
						LASER_POWER_ON = 1;
					}
				}
				making_pen_actoin_flag = 1;
			    //turnon_buz();
			}
			else if(( making_pen_status == 0 )||( making_pen_status == 3 ))//记号笔结束
			{
				#if INSERPOINT_ENABLE
				while( rec1_total_counter > 0 )
						rec_com();
				
				tb4s = 0;
				laser_cutter_aciton_flag = 0; 
				tra1_ind_w = 0;
				tra1_ind_r = 0;
				#endif	
				
				if( para.laser_function_enable == 1)
				{
					LASER_POWER_ON = 0;
				}
				if( making_pen_status == 0 )
					PEN_SIGNAL = 0;
				else if( making_pen_status == 3 )
					LASER_SIGNAL = 0;
				delay_ms(200);
				if( marking_finish_flag ==0)//没执行过结束
				{
					if( (x_bios_offset != 0)||(y_bios_offset != 0) )
					{
						if( making_pen_offset_done == 1)
						{
							making_pen_nopmove_flag =1;
						 	go_commandpoint(allx_step - x_bios_offset ,ally_step - y_bios_offset);
							making_pen_offset_done = 0;
							making_pen_nopmove_flag =0;
						}
						marking_finish_flag = 1;
					}
				}
		
				making_pen_actoin_flag = 0;
				//turnoff_buz();
			}
	}
	else //倒退的过程
	{
		if(( making_pen_status == 1 )||( making_pen_status == 4 ))//记号笔开始
		{
			if( making_pen_status == 1 )
				PEN_SIGNAL = 0;
			else if( making_pen_status == 4 )
				LASER_SIGNAL = 0;
			delay_ms(200);
			if( (x_bios_offset != 0)||(y_bios_offset != 0) )
			{
				if( making_pen_offset_done == 1)
				{
					making_pen_nopmove_flag =1;
				 	go_commandpoint(allx_step - x_bios_offset ,ally_step - y_bios_offset);
					making_pen_offset_done = 0;
					making_pen_nopmove_flag =0;
				}
				marking_finish_flag = 1;
			}
		}
		else if(( making_pen_status == 0 )||( making_pen_status == 3 ))//退回到记号笔结束
		{
			if( (x_bios_offset != 0)||(y_bios_offset != 0) )//
			{
				tmpx = x_bios_offset + allx_step;
				tmpy = y_bios_offset + ally_step;
				if(tmpx < -RESOLUTION*u213 || tmpx > RESOLUTION*u214 || tmpy < -RESOLUTION*u216 || tmpy > RESOLUTION*u215)
				{
					sys.error = ERROR_15;
					StatusChangeLatch = ERROR;
				}
				else
				{
					making_pen_nopmove_flag =1;
					go_commandpoint( tmpx , tmpy );
					making_pen_offset_done = 1;
					making_pen_nopmove_flag =0;
				}
			}
			delay_ms(200);
			if( making_pen_status == 0 )
				PEN_SIGNAL = 1;
			else if( making_pen_status == 3 )
				LASER_SIGNAL = 1;
		}
	}
	delay_ms(200);
}


/*
            传感器信号逻辑（换算到CPU引脚）
			大森                    美机
			挡上       不挡         挡上        不挡
X轴原点     1          0            0           1           左移框架发+数据    右移框架发-数据
Y轴原点     1          0            1           0

中压脚原点  0--下边    1--上边    向上逆时针
剪线原点    1--退回来  0--伸出去  逆时针送出去（-）  顺时针收回来（+）
   
*/
void find_start_point_x(void)
{
	INT32 run_counter,range,i;
	UINT32 quick_time;
	UINT8 speed_tmp;
	UINT16 tmpx,temp16_max,j;
	range =15000; //12000;
	
	if( ally_step != 0)
	{
		go_commandpoint(allx_step,0);
		delay_ms(100);
	}
	
	movestepx_time = 1;
	//15000~50000mm/min  delta:2500
	//60000~18000    
	quick_time = 6000 - (UINT32)delay_of_go_setout * 420;//快走协议走
	
	if(XORG == para.x_sensor_open_level)	//如果传感器没挡上	
	{
		if( x_sensor_pos == X_SENSOR_AT_LEFT)//left
		    x_quickmove(quick_time,range);	
		else
			x_quickmove(quick_time,-range);
		
		for( i=0;i< quick_time+30; i++)
		{
			delay_ms(1);
			if( XORG != para.x_sensor_open_level)
			{
				delay_ms(20);
				if( XORG != para.x_sensor_open_level)
				{
					nop_move_emergency(500,0);
					for(j=0;j<2000;j++)
					 {
						delay_ms(1);
						if( check_motion_done() )
						   break;
				  	 }
				    delay_ms(10);
					tmpx = get_x_distance();
					break;
				}
			}
		}
	    if( (i>=quick_time+30)&&(XORG == para.x_sensor_open_level) )
		{
			sys.status = ERROR;
			StatusChangeLatch = ERROR;
			if( sys.error == 0)
	      	    sys.error = ERROR_25;
	      	return;
		}
	}
	
	if(XORG != para.x_sensor_open_level)	//如果已经挡上											
	{
		if( x_sensor_pos == X_SENSOR_AT_LEFT)//left
			x_quickmove(quick_time,-range);	
		else
			x_quickmove(quick_time,range);	
		
		for( i=0;i< quick_time+30; i++)
		{
			delay_ms(1);
			if( XORG == para.x_sensor_open_level)
			{
				delay_ms(20);
				if( XORG ==para.x_sensor_open_level)
				{
					nop_move_emergency(500,0);
					for(j=0;j<2000;j++)
					{
						delay_ms(1);
						if( check_motion_done() )
						   break;
				  	}
				    delay_ms(10);
					tmpx = get_x_distance();					 
					break;
				}
			}
		}
	    if( XORG == para.x_sensor_open_level)//如果已经冲出传感器了,再次速退回来
		{
		    run_counter = 0;
			while(XORG ==para.x_sensor_open_level)						
			{
				movestepx_time = 1;
				if( x_sensor_pos == 0)//left
		    		movestep_x(1); //向左
				else
					movestep_x(-1);
				run_counter++;
				if(run_counter > range)
				{
					sys.status = ERROR;
					StatusChangeLatch = ERROR;
					if( sys.error == 0)
	      			    sys.error = ERROR_25;
	      			return;
				}
				delay_ms(1);	
			}
		}

	}

	
	delay_ms(20);
	
	if( x_origin_offset !=0 )
	{
		temp16_max = fabsm( x_origin_offset );

		quick_time = Calculate_QuickMove_Time(temp16_max,0);

		x_quickmove(quick_time,-x_origin_offset);	

  		delay_ms(quick_time);
		for(i=0;i<quick_time;i++)
		{
			delay_ms(2);
			if( check_motion_done() )
			{
			   delay_ms(50);
			   break;
			}
		}
		delay_ms(20);
	}
}


void disable_24V_output(void)
{
	if( para.platform_type == FIFTH_GENERATION )
		SNT5_ON = 1;
	else
		SNT4_ON = 1;
}
void enable_24V_output(void)
{
	if( para.platform_type == FIFTH_GENERATION )
		SNT5_ON = 0;
	else
		SNT4_ON = 0;
}
/********************************************************************/ 
/* 函数名: Judge_Quadrant */ 
/* 功能：判断参数坐标的所在象限并返回相应象限值*/ 

/********************************************************************/ 
UINT8 Judge_Quadrant(INT32 x, INT32 y) 
{ 
	if (x>=0) 
	{ //象限判断
		if (y>=0)  
		{ 
			return 1; 
		} 
		else 
		{ 
			return 4; 
		}  
	} 
	else  
	{ 
		if (y>=0) 
		{  
			return 2;   
		}   
		else    
		{    
			return 3;   
		}  
	}  
} 
/*
 用一个缓冲区作为分解后的指令缓冲区，然后后台一直往里输入指令，前台中断从缓冲区拿出指令进行执行。
 转速的规划放到定时里去做了
 
*/
void add_one_action( UINT8 coder)
{
	while((tra1_ind_w == tra1_ind_r)&&(rec1_total_counter==250) )//缓冲区满了
		rec_com();
	tra1_buf[tra1_ind_w] = coder;
	tra1_ind_w = (tra1_ind_w + 1) % 250;
	rec1_total_counter++;
	
}

void PBP_Line (UINT8 stitch_cnt) 
{   
	INT32  lDevVal;              	//偏差值  	
	INT32  XEnd,YEnd; 			 	//目标位置、当前位置
//	UINT16 StepCount,StepMount;	 	//插补次数计数器  
	UINT8  x_count,y_count;
	UINT8  nDir;			     	//当前点所在象限     

//	StepCount = 0;
	XEnd    = xstep_cou;
	YEnd    = ystep_cou;			//目标位置	
	nDir=Judge_Quadrant(XEnd,YEnd); //象限判断 	
	lDevVal = 0;
	XEnd = fabs(XEnd);  
	YEnd = fabs(YEnd);     
	x_count = XEnd;
	y_count = YEnd;
//	StepMount = (UINT16) (XEnd+YEnd);    

	while ((x_count>0)||(y_count>0))//(StepCount < StepMount) //终点判别  
	{
		if (lDevVal>=0)  //偏差〉=0       
		{ 
			switch(nDir) 
			{    
				case 1:   
				case 4: 
						if( x_count > 0)
						{
							add_one_action(1);//x-1
							x_count--;
						}
					break;    
				case 2: 
				case 3:   
					    if( x_count > 0)
						{
							add_one_action(2);//x+1
							x_count--;//x_count++;
						}
					break;    
			}  
			lDevVal-=YEnd; 
		} 
		else
		{ //偏差<0 
 
			switch(nDir) 
			{ 
				case 1:  	
				case 2:
						if( y_count > 0)
						{
							add_one_action(3);//y-1
							y_count --;
						}
				break;  
				case 3:  	
				case 4: 
						if( y_count > 0)
						{
							add_one_action(4);//y+1
							y_count --;
						}
				break; 
			}  
			lDevVal+=XEnd; 
		} 
//		StepCount++; 
	} 
} 


//         COPYRIGHT(C) 2006 Beijing xindahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
