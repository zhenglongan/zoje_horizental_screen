 //--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : 
//  Description: 
//  Version    Date     Author    Description
//  0.01     09/08/08   lm        created
//  ... 
//  ...
//--------------------------------------------------------------------------------------
#ifndef ACTION_H
#define ACTION_H

extern void go_origin_x(void);
extern void go_origin_y(void);
extern void x_converse_check(void);
extern void foot_up(void);
extern void element_calaulate(void);
extern void foot_down(void);
extern void go_origin_allmotor(void);
extern void go_origin_xy(void);
extern void half_current(void);
extern void xy_test(void);
extern void inpress_to(INT16 a);
extern void find_dead_center(void);
extern void go_startpoint(void);
extern void check_data(UINT8 control_flag);
extern void calculate_angle(void);
extern void process_data(void);
extern void go_beginpoint(UINT8 FirstNopmoveFlag);
extern void conprocess_data(void);
extern void process_sewingtest_data(void);
extern void conprocess_sewingtest_data(void);
extern void go_sewingtest_beginpoint(void);
extern void back_endpoint(void);
extern void single_next(void);
extern void single_back(void);
extern void back_startpoint(void);
extern void single_end(void);
extern void single_start(void);
extern void single_stop(void);
extern void move_next(void);
extern void move_back(void);
extern void move_startpoint(void);
extern void course_next(void);
extern void course_back(void);
extern void course_stop(void);
extern void shift_12(void);
extern void shift_01(void);
extern void shift_03(void);
extern void shift_04(void);
extern void shift_06(void);
extern void shift_07(void);
extern void shift_09(void);
extern void shift_10(void);
extern void remove_12(void);
extern void remove_01(void);
extern void remove_03(void);
extern void remove_04(void);
extern void remove_06(void);
extern void remove_07(void);
extern void remove_09(void);
extern void remove_10(void);
extern void remove_stop(void);
extern void go_setoutpoint(void);
extern void turnoff_led(void);
extern void turnoff_buz(void);
extern void turnoff_ledbuz(void);
extern void turnon_led(void);
extern void turnon_buz(void);
extern void flash_led(void);
extern void flash_buz(void);
extern void emergency(void);
extern void para_confirm(void);
extern void sewing_stop(void);
extern UINT8 detect_position(void);
extern void repeat_pattern(void);
extern void clamp_out(void);
extern void clamp_in(void);
extern void clamp_backstep1(void);
extern void clamp_backstep2(void);
extern void clamp_backstep3(void);
extern void go_manualpoint(void);
extern void reset_panel(void);
extern void initial_mainmotor(void);
extern void initial_stepmotor(void);
extern void move_xy(void);
extern void move_ct(void);
extern void zpl_process(void);
extern void go_commandpoint(INT16 commandpointcoorx,INT16 commandpointcoory);

extern void needle_down(void);
extern void single_edit_continue_next(void);
extern void single_edit_continue_back(void);
extern void move_continue_next(void);
extern void move_continue_back(void);

extern void footer_procedure(void);
extern void trim(void);

extern void footer_both_up(void);
extern void footer_both_down(void);

extern UINT8 check_footer_status(void);
extern void stretch_foot_out(void);
extern void stretch_foot_in(void);
extern void sewing_stop_mid(void);
extern INT16 ChangeX(PATTERN_DATA *pp);
extern INT16 ChangeY(PATTERN_DATA *pp);
extern void inpress_to_forsingle(INT16 a);
extern void inpress_down(UINT8 pos);
extern void do_pat_point_add_one(void);
extern void do_pat_point_sub_one(void);

extern void coor_com_fun(void);
extern UINT8 scan_pause_func(UINT8 *pause_flag_tmp,UINT8 system_staus_tmp);
extern UINT32 Calculate_QuickMove_Time(UINT16 temp16_x,UINT16 temp16_y);
extern void test_quickmove_program(void);

extern void go_origin_xy_both(void);
extern void back_endpoint(void);
extern void keep_running_for_dead_center(void);

#if BOBBIN_CHANGER_ENABLE
extern UINT8 detect_bottom_thread_status(void);
extern void bobbin_change_process(void);
#endif



extern const UINT16 spdlimit_10080_345_tab[];
extern const UINT8 MoveTime_Speed_10080_x[];
extern const UINT8 MoveTime_Speed_10080_y[];

extern void process_making_pen_signal(UINT8 flag);

extern UINT8 get_IORG_statu(void);

extern void disable_24V_output(void);
extern void enable_24V_output(void);
extern void tra1_com(UINT8* command,UINT8 length);
extern void PBP_Line (UINT8 stitch_cnt);

extern void take_frame_from_one_side(UINT8 side);
extern void return_frame_back(UINT8 side);
#endif
//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xindahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
