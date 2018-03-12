 //--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xingdahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//  Project Number: sewing_machine_controller 
//  File Name : 
//  Description: 
//  Version    Date     Author    Description
//  0.01     21/02/06   liwenz    created
//  0.02     05/03/08   lm        modify
//  ...
//--------------------------------------------------------------------------------------

#ifndef STEPMOTOR_H
#define STEPMOTOR_H

extern void init_stepmotor_drv(void);
extern void config_stepmotor_drv(void);
extern void movestep_x(int x_data);
extern void movestep_y(int y_data);
extern void fl_solenoid(int yj_data);
extern void fi_solenoid(int zx_data);
extern void ready_quick(void);
extern void readx_quick(void);
extern void quickmove_y(int y_data);
extern void quickmove_x(int x_data);
extern void comX_message(int data);
extern void comY_message(int data);
extern void quickmovex_time(UINT16 x_time);
extern void quickmovey_time(UINT16 y_time);

extern void stepstatus_check(void);
extern void version_check(void);
extern void movestep_zx(int zx_data,UINT16 time);
extern void stepmotor_para(void);    
extern void read_stepmotor_para(void);
extern void switch_stepmotor_open_close(void);

extern UINT8 check_motion_done(void);
extern void jump_to_begin(void);
extern void nop_move_emergency(UINT16 x, UINT16 y);

extern void write_stepmotor_config_para(UINT8 port,UINT8 *pdata);
extern void read_stepmotor_config_para(UINT8 port);
extern UINT8 write_stepmotor_curve(UINT8 port,UINT8 *pdata) ;
extern UINT16 read_stepmotor_curve_crc(UINT8 port);
extern void x_quickmove(UINT16 quick_time,INT32 tempx_step);
extern void y_quickmove(UINT16 quick_time,INT32 tempy_step);
extern void zx_quickmove(UINT16 quick_time,INT32 tempz_step);

extern UINT16 get_CORG_statu(void);
extern void movestep_yj(int zx_data,UINT16 time);

extern void multipule_program_end(UINT8 port) ;
extern UINT16 read_multipule_program_status(UINT8 port);
extern void multipule_program_beginning(UINT8 port);
extern void send_multipule_program_data(UINT8 port) ;
extern void multipule_program_beginning(UINT8 port);

extern void ready_dsp1_time(void);
extern void qd_quickmove(UINT16 quick_time,INT32 tempx_step);
extern void movestep_qd(int zx_data,UINT16 time);
extern void rotated_by_data(INT16 rotated_abs_angle);
#endif

//--------------------------------------------------------------------------------------
//         COPYRIGHT(C) 2006 Beijing xindahao technology Co., Ltd.
//                     ALL RIGHTS RESERVED 
//--------------------------------------------------------------------------------------
