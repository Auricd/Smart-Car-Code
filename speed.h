#ifndef CODE_SPEED_H_
#define CODE_SPEED_H_
#include "zf_common_headfile.h"
#include "image.h"

#define LimitLeft(Left)    (Left = ((Left > steer_left) ? steer_left : Left))
extern uint16 center_line[image_h];
extern float angle;
extern float steer;
extern int16 speed_dif;
extern int16 Speed_Goal_l;
extern int16 Speed_Goal_r;
extern float steer_erro;
extern int16 speedl, speedr,encoder_raw_l,encoder_raw_r,total_encoder_l,total_encoder_r;
extern int16 Speed_PID_OUT_l;
extern int16 Speed_PID_OUT_r;
extern int16 Speed_Erro_l;
extern int16 Speed_Erro_r;
extern double yaw;
extern float yaw_fused;
extern float angle_rate;
extern float yaw_gyro;
extern float total_encoder_l2;
extern float total_encoder_r2;
extern uint16 line[image_h];
extern uint16 L_black_to_white[2];
extern uint8 L_black_to_white_flag;
extern uint16 L_white_to_black[2];
extern uint8 L_white_to_black_flag;
extern uint16 L_black_to_white2[2];
extern uint8 L_black_to_white2_flag;
extern uint16 L_white_to_black2[2];
extern uint8 L_white_to_black2_flag;

extern uint16 R_black_to_white[2];
extern uint8 R_black_to_white_flag;
extern uint16 R_white_to_black[2];
extern uint8 R_white_to_black_flag;
extern uint16 R_black_to_white2[2];
extern uint8 R_black_to_white2_flag;
extern uint16 R_white_to_black2[2];
extern uint8 R_white_to_black2_flag;
extern uint8 U_r_white_to_black_flag;
extern uint8 flag;

extern uint8 l_torus_status;
extern uint8 black;
extern uint8 r_torus_status;
void MOTOR_init(void);
void Encoder_init(void);
void speed_measure(void);
void Speed_decision80(void);
void Speed_decision120(void);
void Speed_decision160(void);
void Speed_decision180(void);
void Speed_decision200(void);
void speed_PIDL(void);
void speed_PIDR(void);
void steer_PID(void);
void element_processing(void);
float calc_encoder_yaw(int16 left_cnt,int16 right_cnt);
float updeta_yaw_fused(int16 left_cnt,int16 right_cnt);
float kalmanupdate(float gyro_measure);
void speed_processing(void);
//void angle_PID(void);
//void Start_gpio_init(void);

#endif /* CODE_SPEED_H_ */
