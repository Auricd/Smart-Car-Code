#ifndef CODE_IMAGE_H_
#define CODE_IMAGE_H_
#include "zf_common_headfile.h"

void image_process(void);

#define USE_num image_h*3   //定义找点的数组成员个数按理说300个点能放下，但是有些特殊情况确实难顶，多定义了一点
#define image_H 120     //原图
#define image_W 188     //原图
#define image_h 80     //图像高度    60    120    80   80
#define image_w 140     //图像宽度    140   188  120   140
#define white_pixel 255
#define black_pixel 0
#define bin_jump_num    1//跳过的点数
#define border_max  image_w-2 //边界最大值
#define border_min  2   //边界最小值
extern uint8 original_image[image_h][image_w];
extern uint8 bin_image[image_h][image_w];//图像数组
#define uesr_RED     0XF800    //红色
#define uesr_GREEN   0X07E0    //绿色
#define uesr_BLUE    0X001F    //蓝色
#define uesr_PINK     0xFE19    //粉色
#define uesr_YELLOW   0xFFE0    //黄色
#define uesr_PURPLE    0xF81F    //紫色

void element_selection(void);
void crop_image(uint8(*mt9v03x_image)[image_W]);
int my_abs(int value);
//左
//void get_l_up_turn_left_point_1(void);
//void get_l_up_turn_left_point_2(uint8 start,uint8 end);
void get_l_up_turn_left_point(void);
void get_l_right_turn_up_point(void);
void get_l_up_turn_right_point(void);
void get_l_up_turn_bottom_left_point(void);
void get_l_top_left_down_point(void);
void get_l_top_right_turn_top_left_point(void);
void get_l_bottom_right_turn_up_point(void);

//右
void get_r_up_turn_right_point(void);
void get_r_left_turn_up_point(void);
void get_r_up_turn_left_point(void);
void get_r_up_turn_bottom_right_point(void);
void get_r_top_right_turn_down_point(void);
void get_r_top_left_turn_top_right_point(void);
void get_r_bottom_left_turn_up_point(void);


void Get_L_Intercept_And_Slope(uint8 start, uint8 end);
void Get_R_Intercept_And_Slope(uint8 start, uint8 end);
void line_repair(uint16 *start,uint16 *end,uint16 *line);

extern uint8 original_image[image_h][image_w];
extern uint8 hightest;
extern uint8 image_thereshold;
extern uint16 points_l[(uint16)USE_num][2];
extern uint16 points_r[(uint16)USE_num][2];
extern uint16 dir_r[(uint16)USE_num];
extern uint16 dir_l[(uint16)USE_num];
extern uint16 data_stastics_l;
extern uint16 data_stastics_r;
extern uint16 center_line[image_h];
extern uint16 l_border[image_h];
extern uint16 r_border[image_h];
extern uint8 hightest;
extern uint8 element;
extern float L_Straightaway_Lope_Rate_A;
extern float L_Straightaway_Lope_Rate_B;
extern float L_Straightaway_Lope_Rate_C;
extern uint8 L_Straight_Flag;
extern float R_Straightaway_Lope_Rate_A;   
extern float R_Straightaway_Lope_Rate_B;   
extern float R_Straightaway_Lope_Rate_C;   
extern uint8 R_Straight_Flag;  
extern uint8 finding_point;
extern int16 u_r;
extern int16 u_l;
extern int16 d_r;
extern int16 d_l;
//左
//上转左点
extern uint8 l_up_turn_left_point[2];//坐标
extern uint8 l_up_turn_left_point_positi;//点数
extern uint8 l_up_turn_left_point_flag;//标志位
extern uint8 l_up_turn_bottom_left_point[2];
extern uint8 l_up_turn_bottom_left_point_position;
extern uint8 l_up_turn_bottom_left_point_flag;
extern uint8 l_top_left_down_point[2];
extern uint8 l_top_left_down_point_position;
extern uint8 l_top_left_down_point_flag;
extern uint8 l_top_left_down_point[2];
extern uint8 l_top_left_down_point_position;
extern uint8 l_top_left_down_point_flag;
//右
//上转右点
extern uint8 r_up_turn_right_point[2];//坐标
extern uint8 r_up_turn_right_point_position;//点数
extern uint8 r_up_turn_right_point_flag;//标志位
extern uint8 r_up_turn_bottom_right_point[2];
extern uint8 r_up_turn_bottom_right_point_position;
extern uint8 r_up_turn_bottom_right_point_flag;
extern uint8 r_top_right_turn_down_point[2];
extern uint8 r_top_right_turn_down_point_position;
extern uint8 r_top_right_turn_down_point_flag;
#endif /* CODE_IMAGE_H_ */
