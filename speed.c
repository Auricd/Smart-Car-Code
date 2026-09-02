#include "zf_common_headfile.h"
#include "speed.h"
#include "image.h"

#define MAX_DUTY            (50 )                                               // 最大 MAX_DUTY% 占空比
#define DIR1                (P18_6)//YOU
#define PWM1                (TCPWM_CH50_P18_7)

#define DIR2                (P00_2)//ZUO
#define PWM2                (TCPWM_CH13_P00_3)

#define ENCODER1                     	(TC_CH09_ENCODER)                      // 编码器接口       
#define ENCODER1_QUADDEC_A             	(TC_CH09_ENCODER_CH1_P05_0)            // A引脚                      
#define ENCODER1_QUADDEC_B            	(TC_CH09_ENCODER_CH2_P05_1)            // B引脚                        
                                                                                
#define ENCODER2                     	(TC_CH07_ENCODER)                      // 编码器接口   
#define ENCODER2_QUADDEC_A            	(TC_CH07_ENCODER_CH1_P02_0)            // A引脚                  
#define ENCODER2_QUADDEC_B            	(TC_CH07_ENCODER_CH2_P02_1)            // B引脚 

//定义
int16 Speed_Goal=0;
int16 speedl, speedr,encoder_raw_l,encoder_raw_r,total_encoder_l,total_encoder_r;
//左轮
int16 Speed_Goal_l;//目标速度
int16 Speed_Encoder_l;//当前速度
float Speed_P_l,Speed_I_l,Speed_D_l;
int16 Speed_Erro_l=0;//当前误差
int16 Speed_PID_OUT_l;//占空比输出
int16 Speed_Lasterro_l=0;//上次误差
int16 Speed_Preverro_l=0;//上上次误差
//右轮
int16 Speed_Goal_r;//目标速度
int16 Speed_Encoder_r;//当前速度
float Speed_P_r,Speed_I_r,Speed_D_r;
int16 Speed_Erro_r=0;//当前误差
int16 Speed_PID_OUT_r;//占空比输出
int16 Speed_Lasterro_r=0;//上次误差
int16 Speed_Preverro_r=0;//上上次误差
//中线
float weighted1,weighted2,weighted3,weighted12,weighted22;
uint16 line[image_h];
float steer;
float steer_erro;
float steer_erro1;
float steer_erro2;
float steer_P,steer_I,steer_D,steer_GKD,steer_P2;
float n;
//角
float angle;
float angle_erro;
//float angle_erro1;
//float angle_erro_sum;
float angle_P,angle_I,angle_D;
int16 speed_dif;       //差速
//计算偏航角
double yaw;
float wheel_base=0.15;//轮距
double diameter=0.000068;//周长
float total_encoder_l2;
float total_encoder_r2;
float encoder_yaw;//编码器偏航角
float yaw_fused=0;//融合偏航角
float a=0.9;//互补系数
float dt=0.0006;//时间
float yaw_gyro;
//滤波，过滤轻微抖动
float Q=0.0005;
float R=0.075;
float P=1;
float K;
float angle_rate=0;
float bias=0;
float bias_q=0;

//初始化
void MOTOR_init(void)
{
  gpio_init(DIR1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                             // GPIO 初始化为输出 默认上拉输出高
  gpio_init(DIR2, GPO, GPIO_HIGH, GPO_PUSH_PULL);                             // GPIO 初始化为输出 默认上拉输出高
  pwm_init(PWM1, 10000, 0);                                                   // PWM 通道初始化频率 17KHz 占空比初始为 0
  pwm_init(PWM2, 10000, 0);                                                   // PWM 通道初始化频率 17KHz 占空比初始为 0
}
void Encoder_init(void)
{
  encoder_dir_init(ENCODER1, ENCODER1_QUADDEC_A, ENCODER1_QUADDEC_B);       // 初始化编码器模块与引脚 带方向增量编码器模式
  encoder_dir_init(ENCODER2, ENCODER2_QUADDEC_A, ENCODER2_QUADDEC_B);       // 初始化编码器模块与引脚 带方向增量编码器模式
}

//主要代码
void speed_measure(void)//速度测量
{
  encoder_raw_l= encoder_get_count(ENCODER1);     // 定时器取值
  speedl=0.2*(float)speedl +(float)encoder_raw_l*0.8;
  total_encoder_l+=encoder_raw_l;
  encoder_clear_count(ENCODER1);// 定时器清空
  encoder_raw_r = -1*encoder_get_count(ENCODER2);     // 定时器取值
  speedr=0.2*(float)speedr+(float)encoder_raw_r*0.8;
  total_encoder_r+=encoder_raw_r;
  encoder_clear_count(ENCODER2);                     // 定时器清空

  //updeta_yaw_fused(total_encoder_l,total_encoder_r);
  //Control_Speed();              //变速控制
}

void speed_processing(void)//速度处理
{
  element_processing();
  //angle_PID();
  //Speed_Goal_r=Speed_Goal-speed_dif;
  //Speed_Goal_l=Speed_Goal+speed_dif;
  Speed_Goal_r=Speed_Goal-(int16)steer;
  Speed_Goal_l=Speed_Goal+(int16)steer;
  //Speed_Goal_r=Speed_Goal;
  //Speed_Goal_l=Speed_Goal;
  //steer_PID();
  /*if(l_torus_status==0)
  {
    Speed_Goal_l=0;
    Speed_Goal_r=0;
  }*/
  speed_PIDL();
  speed_PIDR();
}

void Speed_decision80(void)
{
  Speed_Goal = 80;

  Speed_P_l = 25; //200
  Speed_I_l = 3; //40
  Speed_D_l = 5; //5
    
  Speed_P_r = 25;//200
  Speed_I_r = 2.8; //40
  Speed_D_r = 6;  //5
    
  steer_P=1.05;
  steer_P2=0.02;
  steer_I=0;
  steer_D=2;
  steer_GKD=1.55;
    
  angle_P=1.4;//0.5
  angle_I=0.009;//2
  angle_D=0.3;//1
  
  weighted1=0.65;
  weighted2=0.25;
  weighted3=0.1;
  
  weighted12=0.8;
  weighted22=0.2;
}
void Speed_decision120(void)
{
  Speed_Goal = 120;

  Speed_P_l = 25; //200
  Speed_I_l = 3; //40
  Speed_D_l = 5; //5
    
  Speed_P_r = 25;//200
  Speed_I_r = 2.8; //40
  Speed_D_r = 6;  //5
    
  steer_P=1.27;
  steer_P2=0.03;
  steer_I=0;
  steer_D=3.8;
  steer_GKD=2.8;
  
  weighted1=0.3;
  weighted2=0.5;
  weighted3=0.2;
  
  weighted12=0.4;
  weighted22=1.2;
}
void Speed_decision160(void)
{
 Speed_Goal = 160;

  Speed_P_l = 25; //200
  Speed_I_l = 3; //40
  Speed_D_l = 5; //5
    
  Speed_P_r = 25;//200
  Speed_I_r = 2.8; //40
  Speed_D_r = 6;  //5
    
  steer_P=1.52;
  steer_P2=0.0215;
  steer_I=0;
  steer_D=3.5;
  steer_GKD=4.4;
  
  weighted1=0.2;
  weighted2=0.8;
  weighted3=0.4;
  
  weighted12=0.5;
  weighted22=1.55;
}
void Speed_decision180(void)
{
  Speed_Goal = 180;

  Speed_P_l = 25; //200
  Speed_I_l = 3; //40
  Speed_D_l = 5; //5
    
  Speed_P_r = 25;//200
  Speed_I_r = 2.8; //40
  Speed_D_r = 8;  //5
    
  steer_P=1.65;
  steer_P2=0.022;
  steer_I=0;
  steer_D=5;
  steer_GKD=2.8;
  
  weighted1=0.2;
  weighted2=0.8;
  weighted3=0.4;
  
  weighted12=0.5;
  weighted22=1.55;
}
void Speed_decision200(void)
{
  Speed_Goal = 200;

  Speed_P_l = 25; //200
  Speed_I_l = 3; //40
  Speed_D_l = 6; //5
    
  Speed_P_r = 25;//200
  Speed_I_r = 2.8; //40
  Speed_D_r = 8;  //5
    
  steer_P=2;
  steer_P2=0.035;
  steer_I=0;
  steer_D=8;
  steer_GKD=2.4;
  
  weighted1=0.2;
  weighted2=0.8;
  weighted3=0.4;
  
  weighted12=0.5;
  weighted22=1.55;
}
//pwm_set_duty(PWM_CH1, duty);           // 更新对应通道占空比
//左
void speed_PIDL(void)
{
  Speed_Encoder_l= speedl;//编码器采集当前速度
// gpt12_clear(COUNTER1_TIM);

  Speed_Erro_l =  Speed_Goal_l - Speed_Encoder_l;   //编码器反馈目标值与当前的差值

  Speed_PID_OUT_l += (Speed_P_l * (Speed_Erro_l - Speed_Lasterro_l)+  
                      Speed_I_l *  Speed_Erro_l+
                      Speed_D_l * (Speed_Erro_l- 2 * Speed_Lasterro_l+ Speed_Preverro_l));
  if(Speed_PID_OUT_l<-7000)
      Speed_PID_OUT_l = -7000;
  if(Speed_PID_OUT_l>7000)
      Speed_PID_OUT_l = 7000;
  Speed_Preverro_l =  Speed_Lasterro_l;
  Speed_Lasterro_l =  Speed_Erro_l;
  if(Speed_PID_OUT_l>=0)
  {
    // gpio_set_level(DIR_CH2,0);
    pwm_set_duty(PWM2,Speed_PID_OUT_l);//Speed_PID_OUT_l
    gpio_set_level(P00_2,0);
  }
  else
  {
    pwm_set_duty(PWM2,-Speed_PID_OUT_l);
    gpio_set_level(P00_2,1);
  }  
}
//右
void speed_PIDR(void)
{

  Speed_Encoder_r  =  speedr;//编码器采集当前速度
//gpt12_clear(COUNTER2_TIM);

  Speed_Erro_r =  Speed_Goal_r - Speed_Encoder_r;   //编码器反馈目标值与当前的差值

  Speed_PID_OUT_r += (Speed_P_r * (Speed_Erro_r - Speed_Lasterro_r)+
                      Speed_I_r *  Speed_Erro_r+
                      Speed_D_r * (Speed_Erro_r - 2 * Speed_Lasterro_r + Speed_Preverro_r));
  if(Speed_PID_OUT_r<-7000)
    Speed_PID_OUT_r = -7000;
  if(Speed_PID_OUT_r>7000)
    Speed_PID_OUT_r = 7000;
  Speed_Preverro_r =  Speed_Lasterro_r;
  Speed_Lasterro_r =  Speed_Erro_r;
  //pwm_set_duty(PWM1,0);
  if(Speed_PID_OUT_r>=0)
  {
    pwm_set_duty(PWM1,Speed_PID_OUT_r);
    gpio_set_level(P18_6,0);
  }
  else
  {
    pwm_set_duty(PWM1,-Speed_PID_OUT_r);
    gpio_set_level(P18_6,1);
  }
}

float ABS(float i)
{
  return i < 0 ? -1*i : i;
}

void steer_PID(void)
{
  //imu660ra_get_gyro();
  //angle=imu660ra_gyro_z/100;
  angle_erro=angle/100;
  uint8 m;
  for(int16 x=1;x<image_h-1;x++)
  {
    line[x]=center_line[x];
    m=hightest;
  }
  
  //element_processing();
  int16 n1=0,n2=0,n3=0;
  //int16 j1,j2,j3;
  uint8 i;
  uint8 y;
  /*for(y=points_r[1][1];y>hightest;y--)
  {
    ips200_draw_point(line[y], y, uesr_GREEN);
  }*/
  /*for(i=image_h-2;i>m;i--)
  {
    if(i>80)
    {
      n1+=(line[i]-94);
    }
    else if(i>41)
    {
      n2+=(line[i]-94);
    }
    else n3+=(line[i]-94);
  }
  if(n3>0)
  {
    n=(float)n1/39*0.65+(float)n2/39*0.25+(float)n3/(float)(41-i)*0.1;
  }
  else if(n2>0)
  {
    n=(float)n1/39*0.75+(float)n2/(float)(80-i)*0.25;
  }
  else n=(float)n1/(float)(119-i);*/
  /*for(i=image_h-2;i>m;i--)
  {
    if(i>29)
    {
      n1+=(line[i]-70);
    }
    else if(i>9)
    {
      n2+=(line[i]-70);
    }
    else n3+=(line[i]-70);
  }
  if(n3>0)
  {
    n=(float)n1/29*weighted1+(float)n2/20*weighted2+(float)n3/(float)(9-i)*weighted3;
  }
  else if(n2>0)
  {
    n=(float)n1/29*weighted12+(float)n2/(float)(29-i)*weighted22;
  }
  else n=n1/(58-i)*1.2;
  
  if(n>140)
  {
    n=140;
  }  
  steer_erro=n;*/
  
  for(i=image_h-2;i>m;i--)
  {
    if(i>58)
    {
      n1+=(line[i]-70);
    }
    else if(i>21)
    {
      n2+=(line[i]-70);
    }
    else n3+=(line[i]-70);
  }
  if(abs(n3)>0)
  {
    n=(float)n1/20*weighted1+(float)n2/37*weighted2+(float)n3/(21-i)*weighted3;
  }
  else if(abs(n2)>0)
  {
    n=(float)n1/20*weighted12+(float)n2/(58-i)*weighted22;
  }
  else n=(float)n1/(58-i);
  
  /*if(n>140)
  {
    n=140;
  }  */
  steer_erro=n;
  /*for(i=image_h-2;i>m;i--)
  {
    if(i>58)
    {
      n1+=(line[i]-60);
    }
    else if(i>21)
    {
      n2+=(line[i]-60);
    }
    else n3+=(line[i]-60);
  }
  if(abs(n3)>0)
  {
    n=(float)n1/20*weighted1+(float)n2/37*weighted2+(float)n3/(21-i)*weighted3;
  }
  else if(abs(n2)>0)
  {
    n=(float)n1/20*weighted12+(float)n2/(58-i)*weighted22;
  }
  else n=(float)n1/(78-i)*1.2;*/
  
  /*if(n>120)
  {
    n=120;
  }  */
  steer_erro=n;
  //ips200_show_float (0, 130, n, 8,3);
  //element_processing();
  /*steer=steer_P*(float)steer_erro+
        steer_D*(float)(steer_erro-steer_erro1)+
        steer_I*(float)(steer_erro-2*steer_erro1+steer_erro2);*/
  
  steer=steer_P*(float)steer_erro+
        steer_P2*steer_erro*ABS(steer_erro)+
        steer_D*(float)(steer_erro-steer_erro1)+
        steer_GKD*angle_erro;
  
  //steer_erro2=steer_erro1;
  steer_erro1=steer_erro;
  
  //element_processing();
}
/*void angle_PID(void)
{
  imu660ra_get_gyro();
  angle=imu660ra_gyro_z/100;
  angle_erro=steer+angle;
  angle_erro_sum+=angle_erro;
  if(angle_erro_sum>500)angle_erro_sum=500;
  if(angle_erro_sum<-500)angle_erro_sum=-500;
  speed_dif=angle_P*angle_erro+
            angle_I*angle_erro_sum+
            angle_D*(angle_erro-angle_erro1);
  angle_erro1=angle_erro;
}*/
/*void Start_gpio_init(void)
{
    gpio_init(P20_7,GPI,0,GPI_PULL_DOWN);
    gpio_init(P20_8, GPO, 0, GPO_PUSH_PULL); // P00_0初始化为GPIO功能、输出模式、输出高电平、推挽输出
}*/
uint16 L_black_to_white[2];
uint8 L_black_to_white_flag=0;
uint16 L_white_to_black[2];
uint8 L_white_to_black_flag=0;
uint16 L_black_to_white2[2];
uint8 L_black_to_white2_flag=0;
uint16 L_white_to_black2[2];
uint8 L_white_to_black2_flag=0;


uint16 R_black_to_white[2];
uint8 R_black_to_white_flag=0;
uint16 R_white_to_black[2];
uint8 R_white_to_black_flag=0;
uint16 R_black_to_white2[2];
uint8 R_black_to_white2_flag=0;
uint16 R_white_to_black2[2];
uint8 R_white_to_black2_flag=0;

uint16 U_l_black_to_white[2];
uint8 U_l_black_to_white_flag=0;
uint16 U_r_black_to_white[2];
uint8 U_r_black_to_white_flag=0;
uint16 U_l_white_to_black[2];
uint8 U_l_white_to_black_flag=0;
uint16 U_r_white_to_black[2];
uint8 U_r_white_to_black_flag=0;

/*uint16 U_black_to_white2[2];
uint8 U_black_to_white2_flag=0;
uint16 U_white_to_black2[2];
uint8 U_white_to_black2_flag=0;

uint16 D_white_to_black[2];
uint8 D_white_to_black_flag=0;
uint16 D_black_to_white2[2];
uint8 D_black_to_white2_flag=0;
uint16 D_white_to_black2[2];
uint8 D_white_to_black2_flag=0;*/
uint8 l_torus_status=6;
uint8 black=0;
uint8 r_torus_status=0;
uint8 l_ring_out=0;
uint8 r_ring_out=0;
uint8 flag=0;
void element_processing(void)//元素处理
{
  /*if(element==1)
  {
    Speed_decision160();
  }
  else Speed_decision120();*/
  
  switch (element)
  {
  case 1:
    element=0;
    break;
  case 2://左直角
    
      //int16 i;
      if(L_black_to_white_flag==1)
      {
        steer=-260;
        element=0;
        l_up_turn_left_point_flag=0;
        L_black_to_white_flag=0;
        gpio_set_level(P23_7 , 0);
        /*line_repair(points_l[1],L_black_to_white,line);
        L_black_to_white[0]=40;
        L_black_to_white[1]=0;*/
        //gpio_set_level(P23_7 , 0);
        break;
      }
      /*for(i=1;i<image_h-2;i++)
      {
        if(original_image[i][2]<=image_thereshold&&original_image[i+1][2]>image_thereshold)
        {
          L_white_to_black[0]=2;
          L_white_to_black[1]=i;
          L_white_to_black_flag=1;
          if(L_white_to_black_flag==1)
          {
            element=0;
            L_white_to_black_flag=0;
            line_repair(points_l[1],L_white_to_black,line);
            gpio_set_level(P23_7 , 0);
            break;
          }
        }
      }*/
    
    break;
  case 3://右直角
    
      //int i;
      if(R_black_to_white_flag==1)
      {
        steer=260;
        element=0;
        r_up_turn_right_point_flag=0;
        R_black_to_white_flag=0;
        gpio_set_level(P23_7 , 0);
        /*line_repair(points_r[1],R_black_to_white,line);
        R_black_to_white[0]=40;
        R_black_to_white[1]=0;*/
        //gpio_set_level(P23_7 , 0);
        break;
      }
      /*for(i=1;i<image_h-2;i++)
      {
        if(original_image[i][image_w-3]<=image_thereshold&&original_image[i+1][image_w-3]>image_thereshold)
        {
          R_white_to_black[0]=image_w-3;
          R_white_to_black[1]=i;
          R_white_to_black_flag=1;
          if(R_white_to_black_flag==1)
          {
            element=0;
            R_white_to_black_flag=0;
            line_repair(points_r[1],R_white_to_black,line);
            gpio_set_level(P23_7 , 0);
            break;
          }
        }
      }*/
    
    break;
  case 4:
    /*if(L_black_to_white_flag==1)
      {
        steer=-220;
        element=0;
        L_black_to_white_flag=0;
        break;
      }
    break;*/
  case 5:
    
    break;
  case 6://左环
    /*if(r_up_turn_bottom_right_point[1]>40)
    {
      int i;
      for(i=1;i<image_w-2;i++)
      {
        if(bin_image[3][i]==0&&bin_image[3][i+1]==255)
        {
          U_l_black_to_white[0]=i;
          U_l_black_to_white[1]=3;
          U_l_black_to_white_flag=1;
          break;
        }
      }
      if(U_l_black_to_white_flag==1)
      {
        line_repair(points_l[1],U_l_black_to_white,line);
        U_l_black_to_white_flag=0;
        element=0;
        break;
      }
    }*/
    
    if(r_top_right_turn_down_point[1]>40)
    {
      gpio_set_level(P23_7 , 1);
      l_torus_status=1;
    }
    if(l_torus_status==1&&finding_point==0)
    {
      U_l_white_to_black_flag=0;
      black=1;
      for(int i=image_w-2;i>1;i--)
      {
        if(original_image[1][i]<=image_thereshold&&original_image[1][i+1]>image_thereshold)
        {
          U_l_white_to_black_flag++;
        }
      }
      for(int i=image_h-1;i>0;i--)
      {
        if(original_image[i][image_w-3]>image_thereshold)
        {
          black=0;
        }
      }
      if(U_l_white_to_black_flag==1&&black==1)
      {
        int8 i;
        u_l=0;
        u_r=0;
        d_r=0;
        d_l=0;
        for(i=hightest+1;i<hightest+6;i++)
        {
          u_l+=l_border[i];
          u_r+=r_border[i];
        }
        for(i=image_h-3;i>image_h-8;i--)
        {
          d_r+=l_border[i];
          d_l+=r_border[i];
        }
        if(my_abs((u_l/5+u_r/5)/2-(d_r/5+d_l/5)/2)<=5)
        {
          finding_point=1;
          l_torus_status=2;
          l_ring_out=0;
        }
      }
    }
    if(l_torus_status==2&&finding_point==1)
    {
      if(l_top_left_down_point[1]>40)
      {
        l_ring_out=1;
      }
      if(l_ring_out==1)
      {
        black=2;
        for(int i=image_h-1;i>0;i--)
        {
          if(original_image[i][2]>image_thereshold)
          {
            black=0;
          }
        }
        if(black==2)
        {
          int8 i;
          u_l=0;
          u_r=0;
          d_r=0;
          d_l=0;
          for(i=2;i<7;i++)
          {
            u_l+=l_border[i];
            u_r+=r_border[i];
            d_r+=l_border[image_h-1-i];
            d_l+=r_border[image_h-1-i];
          }
          /*for(i=image_h-3;i>image_h-8;i--)
          {
            d_r+=l_border[i];
            d_l+=r_border[i];
          }*/
          if(my_abs((u_l/5+u_r/5)/2-(d_r/5+d_l/5)/2)<=5)
          {
            l_torus_status=4;
            finding_point=0;
            l_ring_out=0;
            r_top_right_turn_down_point_flag=0;
            l_top_left_down_point_flag=0;
            element=0;
            gpio_set_level(P23_7 , 0);
          }
        }
      }
    }
   /* if(l_torus_status==3&&finding_point==1)
    {
      black=2;
      for(int i=image_h-1;i>0;i--)
      {
        if(bin_image[i][2]==255)
        {
          black=0;
        }
      }
      if(black==2)
      {
        int8 i;
        u_l=0;
        u_r=0;
        d_r=0;
        d_l=0;
        for(i=hightest+1;i<hightest+6;i++)
        {
          u_l+=l_border[i];
          u_r+=r_border[i];
        }
        for(i=58;i>53;i--)
        {
          d_r+=l_border[i];
          d_l+=r_border[i];
        }
        if(my_abs((u_l/5+u_r/5)/2-(d_r/5+d_l/5)/2)<=5)
        {
          l_torus_status=4;
          finding_point=0;
          r_top_right_turn_down_point_flag=0;
          l_top_left_down_point_flag=0;
          element=0;
        }
      }
    }*/
    
    //element=0;
    break;
  case 7://右环
    /*if(r_up_turn_bottom_right_point[1]>40)
    {
      int i;
      for(i=image_w-2;i>0;i--)
      {
        if(bin_image[3][i]==0&&bin_image[3][i-1]==255)
        {
          U_r_black_to_white[0]=i;
          U_r_black_to_white[1]=3;
          U_r_black_to_white_flag=1;
          break;
        }
      }
      if(U_r_black_to_white_flag==1)
      {
        line_repair(points_r[1],U_r_black_to_white,line);
        U_r_black_to_white_flag=0;
        element=0;
        break;
      }
    }*/
    if(r_up_turn_bottom_right_point[1]>20)
    {
      finding_point=1;
      r_torus_status=1;
      gpio_set_level(P23_7 , 1);
    }
    if(r_torus_status==1&&finding_point==1)
    {
      /*U_r_white_to_black_flag=0;
      black=0;
      for(int i=2;i<image_w-2;i++)
      {
        if(bin_image[1][i]==0&&bin_image[1][i-1]==255)
        {
          U_r_white_to_black_flag++;
        }
      }
      for(int i=image_h-1;i>0;i--)
      {
        if(bin_image[i][2]==255)
        {
          black=1;
        }
      }
      if(U_r_white_to_black_flag==1&&black==0)
      {
        int8 i;
        u_l=0;
        u_r=0;
        d_r=0;
        d_l=0;
        for(i=hightest+1;i<hightest+6;i++)
        {
          u_l+=l_border[i];
          u_r+=r_border[i];
        }
        for(i=58;i>53;i--)
        {
          d_r+=l_border[i];
          d_l+=r_border[i];
        }
        if(my_abs((u_l/5+u_r/5)/2-(d_r/5+d_l/5)/2)<=5)
        {
          black=2;
        }
      }*/
      yaw=0;
      yaw_gyro=0;
      yaw_fused=0;
      total_encoder_l=0;
      total_encoder_l2=0;
      total_encoder_r=0;
      total_encoder_r2=0;
      r_torus_status=2;
    }
    if(r_torus_status==2&&finding_point==1)
    {
      updeta_yaw_fused(total_encoder_l,total_encoder_r);
      if(yaw_fused>100)
      {
        gpio_set_level(P23_7 , 0);
        r_up_turn_bottom_right_point_flag=0;
        r_torus_status=0;
        finding_point=0;
        element=0;
        gpio_set_level(P23_7 , 0);
      }
    }
    
    //element=0;
    break;
  case 8:
    
    break;
  case 9:
    
    break;
  }
}

float calc_encoder_yaw(int16 left_cnt,int16 right_cnt)
{
  double left_dist=(left_cnt-total_encoder_l2)*diameter;
  total_encoder_l2=left_cnt;
  double right_dist=(right_cnt-total_encoder_r2)*diameter;
  total_encoder_r2=right_cnt;
  double delta_encoder_yaw=57.32*(left_dist-right_dist)/wheel_base;
  yaw+=delta_encoder_yaw;
  /*if(yaw>3.14)yaw-=2*3.14;
  else if(yaw<-3.14)yaw+=2*3.14;*/
  return yaw;
}
float updeta_yaw_fused(int16 left_cnt,int16 right_cnt)
{
  //a=0.9;
  //dt=0.0006;
  encoder_yaw=calc_encoder_yaw(left_cnt,right_cnt);
  if(kalmanupdate(angle)*dt<-0.1||kalmanupdate(angle)*dt>0.1)
  {
    //yaw_gyro=yaw_fused+angle*dt;
    yaw_gyro=yaw_fused+kalmanupdate(angle)*dt;
  }
  //yaw_gyro=yaw_fused+kalmanupdate(angle)*dt;
  yaw_fused=a*yaw_gyro+(1-a)*encoder_yaw;
  /*if(yaw_fused>3.14)yaw_fused-=2*3.14;
  else if(yaw_fused<-3.14)yaw_fused+=2*3.14;*/
  return yaw_fused;
}
float kalmanupdate(float gyro_measure)
{
  float compensated_measure=gyro_measure-bias;
  P=P+Q;
  K=P/(P+R);
  angle_rate=angle_rate+K*(compensated_measure-angle_rate);
  P=(1-K)*P;
  float residual=compensated_measure-angle_rate;
  bias=bias_q*residual;
  return angle_rate;
}

