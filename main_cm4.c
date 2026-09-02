/*********************************************************************************************************************
* CYT2BL3 Opensourec Library 即（ CYT2BL3 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT2BL3 开源库的一部分
*
* CYT2BL3 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main_cm4
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT2BL3
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-11-19       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "image.h"
#include "speed.h"
#include "serial_port.h"
#include "key.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// *************************** 例程硬件连接说明 ***************************
// 接入总钻风灰度数字摄像头 对应主板摄像头接口 请注意线序
//      模块管脚            单片机管脚
//      TXD                 查看 zf_device_mt9v03x.h 中 MT9V03X_COF_UART_TX 宏定义
//      RXD                 查看 zf_device_mt9v03x.h 中 MT9V03X_COF_UART_RX 宏定义
//      PCLK                查看 zf_device_mt9v03x.h 中 MT9V03X_PCLK_PIN 宏定义
//      VSY                 查看 zf_device_mt9v03x.h 中 MT9V03X_VSYNC_PIN 宏定义
//      D0-D7               查看 zf_device_mt9v03x.h 中 MT9V03X_DATA_PIN 宏定义 从该定义开始的连续八个引脚
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源
// 
//      单排排针 SPI 两寸屏 硬件引脚
//      SCL                 查看 zf_device_ips200.h 中 IPS200_SCL_PIN_SPI  宏定义  B0
//      SDA                 查看 zf_device_ips200.h 中 IPS200_SDA_PIN_SPI  宏定义  B1
//      RST                 查看 zf_device_ips200.h 中 IPS200_RST_PIN_SPI  宏定义  B2
//      DC                  查看 zf_device_ips200.h 中 IPS200_DC_PIN_SPI   宏定义  C19
//      CS                  查看 zf_device_ips200.h 中 IPS200_CS_PIN_SPI   宏定义  B3 
//      BL                  查看 zf_device_ips200.h 中 IPS200_BLk_PIN_SPI  宏定义  C18
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源

// *************************** 例程测试说明 ***************************
// 1.核心板烧录完成本例程 将核心板插在主板上 插到底
// 
// 2.摄像头接在主板的摄像头接口 注意线序1.8寸TFT模块插入主板屏幕接口
// 
// 3.主板上电 或者核心板链接完毕后上电 核心板按下复位按键
// 
// 4.屏幕会显示初始化信息然后显示摄像头图像
// 
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查

// **************************** 代码区域 ****************************
#define IPS200_TYPE     (IPS200_TYPE_SPI)                                // 单排排针 SPI 两寸屏 这里宏定义填写 IPS200_TYPE_SPI
uint16 copy[400]={0};
uint16 copy_n=0;
//extern float yaw_fused;//融合偏航角
//extern float yaw;
int b=0;
uint8 x=0;
uint8 key1_last_flag;

int main(void)
{
    clock_init(SYSTEM_CLOCK_160M); 	// 时钟配置及系统初始化<务必保留>
    debug_info_init();                  // 调试串口信息初始化
    
    //SPI_init();
    // 此处编写用户代码 例如外设初始化代码等
    ips200_init(IPS200_TYPE);
    gpio_init(P23_7, GPO, 0, GPO_PUSH_PULL);
    MOTOR_init();
    Encoder_init();
    Speed_decision180();
    imu660ra_init();
    system_delay_ms (50);
    timer_init(TC_TIME2_CH0, TIMER_US);
    KEY_init();
    pit_ms_init(PIT_CH0, 10);  //中断10ms
    system_delay_ms (10);
   // pit_ms_init(PIT_CH1, 10);   
    //system_delay_ms (50);
    //pit_ms_init(PIT_CH2, 10);
    /*ips200_show_string(0, 0, "mt9v03x init.");
    // 此处编写用户代码 例如外设初始化代码等
    while(1)
    {
        if(mt9v03x_init())
            ips200_show_string(0, 16, "mt9v03x reinit.");
        else
            break;
        system_delay_ms(500);                                                   // 闪灯表示异常
    }

    ips200_show_string(0, 16, "init success.");
    ips200_clear();*/
    
    // 此处编写用户代码 例如外设初始化代码等
    while(1)
    {
        if(mt9v03x_init())
            gpio_toggle_level(LED1);                                            // 翻转 LED 引脚输出电平 控制 LED 亮灭 初始化出错这个灯会闪的很慢
        else
            break;
        system_delay_ms(500);                                                   // 闪灯表示异常
    }
    
    while(true)
    {
        // 此处编写需要循环执行的代码
      
      if(gpio_get_level(KEY1)==1 && key1_last_flag==0)
        {
          
        }
        key1_last_flag=gpio_get_level(KEY1);
      
        if(mt9v03x_finish_flag)
        {
          /*if(b!=100)
          {
            image_process();
            steer_PID();
            //speed_processing();
            if(b<150)
            {
              b++;
            }
          }
          if(b==100)
          {
            timer_start(TC_TIME2_CH0);
            image_process();
            steer_PID();
            //speed_processing();
            timer_stop(TC_TIME2_CH0);
            //b=1;
            b++;
          }*/
          image_process();
          steer_PID();//线
          //speed_processing();
          //angle_PID();
          mt9v03x_finish_flag = 0;
          //line_copy2();
          //line_copy();
          // 发送图像
          //seekfree_assistant_camera_send();
        }
        //steer_PID();
      
        // 此处编写需要循环执行的代码     
       
        //ips200_show_uint (0, 130, flag, 8);
        //ips200_show_uint (0, 130, element, 8);
        //ips200_show_uint (0, 145, finding_point, 8);
        //ips200_show_uint (80, 145, l_top_left_down_point_flag, 8);
        //ips200_show_uint (0, 160, r_torus_status, 8);
        //ips200_show_uint (80, 160, l_torus_status, 8);
        //ips200_show_uint (0, 175, black, 8);
        
        //ips200_show_uint (80, 145,  timer_get(TC_TIME2_CH0), 8);
        //ips200_show_float (0, 130, yaw, 8,3);//*57.32
        //ips200_show_float (0, 145, yaw_fused, 8,3);
        //ips200_show_int (0, 160, total_encoder_l, 8);
        //ips200_show_int (0, 175, total_encoder_r, 8);
        //ips200_show_float (0, 190, kalmanupdate(angle)*0.006, 8, 3);
        //ips200_show_float(0, 205,  yaw_gyro, 8,3);
        //ips200_show_uint(0, 160, hightest, 6);
        /*ips200_show_int(0, 175, Speed_Erro_l, 8);
        ips200_show_int(0, 190, Speed_Erro_r, 8);
        ips200_show_int(80, 175, speedl, 8);
        ips200_show_int(80, 190, speedr, 8);
        ips200_show_int(0, 205, Speed_PID_OUT_l, 8);
        ips200_show_int(0, 220, Speed_PID_OUT_r, 8);*/
        //ips200_show_uint (0, 130, L_Straight_Flag, 8);
        //ips200_show_uint (80, 130, R_Straight_Flag, 8);
        //ips200_show_float (0, 145, L_Straightaway_Lope_Rate_A, 8,3);
        //ips200_show_float (0, 160, L_Straightaway_Lope_Rate_B, 8,3);
        //ips200_show_float (0, 175, L_Straightaway_Lope_Rate_C, 8,3);
        //ips200_show_float (80, 145, R_Straightaway_Lope_Rate_A, 8,3);
        //ips200_show_float (80, 160, R_Straightaway_Lope_Rate_B, 8,3);
        //ips200_show_float (80, 175, R_Straightaway_Lope_Rate_C, 8,3);
        
        //printf("%d,%d\n",Speed_PID_OUT_l,Speed_PID_OUT_r);
        /*if(x==100)
          {
            for(uint16 i=0;i<data_stastics_l;i++)
            {
              copy[i]=dir_l[i];
              
              printf("%d\n",copy[i]);
            }
            //x=0;
          }*/
        
        //line_copy2();
        //line_copy();
        // 发送图像
        //seekfree_assistant_camera_send();

    }
}

// **************************** 代码区域 ****************************
// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
// 
// 问题1：屏幕不显示
//      如果使用主板测试，主板必须要用电池供电 检查屏幕供电引脚电压
//      检查屏幕是不是插错位置了 检查引脚对应关系
//      如果对应引脚都正确 检查一下是否有引脚波形不对 需要有示波器
//      无法完成波形测试则复制一个GPIO例程将屏幕所有IO初始化为GPIO翻转电平 看看是否受控
// 
// 问题2：显示 reinit 字样
//      检查接线是否正常
//      主板供电是否使用电量充足的电池供电
// 
// 问题2：显示图像杂乱 错位
//      检查摄像头信号线是否有松动
