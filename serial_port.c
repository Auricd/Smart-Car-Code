#include "zf_common_headfile.h"
#include "serial_port.h"
#include "image.h"

extern uint8 original_image[image_h][image_w];
extern uint8 bin_image[image_h][image_w];
extern uint16 points_l[(uint16)USE_num][2];//左线
extern uint16 points_r[(uint16)USE_num][2];//右线
extern uint16 r_border[image_h];//一维边线右
extern uint16 l_border[image_h];//一维边线左
extern uint16 center_line[image_h];//中线
extern uint16 data_stastics_l ;
extern uint16 data_stastics_r ;
uint16 l_line_x[(uint16)USE_num],r_line_x[(uint16)USE_num],c_line_x[image_h];//x
uint16 l_line_y[(uint16)USE_num],r_line_y[(uint16)USE_num],c_line_y[image_h];//y

void line_copy(void)
{
  int32 i=0;
  for(i=0;i<image_h;i++)
  {
    //x
    l_line_x[i]=l_border[i];
    r_line_x[i]=r_border[i];
    c_line_x[i]=center_line[i];
    //y
    l_line_y[i]=i;
    r_line_y[i]=i;
    c_line_y[i]=i;
  }
}

void line_copy2(void)
{
  int i=0;
  int j=0;
  for(i=0;i<data_stastics_l;i++)
  {
    l_line_x[i]=points_l[i][0];
    l_line_y[i]=points_l[i][1];
  }
  for(i=0;i<data_stastics_r;i++)
  {
    r_line_x[i]=points_r[i][0];
    r_line_y[i]=points_r[i][1];
  }
  for(j=0;j<image_h;j++)
  {
    c_line_x[i]=center_line[i];
    c_line_y[i]=i;
  }
}

void SPI_init(void)
{
  gpio_init(LED1, GPO, GPIO_LOW, GPO_PUSH_PULL);                                // 初始化 LED1 输出 默认高电平 推挽输出模式
    
    // 此处编写用户代码 例如外设初始化代码等
	if(wireless_uart_init())                                                    // 判断是否通过初始化
    {
        while(1)                                                                // 初始化失败就在这进入死循环
        {
            gpio_toggle_level(LED1);                                            // 翻转 LED 引脚输出电平 控制 LED 亮灭
            system_delay_ms(100);                                               // 短延时快速闪灯表示异常
        }
    }
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);
    
    //line_copy();
    line_copy2();
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, bin_image[0], image_w, image_h);
    seekfree_assistant_camera_boundary_config(XY_BOUNDARY, USE_num, l_line_x, r_line_x, c_line_x, l_line_y, r_line_y, c_line_y);
}

