#include "zf_common_headfile.h"
#include "image.h"
#include "speed.h"


//图像参数
uint8 original_image[image_h][image_w];//灰度图像
uint8 image_thereshold=175;//图像分割阈值
uint8 Threshold = 0;//动态阈值
uint8 bin_image[image_h][image_w];//图像数组
uint8 start_point_l[2] = { 0 };//左边起点的x，y值
uint8 start_point_r[2] = { 0 };//右边起点的x，y值
uint16 points_l[(uint16)USE_num][2] = { {  0 } };//左线
uint16 points_r[(uint16)USE_num][2] = { {  0 } };//右线
uint16 dir_r[(uint16)USE_num] = { 0 };//用来存储右边生长方向
uint16 dir_l[(uint16)USE_num] = { 0 };//用来存储左边生长方向
uint16 data_stastics_l = 0;//统计左边找到点的个数
uint16 data_stastics_r = 0;//统计右边找到点的个数
uint8 hightest = 0;//最高点
uint16 r_border[image_h] = { 0 };//一维边线右
uint16 l_border[image_h] = { 0 };//一维边线左
uint16 center_line[image_h]={0};//中线
uint8 finding_point=0;
//元素处理
//左
//上转左点
uint8 l_up_turn_left_point[2]={0};//坐标
uint8 l_up_turn_left_point_position=0;//点数
uint8 l_up_turn_left_point_flag=0;//标志位
//右转上点
uint8 l_right_turn_up_point[2]={0};//坐标
uint8 l_right_turn_up_point_position=0;//点数
uint8 l_right_turn_up_point_flag=0;//标志位
//上转右点
uint8 l_up_turn_right_point[2]={0};
uint8 l_up_turn_right_point_position=0;
uint8 l_up_turn_right_point_flag=0;
//上转左下点
uint8 l_up_turn_bottom_left_point[2]={0};
uint8 l_up_turn_bottom_left_point_position=0;
uint8 l_up_turn_bottom_left_point_flag=0;
//左上转下点
uint8 l_top_left_down_point[2]={0};
uint8 l_top_left_down_point_position=0;
uint8 l_top_left_down_point_flag=0;
//右上转左上点
uint8 l_top_right_turn_top_left_point[2]={0};
uint8 l_top_right_turn_top_left_point_position=0;
uint8 l_top_right_turn_top_left_point_flag=0;
//右下转上点
uint8 l_bottom_right_turn_up_point[2]={0};
uint8 l_bottom_right_turn_up_point_position=0;
uint8 l_bottom_right_turn_up_point_flag=0;

//右
//上转右点
uint8 r_up_turn_right_point[2]={0};//坐标
uint8 r_up_turn_right_point_position=0;//点数
uint8 r_up_turn_right_point_flag=0;//标志位
//左转上点
uint8 r_left_turn_up_point[2]={0};//坐标
uint8 r_left_turn_up_point_position=0;//点数
uint8 r_left_turn_up_point_flag=0;//标志位
//上转左点
uint8 r_up_turn_left_point[2]={0};
uint8 r_up_turn_left_point_position=0;
uint8 r_up_turn_left_point_flag=0;
//上转右下点
uint8 r_up_turn_bottom_right_point[2]={0};
uint8 r_up_turn_bottom_right_point_position=0;
uint8 r_up_turn_bottom_right_point_flag=0;
//右上转下点
uint8 r_top_right_turn_down_point[2]={0};
uint8 r_top_right_turn_down_point_position=0;
uint8 r_top_right_turn_down_point_flag=0;
//左上转右上点
uint8 r_top_left_turn_top_right_point[2]={0};
uint8 r_top_left_turn_top_right_point_position=0;
uint8 r_top_left_turn_top_right_point_flag=0;
//左上转上点
uint8 r_bottom_left_turn_up_point[2]={0};
uint8 r_bottom_left_turn_up_point_position=0;
uint8 r_bottom_left_turn_up_point_flag=0;

//斜率和截距
float L_Min_Slope_Dif_Thre=0.15;
float L_Straightaway_Lope_Rate_A=0;
float L_Straightaway_Lope_Rate_B=0;
float L_Straightaway_Lope_Rate_C=0;
float L_Intercept=0;
uint8 L_Straight_Flag=0;
float R_Min_Slope_Dif_Thre = 0.15;    //实测得出，当两段线段的斜率低于这个阈值时，可认为斜率相等
float R_Straightaway_Lope_Rate_A = 0;   //第一段的斜率
float R_Straightaway_Lope_Rate_B = 0;   //第二段的斜率
float R_Straightaway_Lope_Rate_C = 0;   //整段边线的斜率
float R_Intercept = 0;      //整段边线的截距
uint8 R_Straight_Flag = 0;      //右侧边线是否为直线标志位

uint8 element=0;//元素处理标志位
int16 u_r=0;
int16 u_l=0;
int16 d_r=0;
int16 d_l=0;


//定义膨胀和腐蚀的阈值区间
#define threshold_max   (image_thereshold+1)*5//255*5//此参数可根据自己的需求调节
#define threshold_min   image_thereshold*2//255*2//此参数可根据自己的需求调节
/*void image_filter(uint8(*original_image)[image_w])//形态学滤波，简单来说就是膨胀和腐蚀的思想
{
    uint16 i, j;
    uint32 num = 0;

    //original_image*/
    /*for (i = 1; i < image_h - 1; i++)
    {
        for (j = 1; j < (image_w - 1); j++)
        {
            //统计八个方向的像素值
            num =bin_image[i - 1][j - 1] + bin_image[i - 1][j] + bin_image[i - 1][j + 1]+ bin_image[i][j - 1] + bin_image[i][j + 1]+ bin_image[i + 1][j - 1] + bin_image[i + 1][j] + bin_image[i + 1][j + 1];
            if (num >= threshold_max && bin_image[i][j] <=image_thereshold)
            {

                bin_image[i][j] = image_thereshold+1;//白  可以搞成宏定义，方便更改
            }
            if (num <= threshold_min && bin_image[i][j] >image_thereshold)
            {

                bin_image[i][j] = image_thereshold;//黑
            }
        }
    }*/
    /*for(i=1;i<data_stastics_l;i++)
    {
      num=original_image[(uint8)points_l[i][1]-1][(uint8)points_l[i][0]-2]+original_image[(uint8)points_l[i][1]-1][(uint8)points_l[i][0]-1]+original_image[(uint8)points_l[i][1]-1][(uint8)points_l[i][0]]
         +original_image[(uint8)points_l[i][1]][(uint8)points_l[i][0]]+original_image[(uint8)points_l[i][1]+1][(uint8)points_l[i][0]]+original_image[(uint8)points_l[i][1]+1][(uint8)points_l[i][0]-1]
         +original_image[(uint8)points_l[i][1]+1][(uint8)points_l[i][0]-2]+original_image[(uint8)points_l[i][1]][(uint8)points_l[i][0]-2];
      if(num>=threshold_max&& original_image[(uint8)points_l[i][1]][(uint8)points_l[i][0]-1] <=image_thereshold)
      {
        original_image[(uint8)points_l[i][1]][(uint8)points_l[i][0]-1]=image_thereshold+1;
      }
      if(num<=threshold_min&&original_image[(uint8)points_l[i][1]][(uint8)points_l[i][0]-1] >image_thereshold)
      {
        original_image[(uint8)points_l[i][1]][(uint8)points_l[i][0]-1]=image_thereshold-1;
      }
    }
    for(i=1;i<data_stastics_r;i++)
    {
      num=original_image[(uint8)points_r[i][1]-1][(uint8)points_r[i][0]]+original_image[(uint8)points_r[i][1]-1][(uint8)points_r[i][0]+1]+original_image[(uint8)points_r[i][1]-1][(uint8)points_r[i][0]+2]
         +original_image[(uint8)points_r[i][1]][(uint8)points_r[i][0]+2]+original_image[(uint8)points_r[i][1]+1][(uint8)points_r[i][0]+2]+original_image[(uint8)points_r[i][1]+1][(uint8)points_r[i][0]+1]
         +original_image[(uint8)points_r[i][1]+1][(uint8)points_r[i][0]]+original_image[(uint8)points_r[i][1]][(uint8)points_r[i][0]];
      if(num>=threshold_max&& original_image[(uint8)points_r[i][1]][(uint8)points_r[i][0]+1] <=image_thereshold)
      {
        original_image[(uint8)points_r[i][1]][(uint8)points_r[i][0]+1]=image_thereshold+1;
      }
      if(num<=threshold_min&&original_image[(uint8)points_r[i][1]][(uint8)points_r[i][0]+1] >image_thereshold)
      {
        original_image[(uint8)points_r[i][1]][(uint8)points_r[i][0]+1]=image_thereshold-1;
      }
    }
    
    for(i=1;i<data_stastics_l;i++)
    {
      num=original_image[(uint8)points_l[i][1]-1][(uint8)points_l[i][0]-1]+original_image[(uint8)points_l[i][1]-1][(uint8)points_l[i][0]]+original_image[(uint8)points_l[i][1]-1][(uint8)points_l[i][0]+1]
         +original_image[(uint8)points_l[i][1]][(uint8)points_l[i][0]+1]+original_image[(uint8)points_l[i][1]+1][(uint8)points_l[i][0]+1]+original_image[(uint8)points_l[i][1]+1][(uint8)points_l[i][0]]
         +original_image[(uint8)points_l[i][1]+1][(uint8)points_l[i][0]-1]+original_image[(uint8)points_l[i][1]][(uint8)points_l[i][0]-1];
      if(num>=threshold_max&& original_image[(uint8)points_l[i][1]][(uint8)points_l[i][0]] <=image_thereshold)
      {
        original_image[(uint8)points_l[i][1]][(uint8)points_l[i][0]]=image_thereshold+1;
      }
      if(num<=threshold_min&&original_image[(uint8)points_l[i][1]][(uint8)points_l[i][0]] >image_thereshold)
      {
        original_image[(uint8)points_l[i][1]][(uint8)points_l[i][0]]=image_thereshold-1;
      }
    }
    for(i=1;i<data_stastics_r;i++)
    {
      num=original_image[(uint8)points_r[i][1]-1][(uint8)points_r[i][0]-1]+original_image[(uint8)points_r[i][1]-1][(uint8)points_r[i][0]]+original_image[(uint8)points_r[i][1]-1][(uint8)points_r[i][0]+1]
         +original_image[(uint8)points_r[i][1]][(uint8)points_r[i][0]+1]+original_image[(uint8)points_r[i][1]+1][(uint8)points_r[i][0]+1]+original_image[(uint8)points_r[i][1]+1][(uint8)points_r[i][0]]
         +original_image[(uint8)points_r[i][1]+1][(uint8)points_r[i][0]-1]+original_image[(uint8)points_r[i][1]][(uint8)points_r[i][0]-1];
      if(num>=threshold_max&& original_image[(uint8)points_r[i][1]][(uint8)points_r[i][0]] <=image_thereshold)
      {
        original_image[(uint8)points_r[i][1]][(uint8)points_r[i][0]]=image_thereshold+1;
      }
      if(num<=threshold_min&&original_image[(uint8)points_r[i][1]][(uint8)points_r[i][0]] >image_thereshold)
      {
        original_image[(uint8)points_r[i][1]][(uint8)points_r[i][0]]=image_thereshold-1;
      }
    }
    
    for(i=1;i<data_stastics_l;i++)
    {
      num=original_image[(uint8)points_l[i][1]-1][(uint8)points_l[i][0]]+original_image[(uint8)points_l[i][1]-1][(uint8)points_l[i][0]+1]+original_image[(uint8)points_l[i][1]-1][(uint8)points_l[i][0]+2]
         +original_image[(uint8)points_l[i][1]][(uint8)points_l[i][0]+2]+original_image[(uint8)points_l[i][1]+1][(uint8)points_l[i][0]+2]+original_image[(uint8)points_l[i][1]+1][(uint8)points_l[i][0]+1]
         +original_image[(uint8)points_l[i][1]+1][(uint8)points_l[i][0]]+original_image[(uint8)points_l[i][1]][(uint8)points_l[i][0]];
      if(num>=threshold_max&& original_image[(uint8)points_l[i][1]][(uint8)points_l[i][0]+1] <=image_thereshold)
      {
        original_image[(uint8)points_l[i][1]][(uint8)points_l[i][0]+1]=image_thereshold+1;
      }
      if(num<=threshold_min&&original_image[(uint8)points_l[i][1]][(uint8)points_l[i][0]+1] >image_thereshold)
      {
        original_image[(uint8)points_l[i][1]][(uint8)points_l[i][0]+1]=image_thereshold-1;
      }
    }
    for(i=1;i<data_stastics_r;i++)
    {
      num=original_image[(uint8)points_r[i][1]-1][(uint8)points_r[i][0]-2]+original_image[(uint8)points_r[i][1]-1][(uint8)points_r[i][0]-1]+original_image[(uint8)points_r[i][1]-1][(uint8)points_r[i][0]]
         +original_image[(uint8)points_r[i][1]][(uint8)points_r[i][0]]+original_image[(uint8)points_r[i][1]+1][(uint8)points_r[i][0]]+original_image[(uint8)points_r[i][1]+1][(uint8)points_r[i][0]-1]
         +original_image[(uint8)points_r[i][1]+1][(uint8)points_r[i][0]-2]+original_image[(uint8)points_r[i][1]][(uint8)points_r[i][0]-2];
      if(num>=threshold_max&& original_image[(uint8)points_r[i][1]][(uint8)points_r[i][0]-1] <=image_thereshold)
      {
        original_image[(uint8)points_r[i][1]][(uint8)points_r[i][0]-1]=image_thereshold+1;
      }
      if(num<=threshold_min&&original_image[(uint8)points_r[i][1]][(uint8)points_r[i][0]-1] >image_thereshold)
      {
        original_image[(uint8)points_r[i][1]][(uint8)points_r[i][0]-1]=image_thereshold-1;
      }
    }
}*/

void image_filter(uint8(*original_image)[image_w])//形态学滤波，简单来说就是膨胀和腐蚀的思想
{
    uint16 i, j;
    uint32 num = 0;

    //bin_image
    for (i = 1; i < image_h - 1; i++)
    {
        for (j = 1; j < (image_w - 1); j++)
        {
            //统计八个方向的像素值
            num =original_image[i - 1][j - 1] + original_image[i - 1][j] + original_image[i - 1][j + 1]+ original_image[i][j - 1] + original_image[i][j + 1]+ original_image[i + 1][j - 1] + original_image[i + 1][j] + original_image[i + 1][j + 1];
            if (num >= threshold_max && original_image[i][j] <=image_thereshold)
            {

                original_image[i][j] = image_thereshold+1;//白  可以搞成宏定义，方便更改
            }
            if (num <= threshold_min && original_image[i][j] >image_thereshold)
            {

                original_image[i][j] = image_thereshold;//黑
            }
        }
    }
}

/*void image_filter(uint8(*bin_image)[image_w])//形态学滤波，简单来说就是膨胀和腐蚀的思想
{
    uint16 i, j;
    uint32 num = 0;


    for (i = 1; i < image_h - 1; i++)
    {
        for (j = 1; j < (image_w - 1); j++)
        {
            //统计八个方向的像素值
            num =bin_image[i - 1][j - 1] + bin_image[i - 1][j] + bin_image[i - 1][j + 1]+ bin_image[i][j - 1] + bin_image[i][j + 1]+ bin_image[i + 1][j - 1] + bin_image[i + 1][j] + bin_image[i + 1][j + 1];
            if (num >= threshold_max && bin_image[i][j] == 0)
            {

                bin_image[i][j] = 255;//白  可以搞成宏定义，方便更改
            }
            if (num <= threshold_min && bin_image[i][j] == 255)
            {

                bin_image[i][j] = 0;//黑
            }
        }
    }
}*/

void image_draw_rectan(uint8(*image)[image_w])//画黑框
{

    uint8 i = 0;
    for (i = 0; i < image_h; i++)
    {
        image[i][0] = 0;
        image[i][1] = 0;
        image[i][image_w - 1] = 0;
        image[i][image_w - 2] = 0;

    }
    for (i = 0; i < image_w; i++)
    {
        image[0][i] = 0;
        //image[1][i] = 0;
        image[image_h-1][i] = 0;

    }
}

int my_abs(int value)//求绝对值
{
if(value>=0) return value;
else return -value;
}
int16 limit_a_b(int16 x, int a, int b)
{
    if(x<a) x = a;
    if(x>b) x = b;
    return x;
}

int16 limit1(int16 x, int16 y)//求最小值
{
    if (x > y)             return y;
    else if (x < -y)       return -y;
    else                return x;
}

/*void crop_image(uint8(*mt9v03x_image)[image_W])//图像裁剪//60*140
{
  uint8 i = 0, j = 0,h=0,w=0;
  for(i=40;i<image_H-20;i++)
  {
    for(j=24;j<image_W-24;j++)
    {
      original_image[h][w]=mt9v03x_image[i][j];
      w++;
    }
    w=0;
    h++;
  }
}*/
/*void crop_image(uint8(*mt9v03x_image)[image_W])//图像裁剪//80*120
{
  uint8 i = 0, j = 0,h=0,w=0;
  for(i=30;i<image_H-10;i++)
  {
    for(j=34;j<image_W-34;j++)
    {
      original_image[h][w]=mt9v03x_image[i][j];
      w++;
    }
    w=0;
    h++;
  }
}*/
void crop_image(uint8(*mt9v03x_image)[image_W])//图像裁剪//80*140
{
  uint8 i = 0, j = 0,h=0,w=0;
  for(i=25;i<image_H-15;i++)
  {
    for(j=24;j<image_W-24;j++)
    {
      original_image[h][w]=mt9v03x_image[i][j];
      w++;
    }
    w=0;
    h++;
  }
}

void Get_image(uint8(*mt9v03x_image)[image_w])//获得灰度图像
{
#define use_num     1   //1就是不压缩，2就是压缩一倍
    uint8 i = 0, j = 0, row = 0, line = 0;
    for (i = 0; i < image_h; i += use_num)          //
    {
        for (j = 0; j <image_w; j += use_num)     //
        {
            original_image[row][line] = mt9v03x_image[i][j];//这里的参数填写你的摄像头采集到的图像
            line++;
        }
        line = 0;
        row++;
    }
}

uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row)//求动态阈值
{
#define GrayScale 256
    uint16 Image_Width  = col;
    uint16 Image_Height = row;
    int X; uint16 Y;
    uint8* data = image;
    int HistGram[GrayScale] = {0};

    uint32 Amount = 0;
    uint32 PixelBack = 0;
    uint32 PixelIntegralBack = 0;
    uint32 PixelIntegral = 0;
    int32 PixelIntegralFore = 0;
    int32 PixelFore = 0;
    double OmegaBack=0, OmegaFore=0, MicroBack=0, MicroFore=0, SigmaB=0, Sigma=0; // 类间方差;
    uint8 MinValue=0, MaxValue=0;
   
    for (Y = 0; Y <Image_Height; Y++) //Y<Image_Height改为Y =Image_Height；以便进行 行二值化
    {
        //Y=Image_Height;
        for (X = 0; X < Image_Width; X++)
        {
        HistGram[(int)data[Y*Image_Width + X]]++; //统计每个灰度值的个数信息
        }
    }

    for (MinValue = 0; MinValue < 255 && HistGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--) ; //获取最大灰度的值

    if (MaxValue == MinValue)
    {
        return MaxValue;          // 图像中只有一个颜色
    }
    if (MinValue + 1 == MaxValue)
    {
        return MinValue;      // 图像中只有二个颜色
    }

    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y];        //  像素总数
    }

    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y;//灰度值总数
    }
    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++)
    {
          PixelBack = PixelBack + HistGram[Y];    //前景像素点数
          PixelFore = Amount - PixelBack;         //背景像素点数
          OmegaBack = (double)PixelBack / Amount;//前景像素百分比
          OmegaFore = (double)PixelFore / Amount;//背景像素百分比
          PixelIntegralBack += HistGram[Y] * Y;  //前景灰度值
          PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
          MicroBack = (double)PixelIntegralBack / PixelBack;//前景灰度百分比
          MicroFore = (double)PixelIntegralFore / PixelFore;//背景灰度百分比
          Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//g
          if (Sigma > SigmaB)//遍历最大的类间方差g
          {
              SigmaB = Sigma;
              Threshold = (uint8)Y;
          }
    }
   return Threshold;
}

void turn_to_bin(void)//图像二值化
{
  uint8 i,j;
 image_thereshold = 140;//otsuThreshold(original_image[0], image_w, image_h);
 //image_thereshold=150;
  for(i = 0;i<image_h;i++)
  {
      for(j = 0;j<image_w;j++)
      {
          if(original_image[i][j]>image_thereshold)bin_image[i][j] = white_pixel;
          else bin_image[i][j] = black_pixel;
      }
  }
}

uint8 get_start_point(uint8 start_row)//寻找两个边界的边界点作为八邻域循环的起始点，输入行数
{
    uint8 i = 0,l_found = 0,r_found = 0;
    //清零
    start_point_l[0] = 0;//x
    start_point_l[1] = 0;//y

    start_point_r[0] = 0;//x
    start_point_r[1] = 0;//y

        //从中间往左边，先找起点
    /*for (i = image_w / 2; i > border_min; i--)
    {
        start_point_l[0] = i;//x
        start_point_l[1] = start_row;//y
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i - 1] == 0)
        {
            //printf("找到左边起点image[%d][%d]\n", start_row,i);
            l_found = 1;
            break;
        }
    }

    for (i = image_w / 2; i < border_max; i++)
    {
        start_point_r[0] = i;//x
        start_point_r[1] = start_row;//y
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i + 1] == 0)
        {
            //printf("找到右边起点image[%d][%d]\n",start_row, i);
            r_found = 1;
            break;
        }
    }*/
    /*if(finding_point==0)
    {
      //从左边开始找起点
      for(i = 1;i<border_max;i++)
      {
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i - 1] == 0)
        {
          start_point_l[0] = i;//x
          start_point_l[1] = start_row;//y
          //printf("找到左边起点image[%d][%d]\n", start_row,i);
          l_found = 1;
          //break;
        }
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i + 1] == 0)
        {
          start_point_r[0] = i;//x
          start_point_r[1] = start_row;//y
          //printf("找到右边起点image[%d][%d]\n",start_row, i);
          r_found = 1;
          break;
        }
      }
    }
    else 
    {
      //从右边开始找起点
      for(i = border_max;i>1;i--)
      {  
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i + 1] == 0)
        {
          start_point_r[0] = i;//x
          start_point_r[1] = start_row;//y
          //printf("找到右边起点image[%d][%d]\n",start_row, i);
          r_found = 1;
          //break;
        }
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i - 1] == 0)
        {
          start_point_l[0] = i;//x
          start_point_l[1] = start_row;//y
          //printf("找到左边起点image[%d][%d]\n", start_row,i);
          l_found = 1;
          break;
        }
      }
    }*/
    
    
    if(finding_point==0)
    {
      //从左边开始找起点
      for(i = 1;i<border_max;i++)
      {
        if (original_image[start_row][i] > image_thereshold && original_image[start_row][i - 1] <=image_thereshold)
        {
          start_point_l[0] = i;//x
          start_point_l[1] = start_row;//y
          //printf("找到左边起点image[%d][%d]\n", start_row,i);
          l_found = 1;
          //break;
        }
        if (original_image[start_row][i] > image_thereshold && original_image[start_row][i + 1] <=image_thereshold)
        {
          start_point_r[0] = i;//x
          start_point_r[1] = start_row;//y
          //printf("找到右边起点image[%d][%d]\n",start_row, i);
          r_found = 1;
          break;
        }
      }
    }
    else 
    {
      //从右边开始找起点
      for(i = border_max;i>1;i--)
      {  
        if (original_image[start_row][i] > image_thereshold && original_image[start_row][i + 1] <=image_thereshold)
        {
          start_point_r[0] = i;//x
          start_point_r[1] = start_row;//y
          //printf("找到右边起点image[%d][%d]\n",start_row, i);
          r_found = 1;
          //break;
        }
        if (original_image[start_row][i] > image_thereshold && original_image[start_row][i - 1] <=image_thereshold)
        {
          start_point_l[0] = i;//x
          start_point_l[1] = start_row;//y
          //printf("找到左边起点image[%d][%d]\n", start_row,i);
          l_found = 1;
          break;
        }
      }
    }

    //从右边开始找起点
    /*for(i = border_max;i>1;i--)
    {
      if (bin_image[start_row][i] == 255 && bin_image[start_row][i + 1] == 0)
        {
          start_point_r[0] = i;//x
          start_point_r[1] = start_row;//y
            //printf("找到右边起点image[%d][%d]\n",start_row, i);
            r_found = 1;
            //break;
        }
      if (bin_image[start_row][i] == 255 && bin_image[start_row][i - 1] == 0)
        {
          start_point_l[0] = i;//x
          start_point_l[1] = start_row;//y
            //printf("找到左边起点image[%d][%d]\n", start_row,i);
            l_found = 1;
            break;
        }
    }*/

    if(l_found&&r_found)return 1;
    else {
        //printf("未找到起点\n");
        return 0;
    }
}

/*
函数名称：void search_l_r(uint16 break_flag, uint8(*image)[image_w],uint16 *l_stastic, uint16 *r_stastic,
                            uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y,uint8*hightest)

功能说明：八邻域正式开始找右边点的函数，输入参数有点多，调用的时候不要漏了，这个是左右线一次性找完。
参数说明：
break_flag_r            ：最多需要循环的次数
(*image)[image_w]       ：需要进行找点的图像数组，必须是二值图,填入数组名称即可
                       特别注意，不要拿宏定义名字作为输入参数，否则数据可能无法传递过来
*l_stastic              ：统计左边数据，用来输入初始数组成员的序号和取出循环次数
*r_stastic              ：统计右边数据，用来输入初始数组成员的序号和取出循环次数
l_start_x               ：左边起点横坐标
l_start_y               ：左边起点纵坐标
r_start_x               ：右边起点横坐标
r_start_y               ：右边起点纵坐标
hightest                ：循环结束所得到的最高高度
函数返回：无
修改时间：2022年9月25日
备    注：
example：
    search_l_r((uint16)USE_num,image,&data_stastics_l, &data_stastics_r,start_point_l[0],
                start_point_l[1], start_point_r[0], start_point_r[1],&hightest);
 */

void search_l_r(uint16 break_flag, uint8(*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*hightest)
{

    uint8 i = 0, j = 0;

    //左边变量
    uint8 search_filds_l[8][2] = { {  0 } };
    //uint8 index_l = 0;
    //uint8 temp_l[8][2] = { {  0 } };
    uint8 center_point_l[2] = {  0 };
    uint16 l_data_statics;//统计左边
    //定义八个邻域
    static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //这个是顺时针

    //右边变量
    uint8 search_filds_r[8][2] = { {  0 } };
    uint8 center_point_r[2] = { 0 };//中心坐标点
    //uint8 index_r = 0;//索引下标
    //uint8 temp_r[8][2] = { {  0 } };
    uint16 r_data_statics;//统计右边
    //定义八个邻域
    static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //这个是逆时针

    l_data_statics = *l_stastic;//统计找到了多少个点，方便后续把点全部画出来
    r_data_statics = *r_stastic;//统计找到了多少个点，方便后续把点全部画出来

    //第一次更新坐标点  将找到的起点值传进来
    center_point_l[0] = l_start_x;//x
    center_point_l[1] = l_start_y;//y
    center_point_r[0] = r_start_x;//x
    center_point_r[1] = r_start_y;//y

        //开启邻域循环
    while (break_flag--)
    {

        //左边
        for (i = 0; i < 8; i++)//传递8F坐标
        {
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
        }
        //中心坐标点填充到已经找到的点内
        points_l[l_data_statics][0] = center_point_l[0];//x
        points_l[l_data_statics][1] = center_point_l[1];//y
        l_data_statics++;//索引加一

        //右边
        for (i = 0; i < 8; i++)//传递8F坐标
        {
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
        }
        //中心坐标点填充到已经找到的点内
        points_r[r_data_statics][0] = center_point_r[0];//x
        points_r[r_data_statics][1] = center_point_r[1];//y

        /*index_l = 0;//先清零，后使用
        for (i = 0; i < 8; i++)
        {
            temp_l[i][0] = 0;//先清零，后使用
            temp_l[i][1] = 0;//先清零，后使用
        }*/

        //左边判断
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_l[i][1]][search_filds_l[i][0]] <= image_thereshold
                && image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] >image_thereshold)
            {
                //temp_l[index_l][0] = search_filds_l[(i)][0];
                //temp_l[index_l][1] = search_filds_l[(i)][1];
                center_point_l[0] = search_filds_l[(i)][0];//x
                center_point_l[1] = search_filds_l[(i)][1];//y
                //index_l++;
                dir_l[l_data_statics - 1] = (i);//记录生长方向
                break;
            }

            /*if (index_l)
            {
                //更新坐标点
                center_point_l[0] = temp_l[0][0];//x
                center_point_l[1] = temp_l[0][1];//y
                for (j = 0; j < index_l; j++)
                {
                    if (center_point_l[1] > temp_l[j][1])
                    {
                        center_point_l[0] = temp_l[j][0];//x
                        center_point_l[1] = temp_l[j][1];//y
                    }
                }
            }*/
        }
        if ((points_r[r_data_statics][0]== points_r[r_data_statics-1][0]&& points_r[r_data_statics][0] == points_r[r_data_statics - 2][0]
            && points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1])
            ||(points_l[l_data_statics-1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics-1][0] == points_l[l_data_statics - 3][0]
                && points_l[l_data_statics-1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics-1][1] == points_l[l_data_statics - 3][1]))
        {
            //printf("三次进入同一个点，退出\n");
            break;
        }
        if (my_abs(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
            && my_abs(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2)
            )
        {
            //printf("\n左右相遇退出\n");
            *hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//取出最高点
            //printf("\n在y=%d处退出\n",*hightest);
            break;
        }
        if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
        {
            //printf("\n如果左边比右边高了，左边等待右边\n");
            continue;//如果左边比右边高了，左边等待右边
        }
        if (dir_l[l_data_statics - 1] == 7
            && (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//左边比右边高且已经向下生长了
        {
            //printf("\n左边开始向下了，等待右边，等待中... \n");
            center_point_l[0] = points_l[l_data_statics - 1][0];//x
            center_point_l[1] = points_l[l_data_statics - 1][1];//y
            l_data_statics--;
        }
        r_data_statics++;//索引加一

        /*index_r = 0;//先清零，后使用
        for (i = 0; i < 8; i++)
        {
            temp_r[i][0] = 0;//先清零，后使用
            temp_r[i][1] = 0;//先清零，后使用
        }*/

        //右边判断
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_r[i][1]][search_filds_r[i][0]] <=image_thereshold
                && image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] >image_thereshold)
            {
                //temp_r[index_r][0] = search_filds_r[(i)][0];
                //temp_r[index_r][1] = search_filds_r[(i)][1];
                //index_r++;//索引加一
                center_point_r[0] = search_filds_r[(i)][0];
                center_point_r[1] = search_filds_r[(i)][1];
                dir_r[r_data_statics - 1] = (i);//记录生长方向
                break;
                //printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
            }
            /*if (index_r)
            {

                //更新坐标点
                center_point_r[0] = temp_r[0][0];//x
                center_point_r[1] = temp_r[0][1];//y
                for (j = 0; j < index_r; j++)
                {
                    if (center_point_r[1] > temp_r[j][1])
                    {
                        center_point_r[0] = temp_r[j][0];//x
                        center_point_r[1] = temp_r[j][1];//y
                    }
                }
            }*/
        }
    }
    //取出循环次数
    *l_stastic = l_data_statics;
    *r_stastic = r_data_statics;
}

/*void search_l_r(uint16 break_flag, uint8(*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*hightest)
{

    uint8 i = 0, j = 0;

    //左边变量
    uint8 search_filds_l[8][2] = { {  0 } };
    uint8 index_l = 0;
    uint8 temp_l[8][2] = { {  0 } };
    uint8 center_point_l[2] = {  0 };
    uint16 l_data_statics;//统计左边
    //定义八个邻域
    static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //这个是顺时针

    //右边变量
    uint8 search_filds_r[8][2] = { {  0 } };
    uint8 center_point_r[2] = { 0 };//中心坐标点
    uint8 index_r = 0;//索引下标
    uint8 temp_r[8][2] = { {  0 } };
    uint16 r_data_statics;//统计右边
    //定义八个邻域
    static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //这个是逆时针

    l_data_statics = *l_stastic;//统计找到了多少个点，方便后续把点全部画出来
    r_data_statics = *r_stastic;//统计找到了多少个点，方便后续把点全部画出来

    //第一次更新坐标点  将找到的起点值传进来
    center_point_l[0] = l_start_x;//x
    center_point_l[1] = l_start_y;//y
    center_point_r[0] = r_start_x;//x
    center_point_r[1] = r_start_y;//y

        //开启邻域循环
    while (break_flag--)
    {

        //左边
        for (i = 0; i < 8; i++)//传递8F坐标
        {
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
        }
        //中心坐标点填充到已经找到的点内
        points_l[l_data_statics][0] = center_point_l[0];//x
        points_l[l_data_statics][1] = center_point_l[1];//y
        l_data_statics++;//索引加一

        //右边
        for (i = 0; i < 8; i++)//传递8F坐标
        {
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
        }
        //中心坐标点填充到已经找到的点内
        points_r[r_data_statics][0] = center_point_r[0];//x
        points_r[r_data_statics][1] = center_point_r[1];//y

        index_l = 0;//先清零，后使用
        for (i = 0; i < 8; i++)
        {
            temp_l[i][0] = 0;//先清零，后使用
            temp_l[i][1] = 0;//先清零，后使用
        }

        //左边判断
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0
                && image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
            {
                temp_l[index_l][0] = search_filds_l[(i)][0];
                temp_l[index_l][1] = search_filds_l[(i)][1];
                index_l++;
                dir_l[l_data_statics - 1] = (i);//记录生长方向
            }

            if (index_l)
            {
                //更新坐标点
                center_point_l[0] = temp_l[0][0];//x
                center_point_l[1] = temp_l[0][1];//y
                for (j = 0; j < index_l; j++)
                {
                    if (center_point_l[1] > temp_l[j][1])
                    {
                        center_point_l[0] = temp_l[j][0];//x
                        center_point_l[1] = temp_l[j][1];//y
                    }
                }
            }
        }
        if ((points_r[r_data_statics][0]== points_r[r_data_statics-1][0]&& points_r[r_data_statics][0] == points_r[r_data_statics - 2][0]
            && points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1])
            ||(points_l[l_data_statics-1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics-1][0] == points_l[l_data_statics - 3][0]
                && points_l[l_data_statics-1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics-1][1] == points_l[l_data_statics - 3][1]))
        {
            //printf("三次进入同一个点，退出\n");
            break;
        }
        if (my_abs(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
            && my_abs(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2)
            )
        {
            //printf("\n左右相遇退出\n");
            *hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//取出最高点
            //printf("\n在y=%d处退出\n",*hightest);
            break;
        }
        if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
        {
            //printf("\n如果左边比右边高了，左边等待右边\n");
            continue;//如果左边比右边高了，左边等待右边
        }
        if (dir_l[l_data_statics - 1] == 7
            && (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//左边比右边高且已经向下生长了
        {
            //printf("\n左边开始向下了，等待右边，等待中... \n");
            center_point_l[0] = points_l[l_data_statics - 1][0];//x
            center_point_l[1] = points_l[l_data_statics - 1][1];//y
            l_data_statics--;
        }
        r_data_statics++;//索引加一

        index_r = 0;//先清零，后使用
        for (i = 0; i < 8; i++)
        {
            temp_r[i][0] = 0;//先清零，后使用
            temp_r[i][1] = 0;//先清零，后使用
        }

        //右边判断
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
                && image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
            {
                temp_r[index_r][0] = search_filds_r[(i)][0];
                temp_r[index_r][1] = search_filds_r[(i)][1];
                index_r++;//索引加一
                dir_r[r_data_statics - 1] = (i);//记录生长方向
                //printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
            }
            if (index_r)
            {

                //更新坐标点
                center_point_r[0] = temp_r[0][0];//x
                center_point_r[1] = temp_r[0][1];//y
                for (j = 0; j < index_r; j++)
                {
                    if (center_point_r[1] > temp_r[j][1])
                    {
                        center_point_r[0] = temp_r[j][0];//x
                        center_point_r[1] = temp_r[j][1];//y
                    }
                }
            }
        }
    }
    //取出循环次数
    *l_stastic = l_data_statics;
    *r_stastic = r_data_statics;
}*/

void get_right(uint16 total_R)//从八邻域边界里提取需要的边线，找到的点的总数
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    //uint16 r_border[120] = { 0 };

    for (i = 0; i < image_h; i++)
    {
        r_border[i] = border_max;//右边线初始化放到最右边，左边线放到最左边，这样八邻域闭合区域外的中线就会在中间，不会干扰得到的数据
    }
    h = image_h - 2;
    //右边
    for (j = 0; j < total_R; j++)
    {
        if (points_r[j][1] == h)
        {
            r_border[h] = points_r[j][0] - 1;
        }
        else continue;//每行只取一个点，没到下一行就不记录
        h--;
        if (h == 0)break;//到最后一行退出
    }
}

void get_left(uint16 total_L)//从八邻域边界里提取需要的边线，找到的点的总数
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = hightest;
    //uint16 l_border[120] = { 0 };

    for (i = 0; i < image_h; i++)
    {
        l_border[i] = 2;//右边线初始化放到最右边，左边线放到最左边，这样八邻域闭合区域外的中线就会在中间，不会干扰得到的数据
    }
    h = image_h - 2;
    //左边
    for (j = 0; j < total_L; j++)
    {
        if (points_l[j][1] == h)
        {
            l_border[h] = points_l[j][0] - 1;
        }
        else continue;//每行只取一个点，没到下一行就不记录
        h--;
        if (h == 0)break;//到最后一行退出
    }
}

void image_process(void)
{
    uint16 i;
    //uint8 h = 0;//定义一个最高行，tip：这里的最高指的是y值的最小
    //uint16 l_border[120]={0};
    //uint16 r_border[120]={0};
    /*这是离线调试用的*/
    //Get_image(mt9v03x_image);
    crop_image(mt9v03x_image);
    //turn_to_bin();
    /*提取赛道边界*/
    //image_filter(bin_image);//滤波
    //image_filter(original_image);//滤波
    //image_draw_rectan(bin_image);//预处理
    image_draw_rectan(original_image);//预处理
    //清零
    data_stastics_l = 0;
    data_stastics_r = 0;
    if (get_start_point(image_h - 2))//找到起点了，再执行八领域，没找到就一直找
    {
        search_l_r((uint16)USE_num, original_image, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &hightest);
        // 从爬取的边界线内提取边线 ， 这个才是最终有用的边线
        get_left(data_stastics_l);
        get_right(data_stastics_r);
        //处理函数放这里，不要放到if外面去了，不要放到if外面去了，不要放到if外面去了，重要的事说三遍
        
        get_l_up_turn_left_point();
        //get_l_right_turn_up_point();
        //get_l_up_turn_right_point();
        //get_l_up_turn_bottom_left_point();
        get_l_top_left_down_point();
        //get_l_top_right_turn_top_left_point();
        //get_l_bottom_right_turn_up_point();
    
        get_r_up_turn_right_point();
        //get_r_left_turn_up_point();
        //get_r_up_turn_left_point();
        get_r_up_turn_bottom_right_point();
        get_r_top_right_turn_down_point();
        //get_r_top_left_turn_top_right_point();
        //get_r_bottom_left_turn_up_point();
        
        //Get_R_Intercept_And_Slope(hightest-3, 115);
        //Get_L_Intercept_And_Slope(hightest-3, 115);
    }
    //显示图像   改成你自己的就行 等后期足够自信了，显示关掉，显示屏挺占资源的
    //ips200_show_gray_image(0, 120,bin_image[0], image_w, image_h, image_w, image_h, otsuThreshold(mt9v03x_image[0], MT9V03X_W, MT9V03X_H));
    //ips200_show_gray_image(0, 120, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
    
    //ips200_show_gray_image(0, 0,original_image[0], image_w, image_h, image_w, image_h, 0);
   // ips200_show_gray_image(0, 0,original_image[0], image_w, image_h, image_w, image_h, 140);
    //ips200_show_gray_image(0, 0,bin_image[0], image_w, image_h, image_w, image_h, 140);
    //ips200_draw_point(0, 0, RGB565_RED);
    //根据最终循环次数画出边界点
    
    
    
    //ips200_show_uint(0, 280, l_up_turn_left_point[0], 5);
    //ips200_show_uint(0, 295, l_up_turn_left_point[1], 5);
    //ips200_show_uint(0, 130, l_right_turn_up_point[0], 5);
    //ips200_show_uint(0, 145, l_right_turn_up_point[1], 5);
    //ips200_show_uint(0, 160, l_up_turn_right_point[0], 5);
    //ips200_show_uint(0, 175, l_up_turn_right_point[1], 5);
    
    //ips200_show_uint(0, 220, l_top_left_down_point[0], 5);
    //ips200_show_uint(0, 235, l_top_left_down_point[1], 5);
    //ips200_show_uint(0, 250, l_bottom_right_turn_up_point[0], 5);
    //ips200_show_uint(0, 265, l_bottom_right_turn_up_point[1], 5);
    
    //ips200_show_uint(80, 280, r_up_turn_right_point[0], 5);
    //ips200_show_uint(80, 295, r_up_turn_right_point[1], 5);
    //ips200_show_uint(80, 130, r_left_turn_up_point[0], 5);
    //ips200_show_uint(80, 145, r_left_turn_up_point[1], 5);
    //ips200_show_uint(80, 160, r_up_turn_left_point[0], 5);
    //ips200_show_uint(80, 175, r_up_turn_left_point[1], 5);
    //ips200_show_uint(80, 190, r_top_right_turn_down_point[0], 5);
    //ips200_show_uint(80, 205, r_top_right_turn_down_point[1], 5);
    //ips200_show_uint(160, 190, r_up_turn_bottom_right_point[0], 5);
    //ips200_show_uint(160, 205, r_up_turn_bottom_right_point[1], 5);
    //ips200_show_uint(80, 220, r_top_left_turn_top_right_point[0], 5);
    //ips200_show_uint(80, 235, r_top_left_turn_top_right_point[1], 5);
    //ips200_show_uint(80, 250, r_bottom_left_turn_up_point[0], 5);
    //ips200_show_uint(80, 265, r_bottom_left_turn_up_point[1], 5);
    
    //ips200_show_uint(160, 130,image_thereshold , 5);
    
   /* for (i = 1; i < data_stastics_l; i++)
    {
        ips200_draw_point(points_l[i][0], points_l[i][1], uesr_BLUE);//显示起点
    }
    for (i = 1; i < data_stastics_r; i++)
    {
        ips200_draw_point(points_r[i][0], points_r[i][1], uesr_RED);//显示起点
    }*/
    for (i = hightest; i <image_h-1; i++)
    {

        center_line[i] = (l_border[i] + r_border[i]) >> 1;//求中线
        //求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
        //当然也有多组边线的找法，但是个人感觉很繁琐，不建议
        //ips200_draw_point(center_line[i], i, uesr_GREEN);//显示起点 显示中线
        //ips200_draw_point(l_border[i], i, uesr_GREEN);//显示起点 显示左边线
        //ips200_draw_point(r_border[i], i, uesr_GREEN);//显示起点 显示右边线
    }
   
    element_selection();
    
}
//**************************************************************************************************************************************************************************************

/**
* 函数功能：      浮点型变量取绝对值
* 特殊说明：      注意调用时参数类型，摄像头代码参数较多，不同类型错误传参可能导致计算结果出现较大问题，严重时卡死单片机运行
* 形  参：        value：       浮点型变量
* 示例：          My_ABS_F();
* 返回值：        传入参数的绝对值
*/
float My_ABS_F(float value)               
{
    if(value >= 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}

uint8 My_ABS_uint8(uint8 value)               
{
    if(value >= 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}


//元素判断*****************************************************************************************************************************************************
//*******************************************************************************************************************************************************
/**
* 函数功能：      最小二乘法计算斜率
* 特殊说明：      无
* 形  参：        uint8 begin                输入起点
*                 uint8 end                  输入终点
*                 uint8 *border              输入需要计算斜率的一维边线数组
* 示例：          Slope_Calculate(start, end, border);
* 返回值：        Result    计算出的斜率
*/
float Slope_Calculate(uint8 begin, uint8 end, uint16 *border)    //注：begin 必须小于 end,一般 begin 位于图像上方， end 位于图像下方
{
    float X_Sum = 0, Y_Sum = 0, XY_Sum = 0, X2_Sum = 0;
    int16 i = 0;
    float Result = 0;
    static float Result_Last;

    for(i = begin; i < end ; i++)
    {
        X_Sum += (float)i;
        Y_Sum += (float)border[i];
        XY_Sum += (float)i * (border[i]);
        X2_Sum += (float)i * i;
    }

    if((end - begin) * X2_Sum - X_Sum * X_Sum)      //防止为0的情况出现
    {
        Result = ((float)(end - begin) * XY_Sum - X_Sum * Y_Sum) / ((float)(end - begin) * X2_Sum - X_Sum * X_Sum);
        Result_Last = Result;
    }
    else
    {
        Result = Result_Last;
    }
    return Result;
}

/**
* 函数功能：      计算斜率截距
* 特殊说明：      调用最小二乘法计算斜率
* 形  参：        uint8 start                输入起点
*                 uint8 end                  输入终点
*                 uint8 *border              输入需要计算斜率的一维边线数组
*                 float *slope_rate          存储斜率的变量地址
*                 float *intercept           存储截距的变量地址
* 示例：          Calculate_Slope_Intercept(start, end, L_Border, &L_Straightaway_Lope_Rate_C, &L_Intercept);
* 返回值：        无
*/
void Calculate_Slope_Intercept(uint8 start, uint8 end, uint16 *border, float *slope_rate, float *intercept)
{
    uint16 i, Num = 0;
    uint16 X_Sum = 0, Y_Sum = 0;
    float Y_Average = 0, X_Average = 0;

    for(i = start; i < end; i++)
    {
        X_Sum += i;
        Y_Sum += border[i];
        Num ++;
    }

    if(Num)
    {
        X_Average = (float)(X_Sum / Num);
        Y_Average = (float)(Y_Sum / Num);
    }

    *slope_rate = Slope_Calculate(start, end, border);
    *intercept = (float)(Y_Average - (*slope_rate) * X_Average);
}
//求方差
void Get_Variance(uint8 start, uint8 end, int16 *fitting_line, uint8 *actual_line, float *variance, uint8 multiple)
{
    uint8 i = 0;
    float S = 0;
    uint8 total_num = 0;
    float Temp = 0;

    if(start % 3 != 0)
    {
        for(i = 1; i < 3; i++)
        {
            start = start + i;
            if(start % 3 == 1)
            {
                break;
            }
        }
    }

    total_num = (uint8)((end - start) / multiple);

    for(i = 0; i < total_num; i++)
    {
        uint8 Temp_Value = 0;
        Temp_Value = i * multiple + start;
        Temp = (float)(actual_line[Temp_Value] - (uint8)fitting_line[Temp_Value / 3]);
        S += Temp * Temp;
    }
    S /= (float)total_num;
    *variance = S;
}

/**
* 函数功能：      求右侧边线斜率截距，并判断是否为直线
* 特殊说明：      无
* 形  参：        uint8 start          //右侧边线起始点Y坐标
*                 uint8 end            //右侧边线终止点Y坐标
*
* 示例：          Get_R_Intercept_And_Slope(R_Line[R_Statics][1] + 5, R_Line[0][1]);
* 返回值：        无
*/
void Get_R_Intercept_And_Slope(uint8 start, uint8 end)
{
    R_Straightaway_Lope_Rate_A = 0;
    R_Straightaway_Lope_Rate_B = 0;
    R_Straightaway_Lope_Rate_C = 0;
    //R_Intercept = 0;
    //R_Straight_Flag = 0;            //所有值先清零

    R_Straightaway_Lope_Rate_A = Slope_Calculate(start, ((end - start) / 2) + start,r_border);
    R_Straightaway_Lope_Rate_B = Slope_Calculate(((end - start) / 2) + start, end, r_border);       //计算两段斜率
    //Calculate_Slope_Intercept(start, end, r_border, &R_Straightaway_Lope_Rate_C, &R_Intercept);     //计算整段边线斜率和截距
    R_Straightaway_Lope_Rate_C =Slope_Calculate(start,end,r_border);

    if(My_ABS_F(R_Straightaway_Lope_Rate_A - R_Straightaway_Lope_Rate_B) <= R_Min_Slope_Dif_Thre &&
       My_ABS_F(R_Straightaway_Lope_Rate_B - R_Straightaway_Lope_Rate_C) <= R_Min_Slope_Dif_Thre &&
       My_ABS_F(R_Straightaway_Lope_Rate_A - R_Straightaway_Lope_Rate_C) <= R_Min_Slope_Dif_Thre)      //判断是否为直线
    {
        R_Straight_Flag = 1;
    }
}
void Get_L_Intercept_And_Slope(uint8 start, uint8 end)
{
    L_Straightaway_Lope_Rate_A = 0;
    L_Straightaway_Lope_Rate_B = 0;
    L_Straightaway_Lope_Rate_C = 0;
    //L_Intercept = 0;
    //L_Straight_Flag = 0;            //所有值先清零

    L_Straightaway_Lope_Rate_A = Slope_Calculate(start, ((end - start) / 2) + start,l_border);
    L_Straightaway_Lope_Rate_B = Slope_Calculate(((end - start) / 2) + start, end, l_border);       //计算两段斜率
    //Calculate_Slope_Intercept(start, end, l_border, &L_Straightaway_Lope_Rate_C, &L_Intercept);     //计算整段边线斜率和截距
    L_Straightaway_Lope_Rate_C =Slope_Calculate(start,end,l_border);

    if(My_ABS_F(L_Straightaway_Lope_Rate_A - L_Straightaway_Lope_Rate_B) <= L_Min_Slope_Dif_Thre &&
       My_ABS_F(L_Straightaway_Lope_Rate_B - L_Straightaway_Lope_Rate_C) <= L_Min_Slope_Dif_Thre &&
       My_ABS_F(L_Straightaway_Lope_Rate_A - L_Straightaway_Lope_Rate_C) <= L_Min_Slope_Dif_Thre)      //判断是否为直线
    {
        L_Straight_Flag = 1;
    }
}

//补线y=k*x+b
void line_repair(uint16 *start,uint16 *end,uint16 *line)
{
  int i=0;
  float k=0;
  float b=0;
  int s[2]={0};
  int e[2]={0};
  s[0]=start[0];
  s[1]=-1*start[1];
  e[0]=end[0];
  e[1]=-1*end[1];

  k=(float)((float)s[1]-(float)e[1])/(float)((float)s[0]-(float)e[0]);//
  b=(float)s[1]-k*(float)s[0];
  for(i=s[1];i<e[1];i++)
  {
    line[-1*i]=(i-b)/k;
  }
}

//左边***********************************************************************************************************************************************************

//上转左点
void get_l_up_turn_left_point(void)
{
  l_up_turn_left_point[0]=0;
  l_up_turn_left_point[1]=0;
  l_up_turn_left_point_position=0;
  int i=0;
  for(i=0;i<data_stastics_l;i++)
  {
    if((dir_l[i]==3||dir_l[i]==2)&&(dir_l[i-2]==4||dir_l[i-2]==5)&&dir_l[i-4]==4&&(dir_l[i+2]==2||dir_l[i+2]==3)&&dir_l[i+4]==2)
    {
      l_up_turn_left_point[0]=points_l[i][0];
      l_up_turn_left_point[1]=points_l[i][1];
      l_up_turn_left_point_position=i;
      l_up_turn_left_point_flag=1;
      break;
    }
  }
}
//右转上点
void get_l_right_turn_up_point(void)
{
  l_right_turn_up_point[0]=0;
  l_right_turn_up_point[1]=0;
  l_right_turn_up_point_position=0;
  int i=0;
  for(i=0;i<data_stastics_l;i++)
  {
    if(dir_l[i]==5&&dir_l[i+2]==4&&dir_l[i+4]==4&&dir_l[i-2]==6&&dir_l[i-4]==6)
    {
      l_right_turn_up_point[0]=points_l[i][0];
      l_right_turn_up_point[1]=points_l[i][1];
      l_right_turn_up_point_position=i;
      l_right_turn_up_point_flag=1;
      break;
    }
  }
}
//上转右点
void get_l_up_turn_right_point(void)
{
  l_up_turn_right_point[0]=0;
  l_up_turn_right_point[1]=0;
  l_up_turn_right_point_position=0;
  int i=0;
  for(i=0;i<data_stastics_l;i++)
  {
    if(dir_l[i]==5&&dir_l[i+2]==6&&dir_l[i+4]==6&&dir_l[i-2]==4&&dir_l[i-4]==4)
    {
      l_up_turn_right_point[0]=points_l[i][0];
      l_up_turn_right_point[1]=points_l[i][1];
      l_up_turn_right_point_position=i;
      l_up_turn_right_point_flag=1;
      break;
    }
  }
}
//环
//上转左下点
void get_l_up_turn_bottom_left_point(void)
{
  l_up_turn_bottom_left_point[0]=0;
  l_up_turn_bottom_left_point[1]=0;
  l_up_turn_bottom_left_point_position=0;
  int i=0;
  for(i=0;i<data_stastics_l;i++)
  {
    if((dir_l[i]==1||dir_l[i]==0)&&(dir_l[i-2]==2||dir_l[i-2]==3||dir_l[i-2]==4)&&(dir_l[i-4]==3||dir_l[i-4]==4||dir_l[i-4]==5)&&(dir_l[i+1]==0||dir_l[i+1]==1)&&(dir_l[i+3]==0||dir_l[i+3]==1))
    {
      l_up_turn_bottom_left_point[0]=points_l[i][0];
      l_up_turn_bottom_left_point[1]=points_l[i][1];
      l_up_turn_bottom_left_point_position=i;
      l_up_turn_bottom_left_point_flag=1;
      break;
    }
  }
}
//左上转下点
void get_l_top_left_down_point(void)
{
  l_top_left_down_point[0]=0;
  l_top_left_down_point[1]=0;
  l_top_left_down_point_position=0;
  int i=0;
  for(i=0;i<data_stastics_l;i++)
  {
    if((dir_l[i]==1||dir_l[i]==0)&&(dir_l[i-2]==3||dir_l[i-2]==4)&&(dir_l[i-4]==3||dir_l[i-4]==4)&&(dir_l[i+1]==0||dir_l[i+1]==1||dir_l[i+1]==7)&&(dir_l[i+3]==0||dir_l[i+3]==1||dir_l[i+3]==7))
    {
      l_top_left_down_point[0]=points_l[i][0];
      l_top_left_down_point[1]=points_l[i][1];
      l_top_left_down_point_position=i;
      l_top_left_down_point_flag=1;
      break;
    }
  }
}
//右上转左上点
void get_l_top_right_turn_top_left_point(void)
{
  l_top_right_turn_top_left_point[0]=0;
  l_top_right_turn_top_left_point[1]=0;
  l_top_right_turn_top_left_point_position=0;
  int i=0;
  for(i=l_up_turn_bottom_left_point_position;i<data_stastics_l;i++)
  {
    if((dir_l[i]==3||dir_l[i]==4)&&dir_l[i+1]==3&&(dir_l[i+3]==3||dir_l[i+3]==4)&&(dir_l[i-2]==5||dir_l[i-2]==4)&&(dir_l[i-4]==5||dir_l[i-4]==4))
    {
      l_top_right_turn_top_left_point[0]=points_l[i][0];
      l_top_right_turn_top_left_point[1]=points_l[i][1];
      l_top_right_turn_top_left_point_position=i;
      l_top_right_turn_top_left_point_flag=1;
      break;
    }
  }
}
//右下转上点
void get_l_bottom_right_turn_up_point(void)
{
  l_bottom_right_turn_up_point[0]=0;
  l_bottom_right_turn_up_point[1]=0;
  l_bottom_right_turn_up_point_position=0;
  int i=0;
  for(i=l_top_right_turn_top_left_point_position;i<data_stastics_l;i++)
  {
    if((dir_l[i]==5||dir_l[i]==4||dir_l[i]==6)&&dir_l[i+1]==4&&(dir_l[i+3]==4||dir_l[i+3]==5)&&(dir_l[i-2]==7||dir_l[i-2]==0)&&(dir_l[i-4]==7||dir_l[i-4]==0))
    {
      l_bottom_right_turn_up_point[0]=points_l[i][0];
      l_bottom_right_turn_up_point[1]=points_l[i][1];
      l_bottom_right_turn_up_point_position=i;
      l_bottom_right_turn_up_point_flag=1;
      break;
    }
  }
}

//***************************************************************************************************************************************
//右边******************************************************************************************************************************************

//上转右点
void get_r_up_turn_right_point(void)
{
  r_up_turn_right_point[0]=0;
  r_up_turn_right_point[1]=0;
  r_up_turn_right_point_position=0;
  int i=0;
  for(i=0;i<data_stastics_r;i++)
  {
    if((dir_r[i]==3||dir_r[i]==2)&&(dir_r[i+2]==2||dir_r[i+2]==3)&&dir_r[i+4]==2&&(dir_r[i-2]==4||dir_r[i-2]==5)&&dir_r[i-4]==4)
    {
      r_up_turn_right_point[0]=points_r[i][0];
      r_up_turn_right_point[1]=points_r[i][1];
      r_up_turn_right_point_position=i;
      r_up_turn_right_point_flag=1;
      break;
    }
  }
}
//左转上点
void get_r_left_turn_up_point(void)
{
  r_left_turn_up_point[0]=0;
  r_left_turn_up_point[1]=0;
  r_left_turn_up_point_position=0;
  int i=0;
  for(i=0;i<data_stastics_r;i++)
  {
    if(dir_r[i]==5&&dir_r[i+2]==4&&dir_r[i+4]==4&&dir_r[i-2]==6&&dir_r[i-4]==6)
    {
      r_left_turn_up_point[0]=points_r[i][0];
      r_left_turn_up_point[1]=points_r[i][1];
      r_left_turn_up_point_position=i;
      r_left_turn_up_point_flag=1;
      break;
    }
  }
}
//上转左点
void get_r_up_turn_left_point(void)
{
  r_up_turn_left_point[0]=0;
  r_up_turn_left_point[1]=0;
  r_up_turn_left_point_position=0;
  int i=0;
  for(i=0;i<data_stastics_r;i++)
  {
    if(dir_r[i]==5&&dir_r[i+2]==6&&dir_r[i+4]==6&&dir_r[i-2]==4&&dir_r[i-4]==4)
    {
      r_up_turn_left_point[0]=points_r[i][0];
      r_up_turn_left_point[1]=points_r[i][1];
      r_up_turn_left_point_position=i;
      r_up_turn_left_point_flag=1;
      break;
    }
  }
}
//右环
//上转右下点
void get_r_up_turn_bottom_right_point(void)
{
  r_up_turn_bottom_right_point[0]=0;
  r_up_turn_bottom_right_point[1]=0;
  r_up_turn_bottom_right_point_position=0;
  int i=0;
  for(i=0;i<data_stastics_r;i++)
  {
    if((dir_r[i]==1||dir_r[i]==0)&&(dir_r[i-2]==4||dir_r[i-2]==3)&&dir_r[i-4]==4&&(dir_r[i+1]==1||dir_r[i+1]==0)&&dir_r[i+3]==1)
    {
      r_up_turn_bottom_right_point[0]=points_r[i][0];
      r_up_turn_bottom_right_point[1]=points_r[i][1];
      r_up_turn_bottom_right_point_position=i;
      if(r_up_turn_bottom_right_point[1]>10)
      {
        r_up_turn_bottom_right_point_flag=1;
      }
      break;
    }
  }
}
//左上转下点
void get_r_top_right_turn_down_point(void)
{
  r_top_right_turn_down_point[0]=0;
  r_top_right_turn_down_point[1]=0;
  r_top_right_turn_down_point_position=0;
  int i=0;
  for(i=0;i<data_stastics_r;i++)
  {
    if((dir_r[i]==1||dir_r[i]==0)&&(dir_r[i-2]==3||dir_r[i-2]==4)&&dir_r[i-4]==3&&dir_r[i+1]==0&&dir_r[i+3]==0)
    {
      r_top_right_turn_down_point[0]=points_r[i][0];
      r_top_right_turn_down_point[1]=points_r[i][1];
      r_top_right_turn_down_point_position=i;
      if(r_top_right_turn_down_point[1]>10)
      {
        r_top_right_turn_down_point_flag=1;
      }
      break;
    }
  }
}
//左上转右上点
void get_r_top_left_turn_top_right_point(void)
{
  r_top_left_turn_top_right_point[0]=0;
  r_top_left_turn_top_right_point[1]=0;
  r_top_left_turn_top_right_point_position=0;
  int i=0;
  for(i=r_up_turn_bottom_right_point_position;i<data_stastics_r;i++)
  {
    if((dir_r[i]==3||dir_r[i]==4)&&dir_r[i+1]==3&&(dir_r[i+3]==3||dir_r[i+3]==4)&&(dir_r[i-2]==5||dir_r[i-2]==4)&&(dir_r[i-4]==5||dir_r[i-4]==4))
    {
      r_top_left_turn_top_right_point[0]=points_r[i][0];
      r_top_left_turn_top_right_point[1]=points_r[i][1];
      r_top_left_turn_top_right_point_position=i;
      r_top_left_turn_top_right_point_flag=1;
      break;
    }
  }
}
//左下转上点
void get_r_bottom_left_turn_up_point(void)
{
  r_bottom_left_turn_up_point[0]=0;
  r_bottom_left_turn_up_point[1]=0;
  r_bottom_left_turn_up_point_position=0;
  int i=0;
  for(i=r_top_left_turn_top_right_point_position;i<data_stastics_r;i++)
  {
    if((dir_r[i]==5||dir_r[i]==4||dir_r[i]==6)&&dir_r[i+1]==4&&(dir_r[i+3]==4||dir_r[i+3]==5)&&(dir_r[i-2]==7||dir_r[i-2]==0)&&(dir_r[i-4]==7||dir_r[i-4]==0))
    {
      r_bottom_left_turn_up_point[0]=points_r[i][0];
      r_bottom_left_turn_up_point[1]=points_r[i][1];
      r_bottom_left_turn_up_point_position=i;
      r_bottom_left_turn_up_point_flag=1;
      break;
    }
  }
}


//*******************************************************************************************************************************************

//元素判断***********************************************************************************************************************************************
uint16 U_black_to_white[2];
uint8 U_black_to_white_flag=0;
uint16 U_white_to_black[2];
uint8 U_white_to_black_flag=0;
uint16 U_black_to_white2[2];
uint8 U_black_to_white2_flag=0;
uint16 U_white_to_black2[2];
uint8 U_white_to_black2_flag=0;
uint16 D_black_to_white[2];
uint8 D_black_to_white_flag=0;
uint16 D_white_to_black[2];
uint8 D_white_to_black_flag=0;
uint16 D_black_to_white2[2];
uint8 D_black_to_white2_flag=0;
uint16 D_white_to_black2[2];
uint8 D_white_to_black2_flag=0;
void element_selection(void)//选择
{
  //直线element=1
  /*if(R_Straight_Flag==1&&L_Straight_Flag==1)//)(l_up_turn_left_point_flag==1&&r_up_turn_right_point_flag==1))
  {
    //element=1;
    R_Straight_Flag=0;
    L_Straight_Flag=0;
  }*/
  
  /*for(i=1;i<188;i++)
  {
    for(j=3;j<8;j++)
    {
      if(bin_image[j][i]==0&&bin_image[j][i+1]==255&&U_black_to_white_flag==0)
      {
        U_black_to_white[0]=i;
        U_black_to_white[1]=j;
        U_black_to_white_flag=1;
      
      }
      if(bin_image[j][i]==0&&bin_image[j][i-1]==255&&U_white_to_black_flag==0)
      {
        U_white_to_black[0]=i;
        U_white_to_black[1]=j;
        U_white_to_black_flag=1;
      }
    }
  }*/
  
  //圆环element=6,7
  if(r_top_right_turn_down_point_flag==1)//左环
  {
    int s;
    int8 l_white_poingt=0;
    int8 r_white_poingt=0;
    /*int16 r_u=0;
    int16 r_d=0;
    for(s=2;s<7;s++)
    {
      r_u+=r_border[s];
      r_d+=r_border[image_h-s-1];
    }
    if(abs(r_u/5-r_d/5)<6)
    {
      element=6;
    }*/
    for(s=1;s<(image_w-2)/2;s++)
    {
      if(original_image[1][s]>image_thereshold)
      {
        l_white_poingt++;
      }
    }
    for(s=(image_w-2)/2;s<image_w-2;s++)
    {
      if(original_image[1][s]>image_thereshold)
      {
        r_white_poingt++;
      }
    }
    if(l_white_poingt>r_white_poingt)
    {
      //Speed_decision120();
      element=6;
    }
    r_top_right_turn_down_point_flag=0;
  }
  if(r_up_turn_bottom_right_point_flag==1)//右环
  {
    int s;
    int8 l_white_poingt=0;
    int8 r_white_poingt=0;
    int16 l_u=0;
    int16 l_d=0;
    for(s=2;s<7;s++)
    {
      l_u+=l_border[s];
      l_d+=l_border[image_h-s-1];
    }
    if(abs(l_u/5-l_d/5)<8)
    {
      //Speed_decision120();
      gpio_set_level(P23_7 , 1);
      element=7;
    }
    /*for(s=1;s<(image_w-2)/2;s++)
    {
      if(original_image[1][s]>image_thereshold)
      {
        l_white_poingt++;
      }
    }
    for(s=(image_w-2)/2;s<image_w-2;s++)
    {
      if(original_image[1][s]>image_thereshold)
      {
        r_white_poingt++;
      }
    }
    if(l_white_poingt<r_white_poingt)
    {
      //Speed_decision120();
      element=7;
    }*/
    r_up_turn_bottom_right_point_flag=0;
  }

  
  
  //直角element=2,3
    if(element!=6&&element!=7)
    {
      if(l_up_turn_left_point_flag==1&&r_up_turn_right_point_flag==0)
      {
        //Speed_decision120();
        uint8 l=0,l2=0,l3=0;
        /*for(int j=2;j<image_h-2;j++)
        {
          if(original_image[j][24]<=image_thereshold&&original_image[j+1][24]>image_thereshold)
          {
            L_black_to_white[0]=24;
            L_black_to_white[1]=j;
            
            l=j;
          }
          if(original_image[j][24]<=image_thereshold&&original_image[j-1][24]>image_thereshold)
          {
            l2=j;
          }
          if(original_image[j][image_w-25]>image_thereshold)
          {
            l3++;
          }
        }
        if(l<l2&&hightest>15&&l>15&&l3<(l2-l)/1.2)
        {
          gpio_set_level(P23_7 , 1);
          element=2;
          for(int j=23;j>1;j--)
          {
            if(original_image[(l+l2)/2][j]<=image_thereshold&&original_image[(l+l2)/2][j-1]>image_thereshold)
            {
              element=4;
            }
          }
          L_black_to_white_flag=1;
          l_up_turn_left_point_flag=0;
        }
        else
        {
          l_up_turn_left_point_flag=0;
          L_black_to_white_flag=0;
          gpio_set_level(P23_7 , 0);
        }*/
         for(int j=2;j<image_h-2;j++)
        {
          if(original_image[j][image_w-21]>image_thereshold)
          {
            l3++;
          }
        }
        if(hightest>10&&l3<6)// 15   5
        {
          gpio_set_level(P23_7 , 1);
          element=2;
          speed_processing();
          //floag++;
          L_black_to_white_flag=1;
          l_up_turn_left_point_flag=0;
        }
        else
        {
          l_up_turn_left_point_flag=0;
          L_black_to_white_flag=0;
          gpio_set_level(P23_7 , 0);
        }
      }
    }
    if(element!=6&&element!=7)
    {
      if(r_up_turn_right_point_flag==1&&l_up_turn_left_point_flag==0)
      {
        //Speed_decision120();
        uint8 r=0,r2=0,r3=0;
        /*for(int j=2;j<image_h-2;j++)
        {
          if(original_image[j][image_w-25]<=image_thereshold&&original_image[j+1][image_w-25]>image_thereshold)
          {
            R_black_to_white[0]=image_h-25;
            R_black_to_white[1]=j;
            
            r=j;
          }
          if (original_image[j][image_w-25]<=image_thereshold&&original_image[j-1][image_w-25]>image_thereshold)
          {
            r2=j;
          }
          if(original_image[j][24]>image_thereshold)
          {
            r3++;
          }
        }
        if(r<r2&&hightest>15&&r>15&&r3<(r2-r)/1.2)
        {
          gpio_set_level(P23_7 , 1);
          element=3;
          for(int j=23;j>1;j--)
          {
            if(original_image[(r+r2)/2][j]<=image_thereshold&&original_image[(r+r2)/2][j+1]>image_thereshold)
            {
              element=5;
            }
          }
          R_black_to_white_flag=1;
          r_up_turn_right_point_flag=0;
        }
        else
        {
          r_up_turn_right_point_flag=0;
          R_black_to_white_flag=0;
          gpio_set_level(P23_7 , 0);
        }*/
        for(int j=2;j<image_h-2;j++)
        {
          if(original_image[j][20]>image_thereshold)
          {
            r3++;
          }
        }
        if(hightest>10&&r3<6)//5
        {
          gpio_set_level(P23_7 , 1);
          element=3;
          speed_processing();
          R_black_to_white_flag=1;
          r_up_turn_right_point_flag=0;
        }
        else
        {
          r_up_turn_right_point_flag=0;
          R_black_to_white_flag=0;
          gpio_set_level(P23_7 , 0);
        }
      }
    }
  //十字element=4
  if(l_up_turn_left_point_flag==1&&r_up_turn_right_point_flag==1)
  {
    l_up_turn_left_point_flag=0;
    r_up_turn_right_point_flag=0;
    //element=4;
  }
  //虚线element=5
  
  
  if(element==0)
  {
    int i;
    int j;
    if(hightest<5)
    {
      u_l=0;
      u_r=0;
      d_r=0;
      d_l=0;
      for(i=hightest+2;i<hightest+7;i++)
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
     //ips200_show_int (0, 130, my_abs((u_l/5+u_r/5)/2-(d_r/5+d_l/5)/2), 8);
      if(my_abs((u_l/5+u_r/5)/2-(d_r/5+d_l/5)/2)<7)
      {
        //element=1;
        //Speed_decision160();
        uint8 l_black=0;
        uint8 r_black=0;
        for(j=1;j<image_h-1;j++)
        {
          if(original_image[j][10]>=image_thereshold)
          {
            l_black++;
          }
          if(original_image[j][image_w-11]>=image_thereshold)
          {
            r_black++;
          }
        }
        if(My_ABS_uint8(l_black-r_black)<5)
        {
          element=1;
        }
      }
    }
  }
  
  //断路element=8
  
  //起点element=9
  
  //折线element=10
  
}
