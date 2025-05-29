#ifndef color_tracer_H
#define color_tracer_H

#include "zf_common_headfile.h"
// 色彩识别代码参考博客
//  https://blog.csdn.net/niruxi0401/article/details/119685347

#define IMG_X               0               // 图片x坐标
#define IMG_Y               0               // 图片y坐标
#define IMG_W               SCC8660_W - 1   // 图片宽度
#define IMG_H               SCC8660_H - 1   // 图片高度

//#define ALLOW_FAIL_PER      10              // 容错率，没1<<ALLOW_FAIL_PER个点允许出现一个错误点，容错率越大越容易识别，但错误率越大
//#define ITERATE_NUM         5               // 迭代次数，迭代次数越多识别越精确，但计算量越大
extern uint8 ALLOW_FAIL_PER;                  // 容错率，没1<<ALLOW_FAIL_PER个点允许出现一个错误点，容错率越大越容易识别，但错误率越大
extern uint8 ITERATE_NUM;                     // 迭代次数，迭代次数越多识别越精确，但计算量越大
//#define CONDI_H_RANGE       60              // 设定色块标定的颜色范围，  在 set_color_target_condi 函数中使用
//#define CONDI_S_RANGE       80              // 设定色块标定的对比度范围，在 set_color_target_condi 函数中使用
//#define CONDI_L_RANGE       50              // 设定色块标定的亮度范围，  在 set_color_target_condi 函数中使用
extern uint8 CONDI_H_RANGE;   // 设定色块标定的颜色范围，  在 set_color_target_condi 函数中使用
extern uint8 CONDI_S_RANGE;   // 设定色块标定的对比度范围，在 set_color_target_condi 函数中使用
extern uint8 CONDI_L_RANGE;   // 设定色块标定的亮度范围，  在 set_color_target_condi 函数中使用

#define min3v(v1, v2, v3)   ((v1)>(v2)? ((v2)>(v3)?(v3):(v2)):((v1)>(v3)?(v3):(v1)))
#define max3v(v1, v2, v3)   ((v1)<(v2)? ((v2)<(v3)?(v3):(v2)):((v1)<(v3)?(v3):(v1)))
#define SWAPBYTE(h) ((((uint16_t)h << 8)&0xFF00) | ((uint16_t)h >> 8))

//#define WHITE_UP		(120)
//#define WHITE_DOWN		(0)
//#define RED_UP			(255)
//#define RED_DOWN		(185)
//#define BLUE_UP			(175)
//#define BLUE_DOWN		(125)
extern uint8 WHITE_UP;
extern uint8 WHITE_DOWN;
extern uint8 RED_UP;
extern uint8 RED_DOWN;
extern uint8 BLUE_UP;
extern uint8 BLUE_DOWN;

#define is_white(color)	((color>=WHITE_DOWN && color<=WHITE_UP)?1:0)
#define is_red(color)	((color>=RED_DOWN && color<=RED_UP)?1:0)
#define is_blue(color)	((color>=BLUE_DOWN && color<=BLUE_UP)?1:0)

typedef struct{
    unsigned char           h_min;          // 目标最小色调
    unsigned char           h_max;          // 目标最大色调

    unsigned char           s_min;          // 目标最小饱和度
    unsigned char           s_max;          // 目标最大饱和度

    unsigned char           l_min;          // 目标最小亮度
    unsigned char           l_max;          // 目标最大亮度

    unsigned int            width_min;      // 目标最小宽度
    unsigned int            hight_min;      // 目标最小高度

    unsigned int            width_max;      // 目标最大宽度
    unsigned int            hight_max;      // 目标最大高度
}target_condi_struct;                       // 判定为目标的条件

typedef struct{
    unsigned int            x;              // 目标的x坐标
    unsigned int            y;              // 目标的y坐标
    unsigned int            w;              // 目标的宽度
    unsigned int            h;              // 目标的高度
}result_struct;                             // 识别结果

typedef struct{
    unsigned char           red;            // [0,255]
    unsigned char           green;          // [0,255]
    unsigned char           blue;           // [0,255]
}color_rgb_struct;                          // RGB格式颜色

typedef struct{
    unsigned char           hue;            // [0,240]
    unsigned char           saturation;     // [0,240]
    unsigned char           luminance;      // [0,240]
}color_hsl_struct;                          // HSL格式颜色

typedef struct{
    unsigned int            x_start;        // 区域x起始位置
    unsigned int            x_end;          // 区域x结束位置
    unsigned int            y_start;        // 区域y起始位置
    unsigned int            y_end;          // 区域y结束位置
}search_area_struct;                        // 区域

extern target_condi_struct target_color_condi;  // 色块颜色阈值信息
extern result_struct target_pos_out;            // 目标位置信息

extern color_rgb_struct rgb;	// 全局 RGB 结构体
extern color_hsl_struct hsl;	// 全局 HSL 结构体

extern color_rgb_struct center_rgb;
extern color_hsl_struct center_hsl;

// 色块追踪函数
int     color_trace             (const target_condi_struct *condition,result_struct *resu);
// 设置色块目标阈值函数
void    set_color_target_condi  (uint16 rgb565_data, target_condi_struct* condition);

void rgbtohsl(const color_rgb_struct* rgb, color_hsl_struct* hsl);
void pixel_rgbtohsl(uint16* pointer);
#endif

