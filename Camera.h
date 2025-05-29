/******************************************************************
 * @Description  : 摄像头
 * @Author       : CMeter
 * @Date         : 2025-05-29 2:02:43
 * @LastEditTime : 2025-05-29 2:02:43
 * @FilePath     : C:\Users\10191\nxp_-ventura\Code\smartvision-YG-V1.0-2024-11-23\project\code\Uart.c
 ******************************************************************/
#ifndef _Camera_h_
#define _Camera_h_

#include "zf_common_headfile.h"

// 赛道信息枚举
typedef enum
{
	STRAIGHT,	// 0	直道
	S_LEFT,		// 1	小左弯
	S_RIGHT,	// 2	小右弯
	B_LEFT_I,	// 3	大左弯内弯
	B_RIGHT_I,	// 4	大右弯内弯
	B_LEFT_O,	// 5	大左弯外弯
	B_RIGHT_O	// 6	大右弯外弯
}track_status_t;

typedef struct
{
    uint16 x1;
    uint16 y1;
    uint16 x2;
    uint16 y2;
}od_result_t;

extern od_result_t od_result;	// 目标坐标
extern track_status_t track_status;

uint8 box_in_saidao(void);
void judge_corner(void);




#endif


