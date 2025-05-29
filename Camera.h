/******************************************************************
 * @Description  : ����ͷ
 * @Author       : CMeter
 * @Date         : 2025-05-29 2:02:43
 * @LastEditTime : 2025-05-29 2:02:43
 * @FilePath     : C:\Users\10191\nxp_-ventura\Code\smartvision-YG-V1.0-2024-11-23\project\code\Uart.c
 ******************************************************************/
#ifndef _Camera_h_
#define _Camera_h_

#include "zf_common_headfile.h"

// ������Ϣö��
typedef enum
{
	STRAIGHT,	// 0	ֱ��
	S_LEFT,		// 1	С����
	S_RIGHT,	// 2	С����
	B_LEFT_I,	// 3	����������
	B_RIGHT_I,	// 4	����������
	B_LEFT_O,	// 5	����������
	B_RIGHT_O	// 6	����������
}track_status_t;

typedef struct
{
    uint16 x1;
    uint16 y1;
    uint16 x2;
    uint16 y2;
}od_result_t;

extern od_result_t od_result;	// Ŀ������
extern track_status_t track_status;

uint8 box_in_saidao(void);
void judge_corner(void);




#endif


