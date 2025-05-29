/******************************************************************
 * @Description  : ����ͷ
 * @Author       : CMeter
 * @Date         : 2025-05-29 2:02:43
 * @LastEditTime : 2025-05-29 2:02:43
 * @FilePath     : C:\Users\10191\nxp_-ventura\Code\smartvision-YG-V1.0-2024-11-23\project\code\Uart.c
 ******************************************************************/
#include "color_tracer.h"
#include "Uart.h"
#include "math.h"

#include "Camera.h"
 
 od_result_t od_result;			// Ŀ������ṹ��
 
 track_status_t track_status;	// ���������ĵ�ǰ����״̬
 
 uint8 out_temp;				// �����־λ

 // �ж������Ƿ��ڵ�ǰ��������
 float  allow_percent=0.6;		// �����൱�ڷ�������ô��ٷֱȵĵط�����ɫʱ�ж�Ϊ�������������������������
 
// �жϵ�ǰĿ���Ƿ�������������һ����
uint8 box_in_saidao(void)
{
	uint16 row=0, col=0;
	uint16 row_blue_number=0;					// ����Ч�����ص���
	uint16 col_blue_number=0;					// һ����ɫ���ص�����
	uint16 wides=0;								// ���ӵĿ��
	
	// ���ҵ������������ϵ�����
	row_blue_number=0;
	wides=abs(od_result.x2-od_result.x1);
	// ������ǽ���ģʽ�Ļ��Ǿ��Ȱ������ϵ����Ӷ���ɸ����
	// ����ǽ���ģʽ�Ļ��Ͳ�ִ�д�ɸѡ�����ˣ�ֱ��ȡ�����������ΪĿ�������
	if(!correct_flag)
	{
		for(col=od_result.x1; col<=od_result.x2; col++)
		{
			col_blue_number=0;
			if(row_blue_number>=wides*allow_percent)	// �ж�Ϊ������
			{
				return 0;
			}
			for(row=od_result.y2+5; row<=SCC8660_H-1; row++)
			{
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + col);	// �������ص���ɫ��ʽת���� HSL
				if(!is_white(hsl.hue)) col_blue_number++;
				if(col_blue_number>=2)
				{
					row_blue_number++;
					break;	// ��һ��
				}
			}
		}
	}
	else	// ����ģʽ�½��ϲ��ֵ����ݸ�ɸ��
	{
		if(od_result.y2 < SCC8660_H*0.4) return 0;
	}
	return 1;
}

// �ж�������������״̬����
// �߽磨��>=2��
uint8 edge=2;

// �����
int16 x1, x2, y1, y2;

// �������ұ߽�
int16 left_line[SCC8660_H];
int16 right_line[SCC8660_H];

// Ŀ�����������ҵ���Ļ�߽������
int16 left_white_row_number;
int16 right_white_row_number;
// ��������������ڵ��ڴ�ֵ����Ϊ�������������
uint8 allow_white_row_number=5;

// ���ұ߽�����仯�Ĵ���
uint16 left_increase_number;
uint16 right_increase_number;
// ����ĵ������������ڵ��ڴ�ֵ����Ϊ�������
uint8 allow_increase_number=8;

// ͳ�ƶ��ߵ���͵���
uint16 diuxian_down_boundary=SCC8660_H-1-40;

// �߽�в�
uint16 cancha;
uint16 cancha_sum;
uint16 cancha_threshold=10;		// �в���ֵ
float  cancha_k=1.0;
uint16 reference_row;			// �в����ײ��ο���
uint16 cancha_up_row;			// �в����������

// ������ж�С���ʱ�����õ�������֮���ܲ���ȡ����
uint16 yijiechafen=0;
uint16 chafen_sum=0;
uint8  allow_chafen_sum=15;
uint8  small_corner_allow_chafen_sum=8;

// �ж�����������Լ����䣩�Լ�ֱ��
void judge_corner(void)
{
	int16 row=0, col=0;
	x1 = od_result.x1;
	x2 = od_result.x2;
	y1 = od_result.y1;
	y2 = od_result.y2;
	for(int16 i=0; i<SCC8660_H; i++) left_line[i]=edge;
	for(int16 i=0; i<SCC8660_H; i++) right_line[i]=SCC8660_W-1-edge;
	left_white_row_number=0;
	right_white_row_number=0;
	left_increase_number=0;
	right_increase_number=0;
	track_status=STRAIGHT;
	
	// �������������㷨Ѱ�ұ���
	// --------------------------����Ϊ���߳���-------------------------- //
	for(row=SCC8660_H-1; row>=y1; row--)
	{
		// �տ�ʼ�ȸ�һ����ֵ
		if(row==SCC8660_H-1)
		{
			left_line[row]=edge;
			right_line[row]=SCC8660_W-1-edge;
			continue;
		}
		// ����������߽�
		col=left_line[row+1];
		left_line[row]=col;	// ����һ����ĺ�����̳�����
		int8 left2=-1, left1=-1, middle=-1, right1=-1, right2=-1;
		pixel_rgbtohsl(scc8660_image + row * SCC8660_W + col);
		middle=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
		// �����������һ����ȥ�Ƿǰ׵㣬��ô�����ң��ڶ���Ϊ�׵㣬��ô������
		if(middle==0)	// ������
		{
			for(int16 i=col; i<=right_line[row+1]-10 && i<=SCC8660_W-1-edge; i++)
			{
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i-2);
				left2=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i-1);
				left1=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i);
				middle=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i+1);
				right1=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i+2);
				right2=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				if(left2==0 && left1==0 && middle==1 && right1==1 && right2==1)
				{
					left_line[row]=i;
					break;
				}
			}
		}
		else	// ������
		{
			for(int16 i=col; i>=edge; i--)
			{
				if(i<=edge)
				{
					left_line[row]=edge;
					break;
				}
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i-2);
				left2=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i-1);
				left1=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i);
				middle=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i+1);
				right1=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i+2);
				right2=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				if(left2==0 && left1==0 && middle==1 && right1==1 && right2==1)
				{
					left_line[row]=i;
					break;
				}
				if(left2==0 && left1==0 && middle==0 && right1==0 && right2==0)
				{
					break;
				}
			}
		}
		// ���������ұ߽�
		col=right_line[row-1];
		right_line[row]=col;	// �ȸ���ֵ
		left2=-1, left1=-1, middle=-1, right1=-1, right2=-1;
		pixel_rgbtohsl(scc8660_image + row * SCC8660_W + col);
		middle=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
		// �����������һ����ȥ�Ƿǰ׵㣬��ô�����ң��ڶ���Ϊ�׵㣬��ô������
		if(middle==0)	// ������
		{
			for(int16 i=col; i>=left_line[row+1]+10 && i>=edge; i--)
			{
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i-2);
				left2=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i-1);
				left1=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i);
				middle=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i+1);
				right1=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i+2);
				right2=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				if(left2==1 && left1==1 && middle==1 && right1==0 && right2==0)
				{
					right_line[row]=i;
					break;
				}
			}
		}
		else	// ������
		{
			for(int16 i=col; i<=SCC8660_W-1-edge; i++)
			{
				if(i>=SCC8660_W-1-edge)
				{
					right_line[row]=SCC8660_W-1-edge;
					break;
				}
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i-2);
				left2=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i-1);
				left1=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i);
				middle=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i+1);
				right1=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i+2);
				right2=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				if(left2==1 && left1==1 && middle==1 && right1==0 && right2==0)
				{
					right_line[row]=i;
					break;
				}
				if(left2==0 && left1==0 && middle==0 && right1==0 && right2==0)
				{
					break;
				}
			}
		}
		// ����ڿ���߽���п�϶������
		if(left_line[row]<=edge && row!=SCC8660_H-1)	// ��ǰ����߽�
		{
			for(col=edge; col<right_line[row+1]-30; col++)	// ���м���
			{
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + col-2);
				left2=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + col-1);
				left1=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + col);
				middle=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + col+1);
				right1=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + col+2);
				right2=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				
				if(left2==0 && left1==0 && middle==1 && right1==1 && right2==1)
				{
					left_line[row]=col;
					break;
				}
			}
		}
		if(right_line[row]>=SCC8660_W-1-edge && row!=SCC8660_H-1)	// ��ǰ���ұ߽�
		{
			for(col=SCC8660_W-1-edge; col>left_line[row+1]+30; col--)	// ���м���
			{
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + col-2);
				left2=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + col-1);
				left1=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + col);
				middle=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + col+1);
				right1=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + col+2);
				right2=(is_white(hsl.hue) | is_red(hsl.hue));	// Ϊ��ɫ���Ǻ�ɫ����Ϊ 1
				
				if(left2==1 && left1==1 && middle==1 && right1==0 && right2==0)
				{
					right_line[row]=col;
					break;
				}
			}
		}
		// ͳ�Ʊ߽��
		if(left_line[row]<=edge && row>=y1 && row<=y2 && row<=diuxian_down_boundary)
		{
			left_white_row_number++;
		}
		if(right_line[row]>=SCC8660_W-1-edge && row>=y1 && row<=y2 && row<=diuxian_down_boundary)
		{
			right_white_row_number++;
		}
	}
	// --------------------------����Ϊ���߳���-------------------------- //
	
	// ȥ���
//	for(row=SCC8660_H-1-5; row>=y1+5; row--)
//	{
//		if(left_line[row]==edge && row>=y1 && row<=y2 && left_line[row-5]>edge && left_line[row+5]>edge)
//		{
//			left_line[row]=(left_line[row-5]+left_line[row+5])*0.5;
//		}
//		if(right_line[row]==SCC8660_W-1-edge && row>=y1 && row<=y2 && right_line[row-5]<SCC8660_W-1-edge && right_line[row+5]<SCC8660_W-1-edge)
//		{
//			right_line[row]=(right_line[row-5]+right_line[row+5])*0.5;
//		}
//		
//		// ����ȥ��
//		if(left_line[row]==edge)
//		{
//			if(left_line[row-2]>edge && left_line[row+2]>edge)
//			{
//				left_line[row]=(left_line[row-2]+left_line[row+2])*0.5;
//			}
//		}
//		if(right_line[row]==SCC8660_W-1-edge)
//		{
//			if(right_line[row-2]<SCC8660_W-1-edge && right_line[row+2]<SCC8660_W-1-edge)
//			{
//				right_line[row]=(right_line[row-2]+right_line[row+2])*0.5;
//			}
//		}
//	}

	// --------------------------����Ϊ״̬�жϳ���-------------------------- //
	// ���ͳ�Ʊ߽��������
	for(row=y1+(y2-y1)*0.2; row<=SCC8660_H-10; row++)
	{
		if(left_line[row]<left_line[row+1] && left_line[row]>edge) left_increase_number++;
		if(right_line[row]>right_line[row+1] && right_line[row]<SCC8660_W-1-edge) right_increase_number++;
	}
	
	// ���߶�������
//	zf_debug_printf("left_white:%d\r\n", left_white_row_number);
//	zf_debug_printf("right_white:%d\r\n", right_white_row_number);
	uint8 flag=1;	// �ų�������������������복��ܽ�������Ļ���䣬����������
	if(left_white_row_number>=allow_white_row_number && right_white_row_number>=allow_white_row_number)
	{
		if(left_increase_number<=allow_increase_number && right_increase_number<=allow_increase_number)
		{
			flag=0;
			if(right_white_row_number>left_white_row_number || right_increase_number>left_increase_number) track_status=B_RIGHT_I;
			else if(right_white_row_number<left_white_row_number || left_increase_number>right_increase_number) track_status=B_LEFT_I;
			else track_status=STRAIGHT;
//			zf_debug_printf("test1\r\n");
		}
		else if(left_increase_number<=allow_increase_number && right_increase_number>allow_increase_number) track_status=B_RIGHT_I;
		else if(left_increase_number>allow_increase_number && right_increase_number<=allow_increase_number) track_status=B_LEFT_I;
//		zf_debug_printf("left_increase:%d\r\n", left_increase_number);
//		zf_debug_printf("right_increase:%d\r\n", right_increase_number);
	}
	else if(left_white_row_number>=allow_white_row_number)
	{
		track_status=B_LEFT_I;	// Ĭ������
	}
	else if(right_white_row_number>=allow_white_row_number)
	{
//		zf_debug_printf("test2\r\n");
		track_status=B_RIGHT_I;	// Ĭ������
	}
	
	// ���������Իع�İ취���������߽��Ƿ�Ϊֱ��
	if(track_status==B_LEFT_I && flag)
	{
		uint16 beilv=1;
		reference_row=y2+15;
		cancha_up_row = y1+(y2-y1)*0.65;
		cancha=0;
		cancha_sum=0;
		
		// �Ҳв�����ϲ�������
		for(row=cancha_up_row; row<=y2+5 && row<=SCC8660_H-2; row++)
		{
			if(right_line[row]<=right_line[row+1] && right_line[row]>=right_line[row-1])
			{
				cancha_up_row=row;
				break;
			}
		}
		// �Ҳв�����²�������
		for(row=y2; row<=SCC8660_H-1; row++)
		{
			if(right_line[row]>=SCC8660_W-1-edge)
			{
				reference_row=row;
				break;
			}
		}
		cancha_k = ((cancha_up_row)-reference_row==0) ? (1) : (fabs(((right_line[cancha_up_row]-right_line[reference_row])*1.0/((cancha_up_row)-reference_row))));
		for(row=reference_row-1; row>=cancha_up_row; row--)
		{
			uint16 need_x = right_line[reference_row] - cancha_k*(beilv++);
			cancha = abs(need_x-right_line[row]);
			if(cancha<=1) cancha=0;
			cancha_sum += cancha;
		}
		if(cancha_sum<=cancha_threshold)	// �в��С���ж�Ϊֱ��
		{
			track_status=STRAIGHT;
			return;
		}
	}
	else if(track_status==B_RIGHT_I && flag)
	{
		uint16 beilv=1;
		reference_row=y2+15;
		cancha_up_row = y1+(y2-y1)*0.65;
		cancha=0;
		cancha_sum=0;
		
		// �Ҳв�����ϲ�������
		for(row=cancha_up_row; row<=y2+5 && row<=SCC8660_H-2; row++)
		{
			if(left_line[row]>=left_line[row+1] && left_line[row]<=left_line[row-1])
			{
				cancha_up_row=row;
				break;
			}
		}
		for(row=y2; row<=SCC8660_H-1; row++)
		{
			if(left_line[row]<=edge)
			{
				reference_row=row;
				break;
			}
		}
		cancha_k = ((cancha_up_row)-reference_row==0) ? (1) : (fabs(((left_line[cancha_up_row]-left_line[reference_row])*1.0/((cancha_up_row)-reference_row))));
		for(row=reference_row-1; row>=cancha_up_row; row--)
		{
			uint16 need_x = left_line[reference_row] + cancha_k*(beilv++);
			cancha = abs(need_x-left_line[row]);
			if(cancha<=1) cancha=0;
			cancha_sum += cancha;
		}
		if(cancha_sum<=cancha_threshold)	// �в��С���ж�Ϊֱ��
		{
			track_status=STRAIGHT;
			return;
		}
	}
	
	// ֱ���ͽ�һ���ж��Ƿ�ΪС���
	// С���Ҳ�ǻ����еģ�����Ҳʹ�ò���ź����ֲ��������
	// ������ж�С���ʱ�����õ�������֮���ܲ���ȡ����
	yijiechafen=0;
	chafen_sum=0;
	if(track_status==STRAIGHT)
	{
		if(left_increase_number>=allow_increase_number)
		{
			for(row=SCC8660_H-2; row>=y1+(y2-y1)*0.2; row--)
			{
				yijiechafen = abs(right_line[row] - right_line[row+1]);
				if(yijiechafen<=1) yijiechafen=0;
				chafen_sum += yijiechafen;
			}
			if(chafen_sum<=small_corner_allow_chafen_sum)	// ˵��������
			{
				track_status=STRAIGHT;
				return;
			}
			else
			{
				track_status=S_LEFT;
				return;
			}
		}
		if(right_increase_number>=allow_increase_number)
		{
			for(row=SCC8660_H-2; row>=y1+(y2-y1)*0.2; row--)
			{
				yijiechafen = abs(left_line[row] - left_line[row+1]);
				if(yijiechafen<=1) yijiechafen=0;
				chafen_sum += yijiechafen;
			}
			if(chafen_sum<=small_corner_allow_chafen_sum)	// ˵��������
			{
				track_status=STRAIGHT;
				return;
			}
			else
			{
				track_status=S_RIGHT;
				return;
			}
		}
	}
}
















