/******************************************************************
 * @Description  : 摄像头
 * @Author       : CMeter
 * @Date         : 2025-05-29 2:02:43
 * @LastEditTime : 2025-05-29 2:02:43
 * @FilePath     : C:\Users\10191\nxp_-ventura\Code\smartvision-YG-V1.0-2024-11-23\project\code\Uart.c
 ******************************************************************/
#include "color_tracer.h"
#include "Uart.h"
#include "math.h"

#include "Camera.h"
 
 od_result_t od_result;			// 目标坐标结构体
 
 track_status_t track_status;	// 箱子所处的当前赛道状态
 
 uint8 out_temp;				// 出界标志位

 // 判断箱子是否在当前赛道参数
 float  allow_percent=0.6;		// 当有相当于方框宽度这么多百分比的地方是蓝色时判断为箱子在赛道外或是在其他赛道
 
// 判断当前目标是否满足在赛道这一条件
uint8 box_in_saidao(void)
{
	uint16 row=0, col=0;
	uint16 row_blue_number=0;					// 行有效蓝像素点量
	uint16 col_blue_number=0;					// 一列蓝色像素点数量
	uint16 wides=0;								// 箱子的宽度
	
	// 先找到所有在赛道上的箱子
	row_blue_number=0;
	wides=abs(od_result.x2-od_result.x1);
	// 如果不是矫正模式的话那就先把赛道上的箱子都先筛出来
	// 如果是矫正模式的话就不执行此筛选程序了，直接取最近的箱子作为目标就行了
	if(!correct_flag)
	{
		for(col=od_result.x1; col<=od_result.x2; col++)
		{
			col_blue_number=0;
			if(row_blue_number>=wides*allow_percent)	// 判断为出赛道
			{
				return 0;
			}
			for(row=od_result.y2+5; row<=SCC8660_H-1; row++)
			{
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + col);	// 将该像素点颜色格式转换成 HSL
				if(!is_white(hsl.hue)) col_blue_number++;
				if(col_blue_number>=2)
				{
					row_blue_number++;
					break;	// 下一列
				}
			}
		}
	}
	else	// 矫正模式下将上部分的数据给筛掉
	{
		if(od_result.y2 < SCC8660_H*0.4) return 0;
	}
	return 1;
}

// 判断箱子所处赛道状态参数
// 边界（需>=2）
uint8 edge=2;

// 坐标点
int16 x1, x2, y1, y2;

// 赛道左右边界
int16 left_line[SCC8660_H];
int16 right_line[SCC8660_H];

// 目标往左往右找到屏幕边界的数量
int16 left_white_row_number;
int16 right_white_row_number;
// 允许的数量，大于等于此值即认为是左弯或是右弯
uint8 allow_white_row_number=5;

// 左右边界递增变化的次数
uint16 left_increase_number;
uint16 right_increase_number;
// 允许的递增次数，大于等于此值即认为在弯道处
uint8 allow_increase_number=8;

// 统计丢线的最低底线
uint16 diuxian_down_boundary=SCC8660_H-1-40;

// 边界残差
uint16 cancha;
uint16 cancha_sum;
uint16 cancha_threshold=10;		// 残差阈值
float  cancha_k=1.0;
uint16 reference_row;			// 残差计算底部参考行
uint16 cancha_up_row;			// 残差计算上区间

// 差分在判断小弯的时候还有用到，看看之后能不能取代掉
uint16 yijiechafen=0;
uint16 chafen_sum=0;
uint8  allow_chafen_sum=15;
uint8  small_corner_allow_chafen_sum=8;

// 判断弯道（左弯以及右弯）以及直道
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
	
	// 采用种子生长算法寻找边线
	// --------------------------以下为搜线程序-------------------------- //
	for(row=SCC8660_H-1; row>=y1; row--)
	{
		// 刚开始先赋一个初值
		if(row==SCC8660_H-1)
		{
			left_line[row]=edge;
			right_line[row]=SCC8660_W-1-edge;
			continue;
		}
		// 先找生长左边界
		col=left_line[row+1];
		left_line[row]=col;	// 将上一个点的横坐标继承下来
		int8 left2=-1, left1=-1, middle=-1, right1=-1, right2=-1;
		pixel_rgbtohsl(scc8660_image + row * SCC8660_W + col);
		middle=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
		// 两种情况，第一种下去是非白点，那么往右找；第二种为白点，那么往左找
		if(middle==0)	// 往右找
		{
			for(int16 i=col; i<=right_line[row+1]-10 && i<=SCC8660_W-1-edge; i++)
			{
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i-2);
				left2=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i-1);
				left1=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i);
				middle=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i+1);
				right1=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i+2);
				right2=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
				if(left2==0 && left1==0 && middle==1 && right1==1 && right2==1)
				{
					left_line[row]=i;
					break;
				}
			}
		}
		else	// 往左找
		{
			for(int16 i=col; i>=edge; i--)
			{
				if(i<=edge)
				{
					left_line[row]=edge;
					break;
				}
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i-2);
				left2=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i-1);
				left1=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i);
				middle=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i+1);
				right1=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i+2);
				right2=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
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
		// 再找生长右边界
		col=right_line[row-1];
		right_line[row]=col;	// 先赋初值
		left2=-1, left1=-1, middle=-1, right1=-1, right2=-1;
		pixel_rgbtohsl(scc8660_image + row * SCC8660_W + col);
		middle=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
		// 两种情况，第一种下去是非白点，那么往右找；第二种为白点，那么往左找
		if(middle==0)	// 往左找
		{
			for(int16 i=col; i>=left_line[row+1]+10 && i>=edge; i--)
			{
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i-2);
				left2=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i-1);
				left1=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i);
				middle=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i+1);
				right1=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i+2);
				right2=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
				if(left2==1 && left1==1 && middle==1 && right1==0 && right2==0)
				{
					right_line[row]=i;
					break;
				}
			}
		}
		else	// 往右找
		{
			for(int16 i=col; i<=SCC8660_W-1-edge; i++)
			{
				if(i>=SCC8660_W-1-edge)
				{
					right_line[row]=SCC8660_W-1-edge;
					break;
				}
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i-2);
				left2=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i-1);
				left1=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i);
				middle=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i+1);
				right1=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + i+2);
				right2=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
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
		// 解决黑块与边界间有空隙的问题
		if(left_line[row]<=edge && row!=SCC8660_H-1)	// 当前在左边界
		{
			for(col=edge; col<right_line[row+1]-30; col++)	// 往中间找
			{
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + col-2);
				left2=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + col-1);
				left1=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + col);
				middle=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + col+1);
				right1=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + col+2);
				right2=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
				
				if(left2==0 && left1==0 && middle==1 && right1==1 && right2==1)
				{
					left_line[row]=col;
					break;
				}
			}
		}
		if(right_line[row]>=SCC8660_W-1-edge && row!=SCC8660_H-1)	// 当前在右边界
		{
			for(col=SCC8660_W-1-edge; col>left_line[row+1]+30; col--)	// 往中间找
			{
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + col-2);
				left2=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + col-1);
				left1=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + col);
				middle=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + col+1);
				right1=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
				pixel_rgbtohsl(scc8660_image + row * SCC8660_W + col+2);
				right2=(is_white(hsl.hue) | is_red(hsl.hue));	// 为白色或是红色即置为 1
				
				if(left2==1 && left1==1 && middle==1 && right1==0 && right2==0)
				{
					right_line[row]=col;
					break;
				}
			}
		}
		// 统计边界点
		if(left_line[row]<=edge && row>=y1 && row<=y2 && row<=diuxian_down_boundary)
		{
			left_white_row_number++;
		}
		if(right_line[row]>=SCC8660_W-1-edge && row>=y1 && row<=y2 && row<=diuxian_down_boundary)
		{
			right_white_row_number++;
		}
	}
	// --------------------------以上为搜线程序-------------------------- //
	
	// 去噪点
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
//		// 正常去噪
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

	// --------------------------以下为状态判断程序-------------------------- //
	// 其次统计边界的增减性
	for(row=y1+(y2-y1)*0.2; row<=SCC8660_H-10; row++)
	{
		if(left_line[row]<left_line[row+1] && left_line[row]>edge) left_increase_number++;
		if(right_line[row]>right_line[row+1] && right_line[row]<SCC8660_W-1-edge) right_increase_number++;
	}
	
	// 两边都丢线了
//	zf_debug_printf("left_white:%d\r\n", left_white_row_number);
//	zf_debug_printf("right_white:%d\r\n", right_white_row_number);
	uint8 flag=1;	// 排除箱子在弯道处，并且离车体很近，在屏幕角落，产生的误判
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
		track_status=B_LEFT_I;	// 默认内弯
	}
	else if(right_white_row_number>=allow_white_row_number)
	{
//		zf_debug_printf("test2\r\n");
		track_status=B_RIGHT_I;	// 默认内弯
	}
	
	// 采用类线性回归的办法计算赛道边界是否为直线
	if(track_status==B_LEFT_I && flag)
	{
		uint16 beilv=1;
		reference_row=y2+15;
		cancha_up_row = y1+(y2-y1)*0.65;
		cancha=0;
		cancha_sum=0;
		
		// 找残差计算上部分区间
		for(row=cancha_up_row; row<=y2+5 && row<=SCC8660_H-2; row++)
		{
			if(right_line[row]<=right_line[row+1] && right_line[row]>=right_line[row-1])
			{
				cancha_up_row=row;
				break;
			}
		}
		// 找残差计算下部分区间
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
		if(cancha_sum<=cancha_threshold)	// 残差过小，判断为直道
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
		
		// 找残差计算上部分区间
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
		if(cancha_sum<=cancha_threshold)	// 残差过小，判断为直道
		{
			track_status=STRAIGHT;
			return;
		}
	}
	
	// 直道就进一步判断是否为小弯道
	// 小弯道也是会误判的，所以也使用差分信号来弥补这个误判
	// 差分在判断小弯的时候还有用到，看看之后能不能取代掉
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
			if(chafen_sum<=small_corner_allow_chafen_sum)	// 说明误判了
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
			if(chafen_sum<=small_corner_allow_chafen_sum)	// 说明误判了
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
















