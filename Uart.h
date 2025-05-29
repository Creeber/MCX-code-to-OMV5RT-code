/******************************************************************
 * @Description  : 串口
 * @Author       : CMeter
 * @Date         : 2025-05-29 2:02:43
 * @LastEditTime : 2025-05-29 2:02:43
 * @FilePath     : C:\Users\10191\nxp_-ventura\Code\smartvision-YG-V1.0-2024-11-23\project\code\Uart.h
 ******************************************************************/
#ifndef _Uart_h_
#define _Uart_h_

#include "zf_common_headfile.h"

#define PARAM_INIT_BYTE		(0xAB)	// 参数初始化标志字节
#define CORRECT_BYTE		(0xAC)	// 矫正模式标志字节

#define HEADER_NUM  		(2)		// 帧头数量
#define FOOTER_NUM  		(1)		// 帧尾数量
#define DATA_SIZE_FLAG_NUM  (1) 	// 数据长度标识符占用字节数
#define CLASSIFY_NUM		(1)		// 数据类型占用字节数
#define HEADER_FOOTER_NUM   (HEADER_NUM+FOOTER_NUM)	// 帧头帧尾总数
#define HEADER1   			(0xAA)  // 帧头1
#define HEADER2   			(0xBB)  // 帧头2
#define FOOTER    			(0xDD)  // 帧尾
#define TEST_CORE_NUM		(1)		// 校验核长度占用字节数

// 串口校验结构体
struct uart_check_data_temp
{
  uint8 data;				// 存储串口读到的1字节数据
  uint8 buffer_temp[128];	// 临时存储已获取数据
  uint8 data_size_now;		// 目前接收到的数据的长度
  uint8 data_size_temp;		// 整个数据长度
  uint8 finish_flag;		// 是否完成数据完整读取
};

// 串口数据结构体
typedef struct uart_data
{
  uint8 data[128];  			// 数据，第0位为数据类型标识
  uint16 data_size; 			// 数据长度(不包含帧头帧尾、数据长度校验位)
  struct uart_check_data_temp check_temp;//校验临时变量
}uart_data;

extern uint8 mcx_send_buffer[128];	// MCX 发送数据数组
extern uart_data MCU1;				// 串口数据结构体
extern uint16 result_buffer[4];		// 结果数组
extern uint8 correct_flag;			// 矫正标志位

void user_uart_send(uint8 * send_buffer, uint32 len);
void user_result_uart_send(void);
void DataFinalProcess(uart_data* uart);
void uart_check_data(uart_data * object);
void serial_rx_interrupt_handler(uart_data* uart);













#endif
