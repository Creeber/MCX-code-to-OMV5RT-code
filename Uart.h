/******************************************************************
 * @Description  : ����
 * @Author       : CMeter
 * @Date         : 2025-05-29 2:02:43
 * @LastEditTime : 2025-05-29 2:02:43
 * @FilePath     : C:\Users\10191\nxp_-ventura\Code\smartvision-YG-V1.0-2024-11-23\project\code\Uart.h
 ******************************************************************/
#ifndef _Uart_h_
#define _Uart_h_

#include "zf_common_headfile.h"

#define PARAM_INIT_BYTE		(0xAB)	// ������ʼ����־�ֽ�
#define CORRECT_BYTE		(0xAC)	// ����ģʽ��־�ֽ�

#define HEADER_NUM  		(2)		// ֡ͷ����
#define FOOTER_NUM  		(1)		// ֡β����
#define DATA_SIZE_FLAG_NUM  (1) 	// ���ݳ��ȱ�ʶ��ռ���ֽ���
#define CLASSIFY_NUM		(1)		// ��������ռ���ֽ���
#define HEADER_FOOTER_NUM   (HEADER_NUM+FOOTER_NUM)	// ֡ͷ֡β����
#define HEADER1   			(0xAA)  // ֡ͷ1
#define HEADER2   			(0xBB)  // ֡ͷ2
#define FOOTER    			(0xDD)  // ֡β
#define TEST_CORE_NUM		(1)		// У��˳���ռ���ֽ���

// ����У��ṹ��
struct uart_check_data_temp
{
  uint8 data;				// �洢���ڶ�����1�ֽ�����
  uint8 buffer_temp[128];	// ��ʱ�洢�ѻ�ȡ����
  uint8 data_size_now;		// Ŀǰ���յ������ݵĳ���
  uint8 data_size_temp;		// �������ݳ���
  uint8 finish_flag;		// �Ƿ��������������ȡ
};

// �������ݽṹ��
typedef struct uart_data
{
  uint8 data[128];  			// ���ݣ���0λΪ�������ͱ�ʶ
  uint16 data_size; 			// ���ݳ���(������֡ͷ֡β�����ݳ���У��λ)
  struct uart_check_data_temp check_temp;//У����ʱ����
}uart_data;

extern uint8 mcx_send_buffer[128];	// MCX ������������
extern uart_data MCU1;				// �������ݽṹ��
extern uint16 result_buffer[4];		// �������
extern uint8 correct_flag;			// ������־λ

void user_uart_send(uint8 * send_buffer, uint32 len);
void user_result_uart_send(void);
void DataFinalProcess(uart_data* uart);
void uart_check_data(uart_data * object);
void serial_rx_interrupt_handler(uart_data* uart);













#endif
