/******************************************************************
 * @Description  : ����
 * @Author       : CMeter
 * @Date         : 2025-05-29 2:02:43
 * @LastEditTime : 2025-05-29 2:02:43
 * @FilePath     : C:\Users\10191\nxp_-ventura\Code\smartvision-YG-V1.0-2024-11-23\project\code\Uart.c
 ******************************************************************/
#include "Camera.h"
#include "color_tracer.h"

#include "Uart.h"

uart_data MCU1;				// �������ݽṹ��

uint8 mcx_send_buffer[128];	// MCX ������������

uint16 result_buffer[4];	// �������

uint8 correct_flag;	// ������־λ

extern uint8 mcx_param_init_flag;

// �û����ڷ�������
void user_uart_send(uint8 * send_buffer, uint32 len)
{
	uint32 i=0;
	uint8 buffer[128]={HEADER1, HEADER2};
	
	memcpy(buffer+HEADER_NUM, send_buffer, len);
	buffer[HEADER_NUM+len] = FOOTER;
	
	user_uart_write_buffer(buffer, sizeof(buffer));
	user_uart_write_buffer(buffer, sizeof(buffer));
	user_uart_write_buffer(buffer, sizeof(buffer));
	user_uart_write_buffer(buffer, sizeof(buffer));
}

// ���ڷ��Ϳ�ѡ���꺯��
void user_result_uart_send(void)
{
	result_buffer[0] = od_result.x1;
	result_buffer[1] = od_result.y1;
	result_buffer[0] = od_result.x2;
	result_buffer[1] = od_result.y2;
	
	user_uart_putchar(HEADER1);
	user_uart_putchar(HEADER2);
	user_uart_putchar(0x0D);	// ���ݰ�����
	user_uart_write_buffer((const uint8*)(result_buffer), sizeof(result_buffer));
	user_uart_putchar(track_status);
	user_uart_putchar(FOOTER);
}

// �������ݴ�����
void DataFinalProcess(uart_data* uart)
{
	if(uart->data[0]==PARAM_INIT_BYTE)		// ���յ�������ʼ������
	{
		uint32 i=1;
		
		CONDI_H_RANGE 	= uart->data[i]; i++;
		CONDI_S_RANGE 	= uart->data[i]; i++;
		CONDI_L_RANGE 	= uart->data[i]; i++;
		ALLOW_FAIL_PER	= uart->data[i]; i++;
		ITERATE_NUM   	= uart->data[i]; i++;
		WHITE_UP 		= uart->data[i]; i++;
		WHITE_DOWN 		= uart->data[i]; i++;
		RED_UP			= uart->data[i]; i++;
		RED_DOWN		= uart->data[i]; i++;
		BLUE_UP   		= uart->data[i]; i++;
		BLUE_DOWN   	= uart->data[i]; i++;
		// ��ʼ���ɹ���־λ
		mcx_param_init_flag=1;
	}
	else if(uart->data[0]==CORRECT_BYTE)	// ���յ�����ģʽ����
	{
		correct_flag = uart->data[1];
	}
	else
	{
		ips200_show_string(170, 0, "Receive_Error!!!");
	}
}

// У�鴮�����ݺ���
void uart_check_data(uart_data * object)
{
	object->check_temp.data = LPUART_ReadByte(USER_USART);
	object->check_temp.buffer_temp[object->check_temp.data_size_now] = object->check_temp.data;
	object->check_temp.data_size_now++;
	if(object->check_temp.data_size_now>128)// ���ݹ�������ձ���
	{
		memset(object->check_temp.buffer_temp, 0, sizeof(object->check_temp.buffer_temp));
		object->check_temp.data_size_temp = 0;
		object->check_temp.data_size_now = 0;
		return;
	}
	// ֡ͷ֡βУ��
	if((object->check_temp.data_size_now == 1 && object->check_temp.data != HEADER1)||(object->check_temp.data_size_now == 2 && object->check_temp.data != HEADER2)||(object->check_temp.data_size_now > 3 && object->check_temp.data_size_now == object->check_temp.data_size_temp && object->check_temp.data != FOOTER))
	{
		// У�鲻ͨ��
		memset(object->check_temp.buffer_temp, 0, sizeof(object->check_temp.buffer_temp));
		object->check_temp.data_size_temp = 0;
		object->check_temp.data_size_now = 0;
	}
	else if(object->check_temp.data_size_now == HEADER_NUM+DATA_SIZE_FLAG_NUM)// ���ݳ���
	{
		object->check_temp.data_size_temp = object->check_temp.data;// �洢���ݳ���
	}
	else if(object->check_temp.data_size_now == object->check_temp.data_size_temp && object->check_temp.data == FOOTER)
	{
		memcpy(object->data, object->check_temp.buffer_temp+HEADER_NUM+DATA_SIZE_FLAG_NUM, object->check_temp.data_size_temp-DATA_SIZE_FLAG_NUM-HEADER_FOOTER_NUM);
		object->data_size = object->check_temp.data_size_temp - HEADER_FOOTER_NUM - DATA_SIZE_FLAG_NUM;
		
		object->check_temp.finish_flag = 1;// ���ȫ��У��
		// ���ñ���
		memset(object->check_temp.buffer_temp, 0, sizeof(object->check_temp.buffer_temp));
		object->check_temp.data_size_temp = 0;
		object->check_temp.data_size_now = 0;
	}
}

// �����жϻص�����
void serial_rx_interrupt_handler(uart_data* uart)
{
	uart_check_data(uart);	// ����У��
	
	if(uart->check_temp.finish_flag == 1)	// ������������ݶ�ȡ��У��
	{
		DataFinalProcess(uart);	// ����ȥ�������ʽ������
		//���ñ���
		uart->check_temp.finish_flag = 0;
		memset(uart->data, 0, sizeof(uart->data));
		uart->data_size = 0;
	}
}
























