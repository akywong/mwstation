#ifndef __SERIAL_RTX_H__
#define __SERIAL_RTX_H__
#include "sys.h"

#define SSB_LEN (128)
#define SSB_LEN_MASK (127)

#define SRB_LEN (64)
#define SRB_LEN_MASK (63)

//表示一个用于串口数据发送的缓冲区
struct serial_send_buffer {
    uint8_t data[SSB_LEN];
    uint8_t write;
    uint8_t read;
    uint8_t send_start;
    USART_TypeDef *uart;
};
//表示一个用于串口数据接收的缓冲区
struct serial_recv_buffer {
    uint8_t data[SRB_LEN];
    uint8_t len;
    uint8_t head;
    void (*cb)(uint8_t *frame, uint8_t len);
    USART_TypeDef *uart;
};
uint8_t ssb_data_len(struct serial_send_buffer *sb);
void srb_recv_byte(struct serial_recv_buffer *sb);
uint8_t ssb_write_frame(struct serial_send_buffer *sb, uint8_t *frame, uint8_t len);
uint8_t ssb_write_data(struct serial_send_buffer *sb, uint8_t *data, uint8_t len);
uint8_t ssb_pool_len(struct serial_send_buffer *sb);
void ssb_send_byte(struct serial_send_buffer *sb);
void ssb_send_start(struct serial_send_buffer *sb);
uint8_t srb_data_len(struct serial_recv_buffer *sb);
#endif
