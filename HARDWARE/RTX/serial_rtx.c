#include "serial_rtx.h"

// 检查目前缓冲区中的数据长度
uint8_t ssb_data_len(struct serial_send_buffer *sb)
{
    uint8_t len = (sb->write + 128) - sb->read;
    return len & SSB_LEN_MASK;
}
// 检查空闲位置的长度
uint8_t ssb_pool_len(struct serial_send_buffer *sb)
{
    return SSB_LEN - ssb_data_len(sb) - 1;
}
uint8_t USART_ReceiveData8(USART_TypeDef *USART_COM)
{
	uint16_t value;
	value = USART_ReceiveData(USART_COM);
	return (uint8_t)(value&0xFF);
}
void USART_SendData8(USART_TypeDef *USART_COM, uint8_t c)
{
	uint16_t value = c;
	USART_SendData(USART_COM, value);
}
// 写入连续的数据
uint8_t ssb_write_data(struct serial_send_buffer *sb, uint8_t *data, uint8_t len)
{
    uint8_t ret = len;
    if(len > ssb_pool_len(sb)) {
        return 0;
    }
    while(len > 0) {
        sb->data[sb->write++] = *data++;
        sb->write &= SSB_LEN_MASK;
        len--;
    }
    return ret;
}
// 写入一帧数据，自动添加c0的同步头和结尾
uint8_t ssb_write_frame(struct serial_send_buffer *sb, uint8_t *frame, uint8_t len)
{
    // 首先需要判断这帧数据最后的长度
    uint8_t *src = frame;
    uint8_t total_len = 0;
    total_len++; // 开始时的0xC0
    while(src < frame + len) {
        if(*src == 0xC0) {
            total_len += 2;
        } else if(*src == 0xDB) {
            total_len += 2;
        } else {
            total_len += 1;
        }
        src++;
    }
    total_len++;// 结尾时的0xC0
    if(len > ssb_pool_len(sb)) {
        return 0;
    }

    src = frame;
    /* 将内容中的0xC0转义为0xDB DC, 将内容中的0xDB转义为0xDB 0xDD */
    /* 填充头部 */
    sb->data[sb->write++] = 0xC0;
    sb->write &= SSB_LEN_MASK;
    while(src < frame + len) {
        uint8_t ts = *src++;
        if(ts == 0xC0) {
            sb->data[sb->write++] = 0xDB;
            sb->write &= SSB_LEN_MASK;
            sb->data[sb->write++] = 0xDC;
            sb->write &= SSB_LEN_MASK;
        } else if(ts == 0xDB) {
            sb->data[sb->write++] = 0xDB;
            sb->write &= SSB_LEN_MASK;
            sb->data[sb->write++] = 0xDD;
            sb->write &= SSB_LEN_MASK;
        } else {
            sb->data[sb->write++] = ts;
            sb->write &= SSB_LEN_MASK;
        }
    }
    sb->data[sb->write++] = 0xC0;
    sb->write &= SSB_LEN_MASK;
    return len;
}

void srb_recv_byte(struct serial_recv_buffer *sb)
{
 uint8_t c = USART_ReceiveData8(sb->uart);
    if(c == 0x24) {
      sb->len = 1;
      sb->head = 1;
      sb->data[0]= 0x24;
    } else if(c == 0x0A){
      if(sb->head == 1){
        sb->data[sb->len++] = 0x0A;
        sb->cb(sb->data,sb->len);
        sb->len = 0;
        sb->head = 0;
      } 
    } else {
      if(sb->head == 1) {
        sb->data[sb->len++] = c;
      }
    }
    if(sb->len > SRB_LEN - 2) {
        sb->len = 0;
        sb->head = 0;
    }
    USART_ClearFlag(sb->uart, USART_FLAG_RXNE);
}

void ssb_send_byte(struct serial_send_buffer *sb)
{
    if(sb->read != sb->write) {
        USART_SendData8(sb->uart, sb->data[sb->read++]);
        sb->read &= SSB_LEN_MASK;
    }
    /* 如果没有更多的数据需要发送,关闭串口中断 */
    if(sb->read == sb->write) {
        sb->send_start = 0;
    } else {
        sb->send_start = 1;
    }
    USART_ClearFlag(sb->uart, USART_FLAG_TC);
}

void ssb_send_start(struct serial_send_buffer *sb)
{
    if(sb->send_start) {
        return;
    } else {
        sb->send_start = 1;
        ssb_send_byte(sb);
    }
}
