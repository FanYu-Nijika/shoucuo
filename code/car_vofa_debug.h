#ifndef CODE_CAR_VOFA_DEBUG_H_
#define CODE_CAR_VOFA_DEBUG_H_

/* 暂停 VOFA 发包和 UART2 测试输出；接收命令仍可更新参数。 */
#define CAR_VOFA_TX_ENABLED (0)

void car_vofa_debug_init(void);
void car_vofa_debug_rx_isr(void);
void car_vofa_debug_capture_5ms(void);
void car_vofa_debug_task(void);

#endif
