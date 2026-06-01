
#ifndef MY_QEP_H_
#define MY_QEP_H_

extern float QEP_Offset;        // QEP的零位offset 第一次运行时 先开环V/F强拖 遇到索引脉冲后自动清零QPOSCNT 开环Vd吸住转自 记录此刻QPOSCNT作为offset
extern void InitEQep1_AS5047P(void);
extern void Calculate_QEP_Angle(void);


#endif /* MY_QEP_H_ */
