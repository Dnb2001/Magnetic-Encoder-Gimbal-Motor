#include "my_epwm.h"
#include "my_adc.h"
#include "my_foc.h"

#define TBPRD_ALL 3750      // 计数周期
#define TBPRD_init 1875     // 计数器初始值

void InitTzPwm(void)
{
    EALLOW;
    // 1. 使能 OSHT (One-Shot) 触发源为 TZ1
    // Trip-Zone Select Register（错误控制选择寄存器）。决定哪些外部引脚信号（TZ1 ~ TZ6）或内部逻辑信号可以触发 PWM 的保护动作 包含OSHT1~6和CBC1~6
    // 使用One-Shot模式， 指定的 TZ 引脚检测到低电平（触发信号），ePWM 模块会立即锁死 PWM 输出,强制为预设的状态， 需要往 TZCLR 寄存器里写 1 来清除这个标志位
    // CBC (Cycle-By-Cycle) 仅对当前 PWM 周期生效，在下一个 PWM 周期的计数器等于 0 时自动恢复
    EPwm1Regs.TZSEL.bit.OSHT1 = 1;
    EPwm2Regs.TZSEL.bit.OSHT1 = 1;
    EPwm3Regs.TZSEL.bit.OSHT1 = 1; // 如果要改为 TZ2，应该写成：    EPwm1Regs.TZSEL.bit.OSHT2 = 1;
    // 2. 定义触发后的动作：强制 A 路和 B 路输出为低电平 (最安全)
    // 0: 高阻, 1: 强制高, 2: 强制低, 3: 无动作
    // 15:4 Reserved， 3:2 TZB， 1:0 TZA
    EPwm1Regs.TZCTL.bit.TZA = 2;
    EPwm1Regs.TZCTL.bit.TZB = 2;
    EPwm2Regs.TZCTL.bit.TZA = 2;
    EPwm2Regs.TZCTL.bit.TZB = 2;
    EPwm3Regs.TZCTL.bit.TZA = 2;
    EPwm3Regs.TZCTL.bit.TZB = 2;
    // 3. 使能 TZ 中断信号产生
    // 当硬件保护触发（跳闸）的瞬间，给 CPU 发一个中断信号,收到中断后，CPU 执行中断服务函数
    // 把状态机从 STATE_RUN 强行切到 STATE_FAULT , 清零电流环的 PI 积分器，并通过 CAN 总线向外发送“逆变器过流”的故障码
    EPwm1Regs.TZEINT.bit.OST = 1;
    EPwm2Regs.TZEINT.bit.OST = 1;
    EPwm3Regs.TZEINT.bit.OST = 1;
    EDIS;
}
void InitEPwm1()  // 周期 相位 计数器初始值 增减模式 时钟 CMPA初值 动作 死区 触发ADC 同步
{
    // --- TB 子模块配置 ---     TB (Time-Base，时间基准)
    EPwm1Regs.TBPRD = TBPRD_ALL;         // Period Register - 周期寄存器  bit0-15全部有效
    EPwm1Regs.TBPHS.half.TBPHS = 0x0000; // 相位为 0
                                         // Phase Register - 相位寄存器 决定当前 ePWM 模块相对于另一个 ePWM 模块的时间差（相位差）
    EPwm1Regs.TBCTR = 0x0000;            // 清零计数器
                                         // Time-Base Counter Register - 计数器寄存器 Bits 0-15 全有效 不断在 0 到 TBPRD 之间变化

    /* TBCTL   FREE32 (Bits 15:14) —— 仿真挂起模式 在 CCS 里点“暂停（Suspend）”时，这个位决定 PWM 硬件是立刻停下、还是数完当前这一个周期再停下、还是完全不管继续跑
     *         PHSDIR (Bit 13)     —— 同步后的计数方向 仅在 CTRMODE 设为 Up-Down 模式时有效。当发生同步事件后，计数器是从 TBPHS 的值向上数（写1）还是向下数（写0）。
     *         CLKDIV (Bits 12:10) —— 标准时钟分频
     *         HSPCLKDIV (Bits 9:7)—— 高频外设时钟分频  两个串联 总分频 = HSPCLKDIV * CLKDIV
     *         SWFSYNC (Bit 6)     —— 软件强制同步脉冲  写 1 可以在软件上人为造出一个同步事件。通常用于离线初始化或者仿真测试
     *         SYNCOSEL (Bits 5:4) —— 同步输出选择 决定当前模块何时向外界（下一个 PWM 模块）发送同步脉冲（SYNCO）
     *                                00：SYNCIN 信号直接透传（作为外界的转接桥梁） 01：当 CTR = 0 时发送同步脉冲
     *                                10：当 CTR = CMPB 时发送同步脉冲    11：禁用同步输出
     *                                F28335 内部的 ePWM 模块同步信号只能是 外部 -> ePWM1 -> ePWM2 -> ePWM3 ... 单向传递
     *                                ePWM1 绝对的老大 必须且只能配置为当  CTR = 0 时发出同步信号 模式，
     *                                ePWM2 老二传话筒 SYNCIN 直通模式 把老大ePWM1的同步信号传给老三ePWM3
     *                                ePWM3 老三队尾闭嘴 禁用同步输出，不往下传任何同步脉冲  不影响后续的ePWM 4 5 6
     *                                保证三相的CTR是一样的
     *         PRDLD (Bit 3)       —— 周期寄存器重载控制 0：当计数器等于 0（CTR=0）时重载 ； 1：立即重载（不使用阴影寄存器 Shadow Register）
     *                                必须选 0；如果选 1（立即重载），你在中断里算完新的占空比或者周期并写入时，
     *                                恰好碰上硬件正在发波，会直接导致当前周期波形畸变，严重时导致逆变器上下桥臂直通爆管
     *         PHSEN (Bit 2)       —— 相位同步使能
     *         CTRMODE (Bits 1:0)  —— 计数模式选择 00向上 01向下 10up down 11冻结
     */

    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // 增减计数模式（对称波形）
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // 系统时钟不分频
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;          // 不分频
    // --- CC 子模块配置 ---
    /*  CMPA  Counter-Compare A Register    16位全有效寄存器（Bits 0-15） 用来存储 ePWMxA 通道的比较值
     *        它的值直接控制了 U、V、W 三相上桥臂的导通时间（占空比）
     *        TBCTR是运动员、TBPRD是终点、CMPA是关卡，往返跑中每遇到一次关卡 就触发一次动作
     *        通常会强行给 CMPA 赋一个 TBPRD/2 的值（也就是 50% 占空比）。这样可以确保逆变器三相中点电压全部为母线电压的一半，全桥处于对称平衡状态，电机死死稳住不乱抖
     *        开启死区模块并配置互补后 不需要手动设置CMPB
     *
     *  CMPCTL (Counter-Compare Control Register) —— 控制寄存器 TI默认开启
     *        LOADAMODE / LOADBMODE (Bits 1:0 和 Bits 3:2)：影子寄存器重载模式
     *                              00：当计数器归零时重载（CTR = 0）。
     *                              01：当计数器数到顶点时重载（CTR = TBPRD）。
     *                              10：在 CTR = 0 或 CTR = TBPRD 时均可重载。
     *                              11：立即加载（禁用影子寄存器）。
     *        SHDWAMODE / SHDWBMODE (Bit 4 和 Bit 6)：影子模式使能
     *                              0：开启影子寄存器模式（Shadow Mode）。
     *                              1：立即模式（Immediate Mode）。
     */
    EPwm1Regs.CMPA.half.CMPA = TBPRD_init;     // 初始占空比 50%
    // --- AQ 子模块配置 (定义 PWM1A 的动作) ---
    /*  AQ（Action-Qualifier，动作限定）子模块
     *  决定 TBCTR在遇到0、CMPA、CMPB、TBPRD时的动作 输出高电平、低电平，还是翻转
     *  AQCTLA (Action-Qualifier Control Register for Output A)：专门控制 EPWMxA 引脚
     *          11:10   CBD 当向下计数且 CTR = CMPB 时 极少使用
     *          9:8     CBU 当向上计数且 CTR = CMPB 时 极少使用
     *          7:6     CAD 当向下计数且 CTR = CMPA 时 常用
     *          5:4     CAU 当向上计数且 CTR = CMPA 时 常用
     *          3:2     PRD 当计数器达到顶点（CTR = TBPRD）时  较少使用
     *          1:0     ZRO 当计数器归零（CTR = 0）时         较少使用
     *   每组都有4种动作
     *          00 (AQ_NO_ACTION)：什么都不做（保持之前的电平）
     *          01 (AQ_CLEAR)：强制拉低（输出 0V）
     *          10 (AQ_SET)：强制拉高（输出 3.3V）
     *          11 (AQ_TOGGLE)：电平翻转
     */
    // 向上计数时比较匹配拉低，向下计数时比较匹配拉高 产生一个以 TBPRD 为中心、对称的 中间凹陷（低电平）、两边高电平 的 PWM 波形
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;
    // --- DB 死区模块配置 (实现互补+死区) ---
    /* DBCTL (Dead-Band Control Register - 控制寄存器)
     *      IN_MODE (源输入选择): 决定死区延时的参考源是谁。
     *              常规设定：DBA_ALL。意思是我们只看 EPWMxA 的波形，上升沿死区和下降沿死区都以 A 路为基准来算。
     *                       （此时 AQ 模块输出的 B 路波形被直接抛弃了）
     *      OUT_MODE (输出使能模式): 决定哪条路应用延时
     *              常规设定：DB_FULL_ENABLE。意思是 A 路应用上升沿延时（RED），B 路应用下降沿延时（FED），双管齐下
     *      POLSEL (极性选择): 这是最核心的一步，决定了 B 路怎么变成 A 路的反相
     * DBRED (Rising Edge Delay - 上升沿延时寄存器)
     *       设定 EPWMxA 变高电平之前，需要“等一等”的时间，单位是时钟周期 系统时钟是 150MHz 时，一个周期是 6.67ns
     * DBFED (Falling Edge Delay - 下降沿延时寄存器)
     *       设定 EPWMxA 变高电平之前，需要“等一等”的时间
     */
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // 开启双边死区
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       // 极性选择：A路不反向，B路反向（实现互补）
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;          // 源输入选择 EPWM1A
    EPwm1Regs.DBRED = 150;    // 上升沿死区时间 (RED) - 150个时钟周期 (约 1us)
    EPwm1Regs.DBFED = 150;    // 下降沿死区时间 (FED) - 150个时钟周期
    // 设置 SOCA (Start of Conversion A)
    /*  ETSEL (Event Trigger Selection) - 触发源选择寄存器
     *      在什么时刻（什么事件），发出触发信号
     *          SOCA / SOCB (Start of Conversion A/B)：专门用来唤醒 ADC 去采样的脉冲。
     *          INT (Interrupt)：用来直接打断 CPU 的中断信号。
     *      SOCASEL 1 (ET_CTR_ZERO)：当计数器 CTR = 0 时触发（FOC 最常用，测平均电流）
     *              2 (ET_CTR_PRD)：当计数器 CTR = PRD（顶点）时触发（FOC 最常用）
     *              4 或 5 (ET_CTRU_CMPA 等)：当计数器遇到 CMPA 时触发
     *  ETPS (Event Trigger Prescale) - 触发分频寄存器
     *      每次事件发生都要触发吗？能不能隔几次触发一次
     *      SOCAPRD (SOCA Period) 1 (ET_1ST)：每次满足条件都触发 ADC。
     *                            2 (ET_2ND)：满足 2 次条件，才触发 1 次 ADC（降频减负）
     *                            3 (ET_3RD)：满足 3 次条件，才触发 1 次 ADC
     *
     */
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;        // 使能 SOCA
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO; // 【关键】在计数器归零时触发 (PWM波形的波谷，通常噪音最小)
    EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;   // 每次事件都触发 (1st event)
    /* ... 触发CPU中断配置 ... ADC当总指挥后 也不需要了
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // 计数器减到0时产生中断
    EPwm1Regs.ETSEL.bit.INTEN = 1;                // 使能中断
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;           // 每次事件都产生中断
    */
    // 同步其他EPwm2和EPwm3
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;     // PHSEN (Bit 2) —— 相位同步使能  老大不需要同步别人
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // 计数到 0 时发信号给小弟们   SYNCOSEL (Bits 5:4) —— 同步输出选择
                                                // 00：SYNCIN 信号直接透传（作为外界的转接桥梁） 01：当 CTR = 0 时发送同步脉冲
                                                // 10：当 CTR = CMPB 时发送同步脉冲   11：禁用同步输出
}
void InitEPwm2()
{
    // --- TB 子模块配置 ---
    EPwm2Regs.TBPRD = TBPRD_ALL;             // 设置周期
    EPwm2Regs.TBPHS.half.TBPHS = 0x0000; // 相位为 0
    EPwm2Regs.TBCTR = 0x0000;            // 清零计数器
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // 增减计数模式（对称波形）
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // 系统时钟不分频
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    // --- CC 子模块配置 ---
    EPwm2Regs.CMPA.half.CMPA = TBPRD_init;     // 初始占空比 50%
    // --- AQ 子模块配置 (定义 PWM1A 的动作) ---
    // 向上计数时比较匹配拉低，向下计数时比较匹配拉高
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;
    // --- DB 死区模块配置 (实现互补+死区) ---
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // 开启双边死区
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       // 极性选择：A路不反向，B路反向（实现互补）
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;          // 源输入选择 EPWM1A
    EPwm2Regs.DBRED = 150;    // 上升沿死区时间 (RED) - 4500个时钟周期 (约 3us)
    EPwm2Regs.DBFED = 150;    // 下降沿死区时间 (FED) - 450个时钟周期
    // --- 【关键：同步设置】 ---
    EPwm2Regs.TBPHS.half.TBPHS = 0;           // 相位偏移设为 0 (绝对对齐)
    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;    // 使能相位加载 (听老大的同步信号)
    EPwm2Regs.TBCTL.bit.PHSDIR = TB_UP;       // 同步后向上计数
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // 将同步信号继续传给 ePWM3 (备用)
}
void InitEPwm3()
{
    // --- TB 子模块配置 ---
    EPwm3Regs.TBPRD = TBPRD_ALL;             // 设置周期
    EPwm3Regs.TBPHS.half.TBPHS = 0x0000; // 相位为 0
    EPwm3Regs.TBCTR = 0x0000;            // 清零计数器
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // 增减计数模式（对称波形）
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // 系统时钟不分频
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    // --- CC 子模块配置 ---
    EPwm3Regs.CMPA.half.CMPA = TBPRD_init;     // 初始占空比 50%
    // --- AQ 子模块配置 (定义 PWM3A 的动作) ---
    // 向上计数时比较匹配拉低，向下计数时比较匹配拉高
    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;
    // --- DB 死区模块配置 (实现互补+死区) ---
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // 开启双边死区
    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       // 极性选择：A路不反向，B路反向（实现互补）
    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;          // 源输入选择 EPWM3A
    EPwm3Regs.DBRED = 150;    // 上升沿死区时间 (RED) - 450个时钟周期 (约 3us)
    EPwm3Regs.DBFED = 150;    // 下降沿死区时间 (FED) - 450个时钟周期
    // --- 【关键：同步设置】 ---
    EPwm3Regs.TBPHS.half.TBPHS = 0;           // 相位偏移设为 0 (绝对对齐)
    EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;    // 使能相位加载 (听老大的同步信号)
    EPwm3Regs.TBCTL.bit.PHSDIR = TB_UP;       // 同步后向上计数
}

int fault_flag;
__interrupt void epwm1_tz_int_isr(void)
{
    fault_flag = 1;
    // 1. 软件紧急停机逻辑
    run_flag = 0; // 关闭你的主算法开关
    GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1; // 拉低 IR2136 EN 脚，双重保险关闭驱动器     使用的是clear 写1时置零
    GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;   // 上拉控制
    // 2. 可以在这里做一些报警记录
    // 3. 清除 TZ 中断标志
    EALLOW;
    EPwm1Regs.TZCLR.bit.OST = 1;
    EPwm1Regs.TZCLR.bit.INT = 1;
    EDIS;
    // 4. 应答 PIE 中断
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;     //手动清除ACK 准备下一次响应
}
