#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"

/*  QEPCTL (QEP 控制寄存器) —— “总司令与发令枪”
 *  这个寄存器决定了 eQEP 模块什么时候开始工作，以及它遇到特殊事件（比如 Z 脉冲、定时器溢出）时该做出什么动作。
 *  15:14   FREE_SOFT       仿真控制模式。
 *  13:12   PCRM            位置计数器复位模式 (Position Counter Reset Mode)。极其关键！
 *                          00: 发生 Index(Z) 脉冲时复位。 (每次过零点就清零，适合绝对定位)
 *                          01: 计数到最大值 (QPOSMAX) 时复位
 *                          10: 发生第一次 Index 脉冲时复位
 *                          11: 发生单位时间事件 (UPEVN) 时复位
 *  11:10   IEI             Index 事件初始化位置计数器使能
 *                          10: 在 Index 脉冲的上升沿，把 QPOSINIT 的值加载到 QPOSCNT 里。（用于零点校准）
 *  9:8     IEL             Index 事件锁存位置计数器使能
 *                          01 或 10: 遇到 Index 脉冲时，把当前的计数值锁存到 QPOSILAT 里
 *  7:6     QPEN            软件初始化。写 1 强行初始化计数器
 *  3       QEPEN           eQEP 模块使能。 终极开关！ 所有的配置写完后，最后一行代码必须是把这位写 1，计步器才开始工作
 */
/*  QEINT (中断使能寄存器): 这是一个开关矩阵
 *      UTO: 单位定时器溢出中断（定时测速用）
 *      IEL: 发生 Index 脉冲中断（电机过零点时，进入中断去清零累积误差）
 *      PCO: 位置计数器溢出/下溢中断（转完一圈了）
 */

/*  QFLG (中断标志寄存器): 只读的监控面板。哪个事件发生了，对应位就变 1
 *
 *
 */
/*  QPOSILAT (Index 位置锁存寄存器) —— “找 d 轴的秘密武器”
 *  大小： 32位
 *  当发生 Index (Z) 脉冲时，硬件会瞬间把当前的 QPOSCNT 拍照存进这里
 *  寻找零位过程： 强行给 Vd 电压，把电机锁死
 *              读当前的 QPOSCNT，这就是绝对的电气零点
 *              但在电机高速旋转时，如果直接用 QPOSCNT 算角度，可能会有误差。所以，我们常常配置在遇到 Z 脉冲时产生中断。
 *              在中断里，我们去读 QPOSILAT。这个值告诉你：“本次 Z 脉冲发生在计步器的哪个位置”。
 *              利用这个值，你可以随时校正你的电气角度补偿量 theta_offset
 */
/*  QDECCTL (Quadrature Decoder Control Register)
 *  这是所有外部编码器信号进入 DSP 的第一道关卡。它的作用是：决定怎么看 A、B、Z 脉冲，要不要反相，要不要交换
 *  15:14   QSRC    位置计数器源选择 Position Counter Source     00: 正交计数模式 (A/B相正交) 【FOC 必选】
 *                                                           01: 方向计数模式 (A输入脉冲，B输入高低电平代表方向)
 *                                                           10: UP 计数模式 (单相测速)
 *                                                           11: DOWN 计数模式
 *  13:12   SOEN/SPSEL  同步输出控制                            将编码器脉冲同步输出给其他模块。普通 FOC 用不到，填 0
 *  11      XCR         外部时钟分辨率  External Clock Rate      0: 2倍/4倍频模式 【FOC 必选】
 *                                                              【FOC 必选】 (A和B的上升/下降沿全都计数，1000线变4000计数值)。
 *                                                           1: 1倍/2倍频模式
 *  10      SWAP        A/B 通道交换  Swap QEPA/QEPB           0: QEPA接A，QEPB接B。   1: 硬件自动把 A 和 B 调换！
 *  9       IGATE       Z信号门控使能                           0: Z信号不受限制
 *                                                           1: 只有在A和B都为特定状态时，Z信号才有效（抗干扰防抖）
 *  8       QAP         QEPA 极性反转                          0: 不反转；1: 把 A 相波形上下颠倒
 *  7       QBP         QEPB 极性反转                          0: 不反转；1: 把 B 相波形上下颠倒
 *  6       QIP         QEPI 极性反转                          0: Z 相不反转；1: Z 相极性反转（取决于你的编码器Z脉冲是高有效还是低有效）
 *  5       QSP         选通引脚极性                            针对 QEPS 引脚的极性反转，通常写 0
 *  4:0     Reserved    保留
 */
/*  QPOSMAX (Maximum Position Count Register) —— “计步器的天花板”
 *  大小： 32位 (全有效)
 *  告诉硬件：电机转完一整圈，总共有多少个步子
 *  设定位置计数器的最大值。当 QPOSCNT 一路往上加，加到和 QPOSMAX 一样大时，再来一个脉冲，会瞬间回卷到 0
 *                    反之，如果从 0 往下减，会瞬间跳回 QPOSMAX
 *  致命防坑公式：如果你买了一个标称 N 线 (PPR) 的增量式编码器，并且你在 QDECCTL 里开启了 4 倍频，
 *  那么：QPOSMAX = N * 4 - 1 举例： 1000 线的编码器，QPOSMAX 必须填 3999。
 *  如果填成 4000，你的电机每转一圈就会多出 1 个计数的误差，转 100 圈你的电角度就彻底错位炸管了！
 */
/*  QPOSINIT (Position Counter Init Register) —— “绝对零点校准器”
 *  大小： 32位 (全有效)
 *  核心功能： 里面存着一个备用数字（通常我们填 0，或者填一个固定的偏移量）
 *  怎么用： 它不是用来读的，它是用来被硬件“砸”进计数器里的。
 *  在 QEPCTL (上次提到的寄存器) 中，我们可以配置：“每次遇到 Z (Index) 脉冲时，把 QPOSINIT 的值覆盖掉 QPOSCNT”
 *  意义： 这样可以完全消除由于电磁干扰导致的“丢步”累积误差。每次转到原点，计数器就被强行“拨乱反正”
 */
/*  QPOSCNT (Position Counter Register) —— “核心实时里程表”
 *  大小： 32位 (全有效)核心功能： 这是 eQEP 模块的心脏。它根据 A/B 相的脉冲，实时在 $0$ 到 QPOSMAX 之间往复跳动。
 *  防坑指南： 理论上，你可以直接在代码里读它 (angle = EQep1Regs.QPOSCNT;) 来算电角度。
 *           但如果你要用它来算速度，绝对不要直接读它！应该去读它的锁存影子寄存器 QPOSLAT（我们在上一节讲过），
 *           否则你会遇到读取时刻不同步带来的严重速度抖动。
 */
/*  QUPRD (Unit Timer Period Register) —— “高速测速的节拍器”
 *  测“电机在固定时间里能走多少个脉冲”(中高速 M 法测速)
 *  大小： 32位 (全有效)
 *  核心功能： 它是单位定时器 (Unit Timer) 的周期设定值。
 *  硬件里有一个定时器以系统时钟(SYSCLKOUT)的频率在跑，当它数到 QUPRD 的值时，就会触发一个 UPEVN (单位时间事件)
 *  硬件联动魔法： 当 UPEVN 触发的这 1 皮秒内，硬件会做两件事：把当前的 QPOSCNT 强行复制一份存进 QPOSLAT
 *                                                 把 QEPSTS.bit.UPEVNT 标志位置 1，并产生中断
 *  实战怎么配：假设你想每 1 毫秒 (1ms) 算一次转速，系统时钟是 150MHz：
 *   QUPRD = 150MHz * 1ms = 150,000,000 * 0.001 = 150,000
 *   你在代码里写 EQep1Regs.QUPRD = 150000;
 *   然后在你的速度环代码里，直接用 (本次 QPOSLAT - 上次 QPOSLAT) / 1ms，这就是极其平滑的高速转速反馈！
 */

void InitEQep1_AS5047P(void)
{
    // =========================================================
    // 1. 使能 eQEP1 外设时钟 (极其重要，否则后续寄存器无法写入)
    // =========================================================
    EALLOW;
    SysCtrlRegs.PCLKCR1.bit.EQEP1ENCLK = 1;  // 开启 eQEP1 模块时钟
    EDIS;

    // =========================================================
    // 2. GPIO 复用与方向配置 (50,51,53 给解码器，52 给 IR2136)
    // =========================================================
    EALLOW;
    // 启用内部上拉电阻
    GpioCtrlRegs.GPBPUD.bit.GPIO50 = 0;   // EQEP1A
    GpioCtrlRegs.GPBPUD.bit.GPIO51 = 0;   // EQEP1B
    GpioCtrlRegs.GPBPUD.bit.GPIO53 = 0;   // EQEP1I (Z脉冲)

    // 信号同步设置 (与系统时钟 SYSCLKOUT 同步)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO50 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO51 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO53 = 0;

    // GPIO 功能复用 (GPBMUX2 控制 GPIO48-63)
    // 1 = eQEP 功能, 0 = 通用 GPIO 功能
    GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 1;  // 配置为 EQEP1A 输入
    GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 1;  // 配置为 EQEP1B 输入
    GpioCtrlRegs.GPBMUX2.bit.GPIO53 = 1;  // 配置为 EQEP1I 输入

    EDIS;

    // 初始化 IR2136 使能引脚电平
    // 注意：请根据你的驱动板逻辑修改。如果是高电平使能，上电先置低电平关闭输出以保护电路
    GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1; // 输出低电平
    // GpioDataRegs.GPBSET.bit.GPIO52 = 1; // 如果需要输出高电平用这句

    // =========================================================
    // 3. eQEP 核心寄存器配置 (解码与找零位)
    // =========================================================
    // 最大计数值：假设测试出来是 1000线，则 1000 * 4 - 1 = 3999
    // 如果今天测试确认是 1024线，请将其改为 4096 - 1 = 4095
    EQep1Regs.QPOSMAX = 4000 - 1;
    EQep1Regs.QPOSINIT = 0;               // 初始位置设定为 0

    // QDECCTL: 解码控制
    EQep1Regs.QDECCTL.bit.QSRC = 00;      // 正交计数模式
    EQep1Regs.QDECCTL.bit.IGATE = 0;      // 禁用 Index 门控 (AS5047P的Z信号通常较宽)
    EQep1Regs.QDECCTL.bit.QAP = 0;        // A 相极性不反转
    EQep1Regs.QDECCTL.bit.QBP = 0;        // B 相极性不反转
    EQep1Regs.QDECCTL.bit.QIP = 0;        // Index 极性不反转

    // QEPCTL: 运行控制与复位逻辑
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2;   // 仿真器挂起（打断点）时，位置计数器不受影响，继续运行

    // 【寻零核心配置】
    EQep1Regs.QEPCTL.bit.PCRM = 00;       // 在检测到 Index (Z脉冲) 事件时，自动复位 QPOSCNT
    EQep1Regs.QEPCTL.bit.IEI = 10;        // 指定 Index 事件由上升沿触发

    // =========================================================
    // 4. 单位定时器配置 (Unit Timer) - 用于后续计算转速
    // =========================================================
    // 假设 F28335 运行在 150MHz 系统时钟下，我们设定 1ms 锁存一次位置
    EQep1Regs.QUPRD = 150000000 / 1000;   // 1ms 对应的时钟周期数

    EQep1Regs.QEPCTL.bit.UTE = 1;         // 使能单位定时器
    EQep1Regs.QEPCTL.bit.QCLM = 1;        // 当 1ms 时间到时，硬件自动将 QPOSCNT 的值锁存到 QPOSLAT 寄存器中

    // =========================================================
    // 5. 启动模块
    // =========================================================
    EQep1Regs.QPOSCNT = 0;                // 计数器清零
    EQep1Regs.QEPCTL.bit.QPEN = 1;        // 全面使能 eQEP1 模块开始工作
}
