#include "DSP2833x_Device.h"
#include "DSP2833x_Examples.h"
#include "my_foc.h"
#include "my_qep.h"
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
/*  QPOSCNT (Position Counter Register) —— “实时位置计数器”
 *
 *  作用：eQEP 根据 A/B 两相信号实时加/减计数，QPOSCNT 在 0 ~ QPOSMAX 之间变化。
 *
 *  本工程中的用途：
 *      1) FOC 实时电角度：直接读取 QPOSCNT。电流环 10 kHz，需要尽可能新的转子位置。
 *      2) M 法测速：不在任意软件时刻读 QPOSCNT，而是让 Unit Timer 在固定时刻
 *         把 QPOSCNT 锁存到 QPOSLAT，再对相邻两次 QPOSLAT 做差。
 *
 *  注意：QPOSCNT 并不是“不能用于测速”。如果软件采样周期足够严格，也可以直接做差测速。
 *        本工程选择 QPOSLAT，是为了让采样时刻由 eQEP 硬件保证，减少 CPU 执行抖动影响。
 */

/*  QUTMR / QUPRD / UTO —— M 法测速的固定时间基准
 *
 *  QUTMR：Unit Timer 当前计数值。
 *  QUPRD：Unit Timer 周期设定值。QEPCTL.UTE=1 后，QUTMR 以 SYSCLKOUT 计数。
 *
 *  UTO = Unit Time-Out，即“单位定时器到期事件/标志”。
 *
 *  本工程：SYSCLKOUT=150MHz，M 法采样频率=250Hz，所以 QUPRD=600000，约每 4ms 一次 UTO。
 *
 *  当 QEPCTL.QCLM=1 时，每次 UTO 到来会把：
 *      QPOSCNT -> QPOSLAT
 *  同时 QFLG.UTO 置 1。软件读取 QPOSLAT 完成一次 M 法计算后，用 QCLR.UTO 清标志。
 *
 *  UTO 和 UPEVNT 不同：
 *      UTO    = 固定“时间”到了，服务于 M 法。
 *      UPEVNT = 固定“位置增量”到了，服务于 T 法。
 */

/*  QCAPCTL (eQEP Capture Control Register) —— T 法测速核心寄存器
 *
 *  T 法问的是：“走固定数量的位置 count，用了多少时间？”
 *
 *  CEN —— Capture Enable
 *      0：关闭 Capture 单元；1：开启 Capture 单元。
 *      修改 UPPS/CCPS 前先 CEN=0，配置后再 CEN=1。
 *
 *  UPPS —— Unit Position Event Prescaler
 *      决定经过多少个 QCLK count 产生一次 UPEVNT。
 *      UPPS=0 -> QCLK/1；UPPS=1 -> QCLK/2；UPPS=2 -> QCLK/4；UPPS=3 -> QCLK/8 ……
 *      本工程 UPPS=0，所以每 1 个 eQEP 位置 count 产生一次 UPEVNT。
 *
 *  CCPS —— Capture Clock Prescaler
 *      决定 Capture Timer(QCTMR) 的计数时钟。
 *      CCPS=0 -> SYSCLK/1；1 -> /2；2 -> /4 …… 7 -> /128。
 *      本工程 CCPS=7，所以 CAPCLK=150MHz/128=1.171875MHz，一个 tick 约 0.853us。
 *
 *  Capture 相关寄存器/标志：
 *      QCTMR：Capture Timer 当前计数值。
 *      QCPRD：每次 UPEVNT 到来时保存上一个位置事件到本次事件之间的 Capture tick 数。
 *      UPEVNT：有新的 Unit Position Event。
 *      COEF：Capture Timer 溢出，本次周期无效。
 *      CDEF：两个位置事件之间发生方向变化，本次周期不适合直接算单一方向速度。
 *
 *  本工程 T 法公式（UPPS=0，即每次事件相隔 1 count）：
 *
 *                   CAPCLK × 1 × 60
 *      rpm = 方向 × ----------------------
 *                  QCPRD × 每圈总 count
 *
 *  QCPRD 与 QCPRDLAT：
 *      当前 QCLM=1，QCPRDLAT 是 UTO 时对 QCPRD 的锁存快照，并非每个 UPEVNT 都更新。
 *      因此 T 法读取最新周期时使用 QCPRD，而不是 QCPRDLAT。
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
    EQep1Regs.QDECCTL.bit.QSRC = 0;      // 正交计数模式
    EQep1Regs.QDECCTL.bit.IGATE = 0;      // 禁用 Index 门控 (AS5047P的Z信号通常较宽)
    EQep1Regs.QDECCTL.bit.QAP = 0;        // A 相极性不反转
    EQep1Regs.QDECCTL.bit.QBP = 0;        // B 相极性不反转
    EQep1Regs.QDECCTL.bit.QIP = 0;        // Index 极性不反转

    // QEPCTL: 运行控制与复位逻辑
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2;   // 仿真器挂起（打断点）时，位置计数器不受影响，继续运行

    // 【寻零核心配置】
    EQep1Regs.QEPCTL.bit.PCRM = 0;       // 在检测到 Index (Z脉冲) 事件时，自动复位 QPOSCNT
    EQep1Regs.QEPCTL.bit.IEI = 2;        // 指定 Index 事件由上升沿触发

    // =========================================================
    // 4. Unit Timer：给 M 法制造严格固定的 4 ms 测量窗口
    // =========================================================
    // M 法：固定时间 Δt 内统计位置变化量 ΔM。
    // SYSCLKOUT=150MHz，QEP_M_SAMPLE_HZ=250Hz -> QUPRD=600000 -> 每约4ms一次UTO。
    // QCLM=1 时，UTO 到来会把 QPOSCNT 锁存到 QPOSLAT。
    EQep1Regs.QUPRD = (Uint32)(DSP_Sys_Clock_Freq / QEP_M_SAMPLE_HZ);
    EQep1Regs.QEPCTL.bit.UTE = 1;         // UTE：Unit Timer Enable，1=启动 QUTMR
    EQep1Regs.QEPCTL.bit.QCLM = 1;        // UTO 时锁存 QPOSCNT -> QPOSLAT

    // =========================================================
    // 5. QCAPCTL：给 T 法配置 eQEP Capture 单元
    // =========================================================
    // UPPS=0 -> 每1个QCLK count产生一次UPEVNT。
    // CCPS=7 -> Capture时钟=SYSCLKOUT/128=1.171875MHz。
    EQep1Regs.QCAPCTL.bit.CEN = 0;        // CEN：0=关闭 Capture，先关再改分频
    EQep1Regs.QCAPCTL.bit.UPPS = QEP_T_UPPS_BITS; // 当前0：QCLK/1，每1个位置count一次事件
    EQep1Regs.QCAPCTL.bit.CCPS = QEP_T_CCPS_BITS; // 当前7：SYSCLK/128
    EQep1Regs.QCAPCTL.bit.CEN = 1;        // 1=开启 Capture，T 法开始工作

    // =========================================================
    // 6. 启动模块
    // =========================================================
    EQep1Regs.QPOSCNT = 0;                // 计数器清零
    EQep1Regs.QEPCTL.bit.QPEN = 1;        // 全面使能 eQEP1 模块开始工作

    QEP_Speed_Reset();
}

    int32 QEP_Offset = 0;       // 使用有符号 32 位整型
    float QEP_Mech_Theta = 0.0f;  // QEP 机械角度
    float QEP_Elc_Theta = 0.0f;   // QEP 电角度
    // 定义你的物理磁极安装夹角 (QEP 比 SPI 大 3.46)
    const float CONST_MAGNETIC_OFFSET = 3.46f;

// ============ 闭环运行时的计算 (放在中断里) ============
    float qep_spi;
void Calculate_QEP_Angle(void)
{
    // 1. 读取当前计数值，并立刻转为有符号整数
    int32 current_qpos = (int32)EQep1Regs.QPOSCNT;

    // 2. 计算相对 Z 脉冲的脉冲差值
    int32 delta_pos =  current_qpos - QEP_Offset ;

    // 3. 处理跨越 0 或 3999 时的物理绕回
    // （假设顺时针转一圈后，delta_pos 会超过 4000，逆时针会小于 0）
    /*if(delta_pos < 0)
    {
        delta_pos += 4000;
    }
    else if(delta_pos >= 4000)
    {
        delta_pos -= 4000;
    }*/

    // 4. 计算机械角度 (弧度)
    QEP_Mech_Theta = (float)delta_pos * (-6.2831853f / QEP_COUNTS_PER_REV_F);

    // 5. 计算电角度 = 机械角度 * 极对数
    QEP_Elc_Theta = ( QEP_Mech_Theta * MOTOR_Pole_pairs ) - CONST_MAGNETIC_OFFSET ;

    // 6. 将电角度归一化到 0 ~ 2*PI 之间
    // 乘以极对数后会跨越多个 2*PI 周期，因此需要归一化。
    while(QEP_Elc_Theta >= 6.2831853f)
    {
        QEP_Elc_Theta -= 6.2831853f;
    }
    while(QEP_Elc_Theta < 0.0f) // 虽然上面的逻辑保证了不会小于0，但加上更保险
    {
        QEP_Elc_Theta += 6.2831853f;
    }
    // 7. 输出给 FOC 算法
    foc.Theta = QEP_Elc_Theta;
    //qep_spi = QEP_Elc_Theta - foc.Theta;
    // 现在，你可以在 Expressions 里安全地把 QEP_Elc_Theta 和你的 SPI 角度进行对比了！
}

/* ============================================================
 * eQEP M/T 测速模块
 *
 * QPOSLAT -> M 法 --\
 *                    -> M/T 选择 -> mech_rpm -> elec_rad_s
 * QCPRD   -> T 法 --/
 * ============================================================ */

/*
 * g_qep_speed：测速模块运行状态与调试变量。
 *
 * mech_rpm：M/T选择后的最终机械转速，单位rpm。
 * elec_rad_s：最终电角速度，单位rad/s；=mech_rpm*2π/60*极对数。
 *
 * m_rpm_raw：M法原始机械rpm，未滤波。
 * m_rpm_filt：M法一阶低通后的机械rpm。
 * m_delta_count：本次4ms窗口内的位置count变化量，已处理跨圈。
 * m_valid：M法是否已有有效样本，0=没有，1=有。
 *
 * t_rpm_raw：T法根据最新QCPRD得到的原始机械rpm。
 * t_rpm_filt：T法一阶低通后的机械rpm。
 * t_capture_period：最近一次QCPRD原始值，即相邻位置事件之间的CAPCLK tick数。
 *                   低速时大，高速时小；约100rpm->176，200rpm->88，450rpm->39。
 * t_valid：T法当前结果是否有效，0=无效，1=有效。
 *
 * method：当前最终速度采用的方法：0=T法，1=M法。
 *         T->M阈值约300rpm，M->T阈值约200rpm，中间100rpm作为滞环。
 *
 * t_overflow_count：QEPSTS.COEF出现次数累计；COEF表示Capture Timer溢出。
 * t_dir_error_count：QEPSTS.CDEF出现次数累计；CDEF表示捕获周期内发生方向变化。
 */
QEP_SPEED_ESTIMATOR_t g_qep_speed = {0};

static int32 s_m_last_pos = 0;      // 上一次UTO时的QPOSLAT，M法本次用当前值减它
static Uint16 s_m_filter_init = 0U; // M法低通滤波器是否已用首个有效样本初始化
static Uint16 s_t_filter_init = 0U; // T法低通滤波器是否已用首个有效样本初始化

/*
 * 连续多少次 QEP_Speed_Update() 没看到新的 UPEVNT。
 * 当前函数在10kHz ISR中调用，1次约0.1ms；800次约80ms。
 * 真正静止时不会再有“下一个边沿”，所以T法必须靠这个超时主动判0rpm。
 */
static Uint16 s_t_no_event_cnt = 0U;

#define QEP_T_ZERO_TIMEOUT_COUNT    800U   // 约80ms无UPEVNT -> 判0rpm
#define QEP_M_LPF_ALPHA             0.25f  // M法一阶低通系数：越大响应越快、滤波越弱
#define QEP_T_LPF_ALPHA             0.20f  // T法一阶低通系数：越大响应越快、滤波越弱
#define QEP_TWO_PI_OVER_60          0.1047197551f // 2π/60，用于rpm -> rad/s

static float QEP_AbsF(float x)
{
    if (x < 0.0f) return -x;
    return x;
}

static int32 QEP_WrapDelta(int32 delta)
{
    const int32 half_count = (int32)(QEP_COUNTS_PER_REV / 2L);

    if (delta > half_count)
    {
        delta -= (int32)QEP_COUNTS_PER_REV;
    }
    else if (delta < -half_count)
    {
        delta += (int32)QEP_COUNTS_PER_REV;
    }
    return delta;
}

void QEP_Speed_Reset(void)
{
    g_qep_speed.mech_rpm = 0.0f;
    g_qep_speed.elec_rad_s = 0.0f;

    g_qep_speed.m_rpm_raw = 0.0f;
    g_qep_speed.m_rpm_filt = 0.0f;
    g_qep_speed.m_delta_count = 0;
    g_qep_speed.m_valid = 0U;

    g_qep_speed.t_rpm_raw = 0.0f;
    g_qep_speed.t_rpm_filt = 0.0f;
    g_qep_speed.t_capture_period = 0U;
    g_qep_speed.t_valid = 0U;

    g_qep_speed.method = (Uint16)QEP_SPEED_METHOD_T;
    g_qep_speed.t_overflow_count = 0U;
    g_qep_speed.t_dir_error_count = 0U;

    s_m_last_pos = (int32)EQep1Regs.QPOSCNT;
    s_m_filter_init = 0U;
    s_t_filter_init = 0U;
    s_t_no_event_cnt = 0U;

    /* 从“现在”开始重新计 4 ms，避免第一次 M 法窗口长度不确定。 */
    EQep1Regs.QUTMR = 0U;

    /* 清除旧的 Unit Timeout 和 Capture 状态。 */
    EQep1Regs.QCLR.bit.UTO = 1;
    EQep1Regs.QEPSTS.all = 0x008CU;   // clear UPEVNT(bit7), COEF(bit3), CDEF(bit2)
}

static void QEP_Update_M_Method(void)
{
    int32 pos_now;     // 本次UTO时刻锁存的QPOSLAT
    int32 delta;       // 本次4ms内的位置增量，已处理跨圈
    float rpm_raw;     // M法原始机械转速，单位rpm

    if (EQep1Regs.QFLG.bit.UTO == 0U)
    {
        return;
    }

    /* QPOSLAT 是硬件在 4 ms Unit Timeout 那一刻锁存的位置。 */
    pos_now = (int32)EQep1Regs.QPOSLAT;
    delta = QEP_WrapDelta(pos_now - s_m_last_pos);
    s_m_last_pos = pos_now;

    g_qep_speed.m_delta_count = delta;

    rpm_raw = (float)delta
            * (60.0f * (float)QEP_M_SAMPLE_HZ / QEP_COUNTS_PER_REV_F)
            * QEP_SPEED_DIRECTION_SIGN;

    g_qep_speed.m_rpm_raw = rpm_raw;

    if (s_m_filter_init == 0U)
    {
        g_qep_speed.m_rpm_filt = rpm_raw;
        s_m_filter_init = 1U;
    }
    else
    {
        g_qep_speed.m_rpm_filt += QEP_M_LPF_ALPHA
                               * (rpm_raw - g_qep_speed.m_rpm_filt);
    }

    g_qep_speed.m_valid = 1U;

    /* UTO 是 QFLG 中的锁存标志，必须通过 QCLR 清除。 */
    EQep1Regs.QCLR.bit.UTO = 1;
}

static void QEP_Update_T_Method(void)
{
    Uint16 status;          // QEPSTS快照：UPEVNT/COEF/CDEF/QDF等
    Uint16 period;          // 最新QCPRD：相邻位置事件之间的Capture tick数
    float direction;        // 当前方向系数：+1.0f或-1.0f
    float rpm_raw;          // T法原始机械转速，单位rpm
    Uint16 clear_mask = 0U; // 本次需要写1清除的QEPSTS状态位集合

    if (s_t_no_event_cnt < 0xFFFFU)
    {
        s_t_no_event_cnt++;
    }

    status = EQep1Regs.QEPSTS.all;

    /* Capture Timer 溢出：当前周期无效。 */
    if ((status & 0x0008U) != 0U)      // COEF bit3
    {
        if (g_qep_speed.t_overflow_count < 0xFFFFU)
        {
            g_qep_speed.t_overflow_count++;
        }
        g_qep_speed.t_valid = 0U;
        clear_mask |= 0x0008U;
    }

    /* 两个 Unit Position Event 之间发生方向翻转：丢弃本次 T 法数据。 */
    if ((status & 0x0004U) != 0U)      // CDEF bit2
    {
        if (g_qep_speed.t_dir_error_count < 0xFFFFU)
        {
            g_qep_speed.t_dir_error_count++;
        }
        g_qep_speed.t_valid = 0U;
        clear_mask |= 0x0004U;
    }

    if ((status & 0x0080U) != 0U)      // UPEVNT bit7
    {
        /*
         * QCPRD 在每个 Unit Position Event 更新；
         * 在 QCLM=1 时不要把 QCPRDLAT 当成“每边沿周期”。
         */
        period = EQep1Regs.QCPRD;
        g_qep_speed.t_capture_period = period;
        s_t_no_event_cnt = 0U;
        clear_mask |= 0x0080U;

        if (((status & 0x000CU) == 0U) && (period != 0U))
        {
            /* QDF=1：位置计数器正向增加；QDF=0：反向减少。 */
            if ((status & 0x0020U) != 0U)
            {
                direction = 1.0f;
            }
            else
            {
                direction = -1.0f;
            }

            rpm_raw = direction
                    * QEP_SPEED_DIRECTION_SIGN
                    * (QEP_T_CAPCLK_HZ * QEP_T_UNIT_COUNTS * 60.0f)
                    / ((float)period * QEP_COUNTS_PER_REV_F);

            g_qep_speed.t_rpm_raw = rpm_raw;

            if (s_t_filter_init == 0U)
            {
                g_qep_speed.t_rpm_filt = rpm_raw;
                s_t_filter_init = 1U;
            }
            else
            {
                g_qep_speed.t_rpm_filt += QEP_T_LPF_ALPHA
                                       * (rpm_raw - g_qep_speed.t_rpm_filt);
            }
            g_qep_speed.t_valid = 1U;
        }
    }

    if (clear_mask != 0U)
    {
        /* QEPSTS 中 UPEVNT/COEF/CDEF 都是写 1 清零。 */
        EQep1Regs.QEPSTS.all = clear_mask;
    }

    /*
     * 真正零速时不会再有边沿，所以 T 法本身“永远等不到下一次周期”。
     * 超过约 80 ms 没有新边沿，直接判为 0 rpm。
     * 以当前配置，Capture 16 位计数器本身的极低速下限约为 0.27 rpm。
     */
    if (s_t_no_event_cnt >= QEP_T_ZERO_TIMEOUT_COUNT)
    {
        g_qep_speed.t_rpm_raw = 0.0f;
        g_qep_speed.t_rpm_filt = 0.0f;
        g_qep_speed.t_valid = 1U;
        s_t_filter_init = 1U;
        s_t_no_event_cnt = QEP_T_ZERO_TIMEOUT_COUNT;
    }
}

static void QEP_Select_MT_Method(void)
{
    float abs_speed;   // 切换只看速度绝对值，不看正反方向

    if (g_qep_speed.method == (Uint16)QEP_SPEED_METHOD_T)
    {
        abs_speed = QEP_AbsF(g_qep_speed.t_rpm_filt);

        /* T -> M：超过 300 rpm，且 M 法已有有效样本。 */
        if ((g_qep_speed.m_valid != 0U) &&
            (((g_qep_speed.t_valid != 0U) && (abs_speed >= QEP_SWITCH_TO_M_RPM)) ||
             (g_qep_speed.t_valid == 0U)))
        {
            g_qep_speed.method = (Uint16)QEP_SPEED_METHOD_M;
        }
    }
    else
    {
        abs_speed = QEP_AbsF(g_qep_speed.m_rpm_filt);

        /* M -> T：降到 200 rpm 以下，并且 T 法已有有效样本。 */
        if ((g_qep_speed.t_valid != 0U) &&
            (abs_speed <= QEP_SWITCH_TO_T_RPM))
        {
            g_qep_speed.method = (Uint16)QEP_SPEED_METHOD_T;
        }
    }

    if ((g_qep_speed.method == (Uint16)QEP_SPEED_METHOD_T) &&
        (g_qep_speed.t_valid != 0U))
    {
        g_qep_speed.mech_rpm = g_qep_speed.t_rpm_filt;
    }
    else if (g_qep_speed.m_valid != 0U)
    {
        g_qep_speed.mech_rpm = g_qep_speed.m_rpm_filt;
    }
    else
    {
        g_qep_speed.mech_rpm = 0.0f;
    }

    /* 机械 rpm -> 电角速度 rad/s */
    g_qep_speed.elec_rad_s = g_qep_speed.mech_rpm
                           * QEP_TWO_PI_OVER_60
                           * MOTOR_Pole_pairs;
}

void QEP_Speed_Update(void)
{
    /* 本函数虽在10kHz ISR每次调用，但M法仅在新UTO时更新，T法仅在新UPEVNT时更新。 */
    QEP_Update_M_Method();   // 有新UTO才更新M法
    QEP_Update_T_Method();   // 有新UPEVNT才更新T法
    QEP_Select_MT_Method();  // 带滞环选择最终速度反馈
}

void InitHallGpio(void)
{
    EALLOW; // 解锁受保护的配置寄存器

    // ==========================================
    // 步骤 1：使能内部上拉电阻 (给信号线绑上3.3V的吊绳)
    // ==========================================
    // GPAPUD 控制 GPIO0 到 GPIO31
    // 0 = 开启内部上拉 (Pull-up Enable)
    // 1 = 禁用内部上拉
    GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0; // 开启 GPIO24 (HALL_U) 上拉
    GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0; // 开启 GPIO25 (HALL_V) 上拉
    GpioCtrlRegs.GPAPUD.bit.GPIO26 = 0; // 开启 GPIO26 (HALL_W) 上拉


    // ==========================================
    // 步骤 2：配置为普通功能 (不复用为 eQEP 或其他外设)
    // ==========================================
    // GPAMUX2 控制 GPIO16 到 GPIO31
    // 00 = 普通数字 I/O 功能
    // 01, 10, 11 = 复用为其他外设功能（例如 01 是复用为 eQEP1）
    GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0; // 配置为普通 GPIO
    GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0; // 配置为普通 GPIO
    GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0; // 配置为普通 GPIO


    // ==========================================
    // 步骤 3：配置引脚方向为“输入”
    // ==========================================
    // GPADIR 控制方向：0 = 输入 (Input)，1 = 输出 (Output)
    GpioCtrlRegs.GPADIR.bit.GPIO24 = 0; // GPIO24 作为输入
    GpioCtrlRegs.GPADIR.bit.GPIO25 = 0; // GPIO25 作为输入
    GpioCtrlRegs.GPADIR.bit.GPIO26 = 0; // GPIO26 作为输入


    // ==========================================
    // 步骤 4【核心抗干扰】：配置硬件数字滤波器 (Qualification)
    // ==========================================
    // 告诉 DSP：这三个脚输入的数据不能来个毛刺就立刻认账，必须连续检查 6 次采样电平都一样，才认为信号真的变了
    // 4.1 设置采样窗大小：让它使用 6 个采样的窗口进行滤波
    // 10 = 6采样窗口 (6-sample window)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO24 = 2;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO25 = 2;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO26 = 2;

    // 4.2 设置采样时钟分频 (GPACTRL)
    // 规定滤波采样时钟的间隔，防止由于高频噪声引起连续 6 次误判
    // 这里的数字越长，滤波时间越长，抗低频干扰越强，但会引入微秒级的检测延时
    // 0x0A 代表将系统时钟分频，这对于 10kHz~20kHz 的霍尔换相信号来说非常安全且干净
    GpioCtrlRegs.GPACTRL.bit.QUALPRD3 = 0x0A; // 控制 GPIO24-31 的滤波采样周期

    EDIS; // 重新锁死受保护的寄存器
}
