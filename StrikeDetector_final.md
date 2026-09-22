# 击打检测 —— 最终方案

目标：**检出击打**，允许延迟 **10 ms**。按压只是过渡环节。
硬件现状：TPA1286 增益 200×，连接已加固。
示波器结论：**锤击瞬间 TPA1286 的 OUT 仍然只有噪声。**

---

# 第 0 部分：先说清楚"OUT 只有噪声"意味着什么

示波器在 OUT 上看到的噪声，是整条链路上最重要的一条信息，它排除了所有软件方案。

## 0.1 三种可能，用两分钟区分

把探头留在 OUT，**探头的弹簧地接放大器的模拟地**（不要用长地线夹），DC 耦合，1 ms/div：

| 实验 | 现象 | 结论 | 对策 |
|---|---|---|---|
| **A** 拔掉压电片，把 IN+ 短接到 2.5 V 基准 | OUT 噪声**消失** | 噪声来自传感器/线缆 | 做 §0.2 |
| **B** 同上 | OUT 噪声**不变** | **放大器自己就在振荡/噪声** | 做 §0.3 ← 最可能 |
| **C** 恢复接传感器，敲击时用 10 µs/div 再看 | 看到几 MHz 的等幅振荡 | **自激振荡**，输出被振荡占据，信号被淹没 | 做 §0.3 |

**你要的"击打检测"，卡在这一步。** 一个输出在静止时就是噪声的放大器，无论后面接多好的 ADC、写多好的滤波器，都检不出击打 —— 因为信息在进 ADC 之前就已经不存在了。

## 0.2 如果噪声来自传感器/线缆

- 压电片是高阻容性源，**线缆必须用屏蔽线，屏蔽层单端接地**（接放大器地，不接传感器外壳）
- 线缆长度压到最短，不要和电机线、电源线并行
- 传感器到 IN+ 之间串 1 kΩ，并联 100 pF 到地（限带 + 抗射频）

## 0.3 如果放大器在自激（最可能）

200 倍增益的仪表放大器直驱线缆 + ADC 引脚的容性负载，**几乎必然振荡**。TPA1286 这类 INA 的容性负载上限通常只有几十 pF。

**修法（按顺序试，每步都用示波器确认）：**

```
                      ┌──── 50 ~ 100 Ω ────┬──── 到 ADC
   TPA1286 OUT ───────┤                    │
                      │                  100 pF
                      └── 100 pF          │
                         到模拟地        模拟地
   （电容紧贴放大器输出引脚，电阻在电容之后）
```

要点：
1. **100 pF–1 nF 直接焊在 TPA1286 输出引脚和模拟地之间**，引线越短越好
2. 输出串 **50–100 Ω**，这个电阻放在小电容**之后**
3. 检查 REF 引脚是不是被低阻驱动（手册要求），悬空或高阻会让 INA 不稳定
4. 检查电源去耦：每个电源脚旁边 100 nF + 10 µF，紧贴引脚
5. 如果还振，在反馈回路里加补偿（TPA1286 手册的 C_F 位置）

**振荡消除后，OUT 应该变成一条安静得多的直流线。** 这时再敲一下 —— 你会第一次看到真正的击打波形。

---

# 第 1 部分：adc.c 的修改

**现在的问题**：`SamplingTime = ADC_SAMPLETIME_1CYCLE_5` = 1.5 周期 @14 MHz = **107 ns**。
STM32F0 采样保持电容约 8 pF，充到 12 位精度需要约 8.3τ，**源阻抗上限只有 1.6 kΩ**。
而且 4 通道扫描让 `adc_raw[0]` 每 4 µs 才刷新一次，Ozone 以 1 kHz 导出 —— **每 250 次转换只留 1 次**。

**改后**：单通道 + 239.5 周期 → **55.6 kHz 有效采样率**，一个 200 µs 的击打瞬态能拿到 **11 个采样点**。

## 1.1 替换 `MX_ADC_Init()` 里的 ADC 全局配置

```c
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;      /* ADC 时钟 = HSI14 = 14 MHz */
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_OVERRUN;             /* 高速连续转换必须用覆盖模式 */
```

> 唯一改动：`ADC_OVR_DATA_PRESERVED` → `ADC_OVR_DATA_OVERRUN`。
> 覆盖模式下，万一处理跟不上，丢的是旧数据而不是让 ADC 停摆 —— 对击打检测更重要。

## 1.2 通道配置：**保留一路，删掉另外三路**

```c
  /* 唯一保留的通道 */
  sConfig.Channel = ADC_CHANNEL_3;                      /* PA3 = ADC_IN3 */
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;    /* 1.5 -> 239.5 周期 */
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* 删掉原来 ADC_CHANNEL_4 / 5 / 6 的三段 HAL_ADC_ConfigChannel 调用 */
```

**转换时间 = (239.5 + 12.5) / 14 MHz = 18 µs → 55.6 kHz。**

## 1.3 采样率汇总

| | 改前 | 改后 |
|---|---|---|
| 采样时间 | 107 ns（1.5 周期） | **18 µs（239.5 周期）** |
| 源阻抗上限 | 1.6 kΩ | **≈270 kΩ** |
| 通道数 | 4 | **1** |
| `adc_buf[0]` 刷新率 | 250 kHz（每 4 µs） | **55.6 kHz（每 18 µs）** |
| 200 µs 瞬态拿到几点 | 1（而且是 250 抽 1） | **11 点，全拿到** |

---

# 第 2 部分：新建 `Core/Inc/strike.h`

```c
#ifndef STRIKE_H
#define STRIKE_H

#include <stdint.h>

/* ================= 击打检测器 =================
 * 采样: 单通道 @ 55.6 kHz (ADC 采样时间 239.5 周期)
 * 原理: 击打 = 短时快变能量突增。
 *       d[n]   = |x[n] - x[n-1]|          整流差分: 抑制缓慢漂移, 对快变敏感
 *       env[n] = Σ_{k=0..W-1} d[n-k]      滑动和 = 整流+平滑包络
 *       env 与本底 floor 做双阈值滞回比较
 *
 * 全部整数运算，无浮点、无乘除。
 * 检测延迟 = DMA 块(1.15 ms) + 包络窗(1.15 ms) + 判定 ≈ 2.3 ms  <  10 ms 预算
 */

#define STK_FS_HZ       55556u  /* ADC 有效采样率 */
#define STK_BLOCK       64u     /* DMA 半/全传输块大小(采样数)。64/55556 = 1.15 ms */
#define STK_W           64u     /* 包络窗(采样数)。64/55556 = 1.15 ms。
                                 * 击打瞬态更短就调小(如 16 = 288 µs);
                                 * 振铃更长就调大(如 256 = 4.6 ms)。
                                 * 最优值 ≈ 击打瞬态持续时间。 */

#define STK_ON_K        6u      /* 触发: env > 6 × floor  */
#define STK_OFF_K       3u      /* 复位: env < 3 × floor  */
#define STK_REFRACT_MS  10u     /* 不应期 = 允许的检测延迟 */
#define STK_FLOOR_SH    12u     /* 本底 EMA 移位(按块更新): τ = 4096 块 = 4.7 s */

void Strike_Init(void);
void Strike_ProcessBlock(const uint16_t *buf, uint32_t n);

/* ---- Ozone 观察变量 ---- */
extern volatile uint32_t stk_env;            /* 当前包络值 */
extern volatile uint32_t stk_floor;          /* 本底(静置时的包络值) */
extern volatile uint32_t stk_peak;           /* 最近一次击打的包络峰值 */
extern volatile uint32_t stk_count;          /* 击打累计次数 */
extern volatile uint32_t stk_state;          /* 0=空闲 1=已触发 */
extern volatile uint32_t stk_blocks;         /* 已处理块数(用于确认在跑) */
extern volatile uint32_t stk_max_env;        /* 上电以来 env 的最大值 */
extern volatile uint32_t stk_reset;          /* 写 1 清零统计 */

#endif /* STRIKE_H */
```

---

# 第 3 部分：新建 `Core/Src/strike.c`

```c
#include "main.h"
#include "strike.h"

volatile uint32_t stk_env     = 0;
volatile uint32_t stk_floor   = 1;
volatile uint32_t stk_peak    = 0;
volatile uint32_t stk_count   = 0;
volatile uint32_t stk_state   = 0;
volatile uint32_t stk_blocks  = 0;
volatile uint32_t stk_max_env = 0;
volatile uint32_t stk_reset   = 1;

static uint16_t stk_prev;
static uint16_t stk_ring[STK_W];
static uint32_t stk_acc;
static uint32_t stk_i;
static uint32_t stk_floor_q;      /* 4.12 定点, floor = stk_floor_q >> 12 */
static uint32_t stk_last_ms;

void Strike_Init(void)
{
    stk_prev = 0;
    stk_acc  = 0;
    stk_i    = 0;
    for (uint32_t i = 0; i < STK_W; i++) stk_ring[i] = 0;
    stk_floor_q  = 1u << 12;      /* floor = 1 */
    stk_floor    = 1;
    stk_env      = 0;
    stk_state    = 0;
    stk_peak     = 0;
    stk_count    = 0;
    stk_blocks   = 0;
    stk_max_env  = 0;
    stk_last_ms  = HAL_GetTick();
}

/* 处理一个 ADC 采样。纯整数, 约 20 条指令。 */
static inline void Strike_Sample(uint16_t s)
{
    /* 1. 整流差分 */
    uint16_t d = (s >= stk_prev) ? (uint16_t)(s - stk_prev) : (uint16_t)(stk_prev - s);
    stk_prev = s;

    /* 2. W 点滑动和 = 包络 */
    stk_acc += d;
    stk_acc -= stk_ring[stk_i];
    stk_ring[stk_i] = d;
    if (++stk_i >= STK_W) stk_i = 0;
    stk_env = stk_acc;

    if (stk_env > stk_max_env) stk_max_env = stk_env;

    /* 3. 双阈值滞回 + 不应期 */
    uint32_t on_thr  = stk_floor * STK_ON_K;
    uint32_t off_thr = stk_floor * STK_OFF_K;

    if (!stk_state)
    {
        if (stk_env > on_thr)
        {
            stk_state = 1;
            stk_peak  = stk_env;
            uint32_t now = HAL_GetTick();
            if ((now - stk_last_ms) >= STK_REFRACT_MS)
            {
                stk_count++;
                stk_last_ms = now;
            }
        }
    }
    else
    {
        if (stk_env > stk_peak) stk_peak = stk_env;
        if (stk_env < off_thr)  stk_state = 0;
    }
}

void Strike_ProcessBlock(const uint16_t *buf, uint32_t n)
{
    if (stk_reset)
    {
        stk_reset   = 0;
        stk_count   = 0;
        stk_peak    = 0;
        stk_max_env = 0;
        stk_blocks  = 0;
    }

    for (uint32_t i = 0; i < n; i++) Strike_Sample(buf[i]);

    /* 4. 本底慢跟踪: 每块更新一次, 且只在空闲状态更新
     *    (击打期间冻结, 否则击打会把本底抬上去, 下一次就漏检) */
    if (!stk_state)
    {
        uint32_t f = stk_floor_q >> 12;
        if (f < 1u) f = 1u;
        if (stk_env < f * 8u)                      /* 只在"像本底"的样本上学习 */
        {
            int32_t target = (int32_t)stk_env << 12;
            stk_floor_q = (uint32_t)((int32_t)stk_floor_q +
                          ((target - (int32_t)stk_floor_q) >> STK_FLOOR_SH));
        }
    }
    uint32_t f = stk_floor_q >> 12;
    if (f < 1u) f = 1u;
    stk_floor = f;

    stk_blocks++;
}
```

---

# 第 4 部分：`Core/Src/main.c` 的修改

## 4.1 顶部 include

```c
/* USER CODE BEGIN Includes */
#include "strike.h"
/* USER CODE END Includes */
```

## 4.2 变量区（`USER CODE BEGIN PV`）

```c
/* ---- 高速采样 DMA 缓冲 ---- */
#define ADC_BUF_LEN   (STK_BLOCK * 2u)          /* 128 个半字 */
volatile uint16_t adc_buf[ADC_BUF_LEN];
```

## 4.3 替换启动流程（`USER CODE BEGIN 2`）

把这段：
```c
  adc_diag_stage = 2;
  HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
  adc_start_status = HAL_ADC_Start_DMA(&hadc, (uint32_t*)adc_raw, 4);
  __HAL_DMA_DISABLE_IT(hadc.DMA_Handle, DMA_IT_HT | DMA_IT_TC);
  HAL_NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
```

**换成：**
```c
  /* ---- 高速连续采样: 单通道 @55.6 kHz, DMA 循环搬运 ---- */
  Strike_Init();
  adc_diag_stage = 2;
  adc_start_status = HAL_ADC_Start_DMA(&hadc, (uint32_t*)adc_buf, ADC_BUF_LEN);
  if (adc_start_status != HAL_OK) Error_Handler();

  /* DMA 半传输/全传输中断必须使能 —— 检测在中断里做, 不靠轮询 */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  adc_diag_stage = 3;
```

> 注意：原来的代码 **特地禁止了** HT/TC 中断（因为那版是轮询的）。现在必须反过来。

## 4.4 在 `main.c` 末尾（`USER CODE BEGIN 4`）加 DMA 完成回调

```c
/* USER CODE BEGIN 4 */

/**
  * @brief  DMA 半传输完成 —— 前半缓冲已满
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  (void)hadc;
  Strike_ProcessBlock((const uint16_t *)&adc_buf[0], STK_BLOCK);
}

/**
  * @brief  DMA 全传输完成 —— 后半缓冲已满
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  (void)hadc;
  Strike_ProcessBlock((const uint16_t *)&adc_buf[STK_BLOCK], STK_BLOCK);
}

/* USER CODE END 4 */
```

## 4.5 主循环

高速检测完全在中断里跑，主循环只剩下：
```c
  while (1)
  {
    adc_diag_stage = 4;
    main_loop_count++;
    /* 击打检测在 DMA 中断里完成，这里只做慢速的按压/基线处理 */
    HAL_Delay(1);
  }
```

> **原来的 `ADC_FilterTick1ms()` / `adc_raw[4]` / 陷波器 / `osc_update` 那一整套都可以停用了。**
> 需要的话把 `adc.c` 里的 `adc_raw` 也改成从 `adc_buf` 取平均做按压路径，但击打检测不依赖它。

---

# 第 5 部分：`Core/Src/stm32f0xx_it.c`

**不用改。** 第 170 行的 `DMA1_Channel1_IRQHandler()` 已经在调用 `HAL_DMA_IRQHandler(&hdma_adc)`，
HAL 会自动分发到第 4.4 节的两个回调。

如果 `SysTick_Handler` 里还在调 `ADC_FilterTick1ms()`，建议注释掉，避免和高速链抢时间。

---

# 第 6 部分：编译后的 Ozone 变量与调参

## 6.1 先确认在跑

| 变量 | 应该看到 |
|---|---|
| `adc_diag_stage` | 4 |
| `adc_start_status` | 0 |
| `stk_blocks` | 每秒 +869 左右 |
| `stk_floor` | 一个稳定的正数 |
| `stk_state` / `stk_count` | 静置时应始终 0 |

## 6.2 关键诊断：`stk_env` 对比 `stk_floor`

**这是整套方案里最有用的一个测量**，它直接告诉你信号在 ADC 输入端有多大：

```
静置时记下 stk_floor 的值 F。
锤一下，看 stk_peak 的值 P。
比值 P / F = 击打的信噪比(在包络域)。
```

| P/F | 含义 | 下一步 |
|---|---|---|
| **≈ 1** | **ADC 端完全没有击打信号** | 回到 §0，信号在模拟端就没了 |
| 1.5 ~ 3 | 有信号但太弱 | 把 `STK_W` 调到接近击打持续时间；`STK_ON_K` 设 3~4 |
| **> 8** | 信号清楚 | `STK_ON_K` 设成 F 波动的 3 倍、P 的 1/3 之间，例如 5~6 |

## 6.3 调 `STK_W`（包络窗）的方法

`STK_W` 的最优值 ≈ **击打瞬态的持续时间**，用示波器量 OUT 上的击打宽度即可。

| 击打瞬态宽度 | `STK_W` | 窗长 |
|---|---|---|
| 200 µs | 11 → 取 **12** | 216 µs |
| 500 µs | 28 → 取 **32** | 576 µs |
| 1 ms | 56 → 取 **64** | 1.15 ms |
| 3 ms | 167 → 取 **128** | 2.30 ms |

窗太长会把噪声也积进来（本底 `stk_floor` 变大，比值下降）；窗太短会让信号只占几个采样点。

## 6.4 调 `STK_ON_K`

1. 静置 60 秒，记下 `stk_env` 的最大值和最小值 → 得到本底波动范围
2. 锤 5 次，记下 `stk_peak` 的 5 个值 → 得到信号范围
3. **`STK_ON_K` 取"本底波动上限 × 3"和"信号最小值 ÷ 3"之间的值**

代码里 `stk_max_env` 会一直保持上电以来的最大值，方便你一次性读出来。
清零统计：往 `stk_reset` 写 1。

## 6.5 延迟核算（预算 10 ms）

| 环节 | 时间 |
|---|---|
| DMA 块缓冲 | 64 / 55556 = **1.15 ms** |
| 包络窗 `STK_W`=64 | 64 / 55556 = **1.15 ms** |
| 判定 + 置位 | 中断内，< 10 µs |
| **合计** | **≈ 2.3 ms** ✅ |

`STK_REFRACT_MS = 10` 是**两次击打的最小间隔**，不是延迟。如果两次击打间隔可能小于 10 ms，把它调小。

---

# 第 7 部分：落地顺序（按这个顺序做，每步都有判据）

| 步 | 做什么 | 判据 | 做不完会怎样 |
|---|---|---|---|
| **1** | §0 用示波器确认 OUT 的噪声来源 | OUT 变成安静的直流 | **后面全部无意义** |
| **2** | §0.3 加 100 pF + 50~100 Ω，消除自激 | 10 µs/div 下没有等幅振荡 | 击打信号永远被淹没 |
| **3** | §1 改 `adc.c`（采样时间 + 单通道 + 覆盖模式） | `stk_floor` 明显下降 | 采样没建立，读的是残留电荷 |
| **4** | §2-5 加入检测器并编译 | `stk_blocks ≈ 869/s` | — |
| **5** | §6.2 测 `stk_peak / stk_floor` | 比值 > 8 | 若 ≈1，回第 1 步 |
| **6** | §6.3/6.4 定 `STK_W` 和 `STK_ON_K` | 锤 5 次 = `stk_count` +5，静置 1 分钟 +0 | — |

---

# 附：为什么不用"平均/低通"检出击打

对**单采样脉冲**（幅度 A、噪声 σ），各级平滑后的残余：

| 平滑点数 N | 脉冲残余 | 噪声 σ | SNR |
|---|---|---|---|
| **1（不平均）** | **A** | **σ** | **A/σ** |
| 5 | A/5 | σ/√5 | 0.45·A/σ |
| 64 | A/64 | σ/8 | 0.125·A/σ |
| 500 | A/500 | σ/22.4 | 0.045·A/σ |

**平均对冲击型信号是负收益**：信号按 1/N 衰减，噪声只按 1/√N 衰减，SNR 掉 √N 倍。

所以击打路径**必须**用"整流差分 + 短窗包络 + 自适应阈值"这类**非线性**结构，
而不能用对付按压那套线性低通。

（按压路径的线性滤波方案见 `SignalChain_design.md`，那套对慢信号是最优的，已实测验证：
输出 σ 9.7 mV、6σ 阈值 58 mV、延迟 247 ms、误触发 0 次。）
