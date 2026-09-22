# 明确修改方案 —— 高速采样 + 峰值保持

**目标**：让 Ozone 以 1 kHz 导出 `adc_raw[0]` 时**不再漏掉快速瞬态**。
**原理**：ADC 以 55.6 kHz 跑，固件每 1 ms 把「该毫秒内偏离均值最大的那个采样」写进 `adc_raw[0]`。
这样 1 kHz 的导出序列里，每一个值都是那一毫秒里最极端的采样 —— 快速击打不会丢。

**改动范围**：`Core/Src/adc.c`（2 处）、`Core/Src/main.c`（4 处）。
`Core/Src/stm32f0xx_it.c` **不用改**。

---

## 改动 1 —— `Core/Src/adc.c`：ADC 配置

### 1.1 找到这一行（约第 59 行）

```c
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
```

**换成：**

```c
  hadc.Init.Overrun = ADC_OVR_DATA_OVERRUN;
```

### 1.2 找到通道配置段（约第 65~97 行）

原来是四段 `HAL_ADC_ConfigChannel`（CHANNEL_3/4/5/6）。

**把整段删掉，换成：**

```c
  /** 只保留一个通道: PA3 = ADC_IN3
    * 采样时间 239.5 周期 @14MHz ADC 时钟 = 17.1 µs
    * 转换时间 = (239.5 + 12.5) / 14MHz = 18 µs  ->  55.6 kHz
    * 源阻抗上限从 1.6 kΩ 放宽到约 270 kΩ
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
```

> **注意**：`MX_ADC_Init()` 的其余部分（ClockPrescaler / ContinuousConvMode / DMAContinuousRequests 等）全部保持不动。

---

## 改动 2 —— `Core/Src/main.c`：新增变量（`USER CODE BEGIN PV`）

在 `/* USER CODE BEGIN PV */` 里**追加**：

```c
/* ================= 高速采样 + 1ms 峰值保持 (fw 20260925) =================
 * ADC 单通道 @55.6 kHz 连续转换, DMA 循环搬到 adc_dma_buf[128].
 * 每半块 (64 采样 = 1.15 ms) 处理一次:
 *   adc_raw[0]  = 该块内【偏离均值最大】的那个原始采样(保留符号)
 *                 -> 1 kHz 的 Ozone 导出不会再漏掉 <1ms 的瞬态
 *   adc_mean[0] = 该块均值 (对照用)
 *   blk_max_ever / blk_min_ever = 上电以来的全局极值
 *   stk_*       = 整流差分包络检波器 (击打检测本体)
 *
 * 原有的 adc_vin[0] / adc_notch_vin[0] 由 SysTick 里的 ADC_FilterTick1ms()
 * 照常从 adc_raw[0] 算出, 不需要改动。
 */
#define ADC_DMA_BLKSZ   64u                      /* 每半块 64 采样 */
#define ADC_DMA_LEN     (ADC_DMA_BLKSZ * 2u)     /* 128 半字 = 256 B */
#define STK_W           64u                      /* 包络窗(采样) = 1.15 ms */
#define STK_ON_K        6u                       /* 触发: env > 6 × floor */
#define STK_OFF_K       3u                       /* 复位: env < 3 × floor */
#define STK_REFRACT_MS  10u                      /* 不应期(ms) */

volatile uint16_t adc_dma_buf[ADC_DMA_LEN];      /* DMA 目标缓冲 */
volatile uint16_t adc_mean[1]  = {0};            /* 块均值原始值 */
volatile uint16_t blk_max_ever = 0;              /* 上电以来全局最大 */
volatile uint16_t blk_min_ever = 0xFFFF;         /* 上电以来全局最小 */
volatile uint32_t blk_reset    = 1;              /* 写 1 清零全局极值 */
volatile uint32_t blk_count    = 0;              /* 已处理块数(确认在跑) */

volatile uint32_t stk_env   = 0;                 /* 当前包络值 */
volatile uint32_t stk_floor = 1;                 /* 本底(静置时的包络值) */
volatile uint32_t stk_peak  = 0;                 /* 最近一次击打的包络峰值 */
volatile uint32_t stk_count = 0;                 /* 击打累计次数 */
volatile uint32_t stk_state = 0;                 /* 0=空闲 1=已触发 */

static uint16_t stk_prev;                        /* 上一个采样 */
static uint16_t stk_ring[STK_W];                 /* 差分滑动窗 */
static uint32_t stk_acc;                         /* 滑动和 */
static uint32_t stk_i;                           /* 环形索引 */
static uint32_t stk_floor_q;                     /* 4.12 定点本底 */
static uint32_t stk_last_ms;                     /* 上次计数时刻 */
```

---

## 改动 3 —— `Core/Src/main.c`：块处理函数（`USER CODE BEGIN 0`）

在 `/* USER CODE BEGIN 0 */` 里**追加**（放在文件末尾也可以，但必须在 `main()` 之前声明）：

```c
/* ---- 处理一个 DMA 块: 峰值保持 + 包络检测 ---- */
static void Strike_ProcessBlock(const uint16_t *buf, uint32_t n)
{
  uint16_t mx = 0, mn = 0xFFFF;
  uint32_t sum = 0;

  if (blk_reset)
  {
    blk_reset    = 0;
    blk_max_ever = 0;
    blk_min_ever = 0xFFFF;
    blk_count    = 0;
    stk_count    = 0;
    stk_peak     = 0;
  }

  for (uint32_t i = 0; i < n; i++)
  {
    uint16_t s = buf[i];

    /* --- 块极值与和 --- */
    if (s > mx) mx = s;
    if (s < mn) mn = s;
    sum += s;

    /* --- 整流差分: d = |x[n] - x[n-1]| --- */
    uint16_t dd = (s >= stk_prev) ? (uint16_t)(s - stk_prev) : (uint16_t)(stk_prev - s);
    stk_prev = s;

    /* --- W 点滑动和 = 包络 --- */
    stk_acc += dd;
    stk_acc -= stk_ring[stk_i];
    stk_ring[stk_i] = dd;
    if (++stk_i >= STK_W) stk_i = 0;
    stk_env = stk_acc;

    /* --- 双阈值滞回 + 不应期 --- */
    if (!stk_state)
    {
      if (stk_env > stk_floor * STK_ON_K)
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
      if (stk_env < stk_floor * STK_OFF_K) stk_state = 0;
    }
  }

  /* --- 全局极值 --- */
  if (mx > blk_max_ever) blk_max_ever = mx;
  if (mn < blk_min_ever) blk_min_ever = mn;

  /* --- 本底慢跟踪: 每块一次, 只在空闲状态 (击打期间冻结) --- */
  if (!stk_state)
  {
    uint32_t f = stk_floor_q >> 12;  if (f < 1u) f = 1u;
    if (stk_env < f * 8u)
    {
      int32_t tgt = (int32_t)stk_env << 12;
      stk_floor_q = (uint32_t)((int32_t)stk_floor_q +
                    ((tgt - (int32_t)stk_floor_q) >> 12));
    }
  }
  {
    uint32_t f = stk_floor_q >> 12;  if (f < 1u) f = 1u;
    stk_floor = f;
  }

  /* --- ★ 关键: 把偏离均值最大的那个采样写进 adc_raw[0] --- */
  {
    uint16_t mean = (uint16_t)(sum / n);
    adc_mean[0] = mean;
    adc_raw[0]  = (((uint32_t)mx - mean) >= ((uint32_t)mean - mn)) ? mx : mn;
  }

  blk_count++;
}
```

---

## 改动 4 —— `Core/Src/main.c`：启动流程（`USER CODE BEGIN 2`）

### 4.1 找到这一整段（约 250~261 行）

```c
  /* Four continuously converted samples generate very frequent HT/TC IRQs.
   * This application polls the DMA buffer and does not use these callbacks.
   * Mask the channel IRQ BEFORE starting, then keep only DMA error IRQs.
   * DMA transfers themselves remain enabled throughout.
   */
  adc_diag_stage = 2;
  HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
  adc_start_status = HAL_ADC_Start_DMA(&hadc, (uint32_t*)adc_raw, 4);
  __HAL_DMA_DISABLE_IT(hadc.DMA_Handle, DMA_IT_HT | DMA_IT_TC);
  HAL_NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  if (adc_start_status != HAL_OK) Error_Handler();
  adc_filter_ready = 1;
  adc_diag_stage = 3;
```

**换成：**

```c
  /* ---- 高速连续采样: 单通道 @55.6 kHz, DMA 循环搬运 ----
   * 与原来相反: 这次【必须】使能 DMA 的 HT/TC 中断, 检测在中断里做。
   */
  stk_prev    = 0;
  stk_acc     = 0;
  stk_i       = 0;
  stk_floor_q = 1u << 12;              /* floor = 1 */
  stk_last_ms = HAL_GetTick();
  for (uint32_t k = 0; k < STK_W; k++) stk_ring[k] = 0;

  adc_diag_stage = 2;
  adc_start_status = HAL_ADC_Start_DMA(&hadc, (uint32_t*)adc_dma_buf, ADC_DMA_LEN);
  if (adc_start_status != HAL_OK) Error_Handler();

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  adc_filter_ready = 1;
  adc_diag_stage = 3;
```

### 4.2 保留后面的 4 秒等待和基线标定（不用改）

```c
  HAL_Delay(4000);
  ADC_CalibrateBaseline(BASELINE_SAMPLES);
```
这两句照常保留，不影响击打测试。

---

## 改动 5 —— `Core/Src/main.c`：DMA 回调（`USER CODE BEGIN 4`）

在 `/* USER CODE BEGIN 4 */` 里**追加**：

```c
/* USER CODE BEGIN 4 */

/**
  * @brief  DMA 半传输完成 —— 前半缓冲 adc_dma_buf[0..63] 已满
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  (void)hadc;
  Strike_ProcessBlock((const uint16_t *)&adc_dma_buf[0], ADC_DMA_BLKSZ);
}

/**
  * @brief  DMA 全传输完成 —— 后半缓冲 adc_dma_buf[64..127] 已满
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  (void)hadc;
  Strike_ProcessBlock((const uint16_t *)&adc_dma_buf[ADC_DMA_BLKSZ], ADC_DMA_BLKSZ);
}

/* USER CODE END 4 */
```

> `HAL_ADC_ConvHalfCpltCallback` / `HAL_ADC_ConvCpltCallback` 是 HAL 的 `__weak` 函数，
> 在 `stm32f0xx_hal_adc.c:1794 / 1809` 声明，由 `ADC_DMAConvCplt` / `ADC_DMAHalfConvCplt` 调用。
> 你在 main.c 里定义同名函数即可自动覆盖，`stm32f0xx_it.c` 不用动。

---

## 改动 6 —— `Core/Src/stm32f0xx_it.c`

**不用改。** 第 170 行的 `DMA1_Channel1_IRQHandler()` 已经在调用 `HAL_DMA_IRQHandler(&hdma_adc)`，
HAL 会自动分发到上面两个回调。

`SysTick_Handler` 里的 `ADC_FilterTick1ms()` **也保留** —— 它负责从 `adc_raw[0]` 算出
`adc_vin[0]` 和 `adc_notch_vin[0]`，正好是我们要导出的。

---

## 编译后要导出给我们的变量

在 Ozone 的 Data Sampling 里加这些，采样率 **1 kHz**：

| 变量 | 含义 |
|---|---|
| **`adc_raw[0]`** | **峰值保持原始值（0~4095）—— 核心** |
| **`adc_notch_vin[0]`** | **陷波后电压 —— 核心** |
| `adc_mean[0]` | 块均值原始值（对照，看基线） |
| `blk_max_ever` | 上电以来全局最大原始值 |
| `blk_min_ever` | 上电以来全局最小原始值 |
| `stk_env` | 包络值 |
| `stk_floor` | 本底 |
| `stk_peak` | 最近一次击打的包络峰值 |
| `stk_count` | 击打累计次数 |
| `blk_count` | 已处理块数（确认在跑：应 ≈ 870/s） |
| `blk_reset` | 写 1 清零全局极值 |

**测试步骤**：
1. 上电，静置 **60 秒**（不要碰板子）
2. 往 `blk_reset` 写 **1**（清零 `blk_max_ever` / `blk_min_ever` / `stk_count`）
3. **锤击 5 次**，每次间隔 1 秒以上
4. 再静置 **30 秒**
5. 导出 CSV 给我（至少覆盖上面全过程）

---

## 判读（先自己看一眼，再发我）

σ 的换算：现在噪声 σ = 311 mV = 386 counts，静置均值约 672 counts。

| 观察 | 数值 | 含义 | 下一步 |
|---|---|---|---|
| `blk_count` | ≈ 870 / 秒 | 高速链在跑 | — |
| `adc_raw[0]` 静置时的波动范围 | 若 ≈ **0 ~ 1500** | 噪声 σ 仍然是 386 counts（采样没建立） | 见下方"A" |
| `adc_raw[0]` 静置时的波动范围 | 若收窄到 **400 ~ 950** | σ 降到约 130 counts，**采样时间改动生效** | 见下方"B" |
| `blk_max_ever` 静置 60 s 后 | ≈ **2600**（若 σ=386） | 纯噪声上界（330 万采样的理论极值 5.1σ） | — |
| `blk_reset=1` 后锤击，`blk_max_ever` | **跳到 3200 以上或 `blk_min_ever` 掉到 0** | **击打信号确实到了 ADC，只是 1 kHz 采样漏掉** | 见下方"B" |
| `blk_reset=1` 后锤击，`blk_max_ever` | **还是 2600 上下** | 击打在 ADC 引脚上也没有 | 见下方"A" |
| `stk_peak / stk_floor` | 锤击时 > 8 | 包络检波器可用 | 定 `STK_ON_K` |
| `stk_count` | 锤 5 次 = +5，静置 +0 | 检测成功 | — |

**A**：`blk_max_ever` 不跳 → 信号到不了 ADC 引脚。下一步在模拟端：查 TPA1286 OUT 是否有自激（输出串 50~100 Ω + 输出脚对地 100 pF），查压电片到 IN+ 的接线和屏蔽。

**B**：`blk_max_ever` 跳了 → 方案成立。下一步：
- 量锤击时 `stk_peak / stk_floor` 的比值
- 用 `blk_max[]` 的宽度定 `STK_W`（把 `blk_max[0..63]` 也加进 Ozone 更直观）
- 定死 `STK_ON_K`

---

## 附：RAM 与 CPU

| 项 | 用量 |
|---|---|
| `adc_dma_buf[128]` | 256 B |
| `stk_ring[64]` | 128 B |
| 其他新变量 | ~40 B |
| **新增合计** | **≈ 424 B** |

如果 RAM 不够（F030C6 只有 4 KB），把 `ADC_DMA_BLKSZ` 和 `STK_W` 同时改成 **32**：
- `adc_dma_buf[64]` = 128 B，`stk_ring[32]` = 64 B → 新增合计 **≈ 232 B**
- 块周期变成 32/55556 = 0.576 ms，延迟更小（预算 10 ms 完全够）

CPU：每采样约 20 条指令 × 55.6 kHz = 1.1 M 指令/s，@48 MHz 约 **2.3%**。
