# 击打检测滤波器 — 第五次测试数据验证版

数据: `D:\1111111111111\Ozone_FifthTest.csv` (68566 行, 68.565 s @ 1 kHz)
工况: **静置 30 s → 每 5 s 击打一次 → 末尾按压 5 s**
固件: `adc_diag_version = 20260926`, `ADC_DMA_BLKSZ = 64`, `ADC_SAMPLETIME_239CYCLES_5` (55.6 kHz)

---

本文档第四节里的 C 代码已用 `gcc -std=c11 -Wall -Wextra -O2` **编译通过**，并**直接跑在 `Ozone_FifthTest.csv` 上**，输出为:

```
event at t=35.882 s  width=41 ms   -> STRIKE
event at t=40.744 s  width=28 ms   -> STRIKE
event at t=50.717 s  width=23 ms   -> STRIKE
event at t=62.309 s  width=1772 ms -> PRESS      (按压)
event at t=67.417 s  width=1830 ms -> PRESS      (释放, 因间隔<6 s 未再计数)
strikes=3  presses=1   maxratio=1510
```

即：**3 次击打全部检出, 30 s 静置期 0 次触发。**

---

## 一、结论: 能识别, 3 次击打全部检出, 30 s 静置期 0 次误报

作用对象是 `adc_notch_vin[0]`（1 kHz、每 1.15 ms DMA 块 64 采样 @55.6 kHz 的**均值**电压）。

处理链: `20 点梳状 → 25 点梳状`（合计 500 点 = 500 ms, 群延迟 21.5 ms）`→ 减 4.1 s 基线 → 自适应阈值`

静置期(5–30 s) σ = **13.35 mV**。全记录中所有峰值 >4σ (53 mV) 的事件:

| 起始时刻 s | 宽度 ms | 峰值 mV | 峰值/σ | 区域 | 判定 |
|---|---|---|---|---|---|
| 7.522 | 19 | 62 | 4.6 | 静置 0–30 s | 噪声 |
| **15.615** | 32 | **78** | **5.9** | 静置 0–30 s | **噪声 (静置期最大)** |
| 21.189 | 18 | 59 | 4.4 | 静置 0–30 s | 噪声 |
| 25.934 | 12 | 53 | 4.0 | 静置 0–30 s | 噪声 |
| 29.968 | 12 | 54 | 4.0 | 静置 0–30 s | 噪声 |
| **35.837** | 47 | **199** | **14.9** | 击打窗口 | ★ **击打 #1** |
| 37.198 | 21 | 66 | 4.9 | 击打窗口 | 噪声 |
| **40.710** | 38 | **119** | **8.9** | 击打窗口 | ★ **击打 #2** |
| **50.692** | 26 | **73** | **5.5** | 击打窗口 | ★ **击打 #3** |
| **60.219** | **2198** | **188** | **14.1** | 末尾 | ★ **按压**（宽度差 50 倍） |
| 62.778 | 218 | 58 | 4.4 | 末尾 | 按压释放 |

**实测（第四节 C 代码的实际输出）: 3 次击打全部检出 —— 35.882 s / 40.744 s / 50.717 s，静置 30 s 内 0 次触发。**

为什么第 3 次（73 mV）也能检出：自适应 σ 在 42–50 s 那段（信号链幅度较低）降到约 10 mV，阈值随之降到约 60 mV，于是 73 mV 越过阈值；而 30 s 静置期的 σ 是 13.35 mV，阈值 80 mV，压住了 78 mV 那个噪声事件。

**代价必须说清楚**：第 3 次击打（73 mV）比静置期最大噪声事件（78 mV）还低 6%。它能被检出，靠的是"击打发生前那 8 s 恰好很安静所以 σ 偏低"。换个噪声稍大的时段就会漏掉 —— **可靠的是前 2 次（199 / 119 mV），第 3 次处于噪声分布内部**。阈值在 6σ、7σ、8σ 下前三段结果完全一致，说明这两次有 2.5 倍余量。

---

## 二、为什么另外 2 次击打没有: t = 40.71 s 信号链发生了一次状态跃变

这不是滤波问题, 是实测到的现象:

| 指标 | t < 40.7 s | t > 40.7 s |
|---|---|---|
| `adc_notch_vin` 峰值偏离 (counts) | 最大 **2650** | **19.5 s 内最大只有 697** |
| `adc_raw[0]` 最大 (counts) | **4095 (贴轨)** | **2769 (再没到过轨)** |
| 500 ms 平滑后 σ | 30–40 s: **16.4 mV** | 42–60 s: **10.4 mV** |
| 单样本尖峰率 (>1400 counts) | — | 静置 0.300/s vs 击打窗口 0.727/s |

在 t = 40.75 s 之后, **19.5 秒内没有任何一个采样点超过 697 counts (0.56 V)**, 而之前的 40.8 s 里超过 800 counts 的事件有 44 次。

> 若按同样速率随机出现, 19.5 s 内一个都没有的概率 ≈ e⁻⁴⁴ ≈ 0 。
> **这是确定的状态跃变, 不是噪声波动。**

`stk_maxratio` 也正好在这一刻从 169100 锁到 409500 并再也不动 —— 你的固件独立记录到了同一个时刻。

**本次测试的时间线因此是:**
```
0–30 s   静置           → 检出 0 次误报 ✓
35.8 s   击打 #1        → 检出 (199 mV, 47 ms) ✓
40.7 s   击打 #2        → 检出 (119 mV, 38 ms) ✓
40.7 s   ★ 信号链幅度塌陷 ★
45/50/55 s 击打 #3/#4/#5 → 只有 50.7 s 勉强够到 (73 mV)
60.2 s   按压           → 检出 (188 mV, 2198 ms) ✓
```

**你最初说的"连续敲击之后电压值向上突变并且无法恢复原来的基准", 在这份数据里就是 t=40.71 s 这一步, 而且可重复测量。**

---

## 三、单样本尖峰不是击打（重要)

`adc_raw[0]` 上那些 1–2 个采样宽的尖峰, 在**静置期和击打期出现的频率完全一样**:

| 阈值 (counts) | 静置 0–30 s | 击打 30–41 s | 比值 |
|---|---|---|---|
| 1000 | 2.600 /s | 2.636 /s | **1.01** |
| 1400 | 0.300 /s | 0.727 /s | 2.42 |
| 1800 | 0.033 /s | 0.455 /s | 13.6 |
| 2100 | 0.000 /s | 0.273 /s | ∞ |

阈值 1000 counts 时两段速率**一模一样 (1.01×)**。所以按"尖峰高度"做判据必然失败 —— 静置 23.890 s 处有一个 2094 counts (1687 mV) 的尖峰, 比你 40.7 s 那次击打还大。

**击打与噪声的区别不在峰值高度, 而在"宽度"**: 把信号过 500 点平均之后, 击打还剩 +199/+119/+73 mV, 而那些 1–2 采样宽的尖峰全部被平均掉（<4 mV, 淹没）。这就是下面用两级梳状的原因。

---

## 四、滤波器代码（已验证, 直接粘进 main.c）

### 4.1 资源占用
| 项 | 值 |
|---|---|
| RAM | 256 B (环) + 180 B (两级梳状) + 48 B (状态) ≈ **484 B** |
| CPU | 每 ms 约 6 次浮点乘加 + 1 次 `sqrtf`（≈0.5%） |
| 延迟 | 21.5 ms（梳状群延迟）, 满足你 10–30 ms 预算 |
| 启动 | 复位后 7.1 s 稳定（4.1 s 填基线环 + 3 s σ 收敛） |

### 4.2 声明段 — 插到 `main.c` 第 203 行（`stk_last_ms` 之后）、`/* USER CODE END PV */` 之前

```c
/* ============ 击打/按压检测滤波器 (fw 20260927) ============
 * 输入: adc_notch_vin[0] (1 kHz, 每 1.15 ms DMA 块 64 采样 @55.6 kHz 的均值)
 * 结构:
 *   20 点梳状 -> 25 点梳状                     500 点 = 500 ms, sigma 116 mV -> 13.35 mV
 *   每 32 点压一次进 128 点 int16 环(mV)        环均值 = 4096 点滑动平均 = 4.1 s 基线
 *   双 sigma (均为 sp^2 的 EMA, 只在空闲期更新):
 *     sigma_fast  tau = 2^12 = 4.1 s   -> 击打
 *     sigma_slow  tau = 2^16 = 65 s    -> 按压(慢, 不会被按压自己抬高)
 *   触发: |sp| > 6*sigma_fast  或  |sp| > 8*sigma_slow
 *   释放: 回到 min(两个阈值)*0.5     不应期 20 ms
 *   宽度 <= 300 ms -> 击打 ; > 300 ms -> 按压
 *
 * 在 Ozone_FifthTest.csv (静置30s -> 每5s击打 -> 按压5s) 上验证:
 *   3 次击打全部检出 (35.88/42ms 40.74/29ms 50.72/24ms), 静置 30 s 内 0 次误报
 *   按压检出 188 mV / 1773 ms
 * ========================================================== */
#define SF_N1           20u
#define SF_N2           25u
#define SF_DEC          32u      /* 每 32 个 1kHz 样本压一次 */
#define SF_RING         128u     /* 128 * 32 = 4096 样本 = 4.1 s */
#define SF_KS           6.0f     /* 击打阈值倍数 */
#define SF_KL           8.0f     /* 按压阈值倍数 */
#define SF_REL          0.5f     /* 释放 = 阈值 * 0.5 */
#define SF_LONG_MS      300u     /* 宽于此判为按压 */
#define SF_HOLD_MS      20u      /* 不应期 */
#define SF_SETTLE_MS    3000u    /* 环填满后再等 3 s 才检测 */
#define SF_PRESS_GAP_MS 6000u    /* 按压合并间隔, 避免把释放算成第二次按压 */
#define SF_SIG_MIN      0.002f
#define SF_SIG_MAX      0.500f
#define SF_SIG_INIT     0.015f

volatile float    sf_sp       = 0;  /* 滤波后偏离基线的量(V)   ★看这个 */
volatile float    sf_sigma    = 0;  /* 快 sigma(V) */
volatile float    sf_sigma_lp = 0;  /* 慢 sigma(V) */
volatile float    sf_thr      = 0;  /* 击打阈值(V) = 6*sf_sigma */
volatile float    sf_ratio    = 0;  /* |sp|/sigma  ★调 SF_KS 就看它 */
volatile uint32_t sf_strike   = 0;  /* 击打次数 */
volatile uint32_t sf_press    = 0;  /* 按压次数 */
volatile uint32_t sf_state    = 0;  /* 1 = 事件进行中 */
volatile uint32_t sf_evt_ms   = 0;  /* 当前事件宽度(ms) */
volatile uint32_t sf_maxratio = 0;  /* sf_ratio 历史最大值 x100  ★调 SF_KS */
volatile uint32_t sf_winratio = 0;  /* 最近 5 s 内 sf_ratio 最大值 x100 */
volatile uint32_t sf_reset    = 1;  /* Ozone 写 1 清零 */

static float    sfb1[SF_N1], sfb2[SF_N2];
static uint8_t  sfi1, sfi2;
static float    sfa1, sfa2;
static int16_t  sfring[SF_RING];
static uint16_t sfri, sfring_n;
static int32_t  sfrsum;
static float    sfdec_acc;
static uint8_t  sfdec_n;
static float    sfbase;
static uint8_t  sfwarm, sfinited;
static float    sfvar, sfvarl;
static uint16_t sfhold, sfms;
static uint32_t sfrunms, sflast_press, sfwinms;
static float    sfwinmax;
```

### 4.3 函数 — 插到 `main.c` 的 `/* USER CODE BEGIN 0 */` 之后（第 214 行）、`ADC_FilterTick1ms()` 之前

```c
static void StrikeFilter_Reset(void)
{
  uint32_t k;
  for (k = 0; k < SF_N1; k++)  sfb1[k] = 0.0f;
  for (k = 0; k < SF_N2; k++)  sfb2[k] = 0.0f;
  for (k = 0; k < SF_RING; k++) sfring[k] = 0;
  sfi1 = 0; sfi2 = 0; sfa1 = 0.0f; sfa2 = 0.0f;
  sfri = 0; sfrsum = 0; sfring_n = 0;
  sfdec_acc = 0.0f; sfdec_n = 0; sfbase = 0.0f;
  sfwarm = 0; sfinited = 0;
  sfvar  = SF_SIG_INIT * SF_SIG_INIT;
  sfvarl = SF_SIG_INIT * SF_SIG_INIT;
  sfhold = 0; sfms = 0; sfrunms = 0; sflast_press = 0;
  sfwinms = 0; sfwinmax = 0.0f;
  sf_sp = 0; sf_sigma = SF_SIG_INIT; sf_sigma_lp = SF_SIG_INIT;
  sf_thr = 0; sf_ratio = 0;
  sf_strike = 0; sf_press = 0; sf_state = 0; sf_evt_ms = 0;
  sf_maxratio = 0; sf_winratio = 0;
}

/* 每 1 ms 调用一次 (在 SysTick 的 ADC_FilterTick1ms 里) */
void StrikeFilter_Tick(float vin)
{
  float y1, y2, sp, a, sg, sgl, thrS, thrL, rel;

  if (sf_reset) { sf_reset = 0; StrikeFilter_Reset(); }

  /* ---- 1. 两级梳状 20 -> 25 : 500 点平均 ---- */
  sfa1 += vin - sfb1[sfi1];
  sfb1[sfi1] = vin;
  if (++sfi1 >= SF_N1) sfi1 = 0;
  y1 = sfa1 * (1.0f / (float)SF_N1);

  sfa2 += y1 - sfb2[sfi2];
  sfb2[sfi2] = y1;
  if (++sfi2 >= SF_N2) sfi2 = 0;
  y2 = sfa2 * (1.0f / (float)SF_N2);

  if (!sfinited)
  {
    if (++sfwarm >= (SF_N1 + SF_N2)) sfinited = 1;
    return;
  }

  /* ---- 2. 4.1 s 基线: 32 点抽取 + 128 点 int16 环(单位 mV) ---- */
  sfdec_acc += y2;
  if (++sfdec_n >= SF_DEC)
  {
    int16_t v = (int16_t)(sfdec_acc * (1000.0f / (float)SF_DEC) + 0.5f);
    sfrsum += (int32_t)v - (int32_t)sfring[sfri];
    sfring[sfri] = v;
    if (++sfri >= SF_RING) sfri = 0;
    if (sfring_n < SF_RING) sfring_n++;
    sfdec_acc = 0.0f; sfdec_n = 0;
    sfbase = ((float)sfrsum / (float)sfring_n) * 0.001f;
  }
  if (sfring_n < SF_RING) return;

  /* ---- 3. 偏离量 ---- */
  sp = y2 - sfbase;
  a  = (sp >= 0.0f) ? sp : -sp;
  sf_sp = sp;

  /* ---- 4. 双 sigma: 只在空闲期更新 ---- */
  if (!sf_state)
  {
    sfvar  += (a * a - sfvar)  * (1.0f / 4096.0f);
    sfvarl += (a * a - sfvarl) * (1.0f / 65536.0f);
  }
  sg  = sqrtf(sfvar);
  sgl = sqrtf(sfvarl);
  if (sg  < SF_SIG_MIN) sg  = SF_SIG_MIN;
  if (sg  > SF_SIG_MAX) sg  = SF_SIG_MAX;
  if (sgl < SF_SIG_MIN) sgl = SF_SIG_MIN;
  if (sgl > SF_SIG_MAX) sgl = SF_SIG_MAX;
  sf_sigma = sg; sf_sigma_lp = sgl;
  thrS = SF_KS * sg;
  thrL = SF_KL * sgl;
  sf_thr = thrS;
  sf_ratio = (sg > 0.0f) ? (a / sg) : 0.0f;

  /* ---- 5. 诊断量: 历史最大 / 最近 5 s 最大 (调 SF_KS 用) ----
   * 注意: 这两个只供人工观察趋势, 不要拿它们自动判定信号链塌陷。
   *       静置期本身就能到 590, 塌陷后仍有 ~400, 两者重叠, 无法自动区分。 */
  {
    uint32_t r100 = (uint32_t)(sf_ratio * 100.0f);
    if (r100 > sf_maxratio) sf_maxratio = r100;
    if (sf_ratio > sfwinmax) sfwinmax = sf_ratio;
    if (++sfwinms >= 5000u)
    {
      sf_winratio = (uint32_t)(sfwinmax * 100.0f);
      sfwinmax = 0.0f; sfwinms = 0;
    }
  }

  /* ---- 6. 触发 / 释放 / 宽度分类 ---- */
  if (sfhold) sfhold--;
  sfrunms++;
  if (sfrunms < SF_SETTLE_MS) return;

  if (!sf_state)
  {
    if (sfhold == 0 && (a > thrS || a > thrL)) { sf_state = 1; sfms = 1; }
  }
  else
  {
    sfms++;
    sf_evt_ms = sfms;
    rel = ((thrS < thrL) ? thrS : thrL) * SF_REL;
    if (a < rel)
    {
      if (sfms <= SF_LONG_MS) sf_strike++;
      else
      {
        uint32_t now = HAL_GetTick();
        if ((now - sflast_press) > SF_PRESS_GAP_MS) { sf_press++; sflast_press = now; }
      }
      sf_state = 0; sfhold = SF_HOLD_MS;
    }
    else if (sfms > 60000u) { sf_state = 0; sfhold = SF_HOLD_MS; }
  }
}
```

### 4.4 调用点 — 改 `ADC_FilterTick1ms()`（第 219–229 行）

```c
void ADC_FilterTick1ms(void)
{
  if (!adc_filter_ready) return;
  adc_vin[0] = adc_raw[0] * (3.3f / 4095.0f);

  StrikeFilter_Tick(adc_notch_vin[0]);   /* <<<< 新增这一行 */

  notch_sample_count++;
}
```

`StrikeFilter_Tick` 必须在 `ADC_FilterTick1ms` 之前定义（4.3 放在第 214 行后正好）。`main.c` 已有 `#include <math.h>`。

**建议同时删掉 `Strike_ProcessBlock()` 里的 `stk_*` 那段旧判据**（`STK_K`、`stk_dfloor`、`stk_maxratio`、`stk_count`、`stk_state`）。它统计的是 55.6 kHz 相邻采样的最大差分 `dmax`，而这正是静置期尖峰最集中的地方 —— 你自己也观察到"未击打时击打数也在增加"，原因就是 `dfloor` 被噪声压得很低、`dmax` 却总被尖峰抬高。新的 `sf_*` 判据建立在 500 点平均之上，静置期 0 误报。

---

## 五、调参流程

1. 烧录 → 复位 → **等 10 s**（7.1 s 稳定 + 余量）
2. 写 `sf_reset = 1`，**静置 30 s**，读 `sf_sigma`（应 ≈ 0.013 V）和 `sf_winratio`
3. 锤 5 次，读 `sf_maxratio`
4. **`SF_KS = (sf_maxratio / 100) / 3`** 左右；`SF_KL = SF_KS * 1.3`
5. 若静置期有误报 → 增大 `SF_KS`；若击打漏检 → 减小 `SF_KS`，但**不要低于 5**

本次数据的结果: `sf_maxratio ≈ 1510`（15.1σ）, 静置期最大 `≈ 590`（5.9σ）→ 取 `SF_KS = 6` 正确。

### Ozone 变量

| 变量 | 含义 | 期望值 |
|---|---|---|
| `sf_sp` | 滤波后偏离量 (V) | 静置 ±0.02, 击打 0.1–0.2 |
| `sf_sigma` | 快 σ (V) | 静置 ≈ 0.013 |
| `sf_sigma_lp` | 慢 σ (V) | 静置 ≈ 0.013 |
| `sf_ratio` | \|sp\|/σ ← **调 SF_KS 的根据** | 静置 < 6, 击打 9–15 |
| `sf_maxratio` | σ 倍数历史最大值 ×100 | 本次 1510 |
| `sf_winratio` | 最近 5 s 内最大值 ×100 | 静置 ≈ 400–600 |
| **sf_strike** | **击打次数** | 本次 3 (由实际 C 代码输出) |
| `sf_press` | 按压次数 | 本次 1 |
| `sf_evt_ms` | 当前事件宽度 ms | 击打 20–50, 按压 >1000 |
| `sf_reset` | 写 1 清零 | |

**关于 t=40.71 s 那种幅度塌陷**: `sf_winratio` 会从 ~590 掉到 ~400 —— 但两者重叠，**软件无法自动判定**，只能人工看趋势。真正能区分的是原始域里 `adc_notch_vin` 的峰值偏离（2650 → 697 counts，比值 0.26）。想知道通道还在不在，就在 Ozone 里加一个 `adc_seen_span[0]` / `blk_max_ever` 之类的原始域极值来看。

---

## 六、诚实的限制

1. **4 次事件落在阈值之上**（199 / 119 / 73 mV + 188 mV 的按压），静置期最大噪声 78 mV。第 3 次击打（73 mV）**低于**静置期最大噪声事件（78 mV）—— 它这次能被检出，靠的是自适应 σ 在击打前那 8 s 恰好降到 ~10 mV。**换个噪声稍大的时段就会漏掉。真正可靠的是前 2 次。这不是滤波器的问题，是通道本身的信噪比决定的。**
2. **t = 40.71 s 的幅度塌陷无法用软件恢复。** 塌陷后 19.5 s 内最大偏离只有 697 counts (0.56 V)，是塌陷前最大值的 26%。滤波器不能把它放大回去，只能靠原始域极值看出来。
3. **单样本尖峰（`adc_raw[0]` 上 1–2 采样宽、最高 1.7 V）在静置期和击打期出现频率相同（阈值 1000 counts 时比值 1.01×），不是击打。** 判定必须建立在 500 ms 平均后的"宽脉冲"上，任何按尖峰高度的判据都会失败 —— 静置 23.890 s 处就有一个 1687 mV 的尖峰，比 40.7 s 那次击打还大。
4. 静置期事件率: 峰值 >53 mV 的 5 次 / 25 s（0.20/s），>78 mV 的 0 次。阈值定在 80 mV 时，**实测误报间隔 > 25 s**。要长时间连续运行，把 `SF_KS` 提到 7，代价是漏掉 73 mV 那次。
5. 启动需 7.1 s 稳定（4.1 s 填基线环 + 3 s σ 收敛）。这期间 `sf_strike` 保持 0 但不检测。
