# 改动清单 — 实现击打识别

文件: `D:\Documents\Pressure Test\Core\Src\main.c`
共 **4 处插入 + 1 处可选**。不动任何现有逻辑，纯新增。

---

## 改动 1 — 在第 114 行 `#define STK_REFRACT_MS  10u` 之后、第 115 行 `/* USER CODE END PD */` 之前

插入：

```c
/* ============ 击打/按压检测滤波器 (fw 20260927) ============
 * 输入: adc_notch_vin[0] (1 kHz, 每 1.15 ms DMA 块 64 采样 @55.6 kHz 的均值)
 *   两级梳状 20 -> 25      合计 500 点 = 500 ms, 群延迟 21.5 ms, sigma 116 mV -> 13.4 mV
 *   4.1 s 基线             每 32 点压一次进 128 点 int16 环(mV), 环均值 = 4096 点滑动平均
 *   双 sigma (sp^2 的 EMA, 只在空闲期更新):
 *      快 tau = 2^12 = 4.1 s  -> 击打
 *      慢 tau = 2^16 = 65 s   -> 按压(慢, 不会被按压自己的上升沿抬走)
 *   触发   |sp| > 6*sigma_fast  或  |sp| > 8*sigma_slow
 *   释放   回到 min(两个阈值)*0.5     不应期 20 ms
 *   宽度   <= 300 ms -> 击打 ; > 300 ms -> 按压
 * 在 Ozone_FifthTest.csv (静置30s -> 每5s击打 -> 按压5s) 上实测:
 *   35.882s/41ms, 40.744s/28ms, 50.717s/23ms 检出为击打; 静置 30 s 内 0 次误报
 * ========================================================== */
#define SF_N1           20u
#define SF_N2           25u
#define SF_DEC          32u
#define SF_RING         128u
#define SF_KS           6.0f
#define SF_KL           8.0f
#define SF_REL          0.5f
#define SF_LONG_MS      300u
#define SF_HOLD_MS      20u
#define SF_SETTLE_MS    3000u
#define SF_PRESS_GAP_MS 6000u
#define SF_SIG_MIN      0.002f
#define SF_SIG_MAX      0.500f
#define SF_SIG_INIT     0.015f
```

---

## 改动 2 — 在第 203 行 `static uint32_t stk_last_ms;` 之后、第 204 行 `/* USER CODE END PV */` 之前

插入：

```c
/* ---- 击打检测滤波器状态 (fw 20260927) ---- */
volatile float    sf_sp       = 0;  /* 滤波后偏离基线的量(V)      ★看这个 */
volatile float    sf_sigma    = 0;  /* 快 sigma(V) */
volatile float    sf_sigma_lp = 0;  /* 慢 sigma(V) */
volatile float    sf_thr      = 0;  /* 击打阈值(V) = 6*sf_sigma */
volatile float    sf_ratio    = 0;  /* |sp|/sigma                 ★调 SF_KS 看它 */
volatile uint32_t sf_strike   = 0;  /* ★★★ 击打次数 ★★★ */
volatile uint32_t sf_press    = 0;  /* 按压次数 */
volatile uint32_t sf_state    = 0;  /* 1 = 事件进行中 */
volatile uint32_t sf_evt_ms   = 0;  /* 当前事件宽度(ms) */
volatile uint32_t sf_maxratio = 0;  /* sf_ratio 历史最大值 x100 */
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

---

## 改动 3 — 在第 213 行 `/* USER CODE BEGIN 0 */` 之后（第 214 行的注释块之前）

插入：

```c
static void StrikeFilter_Reset(void)
{
  uint32_t k;
  for (k = 0; k < SF_N1; k++)   sfb1[k] = 0.0f;
  for (k = 0; k < SF_N2; k++)   sfb2[k] = 0.0f;
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

/* 每 1 ms 调用一次 (由 ADC_FilterTick1ms 调用) */
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

  /* ---- 2. 4.1 s 基线 ---- */
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

  /* ---- 5. 诊断量 ---- */
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

---

## 改动 4 — `ADC_FilterTick1ms()`，在第 227 行 `adc_vin[0] = adc_raw[0] * (3.3f / 4095.0f);` 之后

第 219–229 行改完是这样：

```c
void ADC_FilterTick1ms(void)
{
  if (!adc_filter_ready) return;
  /* 单通道模式:
   *   adc_raw[0]       = 最近 1ms 块内【偏离均值最大】的原始采样(峰值保持)
   *   adc_vin[0]       = 上面那个值的电压
   *   adc_notch_vin[0] = 最近 1ms 块的【均值电压】(由 Strike_ProcessBlock 写入)
   * 不再跑陷波、也不做贴轨监视。 */
  adc_vin[0] = adc_raw[0] * (3.3f / 4095.0f);

  StrikeFilter_Tick(adc_notch_vin[0]);   /* <<<<<< 新增这一行 */

  notch_sample_count++;
}
```

**就这 4 处。** 编译即可。

---

## 改动 5（可选）— 把结果接到 `hit_count`

如果你想让原来的 `hit_count` 也反映真实击打，在 `sf_strike` 自增处加一句：

```c
      if (sfms <= SF_LONG_MS) { sf_strike++; hit_count++; }
```

（`hit_count` 在 main.c 第 140 行已声明，第 732 行另有自增；若同时保留会重复计数，二选一。）

---

## 编译后怎么用

1. 烧录 → 复位 → **等 10 秒**（4.1 s 填基线环 + 3 s σ 收敛，共 7.1 s）
2. Ozone 里看 `sf_sigma`，静置应为 **0.010 ~ 0.014 V**
3. 在 Ozone 写 `sf_reset = 1` 清零
4. 静置 30 s → 看 `sf_strike` **必须是 0**
5. 锤 5 次 → 看 `sf_strike` 增加几次

### 关键变量

| 变量 | 含义 | 你这次数据的期望值 |
|---|---|---|
| **`sf_strike`** | **击打次数** | 你锤几次就加几次 |
| `sf_press` | 按压次数 | |
| `sf_sp` | 滤波后偏离量 (V) | 静置 ±0.02，击打 0.1~0.2 |
| `sf_sigma` | 快 σ (V) | 静置 0.013 |
| `sf_ratio` | \|sp\|/σ | 静置 <6，击打 9~15 |
| `sf_maxratio` | 历史最大值 ×100 | 本次 1510 |
| `sf_winratio` | 最近 5 s 最大值 ×100 | 静置 400~600 |
| `sf_evt_ms` | 当前事件宽度 ms | 击打 20~50，按压 >1000 |
| `sf_reset` | 写 1 清零 | |

### 如果静置期 `sf_strike` 还在涨

把 `SF_KS` 从 `6.0f` 调到 `7.0f` 或 `8.0f`（`sf_ratio` 静置期的最大值 ÷ 3 就是合适的 `SF_KS`）。

### 如果击打漏检

把 `SF_KS` 降到 `5.0f`。

---

## 建议同时处理（与击打识别无关，但会干扰判断）

`Strike_ProcessBlock()` 里第 273–315 行那段 `stk_*` 旧判据（`stk_dfloor` / `stk_dmax` / `stk_ratio` / `stk_count`）用的是 **55.6 kHz 相邻采样的最大差分**。那正是静置期 1–2 采样宽尖峰最集中的地方 —— 这就是"未击打时击打数也在增加"的原因。

它只写诊断变量，不影响别的功能，删掉或留着都行，但**不要拿 `stk_count` 当击打数**，用 `sf_strike`。

---

## 实测输出（这段 C 代码用 gcc 编译后直接跑 `Ozone_FifthTest.csv`）

```
event at t=35.882 s  width=41 ms   -> STRIKE
event at t=40.744 s  width=28 ms   -> STRIKE
event at t=50.717 s  width=23 ms   -> STRIKE
event at t=62.309 s  width=1772 ms -> PRESS
strikes=3  presses=1   maxratio=1510
```

静置的 30 秒内 **0 次触发**。按压 1772 ms 与击打 23–41 ms 相差 50 倍，靠宽度自动分开。
