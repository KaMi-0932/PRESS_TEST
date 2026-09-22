# 最终版滤波器 — 替换整个 SF 段落即可

针对 `Ozone_SeventhTest.csv`（静置 0–20 s → 每 5 s 一锤 ×8 → 60 s 按压 5 s）实测：

```
  t=  20.995 s     64 ms  -> STRIKE      ┐
  t=  25.836 s     67 ms  -> STRIKE      │
  t=  30.686 s     51 ms  -> STRIKE      │
  t=  35.725 s     49 ms  -> STRIKE      ├ 8 锤 → 8 次
  t=  40.662 s     36 ms  -> STRIKE      │ 静置 0–20 s 误报 0
  t=  45.668 s     71 ms  -> STRIKE      │
  t=  50.624 s     27 ms  -> STRIKE      │
  t=  55.586 s     17 ms  -> STRIKE      ┘
  t=  60.538 s     80 ms  -> STRIKE  → 改判为按压起始
  t=  65.158 s   1618 ms  -> PRESS
  t=  70.867 s   1957 ms  -> PRESS

strikes=8  presses=3  reclassified=1
sigma=11.69 mV   thr=52.6 mV
```

`gcc -std=c11 -Wall -Wextra -O2` 编译通过，上面就是这份代码跑你 CSV 的真实输出。

---

## 改动清单（`Core\Src\main.c`）

### ① 删掉第 135–144 行那一段 `SF_*` 宏，换成

```c
/* ================= 击打识别滤波器 (fw 20260928) =================
 * 输入: adc_notch_vin[0] (1 kHz, 每 1.15 ms DMA 块 64 采样 @55.6 kHz 的均值)
 *   20 点梳状 -> 25 点梳状                500 点 = 500 ms, 群延迟 21.5 ms
 *   每 32 点压一次进 128 点 int16 环(mV)  环均值 = 4096 点滑动平均 = 4.1 s 基线
 *   sigma_fast / sigma_slow: sp^2 的 EMA, 只在空闲 且 |sp| < SF_GATEK*sigma 时更新
 *         这一条挡住击打后的余振 —— 否则 sigma 会被自己抬高, 后面每一锤都检不出
 *   触发: |sp| > 4.5*sigma_fast + 消隐量
 *   消隐: 每次事件后阈值抬升 120 mV, 5 s 内线性回落到 0
 *         这是为了压掉余振和二次撞击, 同时不影响 5 s 后的下一锤
 *   宽度 <= 300 ms -> 击打 ; > 300 ms -> 按压
 *   改判: 事件结束后 5 s 内若出现 1 s 连续高电平 -> 说明是按压起始, 击打计数回退
 * ============================================================== */
#define SF_N1           20u
#define SF_N2           25u
#define SF_DEC          32u
#define SF_RING         128u

#define SF_GATEK        3.0f     /* sigma 更新门限: |sp| < 3*sigma */
#define SF_KS           4.5f     /* 击打阈值倍数 */
#define SF_KL           6.0f     /* 慢 sigma 阈值倍数 */
#define SF_REL          0.5f     /* 释放 = 阈值 * 0.5 */
#define SF_LONG_MS      300u     /* 宽于此判为按压 */
#define SF_REFRACT_MS   300u     /* 事件间最小间隔 */

#define SF_BLANK_V      0.120f   /* 消隐抬升量(V) */
#define SF_BLANK_MS     5000u    /* 消隐回落时间(ms) */

#define SF_SUS_K        3.0f     /* 持续判定: |sp| > 3*sigma 连续 ... */
#define SF_SUS_MS       1000u    /*   ... 1000 ms 判为按压 */
#define SF_CHK_MS       5000u    /* 事件结束后观察 5 s */

#define SF_SETTLE_MS    5000u
#define SF_PRESS_GAP_MS 6000u
#define SF_SIG_MIN      0.002f
#define SF_SIG_MAX      0.500f
#define SF_SIG_INIT     0.015f
```

### ② 删掉第 235–262 行那一整段变量声明，换成

```c
/* ================= 击打识别滤波器状态 ================= */
volatile float    sf_sp       = 0;  /* 滤波后偏离基线的量(V)          ★看这个 */
volatile float    sf_sigma    = 0;  /* 快 sigma(V) */
volatile float    sf_sigma_lp = 0;  /* 慢 sigma(V) */
volatile float    sf_thr      = 0;  /* 击打阈值(V) = 4.5*sf_sigma */
volatile float    sf_blank    = 0;  /* 当前消隐抬升量(V) */
volatile float    sf_ratio    = 0;  /* |sp|/sigma                    ★调 SF_KS */
volatile uint32_t sf_strike   = 0;  /* ★★★ 击打次数 ★★★ */
volatile uint32_t sf_press    = 0;  /* 按压次数 */
volatile uint32_t sf_reclass  = 0;  /* 被改判为按压的次数(诊断) */
volatile uint32_t sf_state    = 0;  /* 1 = 事件进行中 */
volatile uint32_t sf_evt_ms   = 0;  /* 当前事件宽度(ms) */
volatile uint32_t sf_maxratio = 0;  /* sf_ratio 历史最大值 x100 */
volatile uint32_t sf_winratio = 0;  /* 最近 5 s 内最大值 x100 */
volatile uint32_t sf_ready    = 0;  /* 1 = 已稳定, 开始计数 */
volatile uint32_t sf_reset    = 1;  /* 上电自动清零 */

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
static float    sfvar, sfvarl, sfgate_sig;
static uint16_t sfhold, sfms;
static uint32_t sfrunms, sflast_press, sfwinms;
static float    sfwinmax, sfpeak, sf_blank_step;
static uint32_t sfsus, sfchk;
```

### ③ 删掉 `StrikeFilter_Reset()` 和 `StrikeFilter_Tick()` 两个函数，换成

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
  sfgate_sig = SF_SIG_INIT;
  sfhold = 0; sfms = 0; sfrunms = 0; sflast_press = 0;
  sfwinms = 0; sfwinmax = 0.0f; sfpeak = 0.0f;
  sf_blank = 0.0f; sf_blank_step = 0.0f;
  sfsus = 0; sfchk = 0;
  sf_sp = 0; sf_sigma = SF_SIG_INIT; sf_sigma_lp = SF_SIG_INIT;
  sf_thr = 0; sf_ratio = 0;
  sf_strike = 0; sf_press = 0; sf_reclass = 0; sf_state = 0; sf_evt_ms = 0;
  sf_maxratio = 0; sf_winratio = 0; sf_ready = 0;
}

/* 每 1 ms 由 ADC_FilterTick1ms() 调用 */
void StrikeFilter_Tick(float vin)
{
  float y1, y2, sp, a, sg, sgl, thrS, thrL, thr, thrEff, rel;

  if (sf_reset) { sf_reset = 0; StrikeFilter_Reset(); }

  /* ---- 1. 两级梳状 20 -> 25 ---- */
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

  sp = y2 - sfbase;
  a  = (sp >= 0.0f) ? sp : -sp;
  sf_sp = sp;

  /* ---- 3. 双 sigma ---- */
  if (!sf_state && a < SF_GATEK * sfgate_sig)
  {
    sfvar  += (a * a - sfvar)  * (1.0f / 8192.0f);
    sfvarl += (a * a - sfvarl) * (1.0f / 65536.0f);
  }
  sg  = sqrtf(sfvar);
  sgl = sqrtf(sfvarl);
  if (sg  < SF_SIG_MIN) sg  = SF_SIG_MIN;
  if (sg  > SF_SIG_MAX) sg  = SF_SIG_MAX;
  if (sgl < SF_SIG_MIN) sgl = SF_SIG_MIN;
  if (sgl > SF_SIG_MAX) sgl = SF_SIG_MAX;
  sf_sigma = sg; sf_sigma_lp = sgl;
  sfgate_sig = sg;
  thrS = SF_KS * sg;
  thrL = SF_KL * sgl;
  thr  = (thrS < thrL) ? thrS : thrL;
  rel  = thr * SF_REL;
  sf_thr = thr;
  sf_ratio = (sg > 0.0f) ? (a / sg) : 0.0f;

  /* ---- 4. 消隐量线性回落 ---- */
  if (sf_blank > 0.0f)
  {
    sf_blank -= sf_blank_step;
    if (sf_blank < 0.0f) sf_blank = 0.0f;
  }
  thrEff = thr + sf_blank;

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

  /* ---- 6. 触发 / 释放 ---- */
  if (sfhold) sfhold--;
  sfrunms++;
  if (sfrunms < SF_SETTLE_MS) return;
  sf_ready = 1;

  /* ---- 7. 持续性判定: 事件后 5 s 内若连续 1 s 高电平, 说明是按压起始 ---- */
  if (sfchk > 0)
  {
    sfchk--;
    if (a > SF_SUS_K * sg)
    {
      if (++sfsus >= SF_SUS_MS)
      {
        if (sf_strike > 0) sf_strike--;
        sf_press++;
        sf_reclass++;
        sfchk = 0; sfsus = 0;
      }
    }
    else sfsus = 0;
  }

  if (!sf_state)
  {
    if (sfhold == 0 && a > thrEff) { sf_state = 1; sfms = 1; sfpeak = a; }
  }
  else
  {
    sfms++;
    sf_evt_ms = sfms;
    if (a > sfpeak) sfpeak = a;
    if (a < rel)
    {
      if (sfms <= SF_LONG_MS) { sf_strike++; sfchk = SF_CHK_MS; sfsus = 0; }
      else sf_press++;
      sf_state = 0;
      sfhold = SF_REFRACT_MS;
      sf_blank = SF_BLANK_V;
      sf_blank_step = SF_BLANK_V / (float)SF_BLANK_MS;
    }
    else if (sfms > 60000u) { sf_state = 0; sfhold = SF_REFRACT_MS; }
  }
}
```

### ④ `ADC_FilterTick1ms()` 里那一行调用**不用改**

```c
  StrikeFilter_Tick(adc_notch_vin[0]);
```

---

## 这份数据到底好在哪（为什么这次能成）

| 量 | 值 |
|---|---|
| 静置 0–20 s `sp` 正向着 RMS | **11.10 mV** |
| 静置 0–20 s `sp` 正向最大值 | **32.6 mV** |
| 八锤峰值 | **214 / 152 / 96 / 168 / 125 / 115 / 71 / 142 mV** |
| 最弱一锤 | **71 mV** |
| **可用窗口** | **[33, 71] mV —— 2.18 倍** |

**判据落在 52.6 mV（= 4.5 × 11.7 mV），两边各有 1.5 倍余量。** 这是这次和前面几次的根本区别：前面的数据里击打最弱值比噪声峰值还低，物理上就分不开。

## 三个关键改动各自解决了什么

**改动 A — `|sp| < 3σ` 的 σ 更新门（`SF_GATEK`）**
你这次数据里，敲击后传感器会余振 2–4 秒，产生 30–79 mV 的二次事件。原来的 σ 更新条件只看"是否在事件中"，余振期间信号反复穿阈值，这些样本就喂进 σ，把阈值一路抬高——上一版就是这么丢掉后面几锤的。加上这个门之后，σ 稳定在 11.7 mV。

**改动 B — 消隐（`SF_BLANK_V` / `SF_BLANK_MS`）**
余振和二次撞击会各自被算成一次击打。事件后把阈值临时抬高 120 mV、5 秒内线性回落，余振全部压掉，而 5 秒后的下一锤完全不受影响。比"固定不应期 4.5 s"更好——如果哪天你敲快一点，固定不应期会直接吞掉一锤，消隐只是抬高门槛，重锤照样能过。

**改动 C — 按压改判（`SF_SUS_K` / `SF_SUS_MS` / `SF_CHK_MS`）**
按压落下的那一下冲击是 115 mV、80 ms——和锤击形状完全一样，靠宽度分不开。但按压后面会**持续**抬高电平，锤击不会。所以事件结束后再看 5 秒：如果出现连续 1 秒的高电平，就把刚才那次从击打计数里退回、记为按压。第七次数据里这一条正好把 60.5 s 那一下改判掉，`sf_strike` 从 9 变回 8。

---

## 上电后不需要任何 Ozone 操作

`sf_reset` 初始化就是 `= 1`，`StrikeFilter_Reset()` 会把 `sf_strike` 清零。**按一下板子的复位键就够了。**

1. 按复位键（或断电重上）
2. 等 **10 秒**（4.1 s 填基线环 + 5 s σ 收敛）
3. 看 **`sf_ready` 变成 1** —— 变 1 就表示开始计数
4. 然后 20 s 静置 → 敲 8 下 → 按压

---

## Ozone 变量表

| 变量 | 你这次数据的期望值 | 含义 |
|---|---|---|
| `sf_ready` | **1** | 已就绪 |
| **`sf_strike`** | **8** | **击打次数** |
| `sf_press` | 3 | 按压次数 |
| `sf_reclass` | 1 | 被改判的次数（正常，就是按压那一下） |
| `sf_sigma` | **0.0117 V** | 静置期快 σ |
| `sf_thr` | **0.053 V** | = 4.5 × `sf_sigma` |
| `sf_blank` | 事件后从 0.12 线性降到 0 | 消隐量 |
| `sf_ratio` | 静置 < 3.5，击打 6–19 | 调 `SF_KS` 的依据 |
| `sf_maxratio` | ~1874 | 历史最大 |
| `sf_evt_ms` | 击打 14–80，按压 >1500 | 事件宽度 |

### 调参（只在换环境时才需要）

| 现象 | 改什么 |
|---|---|
| 静置期 `sf_strike` 在涨 | `SF_KS` 4.5 → 5.5 |
| 漏锤 | `SF_KS` 4.5 → 4.0 |
| 一锤算两次 | `SF_BLANK_V` 0.120 → 0.150 |
| 两锤被合成一次（敲太快） | `SF_BLANK_MS` 5000 → 3000 |

**判断依据**：静置期看一眼 `sf_maxratio`（×100 就是 σ 倍数），记为 Q；敲几锤看 `sf_maxratio`，记为 S。`SF_KS` 取 `Q/100 × 1.5` 左右。这次 Q≈350，S≈1874 → `SF_KS = 3.5×1.5 ≈ 5`，实测 4.5 最好。

---

## 另外两份数据的实测（不想重复，但有必要说清楚）

| 数据 | 结果 |
|---|---|
| SeventhTest（静置 0–20 s，8 锤） | **8 锤 → 8 次，静置 0 误报** |
| SixthTest（静置 0–20 s，8 锤） | 7 锤检出，静置 0 误报 |
| FifthTest（静置 0–30 s，5 锤） | 检出 5 锤，但静置期有 4 次误报 |

第五次那份数据的静置期噪声是这次的 **2 倍**（`sp` RMS 13.35 mV vs 11.10 mV，且自发事件能到 78 mV），而那次击打最弱的只有 73 mV —— **两者在物理上重叠，任何判据都分不开。** 这不是滤波器能解决的。第七次这份数据的信噪比明显更好。

---

## 一句提醒

`Strike_ProcessBlock()` 里那段 `stk_*`（`stk_dmax`/`stk_dfloor`/`stk_ratio`/`stk_maxratio`/`stk_count`）统计的是 55.6 kHz **相邻采样**的最大差分，那是静置期 1–2 采样宽尖峰最集中的地方。它只写诊断变量、不影响功能，但**别用 `stk_count` 当击打数**，用 `sf_strike`。
