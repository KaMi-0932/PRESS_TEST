# v1 → v2 改动 (只改 8 处)

你的固件已经跑上了 v1（第三列能导出 `sf_ratio` 就是证据）。下面是**从 v1 到 v2 的全部差异**。

实测（第六次数据，静置 0–20 s → 每 5 s 一锤 ×8 → 60 s 按压 5 s）：

```
  t=  20.784 s     35 ms  -> STRIKE      ┐
  t=  25.536 s     14 ms  -> STRIKE      │
  t=  30.637 s     26 ms  -> STRIKE      │
  t=  35.786 s     91 ms  -> STRIKE      ├ 8 锤全中
  t=  40.663 s    139 ms  -> STRIKE      │ 间隔 4.75/5.10/5.15/4.88/4.85/5.08/5.25 s
  t=  45.516 s     30 ms  -> STRIKE      │
  t=  50.593 s     34 ms  -> STRIKE      │
  t=  55.842 s     33 ms  -> STRIKE      ┘
  t=  59.631 s    857 ms  -> PRESS
  t=  63.852 s   2721 ms  -> PRESS
  t=  68.810 s   3283 ms  -> PRESS
n=71309  strikes=8 presses=2
静置 0–20 s 误报: 0
```

**v1 在同一份数据上只有 2/8**（20.78 s 报了 7 次、55.84 s 报了 1 次，中间 6 锤全漏）。

---

## 改动 1 — 宏定义（第 114 行之后那段）

```diff
-#define SF_KS           6.0f
-#define SF_KL           8.0f
+#define SF_KS           10.0f
+#define SF_KL           13.0f
+#define SF_GATEK        2.0f     /* ★ 新增: sigma 只在 |sp| < 2*sigma 时更新 */
 #define SF_REL          0.5f
 #define SF_LONG_MS      300u
-#define SF_HOLD_MS      20u
+#define SF_REFRACT_MS   1500u    /* ★ 20 -> 1500: 一次击打会被机械振铃拖 0.8 s */
 #define SF_SETTLE_MS    3000u
 #define SF_PRESS_GAP_MS 6000u
 #define SF_SIG_MIN      0.002f
 #define SF_SIG_MAX      0.500f
 #define SF_SIG_INIT     0.015f
```

---

## 改动 2 — 状态变量（第 203 行之后那段）

在 `static float sfwinmax;` 之后加一行：

```c
static float    sfgate_sig;      /* 上一拍的 sigma, 作为 sigma 门限的比较基准 */
```

---

## 改动 3 — `StrikeFilter_Reset()` 里加一行

在 `sfvar  = SF_SIG_INIT * SF_SIG_INIT;` 那一行**之后**加：

```c
  sfgate_sig = SF_SIG_INIT;
```

---

## 改动 4 — **核心**：`StrikeFilter_Tick()` 里的 σ 更新条件

```diff
-  if (!sf_state)
-  {
-    sfvar  += (a * a - sfvar)  * (1.0f / 4096.0f);
-    sfvarl += (a * a - sfvarl) * (1.0f / 65536.0f);
-  }
+  if (!sf_state && a < SF_GATEK * sfgate_sig)
+  {
+    sfvar  += (a * a - sfvar)  * (1.0f / 4096.0f);
+    sfvarl += (a * a - sfvarl) * (1.0f / 65536.0f);
+  }
```

紧跟着的 σ 限幅、赋值那几行，**在 `sf_sigma = sg; sf_sigma_lp = sgl;` 之后加一行**：

```c
  sfgate_sig = sg;
```

---

## 改动 5 — 触发/释放那段

```diff
   sf_sigma = sg; sf_sigma_lp = sgl;
+  sfgate_sig = sg;
   thrS = SF_KS * sg;
   thrL = SF_KL * sgl;
-  sf_thr = thrS;
+  thr  = (thrS < thrL) ? thrS : thrL;
+  rel  = thr * SF_REL;
+  sf_thr = thr;
   sf_ratio = (sg > 0.0f) ? (a / sg) : 0.0f;
```

然后在 `if (!sf_state) { if (sfhold == 0 && (a > thrS || a > thrL)) {...} }` 这一句改成用 `thr`：

```diff
-    if (sfhold == 0 && (a > thrS || a > thrL)) { sf_state = 1; sfms = 1; }
+    if (sfhold == 0 && a > thr) { sf_state = 1; sfms = 1; }
```

`else` 分支里：

```diff
     sfms++;
     sf_evt_ms = sfms;
-    rel = ((thrS < thrL) ? thrS : thrL) * SF_REL;
     if (a < rel)
     {
       ...
-      sf_state = 0; sfhold = SF_HOLD_MS;
+      sf_state = 0; sfhold = SF_REFRACT_MS;
     }
-    else if (sfms > 60000u) { sf_state = 0; sfhold = SF_HOLD_MS; }
+    else if (sfms > 60000u) { sf_state = 0; sfhold = SF_REFRACT_MS; }
```

（`float ... thr, rel;` 要加进 `StrikeFilter_Tick` 的局部变量声明里。）

---

## 为什么是这一处改动

你的第六次数据里，静置期（0–20 s）的噪声是：

| 量 | 值 |
|---|---|
| `sp` 的 RMS | **3.30 mV** |
| `sp` 的峰值（20 s 内最大） | **17.1 mV** |
| 最弱的一锤（25 s） | **42 mV** |
| 最强的一锤（20 s） | **135 mV** |

**17.1 mV 和 42 mV 之间有一条 2.5 倍的通道 —— 判据本来完全分得开。**

但你的 v1 里 σ 从 3.3 mV 一路涨到 **41.8 mV**（阈值跟着涨到 251 mV），所以后面 6 锤全被埋掉了。

原因是**击打后的机械振铃**：看 `sf_ratio` 在 20.74–21.54 s 的走向

```
20.74 ####...  29.5
20.76 ##########...  36.3     <- 主峰
20.78 #################  11.2
20.80 ##########  6.9
20.82 ####  2.7
20.88 ######  4.0
20.90 ###########  7.4
20.92 #############  9.0
21.00 ###############  9.7     <- 振铃第 2 波
21.16 ###########  7.3
21.26 ############  7.9         <- 第 3 波
21.40 ##########  6.4           <- 第 4 波
21.50 ###########  7.4          <- 第 5 波
```

**一次击打让传感器以约 8 Hz 振铃 0.8 s。** v1 的 σ 更新条件是「不在事件中」，而振铃期间信号反复穿过阈值，`sf_state` 频繁回到 0，于是这些大的振铃样本全被算进 σ。5 s 一锤、σ 时间常数 4.1 s，σ 来不及回落，到第 6 锤就涨到 24 mV 以上。

`a < 2σ` 这个门把振铃完全挡在外面（振铃幅度是 6–36σ，永远大于 2σ），σ 就稳在 3.3–3.5 mV。阈值 35 mV，正好落在 17.1 和 42 mV 中间。

**`SF_REFRACT_MS` 1500 ms 是为了让一锤只算一次**（振铃 0.8 s，20 ms 的不应期会把一次击打算成 7 次）。

---

## 验证结果

| 数据 | v1（你现在的） | v2 |
|---|---|---|
| 第六次：静置 0–20 s | 0 误报 | **0 误报** |
| 第六次：8 锤 | **2/8** | **8/8** |
| 第六次：按压 60 s | 检出 | 检出（宽度分类） |
| 第五次：静置 0–30 s | 0 误报 | 2 次误报 |
| 第五次：30–60 s 击打 | 3 次事件 | 6 次事件 |

**第五次那份数据噪声是第六次的 4 倍**（`sp` RMS 13.35 mV vs 3.30 mV），它静置期的自发事件能到 78 mV，而那几次击打最弱的只有 73 mV —— **那份数据里两者在物理上就是重叠的，任何阈值都分不开。** v2 在那种环境下会漏报也会误报，这不是滤波器的问题。

---

## 上电验证步骤

1. 烧录 → 复位 → 等 10 s
2. 写 `sf_reset = 1`
3. **静置 20 s，`sf_strike` 必须是 0**（此时 `sf_sigma` 应在 0.003–0.004 V）
4. 每 5 s 敲一下，敲 8 下，`sf_strike` 应该是 8
5. 按压 5 s，`sf_press` 应该 +1，`sf_strike` 不变

### 调参

| 变量 | 期望（第六次数据） | 说明 |
|---|---|---|
| `sf_sigma` | **0.0035 V** | 静置期；若明显更大说明环境噪声高 |
| `sf_sigma_lp` | ~0.012 V | 慢 σ，用于按压判据 |
| `sf_thr` | **0.035 V** | = 10 × `sf_sigma` |
| `sf_ratio` | 静置 <3，击打 12–40 | |
| `sf_maxratio` | ~4850 | 调 `SF_KS` 的依据 |
| **`sf_strike`** | **8** | **击打次数** |
| `sf_press` | 1–2 | |

- **静置期 `sf_strike` 有增长** → `SF_KS` 从 10 调到 12 或 14
- **漏锤** → `SF_KS` 降到 8，或 `SF_REFRACT_MS` 降到 1000
- **一锤算两次** → `SF_REFRACT_MS` 提到 2000

---

## 一处与击打识别无关但最好顺手删掉的东西

`Strike_ProcessBlock()` 第 273–315 行那段 `stk_*`（`stk_dmax` / `stk_dfloor` / `stk_maxratio` / `stk_count`）统计的是 55.6 kHz **相邻采样**的最大差分。那正是静置期 1–2 采样宽尖峰最集中的地方 —— 这就是"未击打时击打数也在增加"的来源。它只写诊断变量，留着不影响功能，但**别拿 `stk_count` 当击打数**。
