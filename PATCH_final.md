# 最终改动 — 9 处，全部在 `Core\Src\main.c`

**不需要写 `sf_reset = 1`。** 见文末第五节说明。

改完之后实测（第六次数据，静置 0–20 s → 每 5 s 一锤 ×8 → 60 s 按压 5 s）：

```
  t=  20.784 s     35 ms  -> STRIKE      ┐
  t=  25.536 s     14 ms  -> STRIKE      │
  t=  30.637 s     26 ms  -> STRIKE      │
  t=  35.786 s     91 ms  -> STRIKE      ├ 8 锤全中，一锤一次
  t=  40.663 s    139 ms  -> STRIKE      │
  t=  45.516 s     30 ms  -> STRIKE      │
  t=  50.593 s     34 ms  -> STRIKE      │
  t=  55.842 s     33 ms  -> STRIKE      ┘
  t=  59.631 s    857 ms  -> PRESS
  t=  63.852 s   2721 ms  -> PRESS
  t=  68.810 s   3283 ms  -> PRESS
n=71309  strikes=8  presses=2
静置 0-20 s 误报: 0
```

比对你锤的时刻（20/25/30/35/40/45/50/55）：**检出 20.784 / 25.536 / 30.637 / 35.786 / 40.663 / 45.516 / 50.593 / 55.842，一一对应，不多不少。**

这段代码已用 `gcc -std=c11 -Wall -Wextra -O2` 编译通过并跑出的上面结果。

---

## 编辑 1 — 第 135–139 行

```diff
-#define SF_KS           6.0f
-#define SF_KL           8.0f
+#define SF_KS           10.0f
+#define SF_KL           13.0f
+#define SF_GATEK        2.0f     /* ★新增: sigma 只在 |sp| < 2*sigma 时更新 */
 #define SF_REL          0.5f
 #define SF_LONG_MS      300u
-#define SF_HOLD_MS      20u
+#define SF_REFRACT_MS   1500u    /* ★ 20 -> 1500: 一次击打会机械振铃 0.9 s */
 #define SF_SETTLE_MS    3000u
 #define SF_PRESS_GAP_MS 6000u
```

（`SF_HOLD_MS` 这个名字后面两处用到，一并改名。）

---

## 编辑 2 — 第 247 行之后，加一行

```c
volatile uint32_t sf_ready    = 0;  /* 1 = 滤波器已稳定, 开始计数 */
```

---

## 编辑 3 — 第 262 行 `static float sfwinmax;` 之后，加一行

```c
static float    sfgate_sig;         /* 上一拍的 sigma, 作为 sigma 门限的比较基准 */
```

---

## 编辑 4 — 第 285 行 `sfvarl = SF_SIG_INIT * SF_SIG_INIT;` 之后，加一行

```c
  sfgate_sig = SF_SIG_INIT;
```

---

## 编辑 5 — 第 291 行，改成

```diff
-  sf_maxratio = 0; sf_winratio = 0;
+  sf_maxratio = 0; sf_winratio = 0; sf_ready = 0;
```

---

## 编辑 6（核心）— 第 338 行

```diff
   /* ---- 4. 双 sigma: 只在空闲期更新 ---- */
-  if (!sf_state)
+  if (!sf_state && a < SF_GATEK * sfgate_sig)
   {
     sfvar  += (a * a - sfvar)  * (1.0f / 4096.0f);
     sfvarl += (a * a - sfvarl) * (1.0f / 65536.0f);
   }
```

---

## 编辑 7 — 第 349 行，改成两行

```diff
-  sf_sigma = sg; sf_sigma_lp = sgl;
+  sf_sigma = sg; sf_sigma_lp = sgl;
+  sfgate_sig = sg;
```

---

## 编辑 8 — 第 370 行之后，加一行

```diff
   if (sfrunms < SF_SETTLE_MS) return;   /* 环填满后再等 3 s */
+  sf_ready = 1;                         /* 从这里开始才算数 */
```

---

## 编辑 9 — 第 389 行和第 391 行

```diff
-      sf_state = 0; sfhold = SF_HOLD_MS;
+      sf_state = 0; sfhold = SF_REFRACT_MS;
     }
-    else if (sfms > 60000u) { sf_state = 0; sfhold = SF_HOLD_MS; }
+    else if (sfms > 60000u) { sf_state = 0; sfhold = SF_REFRACT_MS; }
```

---

# 为什么是这两处

## 编辑 6（σ 门限）— 解决"后 6 锤全漏"

你第六次数据静置期（0–20 s）测出来的：

| 量 | 值 |
|---|---|
| `sp` 的 RMS | **3.30 mV** |
| `sp` 峰值（20 s 内最大） | **17.1 mV** |
| 最弱一锤（25 s） | **42 mV** |
| 最强一锤（20 s） | **135 mV** |

**17.1 mV 和 42 mV 之间有一条 2.5 倍的空档，判据本来就够用。**

但你现在固件里的 σ 从 3.3 mV 一路涨到 **41.8 mV**（阈值跟着涨到 251 mV），后 6 锤全被埋掉。原因看你导出的 `sf_ratio` 在 20.74–21.54 s：

```
20.76 ############...  36.3   <- 主峰
20.82 ####              2.7
20.92 #############     9.0   <- 振铃第 2 波
21.00 ###############   9.7
21.26 ############      7.9   <- 第 3 波
21.40 ##########        6.4   <- 第 4 波
21.50 ###########       7.4   <- 第 5 波
```

**一锤让传感器以约 8 Hz 振铃 0.9 秒。** 现在的 σ 更新条件是"不在事件中"，而振铃期间信号反复穿过阈值、`sf_state` 频繁回到 0，这些 6–36σ 的振铃样本就全喂进了 σ。5 s 一锤、σ 时间常数 4.1 s，根本来不及回落。

`a < 2σ` 这个门把振铃完全挡在外面（振铃幅度是 6–36σ，永远大于 2σ）→ σ 稳在 3.5 mV → 阈值 35 mV，正好落在 17.1 和 42 mV 中间。

## 编辑 1 里的 `SF_REFRACT_MS 1500`

振铃 0.9 s，20 ms 的不应期会把**一锤算成 7 次**。1500 ms 让它只算一次。

`SF_KS` 从 6 提到 10（= 35 mV）也是必须的：σ 不再被抬高后，6σ 只有 21 mV，虽然也在空档里，但离静置期最大值 17.1 mV 只有 1.23 倍余量。10σ = 35 mV 更居中。

---

# 上电后不需要任何 Ozone 操作

`sf_reset` 在第 247 行的初始化就是 `= 1`，`StrikeFilter_Reset()` 会把 `sf_strike` 清零。所以：

1. **按一下板子的复位键**（或断电重上）→ `sf_strike` 自动变 0
2. 等 **10 秒**（4.1 s 填基线环 + 3 s σ 收敛）
3. 看 `sf_ready` 是否变成 **1** —— 变 1 就表示开始计数了
4. 然后再开始 20 s 静置 → 敲 8 下 → 按压

**不用在 Ozone 里写任何变量。** `sf_ready` 这个新变量就是给你确认"滤波器已经就绪"用的。

（如果你想在**不**复位的情况下重新开始计数，那还是得写 `sf_reset = 1`。但正常测试流程用复位键就够了。）

---

# Ozone 里该看什么

| 变量 | 期望值 | 含义 |
|---|---|---|
| **`sf_ready`** | **1** | 已稳定，开始计数 |
| **`sf_strike`** | 你敲几下就是几 | **击打次数** |
| `sf_press` | 1–2 | 按压次数 |
| `sf_sigma` | **0.0035 V** | 静置期快 σ |
| `sf_sigma_lp` | ~0.012 V | 慢 σ |
| `sf_thr` | **0.035 V** | = 10 × `sf_sigma` |
| `sf_ratio` | 静置 <3，击打 12–40 | 调 `SF_KS` 的依据 |
| `sf_maxratio` | ~4850 | |
| `sf_evt_ms` | 击打 14–140，按压 >800 | |

### 万一还有偏差

| 现象 | 改什么 |
|---|---|
| 静置期 `sf_strike` 在涨 | `SF_KS` 10 → 12 |
| 漏锤 | `SF_KS` 10 → 8 |
| 一锤算两次 | `SF_REFRACT_MS` 1500 → 2000 |
| 两次快敲被合成一次 | `SF_REFRACT_MS` 1500 → 1000 |

**唯一前提**：`SF_KS` 只有在 `sf_sigma` 稳定在 0.003–0.004 V 时才有意义。如果静置期 `sf_sigma` 明显更大（比如超过 0.008 V），说明测试环境噪声高，那要按 `sf_maxratio` 重新定 `SF_KS`。

---

# 顺带一提（与击打计数无关）

`Strike_ProcessBlock()` 里那段 `stk_*`（`stk_dmax`/`stk_dfloor`/`stk_ratio`/`stk_maxratio`/`stk_count`）统计的是 55.6 kHz **相邻采样**的最大差分，那正是静置期 1–2 采样宽尖峰最集中的地方。它只写诊断变量、不影响功能，但**别用 `stk_count` 当击打数**，用 `sf_strike`。
