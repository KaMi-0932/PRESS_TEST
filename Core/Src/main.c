/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "notch50.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR0_TEST_MODE 1u /* PA3 only; bypass legacy four-channel foreground detection. */
#define ADC_REFERENCE_V 3.3f /* ADC VDDA in volts; replace with measured VDDA if needed. */
#define BOARD_W_MM       200.0f   /* 板宽 W（mm），按实际改 */
#define BOARD_H_MM       150.0f   /* 板高 H（mm），按实际改 */
#define HIT_THRESHOLD    0.3f     /* 击打阈值（V）：按压下降~0.4V，0.3V低于信号、高于空闲噪声 */
#define HIT_WINDOW_MS    30       /* 击打后抓峰值的窗口 ms */
#define BASELINE_SAMPLES 50       /* 开机采安静基线的次数 */
#define LPF_ALPHA        0.1f     /* 一阶低通系数(0~1)；0.1 ≈ 16Hz 截止 @1kHz，270Hz 衰减约 14 倍 */
#define BASELINE_BETA        0.0002f /* 基线慢跟踪系数(τ≈5s)：跟得上慢漂移，跟不动按压/尖峰 */
#define PRESS_ALPHA   0.003f   /* slow EMA for press detection (tau~500ms); 0.001f = ~1s */
#define PRESS_TH      0.03f    /* pressed when voltage drops 0.03V below baseline */
#define RELEASE_TH    0.02f    /* released when within 0.02V of baseline (hysteresis) */
#define STRIKE_MARGIN  0.10f    /* 旧固定阈值，已被下面的自适应阈值取代（保留供对照） */
#define STRIKE_HOLD_MS 100      /* striking 置 1 后保持的毫秒数 */
/* ---- 自适应击打判据 (fw 20260923) ----
 * 实测（Ozone_DataSampling_260920.csv，ch3 空闲）：vin_f 的噪声 σ≈43 mV，
 * 固定 0.10 V 只有 2.3σ，纯噪声就刷出 336 次/分；把阈值抬到 10σ 仍有 25 次/分，
 * 因为噪声是脉冲型而非高斯型，单路幅度阈值分不开。
 * 真正能分开的是【同时性】：敲击是机械事件，四个角在几毫秒内一起响应；
 * 噪声在各通道间独立（实测 ch3 大偏离时 ch1 只有 18.4%、ch2 8.6%、ch0 0.1% 同时偏离）。
 * 实测 6σ + 3/4 路同时(±5ms) → 68.6 s 内 0 次误触发（原逻辑 336 次/分）。 */
#define STRIKE_SIGMA_K      6.0f    /* 单路阈值 = K × 噪声σ */
#define STRIKE_MARGIN_MIN   0.10f   /* 阈值下限(V)：保持与原固定阈值一致，保证不会比原来更敏感 */
#define STRIKE_MARGIN_MAX   1.20f   /* 阈值上限(V) */
#define NOISE_ALPHA         0.0005f /* σ 估计的 EMA 系数，τ≈2 s @1kHz */
#define NOISE_SIGMA_MIN     0.005f  /* σ 下限(V) */
#define NOISE_SIGMA_MAX     1.000f  /* σ 上限(V) */
#define NOISE_CLIP_K        3.0f    /* σ 裁剪：只用 |d| < 3σ 的"安静"样本更新，别让击打本身抬高 σ */
#define COINC_WIN_MS        5u      /* 同时性窗口(ms) */
#define COINC_CH            3u      /* 至少几路在窗口内同时越界才算一次击打 */
#define STRIKE_REFRACT_MS   100u    /* 确认一次后的不应期(ms) */
/* ---- osc_floor 自适应触发 (fw 20260923) ----
 * 原来固定用 osc_floor > 0.85 V，但空闲时 osc_floor 的 p90 就是 0.690、max 0.919，
 * 阈值落在噪声分布里面，状态机自己在 0.85/0.70 之间来回翻转。
 * 改成对 osc_floor 自身的慢参考值和离散度做自适应。 */
#define OSC_FLOOR_K          6.0f    /* 触发阈值 = floor_ref + K × floor_spread */
#define OSC_FLOOR_MARGIN_MIN 0.05f   /* 最小裕量(V) */
#define OSC_FLOOR_REF_ALPHA  0.0002f /* floor 参考值 EMA，τ≈5 s */
#define OSC_FLOOR_SPR_ALPHA  0.0005f /* floor 离散度 EMA，τ≈2 s */
#define OSC_FLOOR_REARM_FRAC 0.5f    /* 复位：回落到 参考+50%裕量 以下 */
#define HIT_PEAK_K           4.0f    /* 该路峰值须超过自身噪声σ的 K 倍才算"有响应" */
#define HIT_STRONG_CH        3u      /* 至少几路有响应才认可这次 hit */
#define OSC_WINDOW           20      /* 振荡幅度窗口：20样本=20ms，覆盖~5个270Hz周期 */
#define OSC_ALPHA            0.05f   /* 振荡幅度低通系数(慢) */
#define OSC_FLOOR_PRESS_TH   0.85f   /* 按压判定：近期下界>0.85V(无低点)=按压；空闲下界会掉到0.6 */
#define OSC_FLOOR_REARM_TH   0.7f    /* 重新武装：近期下界<0.7V(有低点)=空闲 */
#define OSC_FLOOR_LEAK       0.001f  /* 下界上回速率(V/ms)，约250ms遗忘一个低点 */
/* ---- Rail / latch supervision (fw 20260922) ----
 * 实测：连续敲击后模拟前端会整体上移并自锁，ch1/ch2 的 ADC 读到 4095 且持续数十秒。
 * 没有这层保护时，base[] 会一路爬到 3.3V，于是"贴轨"的通道看起来和正常通道一样，
 * 位置解算也会把废弃通道算进去。 */
#define CH_RAIL_HI      4090u   /* raw >= 此值 = 输入达到/超过 ADC 满量程 */
#define CH_RAIL_LO      5u      /* raw <= 此值 = 输入达到/低于 ADC 零点 */
#define CH_RAIL_N      20u      /* 连续多少个 1ms 样本后判定该通道已锁死 */
/* ---- 高速采样 + 1ms 峰值保持 (fw 20260925) ----
 * ADC 单通道 @55.6 kHz (采样时间 239.5 周期) 连续转换, DMA 循环搬运。
 * 每半块(ADC_DMA_BLKSZ 个采样)在 DMA 中断里处理一次:
 *   adc_raw[0]  = 该块内【偏离均值最大】的那个原始采样(保留符号)
 *                 -> Ozone 以 1 kHz 导出时不会再漏掉 <1ms 的快速瞬态
 *   adc_mean[0] = 该块均值(对照)
 *   stk_*       = 整流差分包络检波器(击打检测本体), 延迟 ≈ 2.3 ms
 */
#define ADC_DMA_BLKSZ   64u                    /* DMA 半块/全块 采样数, 64/55556 = 1.15 ms */
#define ADC_DMA_LEN     (ADC_DMA_BLKSZ * 2u)   /* DMA 缓冲总长(半字), 128 = 256 B */
/* ---- 击打检测器 (fw 20260926): 块内 |Δ| 峰值 / 本底 ----
 * 原理: 击打 = 采样点之间的突变。
 *   每块取 max|x[n]-x[n-1]|            -> stk_dmax   (块内最大跳变, 单位 counts)
 *   本底 = 静置时 stk_dmax 的慢跟踪     -> stk_dfloor
 *   比值 = stk_dmax / stk_dfloor × 100  -> stk_ratio
 * 比值无量纲、与增益和偏置无关, 可以直接判断"有没有信号"。
 * ★ stk_maxratio = 清零以来比值的最大值 —— 这是判断能否检出的核心观测量:
 *     静置时约 200~350 (纯噪声的自然起伏)
 *     击打时若远高于它  -> 能检出
 *     击打时也在 300 上下 -> 检不出
 */
#define STK_K           4u      /* 触发阈值: 比值 >= K×100 (即 4 倍本底) */
#define STK_FLOOR_SH    8u      /* 本底跟踪移位(按块): τ = 256 块 = 0.29 s */
#define STK_REFRACT_MS  10u     /* 不应期(ms) = 允许的检测延迟 */
/* ============ 击打/按压检测滤波器 (fw 20260927) ============
 * 输入: adc_notch_vin[0] (1 kHz, 每 1.15 ms DMA 块 64 采样 @55.6 kHz 的均值)
 *   两级梳状 20 -> 25    等效 FIR 长度 44 点 = 44 ms, 群延迟 21.5 ms, sigma 116 mV -> 13.4 mV
 *   4.1 s 基线           每 32 点压一次进 128 点 int16 环(mV), 环均值 = 4096 点滑动平均
 *   双 sigma (sp^2 的 EMA, 只在空闲期更新):
 *      快 tau = 2^12 = 4.1 s  -> 击打
 *      慢 tau = 2^16 = 65 s   -> 按压(慢, 不会被按压自己的上升沿抬走)
 *   触发   |sp| > 6*sigma_fast  或  |sp| > 8*sigma_slow
 *   释放   回到 min(两个阈值)*0.5     不应期 20 ms
 *   宽度   <= 300 ms -> 击打 ; > 300 ms -> 按压
 * 在 Ozone_FifthTest.csv (静置30s -> 每5s击打 -> 按压5s) 上实测:
 *   35.882s/41ms, 40.744s/28ms, 50.717s/23ms 判为击打; 静置 30 s 内 0 次误报
 * ★ 关键: 判据必须建立在 1ms 块均值再 20/25 点串联平均之后。
 *   adc_raw[0] 上 1~2 采样宽的尖峰, 在静置期和击打期出现频率相同(比值 1.01),
 *   不能用来判击打 —— 静置 23.890s 处那个尖峰比 40.7s 那次击打还大。
 * ========================================================== */
#define SF_N1           20u
#define SF_N2           25u
#define SF_DEC          32u
#define SF_RING         128u
#define SF_KS           4.5f    /* 击打阈值 = 4.5 * sigma_fast */
#define SF_KL           6.0f    /* 按压阈值 = 6 * sigma_slow */
#define SF_GATEK        3.0f    /* sigma 只在 |sp| < 3*sigma 时更新 (挡住击打后的余振) */
#define SF_REL          0.5f    /* 释放 = 阈值 * 0.5 */
#define SF_LONG_MS      300u    /* 宽于此判为按压 */
#define SF_REFRACT_MS   300u    /* 事件间最小间隔 */
/* ---- 消隐: 事件后阈值临时抬高, 线性回落 ----
 * 一锤会让传感器余振 2~4 s, 冒出 30~80 mV 的二次事件, 每个都会被算成一次击打。
 * 事件后把阈值抬高, 在 SF_BLANK_MS 内线性回落到 0: 余振全被压掉。
 * ★ 抬升量现在是运行时可调的 sf_blank_cfg (默认 0 = 不消隐),
 *   SF_BLANK_V 只作为"改用锤击时"的参考值保留, 不再直接使用。
 *   代价: 抬升期间灵敏度下降, 相隔 < 5 s 的下一击可能被吞掉。 */
#define SF_BLANK_V      0.120f  /* 参考值: 锤击场景建议 0.100 ~ 0.120 */
#define SF_BLANK_MS     5000u   /* 消隐回落时间(ms) */
/* ---- 按压改判 ----
 * 按压落下那一下是 115 mV / 80 ms, 和锤击形状完全一样, 靠宽度分不开。
 * 但按压之后电平会【持续】抬高, 锤击不会。所以事件结束后再看 5 s:
 * 若出现连续 1 s 的高电平, 就把刚才那次从击打计数退回, 记为按压。 */
#define SF_SUS_K        3.0f    /* 持续判定: |sp| > 3*sigma 连续 ... */
#define SF_SUS_MS       1000u   /*   ... 1000 ms */
#define SF_CHK_MS       5000u   /* 事件结束后观察 5 s */
#define SF_SETTLE_MS    5000u   /* 环填满后再等 5 s */
#define SF_PRESS_GAP_MS 6000u
#define SF_SIG_MIN      0.002f
#define SF_SIG_MAX      0.500f
#define SF_SIG_INIT     0.015f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint16_t adc_raw[4];   /* Legacy: [0] is block extreme, NOT latest conversion; [1..3] inactive. */
volatile float    adc_vin[4];   /* Legacy: [0] is peak-picked voltage. Use sensor0_raw_v for latest sample. */

float            base[4] = {0};      /* 安静基线电压 */
float            peak[4] = {0};      /* 冲击各通道峰值 */
volatile float   vin_f[4] = {0};     /* 低通滤波后的电压（一阶EMA状态，滤270Hz共振）；volatile 便于 Ozone 观察 */
volatile float   vin_slow[4] = {0};   /* slow-filtered voltage (~0.5Hz), for press/release */
volatile uint32_t pressed[4] = {0};    /* per-channel: 1 = pressed, 0 = released (watch in Ozone) */
volatile float    press_dev[4] = {0};  /* per-channel press deviation (base - vin_slow), for Ozone */
volatile uint32_t striking = 0;        /* 1 = striking activity detected (rough) */
volatile uint32_t strike_count = 0;    /* accumulates: +1 per strike (rising edge) */
volatile float   osc_amp = 0;        /* 振荡幅度(V)：原始信号20ms峰峰值，低通后 */
volatile float   osc_floor = 2.0f;   /* 振荡幅度近期下界(V)：空闲~0.6、按压~1.0；供Ozone观察 */
volatile float   hit_x = 0;          /* 击打位置 x（中心原点，右正左负） */
volatile float   hit_y = 0;          /* 击打位置 y（中心原点，上正下负） */
volatile uint32_t hit_count = 0;     /* 累计击打次数 */
/* ---- Rail / latch supervision (fw 20260922) ---- */
volatile uint32_t ch_rail[4] = {0};     /* 1 = 该通道已贴轨锁死（连续 CH_RAIL_N ms 撞轨） */
volatile uint32_t ch_rail_ms[4] = {0};  /* 当前已连续贴轨多少 ms；0 = 未贴轨 */
volatile uint32_t ch_rail_events = 0;   /* 上电以来"进入锁死"的总次数 */
volatile uint32_t trigger_ch = 0;       /* 0..3 = 指定通道；>=4 = 自动选"幅度最大且未贴轨"的一路（默认） */
volatile uint32_t trigger_ch_used = 0;  /* 实际生效的触发通道（选中路贴轨或自动模式下自动切换） */
volatile float    osc_amp_ch[4] = {0};  /* 每路各自的振荡幅度(V)，用来找"哪一路还活着" */
volatile float    peak_up[4] = {0};     /* 抓峰窗口内相对基线的最大【上】偏量(V) */
volatile uint32_t peak_dir = 0;         /* 0 = 位置用下偏 peak[]（原行为），1 = 用上偏 peak_up[] */
volatile uint32_t hit_valid = 0;        /* 1 = 上次触发时至少有 3 路可用，位置可信 */
volatile uint32_t hit_rejected = 0;     /* 因贴轨通道过多而被丢弃的触发次数 */
volatile uint32_t adc_rezero = 0;       /* Ozone 里写 1：立刻重采一次基线并复位滤波器 */
/* ---- 自适应击打判据的观测量 (fw 20260923) ---- */
volatile float    strike_dev[4] = {0};   /* vin_f - base，本路当前的向上偏离量(V) */
volatile float    noise_sigma[4] = {0};  /* 本路 vin_f 的在线噪声 σ 估计(V) */
volatile float    strike_thr[4] = {0};   /* 本路当前实际使用的击打阈值(V) = max(0.05, 6σ) */
volatile uint32_t strike_ch[4] = {0};    /* 1 = 本路本周期越过了自己的阈值 */
volatile uint32_t strike_coinc = 0;      /* 最近 COINC_WIN_MS 窗口内同时越界的通道数 0~4 */
volatile uint32_t strike_reject = 0;     /* 只有 1~2 路越界、因不同时而丢弃的次数 */
volatile float    osc_floor_ref = 0;     /* osc_floor 的自适应参考值(V) */
volatile float    osc_floor_spread = 0;  /* osc_floor 的慢离散度(V) */
volatile float    osc_floor_thr = 0;     /* 当前实际触发阈值(V) = ref + 6×spread */
/* ADC diagnostics: retained software-observed extrema, not a full DMA trace. */
volatile uint32_t adc_diag_version = 20260928u; /* strike filter v4: 消隐 + 按压改判 + sigma 门限 */
volatile uint32_t adc_diag_stage = 0; /* 1=calibrate, 2=start, 3=settle, 4=loop */
volatile uint32_t adc_cal_status = 0xFFFFFFFFu;
volatile uint32_t adc_start_status = 0xFFFFFFFFu;
volatile uint32_t main_loop_count = 0;
volatile uint16_t adc_seen_min[4] = {4095, 4095, 4095, 4095};
volatile uint16_t adc_seen_max[4] = {0};
volatile uint16_t adc_seen_span[4] = {0};
volatile uint32_t adc_diag_reset = 1; /* Write 1 in Ozone to start a new window. */
/* 1 kHz sampled raw voltage and notch output, both in volts for Ozone. */
volatile float adc_notch_vin[4] = {0};
volatile uint32_t notch_sample_count = 0;
static volatile uint32_t adc_filter_ready = 0;
static Notch50State notch_state[4] = {0};
static uint8_t ch_frozen[4] = {0};   /* 该通道上一轮是否处于贴轨冻结状态 */
static float   noise_var[4] = {0};   /* vin_f-base 的在线方差估计 */
static uint8_t coinc_ring[COINC_WIN_MS + 1] = {0}; /* 最近若干采样的"越界位掩码" */
static uint8_t coinc_idx = 0;
static uint32_t strike_last_ms = 0;  /* 上次确认击打的时间戳 */
static uint32_t strike_dropped = 0;  /* 被同时性判据丢掉的候选次数 */

/* ================= 高速采样 + 峰值保持 (fw 20260925) ================= */
volatile uint16_t adc_dma_buf[ADC_DMA_LEN] = {0};  /* DMA 目标缓冲 */
volatile uint16_t adc_mean[1]  = {0};   /* 每块的均值原始值(对照用) */
volatile uint16_t blk_max_ever = 0;     /* 上电以来全局最大原始值 ★ */
volatile uint16_t blk_min_ever = 0xFFFF;/* 上电以来全局最小原始值 ★ */
volatile uint32_t blk_reset    = 1;     /* Ozone 写 1: 清零全局极值和统计 */
volatile uint32_t blk_count    = 0;     /* 已处理块数, 应约 870/s */

volatile uint32_t stk_dmax     = 0;     /* 本块 |Δ| 最大值 (counts) */
volatile uint32_t stk_dfloor   = 1;     /* 本底: 静置时 stk_dmax 的慢跟踪 */
volatile uint32_t stk_ratio    = 0;     /* 本块比值 ×100 = dmax/dfloor*100 */
volatile uint32_t stk_maxratio = 0;     /* ★ 清零以来比值最大值 —— 看这个判断能否检出 */
volatile uint32_t stk_count    = 0;     /* 击打累计次数 */
volatile uint32_t stk_state    = 0;     /* 0=空闲 1=已触发 */
volatile uint32_t stk_reject   = 0;     /* 未达阈值的块数(诊断) */

static uint16_t stk_prev;               /* 上一个采样 */
static uint32_t stk_dfloor_q;           /* 4.12 定点本底 */
static uint32_t stk_last_ms;            /* 上次计数时刻 */

/* ================= 击打/按压检测滤波器状态 (fw 20260927) ================= */
volatile float    sf_sp       = 0;  /* 滤波后偏离基线的量(V)      ★看这个 */
volatile float    sf_sigma    = 0;  /* 快 sigma(V) */
volatile float    sf_sigma_lp = 0;  /* 慢 sigma(V) */
volatile float    sf_thr      = 0;  /* 自适应击打阈值(V) = SF_KS*sf_sigma */
volatile float    sf_blank    = 0;  /* 当前消隐抬升量(V), 事件后从 sf_blank_cfg 线性降到 0 */
volatile float    sf_ratio    = 0;  /* |sp|/sigma                 ★调 SF_KS 看它 */
volatile uint32_t sf_strike   = 0;  /* ★★★ 击打次数 ★★★ */
volatile uint32_t sf_press    = 0;  /* 按压次数 */
volatile uint32_t sf_reclass  = 0;  /* 被改判为按压起始的次数(诊断) */
volatile uint32_t sf_state    = 0;  /* 1 = 事件进行中 */
volatile uint32_t sf_evt_ms   = 0;  /* 当前事件宽度(ms) */
volatile uint32_t sf_maxratio = 0;  /* sf_ratio 历史最大值 x100 */
volatile uint32_t sf_winratio = 0;  /* 最近 5 s 内 sf_ratio 最大值 x100 */
volatile uint32_t sf_reset    = 1;  /* Ozone 写 1 清零 (上电自动为 1) */
volatile uint32_t sf_ready    = 0;  /* 1 = 滤波器已稳定, 开始计数 */

/* =================★ 判定力度调节 (Ozone 里直接改, 不会被 sf_reset 清掉) =================
 *
 * sf_sens_v  —— 【判定力度】, 单位 V。这就是多小的偏离算一次击打。
 *     写 0        -> 用自适应阈值 (SF_KS * sf_sigma), 上电默认
 *     写 0.060    -> 偏离基线 60 mV 就触发
 *     写 0.100    -> 需要 100 mV 才触发
 *     数值越小越灵敏: 轻击能触发, 但误报风险上升
 *
 *  怎么选这个值 (对照导出的 sensor0_delta_v):
 *     1) 静置 20 s, 看 |sensor0_delta_v| 最多到多少 —— 记 N
 *     2) 轻敲一下, 看 sensor0_delta_v 峰值能到多少 —— 记 S
 *     3) sf_sens_v 取 (N + S) / 2 附近, 也就是落在"静置噪声上限"和"击打峰值"正中间
 *  44444.csv 实测: 静置 |delta| 的 100ms 差分 p999 = 45.8 mV, 击打 89~164 mV
 *                  -> sf_sens_v = 0.065f 落在中间
 *  若 N 和 S 靠得太近 (S < 2N), 说明这个力度在这个噪声下分不开, 调不出结果。
 *
 * sf_blank_cfg —— 【消隐抬升量】, 单位 V, 上电默认 0 = 不消隐。
 *     事件关闭后把阈值抬高 sf_blank_cfg, 在 SF_BLANK_MS 内线性回落到 0。
 *     这是给"锤击"用的(锤击余振 2~4 s 会冒出二次事件), 但代价是:
 *     抬升期间灵敏度下降, 相隔 < 5 s 的下一击可能被吞掉。
 *     44444.csv 里击打间隔只有 1.7 s, 120 mV 的消隐吃掉了 16 次里的 5 次,
 *     所以默认改成 0。如果改用锤击, 可以写回 0.100~0.120。
 *
 * 只读诊断量:
 *     sf_thr_used —— 当前实际生效的触发电平(V), 等于 判定力度 + 当前消隐量
 *     sf_sens_sel —— 1 = 正在用手动 sf_sens_v; 0 = 正在用自适应阈值
 */
volatile float    sf_sens_v    = 0.0f;  /* ★★★ 判定力度(V); 0 = 自适应 ★★★ */
volatile float    sf_blank_cfg = 0.0f;  /* 消隐抬升量(V); 0 = 不消隐 */
volatile float    sf_thr_used  = 0;     /* 只读: 当前实际生效的触发电平(V) = 判定力度 + 消隐量 */
volatile uint32_t sf_sens_sel  = 0;     /* 只读: 1 = 正在用手动 sf_sens_v; 0 = 自适应 */
/* =================★ 极性 (Ozone 里直接改, 不会被 sf_reset 清掉) =================
 * 实测 44444.csv: 击打 -> sensor0_delta_v 变负 (-89 ~ -164 mV)
 *                 按压 -> sensor0_delta_v 变正 (+138 ~ +230 mV)
 * sf_pol:
 *     -1  = 只看负向, 也就是"击打"方向  ★默认
 *      0  = 双极性, 正负都算 (旧行为, 按压也会起事件)
 *     +1  = 只看正向
 * 只看负向的好处: 正向的按压根本不会起事件, 不占不应期也不触发消隐,
 * 用 44444.csv 实测, 事件数从 29 降到 24, 击打一次不丢。
 */
volatile int32_t  sf_pol       = -1;    /* ★★★ 极性; -1=只看负向(击打) ★★★ */
volatile float    sf_trig_v    = 0;     /* 只读: 按当前极性算出的有效偏离量(V) */

/* Sensor 0 test observables. Voltages are ADC-pin volts, not piezo input volts.
 * Raw/block statistics update at DMA block completion (~1.152 ms).
 * Filter/detector observables update in SysTick (1 ms). */
volatile uint32_t sensor0_test_version = 1u;
volatile uint16_t sensor0_raw_adc = 0;
volatile float sensor0_raw_v = 0;       /* Last actual sample of the completed DMA block. */
volatile float sensor0_mean_v = 0;      /* Mean of all 64 samples, before temporal filtering. */
volatile float sensor0_block_min_v = 0, sensor0_block_max_v = 0;
volatile float sensor0_filtered_v = 0;  /* Absolute voltage after 20 + 25 point moving averages. */
volatile float sensor0_baseline_v = 0;
volatile float sensor0_delta_v = 0;     /* Filtered voltage minus adaptive baseline. */
volatile float sensor0_threshold_v = 0; /* Actual start threshold INCLUDING blanking uplift. */
volatile float sensor0_last_peak_v = 0; /* Last completed event's absolute baseline deviation. */
volatile uint32_t sensor0_last_width_ms = 0;
volatile uint32_t sensor0_candidate_count = 0; /* Monotonic threshold-crossing event count. */
volatile uint32_t sensor0_hit_count = 0; /* sf_strike: short events; may be reclassified as press. */
volatile uint32_t sensor0_press_count = 0;
volatile uint32_t sensor0_active = 0;    /* Threshold event currently in progress. */
volatile uint32_t sensor0_hit = 0;       /* 100 ms pulse when a short event completes. */
volatile uint32_t sensor0_ready = 0;     /* Quiet startup/reset needs about 9.2 s. */
volatile uint32_t sensor0_reset = 0;     /* Write 1: reset detector, counts and baseline; keep ADC running. */
static uint32_t sensor0_hit_hold = 0;

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
static float    sfgate_sig;             /* 上一拍的 sigma, 作为 sigma 门限的比较基准 */
static float    sfpeak;                 /* 当前事件的峰值(V) */
static float    sf_blank_step;          /* 消隐每毫秒的回落量 */
static uint32_t sfsus;                  /* 持续高电平的连续毫秒数 */
static uint32_t sfchk;                  /* 事件结束后的观察倒计时(ms) */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ================= 击打/按压检测滤波器 (fw 20260928) ================= */
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
  /* 注意: sf_sens_v / sf_blank_cfg / sf_pol 是【用户设定】, 这里故意不清零 ——
   *       按复位键后判定力度和极性保持你上次调好的值, 不用重新写。 */
  sf_thr_used = 0; sf_sens_sel = 0; sf_trig_v = 0;
  sensor0_filtered_v = 0; sensor0_baseline_v = 0; sensor0_delta_v = 0;
  sensor0_threshold_v = 0; sensor0_last_peak_v = 0; sensor0_last_width_ms = 0;
  sensor0_candidate_count = 0; sensor0_hit_count = 0; sensor0_press_count = 0;
  sensor0_active = 0; sensor0_hit = 0; sensor0_hit_hold = 0; sensor0_ready = 0;
}

/* 每 1 ms 调用一次 (由 ADC_FilterTick1ms 调用) */
void StrikeFilter_Tick(float vin)
{
  float y1, y2, sp, a, sg, sgl, thrS, thrL, thr, thrEff, rel, trig;

  if (sf_reset) { sf_reset = 0; StrikeFilter_Reset(); }

  /* Seed both moving averages at the first voltage to avoid a false ramp from zero. */
  if (!sfinited && sfwarm == 0)
  {
    for (uint32_t k = 0; k < SF_N1; k++) sfb1[k] = vin;
    for (uint32_t k = 0; k < SF_N2; k++) sfb2[k] = vin;
    sfa1 = vin * (float)SF_N1; sfa2 = vin * (float)SF_N2;
    sfbase = vin;
    sensor0_baseline_v = vin;
  }
  /* Cascaded 20/25-point moving averages: 44-sample impulse response,
   * 21.5 ms group delay at 1 kHz; NOT a 500 ms moving average. */
  sfa1 += vin - sfb1[sfi1];
  sfb1[sfi1] = vin;
  if (++sfi1 >= SF_N1) sfi1 = 0;
  y1 = sfa1 * (1.0f / (float)SF_N1);

  sfa2 += y1 - sfb2[sfi2];
  sfb2[sfi2] = y1;
  if (++sfi2 >= SF_N2) sfi2 = 0;
  y2 = sfa2 * (1.0f / (float)SF_N2);
  sensor0_filtered_v = y2;

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
    sensor0_baseline_v = sfbase;
  }
  if (sfring_n < SF_RING) return;   /* 前 4.1 s 只填环 */

  /* ---- 3. 偏离基线的量 ---- */
  sp = y2 - sfbase;
  a  = (sp >= 0.0f) ? sp : -sp;
  sf_sp = sp;
  sensor0_delta_v = sp;

  /* ---- 4. 双 sigma: 只在空闲期 且 |sp| < SF_GATEK*sigma 时更新 ----
   * 后一个条件必须加: 击打后传感器余振, 振铃期间信号反复穿过阈值使 sf_state
   * 频繁回到 0。若把这些振铃样本算进 sigma, sigma 会被自己抬高, 后面每一锤
   * 都检不出。加了这个门之后 sigma 稳定在静置噪声本底上。 */
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

  /* ---- ★ 判定力度: sf_sens_v > 0 时用手动绝对值覆盖自适应阈值 ---- */
  if (sf_sens_v > 0.0f)
  {
    thr = sf_sens_v;
    sf_sens_sel = 1;
  }
  else sf_sens_sel = 0;

  rel  = thr * SF_REL;
  sf_thr = thr;
  sf_ratio = (sg > 0.0f) ? (a / sg) : 0.0f;

  /* ---- ★ 极性: 决定往哪个方向算"击打" ----
   * 实测(44444.csv): 击打 -> filtered_v 下凹, delta_v 变负 (-89~-164 mV)
   *                  按压 -> delta_v 变正 (+138~+230 mV)
   * trig 就是"按当前极性算出来的有效偏离量", 只有它越过阈值才起事件。 */
  if (sf_pol < 0)       trig = -sp;              /* 只看负向 = 击打方向 (默认) */
  else if (sf_pol > 0)  trig =  sp;              /* 只看正向 */
  else                  trig =  a;               /* 双极性(正负都算) */
  if (trig < 0.0f) trig = 0.0f;
  sf_trig_v = trig;                              /* 只读: 极性相关量, 调参看它 */

  /* ---- 5. 消隐量线性回落 ---- */
  if (sf_blank > 0.0f)
  {
    sf_blank -= sf_blank_step;
    if (sf_blank < 0.0f) sf_blank = 0.0f;
  }
  thrEff = thr + sf_blank;
  sf_thr_used = thrEff;                 /* 只读: 当前真正生效的触发电平 */
  sensor0_threshold_v = thrEff;

  /* ---- 6. 诊断量: 历史最大 / 最近 5 s 最大 (调 SF_KS 用) ---- */
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

  /* ---- 7. 触发 / 释放 / 宽度分类 ---- */
  if (sfhold) sfhold--;
  sfrunms++;
  if (sfrunms < SF_SETTLE_MS) return;   /* 环填满后再等 5 s */
  sf_ready = 1;                         /* 从这里开始才算数 */

  /* ---- 8. 持续性判定: 事件后 5 s 内若连续 1 s 高电平 -> 是按压起始, 击打计数回退 ---- */
  if (sfchk > 0)
  {
    sfchk--;
    if (trig > SF_SUS_K * sg)
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
    if (sfhold == 0 && trig > thrEff)
    {
      sf_state = 1; sfms = 1; sf_evt_ms = 1; sfpeak = a;
      sensor0_candidate_count++;
    }
  }
  else
  {
    sfms++;
    sf_evt_ms = sfms;
    if (a > sfpeak) sfpeak = a;
    if (trig < rel)
    {
      sensor0_last_peak_v = sfpeak;
      sensor0_last_width_ms = sfms;
      if (sfms <= SF_LONG_MS) { sf_strike++; sfchk = SF_CHK_MS; sfsus = 0; }
      else
      {
        uint32_t now = HAL_GetTick();
        if ((now - sflast_press) > SF_PRESS_GAP_MS) { sf_press++; sflast_press = now; }
      }
      sf_state = 0;
      sfhold = SF_REFRACT_MS;
      /* 消隐: 用运行时可调的 sf_blank_cfg (默认 0 = 不消隐) */
      sf_blank = sf_blank_cfg;
      sf_blank_step = (sf_blank_cfg > 0.0f) ? (sf_blank_cfg / (float)SF_BLANK_MS) : 0.0f;
    }
    else if (sfms > 60000u) { sf_state = 0; sfhold = SF_REFRACT_MS; }
  }
}

/* Called only by the 1 ms SysTick interrupt. The foreground may block for
 * baseline/peak capture; filtering must still continue at exactly 1 kHz.
 * ADC/DMA run independently. This snapshots their latest values, not every
 * hardware conversion; it is not a replacement for analog anti-aliasing.
 */
void ADC_FilterTick1ms(void)
{
  if (!adc_filter_ready || blk_count == 0u) return;
  if (sensor0_reset)
  {
    sensor0_reset = 0;
    sf_reset = 1;
  }
  uint32_t previous_hits = sf_reset ? 0u : sf_strike;
  adc_vin[0] = adc_raw[0] * (ADC_REFERENCE_V / 4095.0f); /* Legacy peak view. */
  StrikeFilter_Tick(sensor0_mean_v);
  sensor0_ready = sf_ready;
  sensor0_active = sf_state;
  sensor0_hit_count = sf_strike;
  sensor0_press_count = sf_press;
  if (sensor0_hit_hold) sensor0_hit_hold--;
  if (sf_strike > previous_hits) sensor0_hit_hold = 100u;
  sensor0_hit = (sensor0_hit_hold != 0u);
  notch_sample_count++; /* Legacy name: counts 1 kHz processing ticks, no biquad here. */
}

/* ---- 处理一个 DMA 块: 峰值保持 + 整流差分包络检波 ----
 * 由 DMA 半传输/全传输中断调用, 每块 ADC_DMA_BLKSZ 个采样。
 * 纯整数运算, 每采样约 20 条指令。
 */
static void Strike_ProcessBlock(const uint16_t *buf, uint32_t n)
{
  uint16_t mx = 0, mn = 0xFFFF, dmax = 0;
  uint32_t sum = 0;

  if (blk_reset)
  {
    blk_reset    = 0;
    blk_max_ever = 0;
    blk_min_ever = 0xFFFF;
    blk_count    = 0;
    stk_count    = 0;
    stk_maxratio = 0;
    stk_reject   = 0;
    stk_dfloor_q = 0;
    stk_dfloor   = 1;
  }

  for (uint32_t i = 0; i < n; i++)
  {
    uint16_t s = buf[i];

    /* --- 块极值与和 --- */
    if (s > mx) mx = s;
    if (s < mn) mn = s;
    sum += s;

    /* --- 相邻采样差分绝对值, 取块内最大 --- */
    uint16_t dd = (s >= stk_prev) ? (uint16_t)(s - stk_prev) : (uint16_t)(stk_prev - s);
    stk_prev = s;
    if (dd > dmax) dmax = dd;
  }
  stk_dmax = dmax;

  /* --- 全局极值 --- */
  if (mx > blk_max_ever) blk_max_ever = mx;
  if (mn < blk_min_ever) blk_min_ever = mn;

  /* --- 本底慢跟踪: 只在空闲状态, 且只用"像本底"的块 --- */
  {
    uint32_t f = stk_dfloor_q >> 12;
    if (f < 1u) f = 1u;
    if (!stk_state && (uint32_t)dmax < f * 3u)
    {
      int32_t tgt = (int32_t)dmax << 12;
      stk_dfloor_q = (uint32_t)((int32_t)stk_dfloor_q +
                    ((tgt - (int32_t)stk_dfloor_q) >> STK_FLOOR_SH));
    }
  }
  {
    uint32_t f = stk_dfloor_q >> 12;
    if (f < 1u) f = 1u;
    stk_dfloor = f;
  }

  /* --- 比值 ×100 --- */
  {
    uint32_t ratio = ((uint32_t)dmax * 100u) / stk_dfloor;
    stk_ratio = ratio;
    if (ratio > stk_maxratio) stk_maxratio = ratio;

    /* --- 双阈值滞回 + 不应期 --- */
    if (!stk_state)
    {
      if (ratio >= STK_K * 100u)
      {
        stk_state = 1;
        uint32_t now = HAL_GetTick();
        if ((now - stk_last_ms) >= STK_REFRACT_MS)
        {
          stk_count++;
          stk_last_ms = now;
        }
      }
      else stk_reject++;
    }
    else
    {
      if (ratio < STK_K * 50u) stk_state = 0;
    }
  }

  /* --- ★ 两个导出量 ---
   *   adc_raw[0]       = 该块内【偏离均值最大】的原始采样(保留符号)
   *                      -> 1 kHz 导出不会漏掉 <1ms 的快速瞬态, 看击打就看它
   *   adc_notch_vin[0] = 该块【均值电压】(V)
   *                      -> 窄带低噪声, 看真实直流基线和本底噪声就看它
   */
  {
    uint16_t mean = (uint16_t)(sum / n);
    sensor0_raw_adc = buf[n - 1u];
    sensor0_raw_v = (float)sensor0_raw_adc * (ADC_REFERENCE_V / 4095.0f);
    sensor0_mean_v = ((float)sum / (float)n) * (ADC_REFERENCE_V / 4095.0f);
    sensor0_block_min_v = (float)mn * (ADC_REFERENCE_V / 4095.0f);
    sensor0_block_max_v = (float)mx * (ADC_REFERENCE_V / 4095.0f);
    adc_mean[0]      = mean;
    adc_notch_vin[0] = sensor0_mean_v; /* Legacy alias: this is a block mean, not notch output. */
    adc_raw[0]       = (((uint32_t)mx - mean) >= ((uint32_t)mean - mn)) ? mx : mn;
  }

  blk_count++;
}
/* USER CODE END 0 */

/* Called from foreground sampling, including the hit peak-capture window.
 * Extrema persist until reset so slow debugger refresh does not erase them.
 * Pulses occurring entirely between these samples can still be missed.
 */
static void ADC_UpdateDiagnostics(void)
{
  uint32_t reset = adc_diag_reset;
  adc_diag_reset = 0;
  for (int i = 0; i < 4; i++)
  {
    uint16_t raw = adc_raw[i];
    if (reset)
    {
      adc_seen_min[i] = raw;
      adc_seen_max[i] = raw;
    }
    else
    {
      if (raw < adc_seen_min[i]) adc_seen_min[i] = raw;
      if (raw > adc_seen_max[i]) adc_seen_max[i] = raw;
    }
    adc_seen_span[i] = adc_seen_max[i] - adc_seen_min[i];
  }
}

/* 开机采 samples 次平均作为安静基线（此时板上不能有压力） */
static void ADC_CalibrateBaseline(uint32_t samples)
{
  float sum[4] = {0, 0, 0, 0};
  for (uint32_t k = 0; k < samples; k++)
  {
    for (int i = 0; i < 4; i++)
      sum[i] += adc_notch_vin[i]; /* Baseline uses the same filtered signal. */
    HAL_Delay(1);
  }
  for (int i = 0; i < 4; i++)
    base[i] = sum[i] / (float)samples;
}

/* ADC → 力 F 的转换（占位）。
   用「电压 - 基线」当作 F（正比于力）。
*/
static float adc_to_F(float voltage, float baseline)
{
  float f = baseline - voltage;   /* 阶跃：按下时电压【下降】，所以 基线-电压 = 按压的力 */
  if (f < 0) f = 0;
  return f;
}

/* 一阶低通（指数滑动平均）：state += alpha * (x - state)。
   每次 1ms 采样调用一次。用于滤掉 ~270Hz 机械共振（击打是 <50Hz 的慢信号）。 */
static float lpf_update(float state, float x, float alpha)
{
  return state + alpha * (x - state);
}

/* 自适应基线：电压高于基线时较快跟上（跟踪空闲/向上漂移），
   低于基线时极慢（按压不拉低基线）。
   解决"固定基线采早/采错/漂移"导致的 hit_count 自增或永远为 0。 */
static float baseline_track(float base, float v)
{
  return base + BASELINE_BETA * (v - base);   /* 对称慢跟踪：不追尖峰也不追按压 */
}

/* 振荡幅度检测：手指压上会压死 270Hz 共振，振荡幅度骤降。
 * 每 1ms 对四路各调一次，更新 osc_amp_ch[]（该路峰峰值，低通后），并返回该路的值。
 * 调用者再决定用哪一路去驱动 osc_amp / osc_floor。 */
static float osc_update(float voltage, int ch)
{
  static float buf[4][OSC_WINDOW];
  static uint8_t idx[4] = {0, 0, 0, 0};
  static uint8_t initialized[4] = {0, 0, 0, 0};
  if (!initialized[ch]) {
    for (int i = 0; i < OSC_WINDOW; i++) buf[ch][i] = voltage;
    initialized[ch] = 1;
  }
  buf[ch][idx[ch]] = voltage;
  idx[ch] = (idx[ch] + 1) % OSC_WINDOW;
  float mn = buf[ch][0], mx = buf[ch][0];
  for (int i = 1; i < OSC_WINDOW; i++) {
    if (buf[ch][i] < mn) mn = buf[ch][i];
    if (buf[ch][i] > mx) mx = buf[ch][i];
  }
  float amp = mx - mn; /* Input is already a voltage after the 50 Hz notch. */
  osc_amp_ch[ch] = lpf_update(osc_amp_ch[ch], amp, OSC_ALPHA);
  return osc_amp_ch[ch];
}

/* 复位某一路的陷波状态。通道刚从轨上下来时必须做，否则滤波器内残留的
 * 直流电平会制造一个假阶跃。 */
static void Notch50_Reset(int ch)
{
  notch_state[ch].x1 = notch_state[ch].x2 = 0.0f;
  notch_state[ch].y1 = notch_state[ch].y2 = 0.0f;
  notch_state[ch].initialized = 0;
}

/* 立刻重采一次安静基线，并把滤波/按压/峰值状态全部复位到新基线。
 * 用于模拟前端已经漂移、但不想断电重启的场合。贴轨通道跳过（3.3V 不是有效基线）。 */
static void ADC_Rezero(void)
{
  float sum[4] = {0, 0, 0, 0};
  for (uint32_t k = 0; k < BASELINE_SAMPLES; k++)
  {
    for (int i = 0; i < 4; i++) sum[i] += adc_notch_vin[i];
    HAL_Delay(1);
  }
  for (int i = 0; i < 4; i++)
  {
    if (ch_rail[i]) continue;
    base[i] = sum[i] / (float)BASELINE_SAMPLES;
    vin_f[i] = base[i];
    vin_slow[i] = base[i];
    press_dev[i] = 0;
    pressed[i] = 0;
    peak[i] = 0;
    peak_up[i] = 0;
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */
  /* Calibrate ADC, then start single-channel PA3 sampling via circular DMA */
  sensor0_test_version = 1u; /* Retain the observation-profile version in the ELF. */
  adc_diag_stage = 1;
  adc_cal_status = HAL_ADCEx_Calibration_Start(&hadc);
  if (adc_cal_status != HAL_OK) Error_Handler();

  /* ---- 高速连续采样: 单通道 @55.6 kHz, DMA 循环搬运 ----
   * 与原来相反: 这次【必须】使能 DMA 的 HT/TC 中断, 检测在中断里做。 */
  stk_prev    = 0;
  stk_dfloor_q = 0;                    /* 本底从 0 开始, 第一块后自动建立 */
  stk_last_ms = HAL_GetTick();

  adc_diag_stage = 2;
  adc_start_status = HAL_ADC_Start_DMA(&hadc, (uint32_t*)adc_dma_buf, ADC_DMA_LEN);
  if (adc_start_status != HAL_OK) Error_Handler();

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  adc_filter_ready = 1;
  adc_diag_stage = 3;

  /* 开机采安静基线（此刻不要碰板子） */
  //HAL_Delay(4000);  /* 传感器开机有~1~2s慢速爬升(实测1.43V→1.65V)，必须等稳定再采基线 */
  ADC_CalibrateBaseline(BASELINE_SAMPLES);

  /* 用基线初始化滤波器状态，避免启动瞬态误触发 */
  for (int i = 0; i < 4; i++) { vin_f[i] = base[i]; vin_slow[i] = base[i]; }

  uint8_t armed = 1;   /* 击打检测状态机：1=可触发，0=等回落 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    adc_diag_stage = 4;
    main_loop_count++;
    ADC_UpdateDiagnostics();
    if (SENSOR0_TEST_MODE)
    {
      /* Independent single-channel test runs entirely in DMA + SysTick.
       * Do not run auto-channel selection, 3/4 coincidence or position solving. */
      if (adc_rezero) { adc_rezero = 0; sensor0_reset = 1; }
      HAL_Delay(1);
      continue;
    }

    /* ---- 0. 手动重零：Ozone 里把 adc_rezero 写 1 ---- */
    if (adc_rezero)
    {
      adc_rezero = 0;
      ADC_Rezero();
    }

    /* ---- 1. 原始 ADC → 电压 → 滤波 + 自适应基线（供峰值/重心用） ---- */
    for (int i = 0; i < 4; i++)
    {
      /* 贴轨通道整体冻结：不滤波、不跟踪基线。
       * 否则 base[] 会一路爬到 3.3V，把"已锁死的通道"伪装成正常通道。 */
      if (ch_rail[i])
      {
        ch_frozen[i] = 1;
        continue;
      }
      float v = adc_notch_vin[i]; /* Fixed-rate notch output from SysTick. */
      if (ch_frozen[i])
      {
        /* 刚从轨上下来：把状态重置到当前电平，否则这个台阶会被当成一次击打。 */
        Notch50_Reset(i);
        v = adc_raw[i] * (ADC_REFERENCE_V / 4095.0f);
        vin_f[i] = v;
        vin_slow[i] = v;
        base[i] = v;
        press_dev[i] = 0;
        pressed[i] = 0;
        peak[i] = 0;
        peak_up[i] = 0;
        ch_frozen[i] = 0;
        continue;
      }
      vin_f[i]     = lpf_update(vin_f[i], v, LPF_ALPHA);
      vin_slow[i]  = lpf_update(vin_slow[i], v, PRESS_ALPHA);
      if (!pressed[i])
        base[i] = baseline_track(base[i], vin_slow[i]);            /* freeze baseline while pressed */
    }

    /* ---- 2. 振荡幅度(270Hz) → 按压检测：手指压上会压死共振，幅度骤降 ----
     * 四路都算；触发用的那一路由 trigger_ch 选。若选中的一路贴轨（没有信号），
     * 自动退回到"幅度最大且未贴轨"的一路，这样单路损坏不会让整个检测失效。 */
    for (int i = 0; i < 4; i++) osc_update(adc_notch_vin[i], i);
    {
      /* 触发通道选择：
       *   trigger_ch = 0..3  → 固定用该路；若该路贴轨则临时退回到最大幅度的一路
       *   trigger_ch >= 4    → 自动：选 osc_amp_ch 最大且未贴轨的一路（带 2× 迟滞防抖）
       * 实测这块板上 ch0 空闲时 1ms 抖动只有 0.4 mV（等于死通道），
       * 而原来的两套检测都只读 ch0 —— 自动模式直接解决这个问题。 */
      uint32_t tc;
      if (trigger_ch <= 3u && !ch_rail[trigger_ch]) {
        tc = trigger_ch;
      } else {
        uint32_t prev = trigger_ch_used;                 /* 上一轮的选择，用于迟滞 */
        if (prev > 3u || ch_rail[prev]) prev = 0u;
        uint32_t best = prev;
        float best_amp = -1.0f;
        for (int i = 0; i < 4; i++)
          if (!ch_rail[i] && osc_amp_ch[i] > best_amp) { best_amp = osc_amp_ch[i]; best = (uint32_t)i; }
        /* 自动模式：新通道要明显更强(>2×)才切换，避免在两路之间抖动 */
        if (trigger_ch > 3u && best != prev && best_amp < 2.0f * osc_amp_ch[prev]) best = prev;
        tc = best;
      }
      trigger_ch_used = tc;
      osc_amp = osc_amp_ch[tc];
      /* 下界(近期最低)：快速下跟、缓慢上回。空闲掉到0.6，按压停在1.0 */
      osc_floor = (osc_amp < osc_floor + OSC_FLOOR_LEAK) ? osc_amp : (osc_floor + OSC_FLOOR_LEAK);
      /* osc_floor 自身的慢参考值与离散度 → 自适应触发阈值 */
      {
        float ref = osc_floor_ref, spr = osc_floor_spread;
        ref += OSC_FLOOR_REF_ALPHA * (osc_floor - ref);
        spr += OSC_FLOOR_SPR_ALPHA * (fabsf(osc_floor - ref) - spr);
        osc_floor_ref = ref;
        osc_floor_spread = spr;
        float thr = ref + OSC_FLOOR_K * spr;
        if (thr < ref + OSC_FLOOR_MARGIN_MIN) thr = ref + OSC_FLOOR_MARGIN_MIN;
        osc_floor_thr = thr;
      }
    }

    /* ---- press/release detection: per-channel slow voltage vs frozen baseline ---- */
    for (int i = 0; i < 4; i++)
    {
      if (ch_rail[i]) { press_dev[i] = 0; pressed[i] = 0; continue; }
      press_dev[i] = base[i] - vin_slow[i];   /* >0 = pressed (voltage dropped) */
      if (!pressed[i])
      {
        if (press_dev[i] > PRESS_TH) pressed[i] = 1;
      }
      else
      {
        if (press_dev[i] < RELEASE_TH) pressed[i] = 0;
      }
    }

    /* ---- rough strike detection (fw 20260923) ----
     * 每路各用自己的自适应阈值 (max(0.05V, 6σ)) 判断是否向上越界，
     * 再要求最近 COINC_WIN_MS 内至少 COINC_CH 路同时越界。
     * 单路越界只算候选，不计数；这是把"每路独立的噪声"和"共同激励的机械敲击"分开的关键。 */
    {
      uint8_t mask = 0;
      for (int i = 0; i < 4; i++)
      {
        float d = vin_f[i] - base[i];
        strike_dev[i] = d;

        /* σ 在线估计：只用 |d| < 3σ 的"安静"样本，否则击打本身会把 σ 抬上去 */
        float s = noise_sigma[i];
        if (s < NOISE_SIGMA_MIN) s = NOISE_SIGMA_MIN;
        if (fabsf(d) < NOISE_CLIP_K * s)
        {
          noise_var[i] += NOISE_ALPHA * (d * d - noise_var[i]);
          if (noise_var[i] < 0.0f) noise_var[i] = 0.0f;
        }
        float sn = sqrtf(noise_var[i]);
        if (sn < NOISE_SIGMA_MIN) sn = NOISE_SIGMA_MIN;
        if (sn > NOISE_SIGMA_MAX) sn = NOISE_SIGMA_MAX;
        noise_sigma[i] = sn;

        float thr = STRIKE_SIGMA_K * sn;
        if (thr < STRIKE_MARGIN_MIN) thr = STRIKE_MARGIN_MIN;
        if (thr > STRIKE_MARGIN_MAX) thr = STRIKE_MARGIN_MAX;
        strike_thr[i] = thr;

        if (!ch_rail[i] && d > thr) { strike_ch[i] = 1; mask |= (uint8_t)(1u << i); }
        else                          strike_ch[i] = 0;
      }

      /* 同时性：把最近 COINC_WIN_MS+1 个采样的越界掩码 OR 起来 */
      coinc_ring[coinc_idx] = mask;
      coinc_idx = (uint8_t)((coinc_idx + 1u) % (COINC_WIN_MS + 1u));
      uint8_t acc = 0;
      for (uint32_t i = 0; i <= COINC_WIN_MS; i++) acc |= coinc_ring[i];
      uint32_t n = 0;
      for (int i = 0; i < 4; i++) if (acc & (uint8_t)(1u << i)) n++;
      strike_coinc = n;

      uint32_t now = HAL_GetTick();
      if (n >= COINC_CH)
      {
        if ((now - strike_last_ms) >= STRIKE_REFRACT_MS)
        {
          strike_count++;                     /* rising edge = one strike */
          strike_last_ms = now;
        }
        else strike_dropped++;                /* 不应期内，忽略 */
      }
      else if (n > 0) strike_dropped++;
      strike_reject = strike_dropped;

      striking = ((now - strike_last_ms) < STRIKE_HOLD_MS) ? 1u : 0u;
    }

    if (armed)
    {
      if (osc_floor > osc_floor_thr)
      {
        /* ---- 3. 触发后开 HIT_WINDOW_MS 窗口抓各通道峰值 ---- */
        for (int i = 0; i < 4; i++) { peak[i] = 0; peak_up[i] = 0; }
        uint32_t t0 = HAL_GetTick();
        while (HAL_GetTick() - t0 < HIT_WINDOW_MS)
        {
          ADC_UpdateDiagnostics();
          for (int i = 0; i < 4; i++)
          {
            if (ch_rail[i]) continue;
            float v = adc_notch_vin[i];
            vin_f[i] = lpf_update(vin_f[i], v, LPF_ALPHA);   /* 同样走滤波，保持 1ms 采样 */
            float f = adc_to_F(vin_f[i], base[i]);           /* 向下偏离量 */
            float u = adc_to_F(base[i], vin_f[i]);           /* 向上偏离量 */
            if (f > peak[i]) peak[i] = f;
            if (u > peak_up[i]) peak_up[i] = u;
          }
          HAL_Delay(1);   /* 保持 1ms 采样率，滤波系数才对应 ~16Hz 截止 */
        }

        /* ---- 4. 峰值重心 → 位置（中心原点，四象限）----
           象限映射：ch0 右上(第一)、ch1 左上(第二)、
                     ch2 左下(第三)、ch3 右下(第四)
           贴轨通道不参与：它给出的是电源轨，不是力。
           并且沿用和 strike_count 相同的"同时性"原则：至少 HIT_STRONG_CH 路的
           峰值超过各自噪声的 HIT_PEAK_K 倍，才认可这是一次共同激励的敲击。 */
        {
          volatile float *pv = peak_dir ? peak_up : peak;
          float Sp = 0;
          uint32_t live = 0, strong = 0;
          for (int i = 0; i < 4; i++)
          {
            if (ch_rail[i]) continue;
            Sp += pv[i];
            live++;
            if (pv[i] > HIT_PEAK_K * noise_sigma[i]) strong++;
          }

          if (live < 3 || strong < HIT_STRONG_CH)
          {
            /* 可用通道不足、或没有形成多路共同响应 → 丢弃，不给假坐标也不计数。 */
            hit_valid = 0;
            hit_rejected++;
          }
          else if (Sp > 1e-3f)
          {
            hit_x = (BOARD_W_MM / 2.0f) * ((pv[0] + pv[3]) - (pv[1] + pv[2])) / Sp;
            hit_y = (BOARD_H_MM / 2.0f) * ((pv[0] + pv[1]) - (pv[2] + pv[3])) / Sp;
            hit_valid = 1;
            hit_count++;
          }
        }
        armed = 0;   /* 等回落，防止同一次击打重复算 */
      }
    }
    else
    {
      /* ---- 5. 下界掉回(手指离开，低点恢复) → 重新武装 ----
       * 复位点取"参考值 → 阈值"之间的一半，避免阈值附近来回翻转。 */
      if (osc_floor < osc_floor_ref + OSC_FLOOR_REARM_FRAC * (osc_floor_thr - osc_floor_ref))
        armed = 1;
    }

    HAL_Delay(1);   /* 采样周期 1ms */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  DMA 半传输完成 —— 前半缓冲 adc_dma_buf[0 .. ADC_DMA_BLKSZ-1] 已满
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  (void)hadc;
  Strike_ProcessBlock((const uint16_t *)&adc_dma_buf[0], ADC_DMA_BLKSZ);
}

/**
  * @brief  DMA 全传输完成 —— 后半缓冲 adc_dma_buf[ADC_DMA_BLKSZ .. 2*ADC_DMA_BLKSZ-1] 已满
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  (void)hadc;
  Strike_ProcessBlock((const uint16_t *)&adc_dma_buf[ADC_DMA_BLKSZ], ADC_DMA_BLKSZ);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
