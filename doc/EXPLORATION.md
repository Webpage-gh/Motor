# 探索记录：从"让它震"到"让它唱"

> 这份文档记录的是**过程与决策**。
> 设备：Redmi Note 11（`evergo`）/ Android 13 / kernel 4.14.186 / MT6833 / root。
> 执行器：AWINIC **AW8697** LRA，i2c9 @ `0x5a`，`/dev/input/event2`（`awinic_haptic`），f0 ≈ 170Hz。

---

## 0. 一句话结论

它不是"会响的马达"，而是**一台 100~250Hz 的激振器 + 一块 1k~10k 会共振的板子**。
往 100~250Hz 喂能量 → 手上的振动；往 1k~6k 喂能量 → 耳朵里的声音；
**喂到别处的能量只会变成嗡嗡声和共振。**

---

## 1. 从触感栈倒推

```
vendor.xiaomi.hardware.vibratorfeature.service
        ↓  evdev FF
/dev/input/event2  (awinic_haptic)
        ↓  i2c
AW8697 (0x5a)
```

`EVIOCGBIT(EV_FF)` 报告可用：`FF_RUMBLE FF_PERIODIC FF_CONSTANT FF_CUSTOM FF_GAIN`，
`sizeof(struct ff_effect) = 48`，32 个 slot，**没有 `FF_SINE`** —— 这个细节后来卡了一整天。

## 2. 第一次尝试：FF_CONSTANT

能震，但发现**驱动完全忽略 `gain` 与 `effect.level`**（`level` 恒 `0x80`）。
于是"调音量"这条路堵死，只能从波形本身想办法。

## 3. "偶发不震" → `activate_mode`

| 值 | 含义 |
|---|---|
| 0 | RAM 单次（无定时器，偶发不震的元凶） |
| 1 | CONT |
| 2 | RTP |
| 3 | RAM_LOOP（稳定） |

## 4. 抓日志：真正的通路是"流式 PCM"

清空环形缓冲（`dmesg -C`）之后再抓，才看清：

```
aw8697_rtp_work_routine: effect_id = 197  state=1  activate_mode = 2
buffer size 512,  availbe size 8192
aw8697_irq: rtp update complete
```

系统播游戏音效走的是 **custom_wave 流式 PCM**（`CUSTOME_WAVE_ID = 197`），
不是"多个 RAM 单次脉冲"（早期按这个理解复刻过，听感完全不同，方向被纠正）。

## 5. 读内核源码的三处关键发现

参考 `xiaomi-mt6833-dev/kernel_xiaomi_mt6833`（`lineage-23.2`，`drivers/input/misc/aw8697_haptic/`）：

1. **`awinic_rtp_name[]` 只有一项**（`keyboard_linear_up_rtp.bin`），
   任何 `effect_id` 都被 clamp 到 0 → 永远播同一个 500B 键盘音。
2. **`is_custom_wave` 的开关**：periodic 上传且 `custom_data[0] == CUSTOME_WAVE_ID` 时置 1，
   非 periodic 上传时清 0 → 这是进出流式通路的门。
3. **`ringbuffer.c`**：写入长度为 512 整数倍才算数据，**否则等价于 EOF**；
   重新上传 periodic 会 `rb_init()` 清零读写索引。

> 源码是 lineage 分支，不是出厂内核，行号未必对应。
> 用法是"源码提假设，dmesg 给证据"。

## 6. 打通 FF_PERIODIC（最卡的一步）

`EVIOCSFF(FF_PERIODIC)` 一直返回 `EINVAL`。真因**不是** `effects_count == 0`，
而是 `ff-core` 会校验 `periodic.waveform` 必须在设备 ffbit 内——本机没有 `FF_SINE`。

```c
e.u.periodic.waveform   = FF_CUSTOM;   /* 0x5d */
e.u.periodic.custom_len = 2;
e.u.periodic.custom_data[0] = 197;
```

改用 `FF_CUSTOM` 立刻成功。播 `161_gameboxLaunch_RTP.bin`（55293B），
确认就是系统"游戏空间"启动时的汽车声——**从这一刻起，马达就是一个 PCM 播放器**。

## 7. 参数标定

| 项目 | 结论 | 来源 |
|---|---|---|
| 采样率 | ≈ **24000 B/s**，8bit 有符号单声道 | 写入耗时 ≈ 播放时长 |
| 音量 | `0x7FFF→0x80`、`0x5FFF→64`、`0x3FFF→0x1E` | 源码 + 实测 |
| 缓冲 | `period=512`，`max=8192` | 读 `custom_wave` |
| 写满 | 阻塞，不丢数据 | 实测 |

## 8. 收尾（缺一不可）

早期症状：尾巴被砍、"震几下就不震"、把系统触感弄坏。修法：

```
停播 → 重新上传 periodic(197) 触发 rb_init() 清索引
     → 上传并撤销 FF_CONSTANT（清 is_custom_wave）
     → activate=0 / activate_mode=0
     → 等 free_size 回到 max_size
```

**教训**：播放被中途打断（Ctrl-C / 超时 / 被杀）必须补一次复位，
否则驱动停在半播状态，之后所有播放都会出现"共振 + 混乱高频"。

## 9. 频响：它到底能抖多快

早期用听感做粗测：200Hz 强、250Hz 极弱、300Hz 无、600Hz 无、100Hz 难辨、50Hz 几乎无
→ 机械振动带 ≈ **150~230Hz**，峰值 170~200Hz，属机械限制。

后来用 `sweep` + `reference/频率-振幅测试表.csv` 做了系统标定（见 `README.md` 安全层）。

## 10. 发声：板子在替你唱

把简谱映射到 1.6k~4.7kHz 播放，主观感受是"**不用靠近就能听见**"。
听感打分（纯音，1~20 分）：

| 频率 | 得分 |
|---|---|
| **2401Hz** | **20** |
| 4601 / 7201Hz | 15 |
| 6201Hz | 6（谷） |
| 7400~10600Hz | 13（平坦） |
| 1~400Hz | 几乎听不到 |

结论：**发声来自机身板子的声学共振，不是马达"发声"**。这也是后来"人声能听清"的物理基础。

## 11. 失败记录：直接播音乐 / MIDI

| 尝试 | 结果 |
|---|---|
| 整首 Phigros 原频率播放（带安全 EQ） | "能听到鼓点，但糊成一团" |
| 音高 +2 八度 | "像手机坏掉了" |
| MIDI 抽"最高音"当旋律 | "奇怪 + 共振" |
| MIDI 按通道抽旋律（排除鼓） | "还是奇怪" |

**定量归因**（关键一课）：

| 指标 | 厂商 161 汽车声 | 整首音乐 | MIDI 旋律 |
|---|---|---|---|
| RMS | 15.2 | 33.1 | 18.6 |
| 波峰因数 | 8.3 | 3.8 | 3.2 |
| 静音占比 | 33% | 3.5% | 12.5% |
| 能量 < 600Hz | **99.6%** | 70.7% | **0.3%** |

厂商素材把能量全放在马达转得动的频段，靠"短促大峰值 + 大量静音"；
直接播音乐则把 29%~99.6% 的能量喂给了 600Hz 以上——**动子跟不上，只能嗡嗡 + 带动机身共振**。

**结论**：不是"马达放不了音乐"，是"多声部宽频内容不适合当激励源"。
于是引出主线方案：**先提取主旋律 → 单声部 → 再播**（`extract` + `mplay.sh`）。

## 12. 主线方案成立

`extract`（STFT 谐波掩码提主旋律）→ `mplay.sh`（解码、量化、安全层、播放）。
两个实测样本：

| 素材 | 主音高覆盖 | 音域 | RMS | 听感 |
|---|---|---|---|---|
| Hans Zimmer – Cornfield Chase | 92% | 2.9 八度 | 20.6 | "效果不算好，但够用" |
| 周深 – 亲爱的旅人啊 | 80% | **1.4 八度** | 42.1 | "**快能听见歌词了**" |

差异的原因：人声是单声部、连续、音域窄，且它的**高次谐波落在板子的共振区**——
真正"播"元音辅音的是机身，不是马达。

后续的调音实验见 **`TUNING.md`**。

## 13. 可复用教训

1. 设备的 `ffbit` 决定 `EVIOCSFF` 能否通过；报 `EINVAL` 先查 ffbit，别猜驱动内部状态。
2. 抓日志前先 `dmesg -C`，否则会被旧日志误导。
3. 别 `pkill -f "dmesg -w"`（会杀掉自己的 shell），按 PID kill。
4. `su -c '~/xxx'` 不展开 `~`，root 侧一律写绝对路径。
5. 播放被打断必须补复位，否则症状是"共振 + 混乱高频"。
6. 安全余量只乘在共振带内；带外乘会误伤厂商素材。
7. "能不能响"和"算不算振动"是两件事：>600Hz 是声学共振，不是动子运动。
8. 长期高强度测试会改变 `f0_cali_lra`（观测到 5 → 61），开工前先复测。
