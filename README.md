# motor —— 最小马达调用库

> Redmi Note 11 (`evergo`) / AWINIC **AW8697** LRA / `/dev/input/event2`。
> 一个 C 文件 + 一条编译命令，能直接把任意 8bit PCM 喂给马达。
> **必须 root**（`/dev/input/event2` 与 sysfs 都只有 root 可写）。

## 1. 30 秒上手

```bash
cd ~/motor
./build.sh                                  # 产出 ./motor

su -c '~/motor/motor tone 600 1000 60'      # 600Hz 响 1 秒
su -c '~/motor/motor play ~/xxx.bin 0x5FFF' # 播一段 PCM
su -c '~/motor/motor stop'                  # 立即静音
```

> Termux 里 `su -c` 内是 `/system/bin/sh`，**不会展开 `~`**，
> 所以给 `su` 的路径要写全：`/data/data/com.termux/files/home/motor/motor`。

## 2. 库接口（只有 4 个函数）

```c
int  motor_open(void);                                       // 0 = 成功
int  motor_play(const signed char *pcm, size_t n, int mag);  // 阻塞播放，自动收尾
int  motor_tone(double hz, int ms, int peak, int mag);       // 内置正弦
void motor_stop(void);                                        // 立即静音
```

- `pcm`：**8bit 有符号、单声道、24000 Hz**（1 秒 = 24000 字节）
- `peak`：样本峰值 0~127
- `mag`：`0x7FFF` = 100%，`0x5FFF` = 50%，`0x3FFF` ≈ 25%（见 §4）

## 3. 协议（踩了三天坑才通的唯一正门）

```
1) ioctl(EVIOCSFF, FF_PERIODIC)
     .u.periodic.waveform    = FF_CUSTOM (0x5d)   ← 设备 ffbit 里没有 FF_SINE，
     .u.periodic.custom_len  = 2                     用 FF_SINE/FF_SQUARE 一律 EINVAL
     .u.periodic.custom_data = {197, 0, 0, 0}     ← 197 = CUSTOME_WAVE_ID，进流式通路
     .u.periodic.magnitude   = mag                ← 等价 vmax，就是"全局音量"
2) write(/dev/input/event2, {EV_FF, id, 1})       ← 开播
3) open 写 /sys/.../9-005a/custom_wave，每 512 字节 write() 一次（写满会阻塞，不丢数据）
4) 轮询读 custom_wave 的 free_size，回到 max_size(8192) 才算播完
5) 立刻再写 8×512 字节零（≈171ms 静音尾巴）
6) 收尾：EV_FF stop → EVIOCRMFF → 重新上传 periodic(197) 让内核 rb_init() 清索引
        → 上传并撤销一个 FF_CONSTANT（清 is_custom_wave）→ activate=0, activate_mode=0
```

**第 6 步不能省**：跳过它，驱动会留在半播状态，之后所有播放都会出现
"共振 + 混乱高频"（芯片里残留的旧数据被冲出来）。

## 4. 音量 `mag` → 驱动 `level`

| mag | level | 相对音量 |
|---|---|---|
| `0x7FFF` | `0x80` | 100% |
| `0x5FFF` | 64 | 50% |
| `0x3FFF` | `0x1E` | ~23% |

换算：`level = (mag - 16383) / 128`，`mag >= 0x7FFF` 取 `0x80`，`<= 0x3FFF` 取 `0x1E`。
`mag` 只影响振幅，**不影响芯片内部 F0 与波形**。

## 5. 内置安全层（自动，默认开启）

`motor_play()` / `motor_tone()` 在**上传前**自动对 PCM 做一遍处理，规则全部来自实测：

| 检查 | 规则 | 触发后 |
|---|---|---|
| 共振带 | 152~190Hz / 485~581Hz 的安全峰值表 | 逐块测量，把该段按比例压下去（平滑过渡，不产生新陡边） |
| 档位放宽 | 低档位按 `×128/level` 线性放宽 | `0x5FFF` 时允许值翻倍 |
| 带内余量 | 带内再 `×0.8`（带外不乘，避免误伤厂商素材） | — |
| RMS | **按 2048 样本分块判**：带内 ≤ 15，带外 ≤ 90（= peak 127） | 块内缩放 + 块间过渡。带外不用 15 压：实测表允许满幅，压到 peak 21 属误伤 |
| 瞬态 | 每样本跳变 ≤ max(40, 局部 RMS)（前视 64 样本） | 压该段增益（腐蚀提前下压 + 平滑恢复）：满幅 click 127 → 40 |
| 能量预算 | `剂量 = Σ rms²·dt > 121500`（≈满幅 15s）或 `>300s` | **事前拒绝播放**（退出码 3），设备一个字节都不写；`--force` 绕过，`--max-ms` 截短 |

实测校验（`motor check`，不播放）：

```
170Hz 满幅 0x7FFF → 共振带 170Hz 压到 20%，RMS 17.1 → ×0.88
170Hz 满幅 0x5FFF → 共振带 170Hz 压到 39%，RMS 34.7 → ×0.43   ← 档位放宽生效
540Hz 满幅 0x7FFF → 共振带 539Hz 压到 12%
540Hz 满幅 0x5FFF → 共振带 539Hz 压到 23%，RMS 20.3 → ×0.74
600Hz 满幅 127 / 300ms → 通过 (RMS 90.0, 最大跳变 20)    ← 带外放行
600Hz 300ms peak 100     → 通过（此前被 RMS 15 压到 peak 21）
满幅单样本 click     → 瞬态压到 31%（跳变 127 → 40）
0.5s 满幅 300Hz + 59.5s 静音 → RMS 89.9 → ×0.17（分块判才抓得到）
300Hz 正弦 peak 15   → 通过 (RMS 10.7, 最大跳变 2)          ← 正弦不误伤
`--max-ms 1000` 的 8s 文件 → 长度 8.00s → 1.00s（真截断）
```

```bash
motor check <file.bin> [mag]      # 只体检，不振动（不需要 root）
motor sweep <f0> <f1> <ms> [peak] [mag] [lin]   # 扫频，默认对数
motor --dry sweep 40 1200 4000   # 扫频只体检（普通用户可跑）
motor --max-ms 5000 play ...      # 真正截断到 5 秒（非负整数，非法值报错退出）
motor --force play long.bin       # 绕过能量预算/时长上限（超量时才需要）
motor --no-safety play ...        # 跳过安全层（不建议）
```

**扫频 `sweep`**

```bash
motor sweep 40 1200 4000          # 40→1200Hz，4 秒，对数（每倍频程等时）
motor sweep 1200 40 4000 60       # 反向扫频，峰值 60
motor sweep 200 1000 4000 100 lin # 线性扫频（等 Hz/秒）
```

- 对数默认：低频段停留更久，探共振/听死角更公平；线性适合找某一窄带；
- 自带 20ms 起落，不会因为切口产生 click；
- **同样过安全层**：扫过 152–190 / 485–581 时会被自动压制（这是特性不是 bug，
  想听“未处理的原始扫频”得加 `--no-safety`，但那会真的撞共振带）。

**健壮性约定**

- 安全检查内存不足时 **`motor_play()` 返回 -2 并拒绝播放**（不会"放行未检查的内容"）
- 瞬态限幅用 `float`，3 条缓冲共 `3×4×n` 字节（22s ≈ 2.6MB）
- 报告拼接有长度钳制，不会栈溢出

**已知行为**：>600Hz 的满幅内容若跳变超过"局部 RMS"，会被瞬态规则连带压下去
（例如 8kHz/峰值60 → 压到 ~34%）。这条优先于"带外一律 127"，因为高频满幅本身
就是嗡嗡声而不是振动。

```
```

能量预算为什么是"事前拒绝"而不是"越放越小声"：剂量在预处理里一次算完，
超了直接不播（`退出码 3`），所以不会出现"响一段突然变弱"；想放长就用
`--max-ms` 截到预算内，或者 `--force` 自己担。

已知缺口：安全层只管 152~581Hz 两个共振带与跳变/RMS（带外 RMS 上限 90，
发热由能量预算控制），超低频由 25Hz 二级高通压掉
（常数 10 的文件会被判"通过"，静态推力会持续加热线圈），也不管 >581Hz
的声学共振。自己合成时先 `motor check`，别赌。

## 6. 安全三条（血泪，写给写波形的人）

1. **只用正弦**。方波/锯齿的陡边是宽频激励，会撞壳。
2. **避开共振带**：`152~190Hz` 与 `485~581Hz` 是机身共振区，满档安全峰值分别只有
   `31`（170Hz）和 `18`（540Hz）。带内振幅要压到 ~20%，否则"敲壳"。
   安全层会自动压，但压完基本只剩"微振"。
3. **带内**别超 RMS 15（满幅 170Hz 持续正弦会撞限位）；**带外**按实测表可到
   peak 127，RMS 上限随之放到 90。
4. **别用 click 当鼓点**。安全层会把单个满幅跳变压到 40，但连续瞬态依旧是宽频激励。

单次播放建议 ≤5 秒；长音频先切片。

## 7. 高频的事实

马达**不是扬声器**：>600Hz 它能"发声"（靠机身声学共振，2.4kHz 最响），
但多声部/宽频带内容只会变成"坏掉的手机"。可用的只有：
单声部纯音（能听出旋律）、低频包络（能摸到节奏）、瞬态（鼓点）。
