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
| RMS | 持续音 ≤ 15 | 整体缩放 |
| 瞬态 | 每样本跳变 ≤ max(40, 局部 RMS)（前视 64 样本） | 压该段增益（腐蚀提前下压 + 平滑恢复）：满幅 click 127 → 40 |
| 时长 | 默认**不截断**（共振是逐块检查的，不因时长变弱） | 超过 10s 且 RMS>10 时只提示发热；`--max-ms N` 会真截断并把尾巴清零 |

实测校验（`motor check`，不播放）：

```
170Hz 满幅 0x7FFF → 共振带 170Hz 压到 20%，RMS → ×0.69
170Hz 满幅 0x5FFF → 共振带 170Hz 压到 39%，RMS → ×0.40   ← 档位放宽生效
540Hz 满幅 0x7FFF → 共振带 539Hz 压到 12%
满幅单样本 click     → 瞬态压到 31%（跳变 127 → 40）
300Hz 正弦 peak 15   → 通过 (RMS 10.7, 最大跳变 2)          ← 正弦不误伤
`--max-ms 1000` 的 8s 文件 → 长度 8.00s → 1.00s（真截断）
```

```bash
motor check <file.bin> [mag]      # 只体检，不振动（不需要 root）
motor --max-ms 5000 play ...      # 真正截断到 5 秒
motor --no-safety play ...        # 跳过安全层（不建议）
```

已知缺口：安全层只管 152~581Hz 两个共振带与跳变/RMS，**不检测 DC 偏置**
（常数 10 的文件会被判"通过"，静态推力会持续加热线圈），也不管 >581Hz
的声学共振。自己合成时先 `motor check`，别赌。

## 6. 安全三条（血泪，写给写波形的人）

1. **只用正弦**。方波/锯齿的陡边是宽频激励，会撞壳。
2. **避开共振带**：`152~190Hz` 与 `485~581Hz` 是机身共振区，满档安全峰值分别只有
   `31`（170Hz）和 `18`（540Hz）。带内振幅要压到 ~20%，否则"敲壳"。
   安全层会自动压，但压完基本只剩"微振"。
3. **自己合成时别超 RMS 15**；满幅 170Hz 持续正弦会撞限位。
   注意 RMS 上限**不随档位放宽**：`0x3FFF` 是"硬件压低 × PCM 压低"的双重衰减。
4. **别用 click 当鼓点**。安全层会把单个满幅跳变压到 40，但连续瞬态依旧是宽频激励。

单次播放建议 ≤5 秒；长音频先切片。

## 7. 高频的事实

马达**不是扬声器**：>600Hz 它能"发声"（靠机身声学共振，2.4kHz 最响），
但多声部/宽频带内容只会变成"坏掉的手机"。可用的只有：
单声部纯音（能听出旋律）、低频包络（能摸到节奏）、瞬态（鼓点）。
