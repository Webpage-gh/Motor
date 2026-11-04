/* motor.c —— 最小马达调用库（AW8697 / custom_wave 流式 PCM）
 *
 * 用法:
 *   motor tone <Hz> <ms> [peak] [mag]     内置正弦，直接放
 *   motor play <file.bin> [mag]           播放 8bit 有符号单声道 PCM（24kHz）
 *   motor stop                            立即静音（紧急用，不做收尾）
 *
 * 库接口:
 *   int  motor_open(void);
 *   int  motor_play(const signed char *pcm, size_t n, int mag);   // 阻塞，自动收尾
 *   int  motor_tone(double hz, int ms, int peak, int mag);
 *   void motor_stop(void);
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <linux/input.h>

#define EVDEV   "/dev/input/event2"
#define SYS     "/sys/devices/platform/11d01000.i2c9/i2c-9/9-005a"
#define CW_ID   197      /* CUSTOME_WAVE_ID：写进 custom_data[0] 才会进流式通路 */
#define PERIOD  512      /* 芯片每次取走的字节数 */
#define MAXBUF  8192     /* 环形缓冲大小 */
#define SR      24000    /* 采样率（实测 ≈24kB/s） */

#ifndef FF_CUSTOM
#define FF_CUSTOM 0x5d
#endif

static int g_fd = -1;
static int g_safety = 1;      /* 1=自动安全检查（默认） */
static int g_max_ms = 0;      /* 0=不截断（默认）；可 --max-ms 指定 */

/* ================= 安全层 =================
 * 规则全部来自实测，不含新东西：
 *  1) 共振带安全峰值表（mag=0x7FFF / level=128 时测得）
 *  2) 档位放宽：低档位按 ×128/level 线性放宽
 *  3) 带内再乘 0.8 余量（带外不乘，否则误伤厂商素材）
 *  4) 持续音 RMS ≤ 15
 *  5) 瞬态：每样本跳变 ≤ max(40, 局部 RMS)（满幅 click 127 → 40）
 *  6) 时长：默认不截断，超过 10s 且 RMS>10 只提示；--max-ms N 真截断并清零尾巴
 *  7) 已知缺口：不检测 DC 偏置，也不管 >581Hz 的声学共振
 */
typedef struct { int f, pk; } SafePt;
static const SafePt SAFE[] = {
    {152,127},{153,122},{155,101},{160,77},{165,52},{170,31},
    {180,68},{185,94},{189,118},{190,127},
    {485,127},{490,109},{500,79},{510,53},{525,30},{530,26},{535,19},
    {540,18},{545,25},{550,34},{555,44},{560,54},{570,83},{575,104},{581,127}
};
#define NSAFE ((int)(sizeof SAFE / sizeof SAFE[0]))
#define RMS_CAP 15.0
#define SLEW_MIN 40.0   /* 允许的每样本最大跳变（厂商素材实测 ≤10） */

static int level_of(int mag)
{
    if (mag >= 0x7fff) return 0x80;
    if (mag <= 0x3fff) return 0x1e;
    return (mag - 16383) / 128;
}

/* 该频率允许的样本峰值（已含档位放宽与带内余量） */
static double safe_limit(double f, int level)
{
    double v = 127.0, relax = 128.0 / (level > 0 ? level : 1);
    int inband = 0;
    if (f >= SAFE[0].f && f <= SAFE[NSAFE-1].f) {
        for (int i = 0; i < NSAFE - 1; i++) {
            if (f >= SAFE[i].f && f <= SAFE[i+1].f) {
                double t = (f - SAFE[i].f) / (double)(SAFE[i+1].f - SAFE[i].f);
                v = SAFE[i].pk + (SAFE[i+1].pk - SAFE[i].pk) * t;
                inband = 1;
                break;
            }
        }
        /* 落在两个共振带之间的空隙（190~485）不算带内 */
        if (f > 190.0 && f < 485.0) inband = 0;
    }
    v = v * relax;
    if (inband) v *= 0.8;
    return v > 127.0 ? 127.0 : v;
}

/* Hann 窗 Goertzel 幅度估计（返回该频率的正弦幅度） */
static double goertzel_amp(const signed char *x, int n, double f)
{
    double w = 2.0 * M_PI * f / SR;
    double coeff = 2.0 * cos(w), s0 = 0, s1 = 0, s2 = 0;
    for (int i = 0; i < n; i++) {
        double win = 0.5 - 0.5 * cos(2.0 * M_PI * i / (n - 1));
        s0 = win * x[i] + coeff * s1 - s2;
        s2 = s1; s1 = s0;
    }
    double p = s1 * s1 + s2 * s2 - coeff * s1 * s2;
    if (p < 0) p = 0;
    return 2.0 * sqrt(p) / (0.5 * n);     /* Hann 相干增益 0.5 */
}

/* 扫描共振带：返回需要的最小增益，并记录肇事频率 */
static double band_scan(const signed char *x, int n, int level, double *worst_f, double *worst_a)
{
    double g = 1.0;
    for (double f = 152.0; f <= 190.0; f += 2.0) {
        double a = goertzel_amp(x, n, f), lim = safe_limit(f, level);
        if (a > lim) { double k = lim / a; if (k < g) { g = k; if (worst_f) *worst_f = f; } }
        if (worst_a && a > *worst_a) *worst_a = a;
    }
    for (double f = 485.0; f <= 581.0; f += 2.0) {
        double a = goertzel_amp(x, n, f), lim = safe_limit(f, level);
        if (a > lim) { double k = lim / a; if (k < g) { g = k; if (worst_f) *worst_f = f; } }
        if (worst_a && a > *worst_a) *worst_a = a;
    }
    return g;
}

static signed char clip127(double v)
{
    if (v > 127.0) v = 127.0;
    if (v < -128.0) v = -128.0;
    return (signed char)lrint(v);
}

/* 追加一段报告文字：used 是“已写长度”，绝不越界 */
static size_t rep_add(char *msg, size_t cap, size_t used, const char *fmt, ...)
{
    if (used + 1 >= cap) return cap - 1;
    va_list ap;
    va_start(ap, fmt);
    int r = vsnprintf(msg + used, cap - used, fmt, ap);
    va_end(ap);
    if (r < 0) return used;
    size_t nu = used + (size_t)r;
    return (nu < cap) ? nu : cap - 1;
}

/* ---- 瞬态限幅（治"撞壳"的 click） ----
 * 纯正弦的每样本最大跳变 ≈ 2πf/SR × peak：2.4kHz 满幅也只有 0.62×RMS；
 * 静音里单个满幅样本的跳变却是 127，宽频瞬态远超正弦。故：
 *   允许步长 = max(SLEW_MIN, 前视 64 样本的局部 RMS)
 * 需要的增益先做 ±16 样本最小值腐蚀（提前下压），再 32 点盒式平滑（不造新陡边）。
 * 返回最小增益（1.0 = 未处理）；**返回 -1 = 内存不足，调用方必须拒绝播放**。
 */
static double transient_guard(signed char *pcm, size_t n, int *maxj_out)
{
    const int LOOK = 64, WIN = 128, ERODE = 16, SMOOTH = 32;

    if (n < 2) { if (maxj_out) *maxj_out = 0; return 1.0; }

    float *e = malloc(n * sizeof *e);    /* 过去的局部 RMS */
    float *g = malloc(n * sizeof *g);    /* 需要的增益 / 平滑后的增益 */
    float *s = malloc(n * sizeof *s);    /* 腐蚀后的增益 */
    if (!e || !g || !s) { free(e); free(g); free(s); return -1.0; }   /* 失败不放行 */

    /* 前视 LOOK 的滑动 RMS：窗口 = [i-LOOK-WIN+1, i-LOOK] */
    double acc = 0;
    for (size_t i = 0; i < n; i++) {
        long end = (long)i - LOOK;
        if (end < 0) { e[i] = 0; continue; }
        acc += (double)pcm[end] * pcm[end];
        long beg = end - WIN + 1;
        if (beg - 1 >= 0) acc -= (double)pcm[beg - 1] * pcm[beg - 1];
        if (acc < 0) acc = 0;
        double cnt = (double)(end - (beg < 0 ? 0 : beg) + 1);
        e[i] = sqrt(acc / (cnt < 1 ? 1 : cnt));
    }

    /* 每个样本需要的增益 */
    for (size_t i = 0; i < n; i++) {
        double step = i ? fabs((double)pcm[i] - (double)pcm[i-1]) : fabs((double)pcm[i]);
        double allow = (e[i] > SLEW_MIN) ? e[i] : SLEW_MIN;
        g[i] = (step > allow) ? allow / step : 1.0;
    }

    /* 最小值腐蚀：增益在 click 到来之前就降下去 */
    double ming = 1.0;
    for (size_t i = 0; i < n; i++) {
        size_t a = (i > (size_t)ERODE) ? i - ERODE : 0;
        size_t b = (i + ERODE < n) ? i + ERODE : n - 1;
        double m = 1.0;
        for (size_t j = a; j <= b; j++) if (g[j] < m) m = g[j];
        s[i] = m;
        if (m < ming) ming = m;
    }

    if (ming < 1.0) {                     /* 需要压：盒式平滑 + 施加 */
        double run = 0;
        for (size_t i = 0; i < n; i++) {
            run += s[i];
            if (i >= (size_t)SMOOTH) run -= s[i - SMOOTH];
            double cnt = (double)((i < (size_t)SMOOTH) ? i + 1 : SMOOTH);
            /* 取 min(平滑值, 腐蚀值)：下压不因平滑被"平均"回去，恢复才是缓的 */
            g[i] = (run / cnt < s[i]) ? run / cnt : s[i];
        }
        ming = 1.0;
        for (size_t i = 0; i < n; i++) {
            pcm[i] = clip127((double)pcm[i] * g[i]);
            if (g[i] < ming) ming = g[i];
        }
    }

    if (maxj_out) {
        int mj = 0;
        for (size_t i = 1; i < n; i++) {
            int d = abs(pcm[i] - pcm[i-1]);
            if (d > mj) mj = d;
        }
        *maxj_out = mj;
    }
    free(e); free(g); free(s);
    return ming;
}

/* 就地安全处理：分块测共振带 → 限 RMS → 限瞬态 → 时长截断。
 * *np 传入长度、回写生效长度；返回 1 = 有改动 */
static int motor_sanitize(signed char *pcm, size_t *np, int mag, char *rep, size_t replen)
{
    size_t n = *np;
    int level = level_of(mag);
    const int NF = 2048, HOP = 1024;
    double ming = 1.0, worst_f = 0;
    int changed = 0, cut = 0;

    /* 时长截断：必须真的截掉，并清零尾部（否则尾巴会以原幅裸奔） */
    if (g_max_ms > 0 && (int)n > g_max_ms * SR / 1000) {
        size_t keep = (size_t)g_max_ms * SR / 1000;
        memset(pcm + keep, 0, n - keep);
        n = keep;
        changed = cut = 1;
    }

    /* 逐块扫描并施加带平滑的增益 */
    if ((int)n >= NF) {
        size_t pos = 0; double prevg = 1.0;
        while (pos < n) {
            int len = (int)((n - pos > (size_t)NF) ? NF : (n - pos));
            double wf = 0, wa = 0;
            double g = (len >= 512) ? band_scan(pcm + pos, len, level, &wf, &wa) : 1.0;
            if (g < 1.0) { changed = 1; if (g < ming) { ming = g; worst_f = wf; } }
            size_t blk = (size_t)HOP; if (pos + blk > n) blk = n - pos;
            for (size_t i = 0; i < blk; i++) {
                double t = (double)i / (double)blk;
                double gg = prevg + (g - prevg) * t;
                double v = pcm[pos + i] * gg;
                pcm[pos + i] = (signed char)(v > 127 ? 127 : (v < -128 ? -128 : v));
            }
            prevg = g;
            pos += blk;
        }
    }

    /* 持续音 RMS 上限 */
    double s2 = 0;
    for (size_t i = 0; i < n; i++) s2 += (double)pcm[i] * pcm[i];
    double rms = sqrt(s2 / (n ? n : 1));
    char rmsmsg[96] = "";
    if (rms > RMS_CAP) {
        double k = RMS_CAP / rms;
        for (size_t i = 0; i < n; i++) pcm[i] = (signed char)lrint(pcm[i] * k);
        snprintf(rmsmsg, sizeof rmsmsg, "RMS %.1f>%.0f → ×%.2f", rms, RMS_CAP, k);
        changed = 1;
        rms = RMS_CAP;
    }

    /* 瞬态：厂商素材最大跳变仅 10，我们合成的 click 能到 127 → 这里直接压 */
    int maxj = 0;
    double tg = transient_guard(pcm, n, &maxj);
    if (tg < 0) {
        if (rep) snprintf(rep, replen, "安全检查失败（内存不足），拒绝播放");
        *np = n;
        return -1;
    }
    if (tg < 1.0) changed = 1;

    /* 长时播放只提示（共振是逐块检查的，不因时长变弱）；时长只关系发热 */
    if ((int)n > 10 * SR && rms > 10.0 && rep)
        printf("[提示] 时长 %.0fs 且 RMS %.1f，注意马达发热（可 --max-ms 限制）\n",
               (double)n / SR, rms);

    if (rep) {
        char msg[256] = "";
        size_t used = 0;
        if (cut) used = rep_add(msg, sizeof msg, used, "截断到 %dms，", g_max_ms);
        if (ming < 1.0)
            used = rep_add(msg, sizeof msg, used,
                           "共振带 %.0fHz 压到 %.0f%%，", worst_f, ming * 100);
        if (tg < 1.0)
            used = rep_add(msg, sizeof msg, used,
                           "瞬态压到 %.0f%%（跳变 %d），", tg * 100, maxj);
        if (rmsmsg[0]) used = rep_add(msg, sizeof msg, used, "%s，", rmsmsg);
        if (used == 0) {
            snprintf(msg, sizeof msg, "通过 (RMS %.1f, 最大跳变 %d)", rms, maxj);
        } else if (used >= 3 &&
                   (unsigned char)msg[used-3] == 0xEF &&
                   (unsigned char)msg[used-2] == 0xBC &&
                   (unsigned char)msg[used-1] == 0x8C) {     /* 去掉末尾全角逗号 */
            used -= 3;
            msg[used] = 0;
        }
        snprintf(rep, replen, "%s", msg);
    }
    *np = n;
    return changed;
}

static void nap(int ms)
{
    struct timespec ts = { ms / 1000, (long)(ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
}

/* custom_wave 返回: period_size=512;max_size=8192;free_size=N;custom_wave_id=197; */
static int cw_free(void)
{
    char b[192] = {0};
    int fd = open(SYS "/custom_wave", O_RDONLY);
    if (fd < 0) return -1;
    int n = read(fd, b, sizeof b - 1);
    close(fd);
    if (n <= 0) return -1;
    int p, m, f = -1;
    if (sscanf(b, "period_size=%d;max_size=%d;free_size=%d;", &p, &m, &f) != 3) return -1;
    return f;
}

static void sys_write(const char *name, const char *val)
{
    char path[256];
    snprintf(path, sizeof path, "%s/%s", SYS, name);
    int fd = open(path, O_WRONLY);
    if (fd >= 0) { if (write(fd, val, strlen(val)) < 0) {} close(fd); }
}

/* 关键：FF_PERIODIC + waveform=FF_CUSTOM(0x5d) + custom_len=2 + custom_data[0]=197
 * 设备 ffbit 里没有 FF_SINE，用别的 waveform 会 EINVAL */
static int upload_periodic(int mag)
{
    struct ff_effect e;
    static short data[4] = { CW_ID, 0, 0, 0 };
    memset(&e, 0, sizeof e);
    e.type = FF_PERIODIC;
    e.id = -1;
    e.u.periodic.waveform = FF_CUSTOM;
    e.u.periodic.period = 0;
    e.u.periodic.magnitude = mag;      /* = vmax，驱动据此换算 level（全局音量） */
    e.u.periodic.custom_len = 2;
    e.u.periodic.custom_data = data;
    if (ioctl(g_fd, EVIOCSFF, &e) < 0) {
        fprintf(stderr, "EVIOCSFF 失败: %s\n", strerror(errno));
        return -1;
    }
    return e.id;
}

static void ff_play(int id, int on)
{
    struct input_event ev;
    memset(&ev, 0, sizeof ev);
    ev.type = EV_FF;
    ev.code = id;
    ev.value = on;
    if (write(g_fd, &ev, sizeof ev) < 0) perror("EV_FF");
}

/* 上传一个 FF_CONSTANT 再撤销 → 驱动在此路径把 is_custom_wave 清 0，退出流式状态 */
static void clear_custom_flag(void)
{
    struct ff_effect e, rm;
    memset(&e, 0, sizeof e);
    e.type = FF_CONSTANT;
    e.id = -1;
    e.u.constant.level = 0x1000;
    if (ioctl(g_fd, EVIOCSFF, &e) == 0) {
        memset(&rm, 0, sizeof rm);
        rm.id = e.id;
        ioctl(g_fd, EVIOCRMFF, &rm);
    }
}

/* 收尾：不清理会把驱动留在半播状态，导致后续"共振 / 混乱高频" */
static void cleanup(int id)
{
    ff_play(id, 0);
    nap(60);
    { struct ff_effect rm; memset(&rm, 0, sizeof rm); rm.id = id; ioctl(g_fd, EVIOCRMFF, &rm); }
    /* 重新上传 periodic → 内核 rb_init()，wr/rd 索引清零，冲掉残留数据 */
    int id2 = upload_periodic(0x7fff);
    if (id2 >= 0) {
        struct ff_effect rm; memset(&rm, 0, sizeof rm); rm.id = id2;
        ioctl(g_fd, EVIOCRMFF, &rm);
    }
    nap(30);
    clear_custom_flag();
    sys_write("activate", "0");
    sys_write("activate_mode", "0");
    for (int i = 0; i < 150 && cw_free() < MAXBUF; i++) nap(20);
}

int motor_open(void)
{
    g_fd = open(EVDEV, O_RDWR);
    if (g_fd < 0) { perror("open " EVDEV); return -1; }
    return 0;
}

void motor_stop(void)
{
    sys_write("activate", "0");
}

int motor_play(const signed char *pcm, size_t n, int mag)
{
    signed char *buf = NULL;
    if (g_safety) {
        /* 安全检查：共振带 → RMS → 瞬态 → 时长截断；返回值是生效长度 */
        size_t cap = n;
        buf = malloc(cap ? cap : 1);
        if (!buf) return -1;
        memcpy(buf, pcm, cap);
        char rep[256] = "";
        int sr = motor_sanitize(buf, &n, mag, rep, sizeof rep);
        printf("[安全] %s\n", rep);
        if (sr < 0) { free(buf); return -2; }   /* 安全检查失败 → 不播 */
        pcm = buf;
    }

    int id = upload_periodic(mag);
    if (id < 0) { free(buf); return -1; }
    int cw = open(SYS "/custom_wave", O_WRONLY);
    if (cw < 0) { perror("open custom_wave"); free(buf); return -1; }

    ff_play(id, 1);
    const unsigned char *p = (const unsigned char *)pcm;
    size_t done = 0;
    while (done < n) {
        unsigned char buf[PERIOD];
        size_t c = (n - done > PERIOD) ? PERIOD : (n - done);
        memset(buf, 0, PERIOD);          /* 末尾不足 512 用零补齐 */
        memcpy(buf, p + done, c);
        if (write(cw, buf, PERIOD) < 0) { perror("write custom_wave"); break; }
        done += c;
    }
    /* 静音尾巴 8×512B ≈171ms：让停止落在静音里，避免"硬切+刹车脉冲"的杂音 */
    {
        unsigned char z[PERIOD];
        memset(z, 0, sizeof z);
        for (int i = 0; i < 8; i++) if (write(cw, z, PERIOD) < 0) break;
    }
    close(cw);
    /* 真正的"播完"= free_size 回到 max_size（不能靠猜时间） */
    for (int i = 0; i < 750 && cw_free() < MAXBUF; i++) nap(20);
    nap(250);
    cleanup(id);
    free(buf);
    return 0;
}

int motor_tone(double hz, int ms, int peak, int mag)
{
    int n = (int)((long long)SR * ms / 1000);
    signed char *b = malloc(n);
    if (!b) return -1;
    int fade = SR * 20 / 1000;                 /* 20ms 起落，去掉陡边（陡边会撞壳） */
    if (fade * 2 > n) fade = n / 2;
    for (int i = 0; i < n; i++) {
        double t = (double)i / SR;
        double e = 1.0;
        if (i < fade) e = (double)i / fade;
        else if (i > n - fade) e = (double)(n - i) / fade;
        b[i] = (signed char)lrint(peak * e * sin(2 * M_PI * hz * t));
    }
    int r = motor_play(b, n, mag);
    free(b);
    return r;
}

int main(int argc, char **argv)
{
    /* 前置选项：--max-ms N 与 --no-safety 可任意顺序、可重复 */
    while (argc > 1) {
        if (!strcmp(argv[1], "--max-ms") && argc > 2) {
            g_max_ms = atoi(argv[2]);
            for (int i = 1; i < argc - 2; i++) argv[i] = argv[i+2];
            argc -= 2;
        } else if (!strcmp(argv[1], "--no-safety")) {
            g_safety = 0;
            for (int i = 1; i < argc - 1; i++) argv[i] = argv[i+1];
            argc--;
        } else break;
    }
    const char *cmd = argc > 1 ? argv[1] : "";
    int mag = 0x7fff;

    /* check 不需要打开设备：只做安全体检 */
    if (!strcmp(cmd, "check") && argc >= 3) {
        if (argc > 3) mag = (int)strtol(argv[3], NULL, 0);
        FILE *f = fopen(argv[2], "rb");
        if (!f) { perror(argv[2]); return 1; }
        fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
        signed char *b = malloc(n);
        if (!b || fread(b, 1, n, f) != (size_t)n) { fputs("读文件失败\n", stderr); return 1; }
        fclose(f);
        char rep[256] = "";
        size_t cap = (size_t)n, keep = cap;
        int ch = 0;
        if (g_safety) {
            ch = motor_sanitize(b, &keep, mag, rep, sizeof rep);
            if (ch < 0) { printf("  ⇒ %s\n", rep); free(b); return 1; }
        } else {
            int mj = 0;
            for (size_t i = 1; i < keep; i++) { int d = abs(b[i] - b[i-1]); if (d > mj) mj = d; }
            snprintf(rep, sizeof rep, "--no-safety：原样（最大跳变 %d）", mj);
            keep = cap;
        }
        printf("%s  长度 %.2fs → %.2fs  mag=0x%04X (level %d)\n",
               argv[2], (double)n / SR, (double)keep / SR, mag, level_of(mag));
        if (keep != cap)
            printf("  注意：实际播放只会到 %.2fs（尾部已清零）\n", (double)keep / SR);
        printf("  ⇒ %s\n", rep);
        printf("  %s\n", ch ? "已修改（播放时会用这一版）" : "不需修改");
        free(b);
        return 0;
    }

    if (motor_open() < 0) return 1;

    if (!strcmp(cmd, "stop")) { motor_stop(); puts("已静音"); return 0; }

    if (!strcmp(cmd, "tone") && argc >= 4) {
        double hz = atof(argv[2]);
        int ms = atoi(argv[3]);
        int peak = argc > 4 ? atoi(argv[4]) : 60;
        if (argc > 5) mag = (int)strtol(argv[5], NULL, 0);
        if (peak > 127) peak = 127;
        printf("tone %.0fHz %dms peak=%d mag=0x%04X\n", hz, ms, peak, mag);
        return motor_tone(hz, ms, peak, mag) < 0;
    }

    if (!strcmp(cmd, "play") && argc >= 3) {
        if (argc > 3) mag = (int)strtol(argv[3], NULL, 0);
        FILE *f = fopen(argv[2], "rb");
        if (!f) { perror(argv[2]); return 1; }
        fseek(f, 0, SEEK_END);
        long n = ftell(f);
        fseek(f, 0, SEEK_SET);
        signed char *b = malloc(n);
        if (!b || fread(b, 1, n, f) != (size_t)n) { fputs("读文件失败\n", stderr); return 1; }
        fclose(f);
        printf("读入 %ld 字节 (%.2fs) mag=0x%04X\n", n, (double)n / SR, mag);
        int r = motor_play(b, n, mag);
        free(b);
        return r < 0;
    }

    fprintf(stderr,
        "motor —— 最小马达调用（内置安全层）\n"
        "  motor tone <Hz> <ms> [peak] [mag]\n"
        "  motor play <file.bin> [mag]\n"
        "  motor check <file.bin> [mag]     只做安全体检，不播放\n"
        "  motor stop\n"
        "  motor --max-ms <N> ...           限制单次时长（0=不限，默认）\n"
        "  motor --no-safety ...            跳过安全检查\n");
    return 2;
}
