/* extract.c —— extract_lead.py 的 C 移植（逐行对应，供对拍）
 *
 * 用法: ./extract 输入 输出(.flac/.wav/.mp3)
 * 做法同 Python 版：STFT 上按估计的 f0 做谐波软掩码，用原始相位 overlap-add 重建。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#define SR 48000
#define NFFT 4096
#define HOP 512
#define PAD (NFFT / 2)
#define LO_N 40
#define HI_N 91
#define NCAND (HI_N - LO_N + 1)
#define FH (NFFT / 2 + 1)          /* 2049 */
#define CHUNK 512
#define NYQ_LIM 18000.0
static const double DF = (double)SR / NFFT;   /* 11.71875 */

/* ---------------- FFT (radix-2, 就地) ---------------- */
typedef struct { float re, im; } cplx;

static int g_nfft;
static cplx *g_tw;                  /* exp(-2πi k/n), k<n/2 */
static int *g_rev;

static void fft_init(int n)
{
    g_nfft = n;
    g_tw = malloc(sizeof(cplx) * (n / 2));
    g_rev = malloc(sizeof(int) * n);
    for (int k = 0; k < n / 2; k++) {
        double a = -2.0 * M_PI * k / n;
        g_tw[k].re = (float)cos(a);
        g_tw[k].im = (float)sin(a);
    }
    int bits = 0;
    while ((1 << bits) < n) bits++;
    for (int i = 0; i < n; i++) {
        int r = 0;
        for (int b = 0; b < bits; b++) if (i & (1 << b)) r |= 1 << (bits - 1 - b);
        g_rev[i] = r;
    }
}

static void fft(cplx *a, int inverse)
{
    const int n = g_nfft;
    for (int i = 0; i < n; i++) {
        int r = g_rev[i];
        if (i < r) { cplx t = a[i]; a[i] = a[r]; a[r] = t; }
    }
    for (int len = 2; len <= n; len <<= 1) {
        int half = len >> 1, step = n / len;
        for (int i = 0; i < n; i += len) {
            for (int j = 0; j < half; j++) {
                cplx w = g_tw[j * step];
                if (inverse) w.im = -w.im;
                cplx u = a[i + j], v = a[i + j + half];
                cplx t;
                t.re = v.re * w.re - v.im * w.im;
                t.im = v.re * w.im + v.im * w.re;
                a[i + j].re = u.re + t.re;  a[i + j].im = u.im + t.im;
                a[i + j + half].re = u.re - t.re;
                a[i + j + half].im = u.im - t.im;
            }
        }
    }
    if (inverse) {
        float s = 1.0f / n;
        for (int i = 0; i < n; i++) { a[i].re *= s; a[i].im *= s; }
    }
}

/* rfft：实序列 → 前 FH 个复谱；buf 长度 NFFT */
static void rfft(const float *x, cplx *out)
{
    static cplx *tmp = NULL;
    if (!tmp) tmp = malloc(sizeof(cplx) * NFFT);
    for (int i = 0; i < NFFT; i++) { tmp[i].re = x[i]; tmp[i].im = 0.0f; }
    fft(tmp, 0);
    memcpy(out, tmp, sizeof(cplx) * FH);
}

/* irfft：前 FH 个复谱 → 实序列（未乘 NFFT，等价 numpy.irfft 的 1/n 已在 fft 里） */
static void irfft(const cplx *X, float *out)
{
    static cplx *tmp = NULL;
    if (!tmp) tmp = malloc(sizeof(cplx) * NFFT);
    for (int i = 0; i < FH; i++) tmp[i] = X[i];
    for (int i = 1; i < FH - 1; i++) {          /* Hermitian */
        tmp[NFFT - i].re = X[i].re;
        tmp[NFFT - i].im = -X[i].im;
    }
    fft(tmp, 1);
    for (int i = 0; i < NFFT; i++) out[i] = tmp[i].re;
}

/* ---------------- 工具 ---------------- */
static void *xmalloc(size_t n)
{
    void *p = malloc(n);
    if (!p) { fputs("内存不足\n", stderr); exit(1); }
    return p;
}

static int cmp_f(const void *a, const void *b)
{
    float x = *(const float *)a, y = *(const float *)b;
    return (x > y) - (x < y);
}

/* numpy.percentile(..., q) 默认 linear 插值；q 传 0..100 */
static float percentile(const float *v, size_t n, double q)
{
    float *s = xmalloc(sizeof(float) * n);
    memcpy(s, v, sizeof(float) * n);
    qsort(s, n, sizeof(float), cmp_f);
    double pos = (q / 100.0) * (double)(n - 1);
    size_t lo = (size_t)floor(pos);
    double frac = pos - (double)lo;
    float r = (lo + 1 < n) ? (float)(s[lo] * (1.0 - frac) + s[lo + 1] * frac) : s[lo];
    free(s);
    return r;
}

static float median3f(float a, float b, float c)
{
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { b = c; }
    if (a > b) { b = a; }
    return b;
}

static int median3i(int a, int b, int c)
{
    if (a > b) { int t = a; a = b; b = t; }
    if (b > c) b = c;
    if (a > b) b = a;
    return b;
}

/* ---------------- 解码：ffmpeg → f32le 立体声 48k ---------------- */
static float *decode(const char *path, size_t *nsamp)
{
    char cmd[4096];
    /* 单引号包裹路径，内部单引号用 '\'' 转义 */
    char q[2048];
    size_t o = 0;
    for (const char *p = path; *p && o < sizeof q - 8; p++) {
        if (*p == '\'') { memcpy(q + o, "'\\''", 4); o += 4; }
        else q[o++] = *p;
    }
    q[o] = 0;
    snprintf(cmd, sizeof cmd,
             "ffmpeg -v error -i '%s' -map 0:a:0 -vn -ac 2 -ar %d "
             "-f f32le -acodec pcm_f32le pipe:1", q, SR);
    FILE *fp = popen(cmd, "r");
    if (!fp) { fputs("无法启动 ffmpeg\n", stderr); exit(1); }
    size_t cap = SR * 2 * 8, n = 0;            /* 8 秒起步 */
    float *buf = xmalloc(cap * sizeof(float));
    for (;;) {
        if (n + 65536 > cap) {
            cap *= 2;
            float *g = realloc(buf, cap * sizeof(float));
            if (!g) { fputs("内存不足\n", stderr); exit(1); }
            buf = g;
        }
        size_t got = fread(buf + n, sizeof(float), 65536, fp);
        if (got == 0) break;
        n += got;
    }
    int rc = pclose(fp);
    if (rc != 0 || n < 2) { fputs("ffmpeg 解码失败\n", stderr); exit(1); }
    *nsamp = n / 2;
    return buf;
}

/* ---------------- 主流程 ---------------- */
int main(int argc, char **argv)
{
    if (argc < 3) {
        fputs("用法: extract 输入 输出(.flac/.wav/.mp3)\n", stderr);
        return 2;
    }
    const char *src = argv[1], *dst = argv[2];

    size_t n;
    float *y = decode(src, &n);
    float *L = xmalloc(sizeof(float) * n), *R = xmalloc(sizeof(float) * n);
    float *mono = xmalloc(sizeof(float) * n);
    for (size_t i = 0; i < n; i++) {
        L[i] = y[2 * i];
        R[i] = y[2 * i + 1];
        mono[i] = 0.5f * (L[i] + R[i]);
    }
    free(y);
    printf("解码: %.3fs, %zu 帧, 48kHz 立体声\n", (double)n / SR, n);

    fft_init(NFFT);
    const int T = (int)(n / HOP) + 1;
    const int F = FH;
    cplx *spec = xmalloc(sizeof(cplx) * FH);
    float *S = xmalloc(sizeof(float) * (size_t)T * F);     /* [t][f] */

    float *win = xmalloc(sizeof(float) * NFFT);
    float *win2 = xmalloc(sizeof(float) * NFFT);
    float *fr = xmalloc(sizeof(float) * NFFT);
    for (int i = 0; i < NFFT; i++) {
        win[i] = (float)(0.5 - 0.5 * cos(2.0 * M_PI * i / (NFFT - 1)));
        win2[i] = win[i] * win[i];
    }

    /* --- STFT --- */
    for (int t = 0; t < T; t++) {
        long st = (long)t * HOP - PAD;
        for (int i = 0; i < NFFT; i++) {
            long k = st + i;
            float v = (k >= 0 && (size_t)k < n) ? mono[k] : 0.0f;
            fr[i] = v * win[i];
        }
        rfft(fr, spec);
        for (int f = 0; f < F; f++)
            S[(size_t)t * F + f] = sqrtf(spec[f].re * spec[f].re +
                                         spec[f].im * spec[f].im);
    }

    /* --- 候选音高与响度 --- */
    int cand[NCAND];
    float f0q[NCAND], wgt[NCAND];
    for (int i = 0; i < NCAND; i++) {
        cand[i] = LO_N + i;
        f0q[i] = (float)(440.0 * pow(2.0, (cand[i] - 69) / 12.0));
        double c1 = ((double)f0q[i] - 95.0) / 260.0;
        if (c1 < 0) c1 = 0; if (c1 > 1) c1 = 1;
        double c2 = (double)f0q[i] / 400.0;
        if (c2 < 0.75) c2 = 0.75; if (c2 > 1.25) c2 = 1.25;
        wgt[i] = (float)((0.30 + 0.70 * c1) * pow(c2, 0.3));
    }

    float *sal = xmalloc(sizeof(float) * (size_t)NCAND * T);   /* [i][t] */
    float *acc = xmalloc(sizeof(float) * T);
    for (int i = 0; i < NCAND; i++) {
        memset(acc, 0, sizeof(float) * T);
        for (int k = 1; k <= 5; k++) {
            double fk = (double)f0q[i] * k;
            if (fk >= NYQ_LIM) break;
            int c = (int)lround(fk / DF);
            int lo = c - 1; if (lo < 1) lo = 1;
            int hi = (c + 1 < F - 1) ? c + 1 : F - 1;      /* 闭区间上界 */
            double div = pow((double)k, 0.7);
            for (int t = 0; t < T; t++) {
                const float *row = S + (size_t)t * F;
                float mx = row[lo];
                for (int b = lo + 1; b <= hi; b++) if (row[b] > mx) mx = row[b];
                acc[t] += (float)(mx / div);
            }
        }
        for (int t = 0; t < T; t++) sal[(size_t)i * T + t] = acc[t] * wgt[i];
    }
    free(acc);

    /* --- Viterbi（抑制八度误判） --- */
    float *trans = xmalloc(sizeof(float) * NCAND * NCAND);
    for (int i = 0; i < NCAND; i++)
        for (int j = 0; j < NCAND; j++) {
            int d = abs(cand[i] - cand[j]);
            float v;
            if (d == 0) v = 0.0f;
            else if (d <= 2) v = -0.15f * d;
            else if (d <= 5) v = -0.35f * d;
            else if (d <= 11) v = -0.60f * d;
            else v = -1.20f * d;
            if (d == 12) v -= 1.5f;
            trans[i * NCAND + j] = v;
        }

    float *prev = xmalloc(sizeof(float) * NCAND);
    float *cur = xmalloc(sizeof(float) * NCAND);
    float *emit = xmalloc(sizeof(float) * NCAND);
    unsigned char *ptr = xmalloc((size_t)T * NCAND);
    for (int i = 0; i < NCAND; i++) emit[i] = 0.0f;
    for (int t = 0; t < T; t++) {
        float mx = 0.0f;
        for (int i = 0; i < NCAND; i++) {
            float v = sal[(size_t)i * T + t];
            if (v > mx) mx = v;
        }
        for (int i = 0; i < NCAND; i++)
            emit[i] = sal[(size_t)i * T + t] / (mx + 1e-6f) * 8.0f;
        if (t == 0) { memcpy(prev, emit, sizeof(float) * NCAND); continue; }
        for (int j = 0; j < NCAND; j++) {
            float best = -1e30f; int bi = 0;
            for (int i = 0; i < NCAND; i++) {
                float v = prev[i] + trans[i * NCAND + j];
                if (v > best) { best = v; bi = i; }
            }
            ptr[(size_t)t * NCAND + j] = (unsigned char)bi;
            cur[j] = best + emit[j];
        }
        memcpy(prev, cur, sizeof(float) * NCAND);
    }
    int *path = xmalloc(sizeof(int) * T);
    {
        float best = -1e30f; int bi = 0;
        for (int i = 0; i < NCAND; i++) if (prev[i] > best) { best = prev[i]; bi = i; }
        path[T - 1] = bi;
        for (int t = T - 1; t > 0; t--)
            path[t - 1] = ptr[(size_t)t * NCAND + path[t]];
    }
    free(prev); free(cur); free(emit); free(ptr); free(trans);

    /* --- midi 序列（medfilt 3，零填充） + 逐帧统计 --- */
    float *top = xmalloc(sizeof(float) * T);
    float *energy = xmalloc(sizeof(float) * T);
    float *conf = xmalloc(sizeof(float) * T);
    float *sharp = xmalloc(sizeof(float) * T);
    for (int t = 0; t < T; t++) {
        float e = 0.0f, sm = 0.0f;
        const float *row = S + (size_t)t * F;
        for (int f = 0; f < F; f++) e += row[f];
        float tp = sal[(size_t)path[t] * T + t];
        for (int i = 0; i < NCAND; i++) sm += sal[(size_t)i * T + t];
        energy[t] = e;
        top[t] = tp;
        conf[t] = tp / (e + 1e-6f);
        sharp[t] = tp / (sm / NCAND + 1e-6f);
    }
    int *midi = xmalloc(sizeof(int) * T);
    for (int t = 0; t < T; t++) {
        int a = (t > 0) ? cand[path[t - 1]] : 0;
        int b = cand[path[t]];
        int c = (t + 1 < T) ? cand[path[t + 1]] : 0;
        midi[t] = median3i(a, b, c);
    }

    /* --- valid 判据 --- */
    float p20 = percentile(energy, T, 20.0);
    float medc = percentile(conf, T, 50.0);
    float meds = percentile(sharp, T, 50.0);
    unsigned char *valid = xmalloc(T);
    int nvalid = 0;
    for (int t = 0; t < T; t++) {
        valid[t] = (energy[t] > p20 * 0.70f) && (conf[t] > medc * 0.45f) &&
                   (sharp[t] > meds * 0.50f);
        nvalid += valid[t];
    }

    /* --- 连续 f0 细化 --- */
    float *f0 = xmalloc(sizeof(float) * T);
    for (int t = 0; t < T; t++) {
        double f = 440.0 * pow(2.0, (midi[t] - 69) / 12.0);
        int lo = (int)(f * pow(2.0, -0.5 / 12) / DF);
        if (lo < 1) lo = 1;
        int hi = (int)(f * pow(2.0, 0.5 / 12) / DF) + 1;
        if (hi > F - 2) hi = F - 2;
        if (hi <= lo) { f0[t] = (float)f; continue; }
        const float *row = S + (size_t)t * F;
        int p = 0; float bv = row[lo];
        for (int b = lo + 1; b <= hi; b++) if (row[b] > bv) { bv = row[b]; p = b - lo; }
        double c = lo + p;
        int len = hi - lo + 1;
        if (p > 0 && p < len - 1) {
            double a = row[lo + p - 1], bb = row[lo + p], cc = row[lo + p + 1];
            double den = a - 2 * bb + cc;
            double off = (fabs(den) > 1e-9) ? 0.5 * (a - cc) / den : 0.0;
            if (off < -0.5) off = -0.5;
            if (off > 0.5) off = 0.5;
            c += off;
        }
        f0[t] = (float)(c * DF);
        if (f0[t] < 20.0f) f0[t] = 20.0f;
    }
    {
        float *tmp = xmalloc(sizeof(float) * T);
        for (int t = 0; t < T; t++) {
            float a = (t > 0) ? f0[t - 1] : 0.0f;
            float b = f0[t];
            float c = (t + 1 < T) ? f0[t + 1] : 0.0f;
            tmp[t] = median3f(a, b, c);
        }
        memcpy(f0, tmp, sizeof(float) * T);
        free(tmp);
    }
    {
        int mn = 999, mx = -1;
        for (int t = 0; t < T; t++) if (valid[t] && midi[t] > 0) {
            if (midi[t] < mn) mn = midi[t];
            if (midi[t] > mx) mx = midi[t];
        }
        printf("分析帧 %d, 有主音高 %d (%.0f%%), 音域 %d~%d MIDI\n",
               T, nvalid, 100.0 * nvalid / T, mn, mx);
    }
    free(S); free(top); free(conf); free(sharp); free(energy); free(midi);

    /* --- 掩码：逐帧按谐波写 M[T][F]，再沿时间做 3 点平滑（因此存两份，各 T×F） --- */
    float *M = xmalloc(sizeof(float) * (size_t)T * F);
    {
        float *row = xmalloc(sizeof(float) * F);
        for (int t = 0; t < T; t++) {
            float *m = M + (size_t)t * F;
            memset(m, 0, sizeof(float) * F);
            if (!valid[t]) continue;
            double f = f0[t];
            for (int k = 1; k <= 23; k++) {
                double fk = f * k;
                if (fk > NYQ_LIM) break;
                double half = 20.0;
                if (fk * 0.029 > half) half = fk * 0.029;
                double c = fk / DF;
                int lo = (int)floor(c - half / DF) - 1;
                if (lo < 1) lo = 1;
                int hi = (int)ceil(c + half / DF) + 1;
                if (hi > F - 1) hi = F - 1;
                if (hi < lo) continue;
                for (int b = lo; b <= hi; b++) {
                    double dist = ((double)b - c) / (half / DF);
                    double v = 0.0;
                    if (fabs(dist) <= 1.0) {
                        double x = dist;
                        if (x < -1) x = -1; if (x > 1) x = 1;
                        v = 0.5 * (1.0 + cos(M_PI * x));
                    }
                    if (v > m[b]) m[b] = (float)v;
                }
            }
        }
        free(row);
        /* uniform_filter1d(size=3, mode="nearest") */
        float *sm = xmalloc(sizeof(float) * (size_t)T * F);
        for (int t = 0; t < T; t++) {
            const float *a = M + (size_t)(t > 0 ? t - 1 : 0) * F;
            const float *b = M + (size_t)t * F;
            const float *c = M + (size_t)(t + 1 < T ? t + 1 : T - 1) * F;
            float *o = sm + (size_t)t * F;
            for (int f = 0; f < F; f++) o[f] = (a[f] + b[f] + c[f]) / 3.0f;
        }
        free(M);
        M = sm;
    }
    free(f0); free(valid);

    /* --- 重建（分块 OLA） --- */
    float *oL = xmalloc(sizeof(float) * (n + 2 * PAD));
    float *oR = xmalloc(sizeof(float) * (n + 2 * PAD));
    float *den = xmalloc(sizeof(float) * (n + 2 * PAD));
    memset(oL, 0, sizeof(float) * (n + 2 * PAD));
    memset(oR, 0, sizeof(float) * (n + 2 * PAD));
    memset(den, 0, sizeof(float) * (n + 2 * PAD));

    for (int s = 0; s < T; s += CHUNK) {
        int e = s + CHUNK; if (e > T) e = T;
        for (int t = s; t < e; t++) {
            long st = (long)t * HOP;
            const float *m = M + (size_t)t * F;
            for (int ch = 0; ch < 2; ch++) {
                const float *src_ch = ch ? R : L;
                long fs = (long)t * HOP - PAD;
                for (int i = 0; i < NFFT; i++) {
                    long k = fs + i;
                    float v = (k >= 0 && (size_t)k < n) ? src_ch[k] : 0.0f;
                    fr[i] = v * win[i];
                }
                rfft(fr, spec);
                for (int f = 0; f < F; f++) { spec[f].re *= m[f]; spec[f].im *= m[f]; }
                float out[NFFT];
                irfft(spec, out);
                float *acc_ch = ch ? oR : oL;
                for (int i = 0; i < NFFT; i++) acc_ch[st + i] += out[i] * win[i];
                if (ch == 0)
                    for (int i = 0; i < NFFT; i++) den[st + i] += win2[i];
            }
        }
    }
    free(M);

    float peak = 0.0f;
    for (size_t i = 0; i < n; i++) {
        long k = (long)i + PAD;
        if (den[k] > 1e-6f) { oL[k] /= den[k]; oR[k] /= den[k]; }
        float a = fabsf(oL[k]), b = fabsf(oR[k]);
        if (a > peak) peak = a;
        if (b > peak) peak = b;
    }
    printf("重建完成: %zu 样本 (%.3fs)\n", n, (double)n / SR);
    printf("峰值 %.4f (未做增益)\n", peak);
    if (peak > 0.99f) {
        float g = 0.99f / peak;
        for (size_t i = 0; i < n; i++) { long k = (long)i + PAD; oL[k] *= g; oR[k] *= g; }
    }

    /* --- 写 16bit WAV → 再转目标格式 --- */
    char tmp[4096];
    snprintf(tmp, sizeof tmp, "%s.tmp.wav", dst);
    FILE *fp = fopen(tmp, "wb");
    if (!fp) { fprintf(stderr, "无法写入 %s\n", tmp); return 1; }
    uint32_t datasz = (uint32_t)(n * 4), riffsz = 36 + datasz;
    uint16_t ch = 2, bps = 16, align = 4, fmt = 1;
    uint32_t rate = SR, byterate = SR * 4, fmtsz = 16;
    fwrite("RIFF", 1, 4, fp); fwrite(&riffsz, 4, 1, fp); fwrite("WAVE", 1, 4, fp);
    fwrite("fmt ", 1, 4, fp); fwrite(&fmtsz, 4, 1, fp);
    fwrite(&fmt, 2, 1, fp); fwrite(&ch, 2, 1, fp); fwrite(&rate, 4, 1, fp);
    fwrite(&byterate, 4, 1, fp); fwrite(&align, 2, 1, fp); fwrite(&bps, 2, 1, fp);
    fwrite("data", 1, 4, fp); fwrite(&datasz, 4, 1, fp);
    for (size_t i = 0; i < n; i++) {
        long k = (long)i + PAD;
        int16_t l = (int16_t)(oL[k] * 32767.0f);
        int16_t r = (int16_t)(oR[k] * 32767.0f);
        fwrite(&l, 2, 1, fp); fwrite(&r, 2, 1, fp);
    }
    fclose(fp);

    const char *dot = strrchr(dst, '.');
    int lossless = dot && (!strcasecmp(dot, ".flac") || !strcasecmp(dot, ".wav"));
    char title[1024];
    {
        const char *bs = strrchr(src, '/');
        bs = bs ? bs + 1 : src;
        const char *de = strrchr(bs, '.');
        int len = de ? (int)(de - bs) : (int)strlen(bs);
        if (len > 900) len = 900;
        snprintf(title, sizeof title, "%.*s (lead extract)", len, bs);
    }
    char cmd[8192];
    snprintf(cmd, sizeof cmd,
             "ffmpeg -y -v error -i '%s' %s -ar %d -metadata 'title=%s' '%s'",
             tmp, lossless ? "-c:a flac" : "-b:a 320k -ac 2", SR, title, dst);
    int rc = system(cmd);
    remove(tmp);
    if (rc != 0) { fputs("编码失败\n", stderr); return 1; }
    printf("输出 -> %s\n", dst);
    return 0;
}
