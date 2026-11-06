#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
音调提取 -> mp3（不重新发声、不设音色/响度/包络参数）

做法：从原曲估出逐帧主音高，只把它当作「选区」，
在原始 STFT 上保留该音高及其谐波附近的谱线（软窗），
用原始相位 overlap-add 重建。原曲的波形、音色、动态、
时间轴、声道都保留，没有任何合成的自由度。
"""
import sys
import os
import subprocess
import gc
import numpy as np
from scipy.signal import medfilt
from scipy.ndimage import uniform_filter1d
from scipy.io import wavfile
from numpy.lib.stride_tricks import sliding_window_view

SR = 48000
NFFT = 4096
HOP = 512
PAD = NFFT // 2
LO_N, HI_N = 40, 91
DF = SR / NFFT
NYQ_LIM = 18000.0
CHUNK = 512          # 分块重建，控制内存


def decode(path):
    # -map 0:a:0 明确只取第一条音频流；-vn 排除封面/视频流（m4a、mp4 常带）
    raw = subprocess.check_output([
        "ffmpeg", "-v", "error",
        "-i", path,
        "-map", "0:a:0", "-vn",
        "-ac", "2", "-ar", str(SR),
        "-f", "f32le", "-acodec", "pcm_f32le", "pipe:1",
    ])
    return np.frombuffer(raw, dtype="<f4").reshape(-1, 2)


def estimate(y_mono):
    win = np.hanning(NFFT).astype(np.float32)
    yp = np.pad(y_mono, (PAD, PAD))
    frames = sliding_window_view(yp, NFFT)[::HOP]
    T = frames.shape[0]
    S = np.abs(np.fft.rfft(frames.astype(np.float32) * win[None, :], axis=1)).T
    S = S.astype(np.float32)                      # (F, T)
    del frames, yp
    gc.collect()
    F = S.shape[0]

    cand = np.arange(LO_N, HI_N + 1)
    f0q = (440.0 * 2 ** ((cand - 69) / 12.0)).astype(np.float32)
    N = len(cand)

    sal = np.zeros((N, T), np.float32)
    for i, f in enumerate(f0q):
        acc = np.zeros(T, np.float32)
        for k in range(1, 6):
            fk = f * k
            if fk >= NYQ_LIM:
                break
            c = int(round(fk / DF))
            lo, hi = max(1, c - 1), min(F - 1, c + 1) + 1
            acc += S[lo:hi].max(axis=0) / (k ** 0.7)
        sal[i] = acc

    w = 0.30 + 0.70 * np.clip((f0q - 95.0) / 260.0, 0, 1)
    w *= np.clip(f0q / 400.0, 0.75, 1.25) ** 0.3
    sal *= w[:, None]

    # Viterbi：抑制八度误判
    d = np.abs(cand[:, None] - cand[None, :])
    trans = np.where(d == 0, 0.0,
             np.where(d <= 2, -0.15 * d,
             np.where(d <= 5, -0.35 * d,
             np.where(d <= 11, -0.60 * d, -1.20 * d))))
    trans[d == 12] -= 1.5
    emit = sal / (sal.max(axis=0, keepdims=True) + 1e-6) * 8.0
    prev = emit[:, 0].copy()
    ptr = np.zeros((T, N), dtype=np.int16)
    for t in range(1, T):
        m = prev[:, None] + trans
        bp = m.argmax(axis=0)
        prev = m[bp, np.arange(N)] + emit[:, t]
        ptr[t] = bp
    path = np.zeros(T, dtype=np.int16)
    path[-1] = int(prev.argmax())
    for t in range(T - 1, 0, -1):
        path[t - 1] = ptr[t, path[t]]

    midi = medfilt(np.array(cand[path], dtype=np.int32), kernel_size=3)
    top = sal[path, np.arange(T)]
    energy = S.sum(axis=0)
    conf = top / (energy + 1e-6)
    sharp = top / (sal.mean(axis=0) + 1e-6)
    valid = (
        (energy > np.percentile(energy, 20) * 0.70)
        & (conf > np.median(conf) * 0.45)
        & (sharp > np.median(sharp) * 0.50)
    )

    # 连续 f0 细化：在 ±0.5 半音内找真实谱峰并抛物线插值
    f0 = np.zeros(T, np.float32)
    for t in range(T):
        f = 440.0 * 2 ** ((int(midi[t]) - 69) / 12.0)
        lo = max(1, int(f * 2 ** (-0.5 / 12) / DF))
        hi = min(F - 2, int(f * 2 ** (0.5 / 12) / DF) + 1)
        if hi <= lo:
            f0[t] = f
            continue
        col = S[lo:hi + 1, t]
        p = int(np.argmax(col))
        c = lo + p
        if 0 < p < len(col) - 1:
            a, b, cc = col[p - 1], col[p], col[p + 1]
            den = a - 2 * b + cc
            off = 0.5 * (a - cc) / den if abs(den) > 1e-9 else 0.0
            c = c + np.clip(off, -0.5, 0.5)
        f0[t] = max(20.0, c * DF)
    f0 = medfilt(f0.astype(np.float32), kernel_size=3).astype(np.float32)
    return S, midi, f0, valid, T, F


def build_mask(f0, valid, T, F):
    M = np.zeros((F, T), np.float32)
    bins = np.arange(F, dtype=np.float32)
    for t in range(T):
        if not valid[t]:
            continue
        f = float(f0[t])
        for k in range(1, 24):
            fk = f * k
            if fk > NYQ_LIM:
                break
            half = max(20.0, fk * 0.029)      # ~±0.5 半音，容纳 detune/vibrato
            c = fk / DF
            lo = max(1, int(np.floor(c - half / DF)) - 1)
            hi = min(F - 1, int(np.ceil(c + half / DF)) + 1)
            if hi < lo:
                continue
            dist = (bins[lo:hi + 1] - c) / (half / DF)
            w = np.where(np.abs(dist) <= 1.0,
                         0.5 * (1.0 + np.cos(np.pi * np.clip(dist, -1, 1))), 0.0)
            np.maximum(M[lo:hi + 1, t], w.astype(np.float32), out=M[lo:hi + 1, t])
    # 时间方向轻微平滑，消除掩码跳变造成的音乐噪声
    M = uniform_filter1d(M, size=3, axis=1, mode="nearest")
    return M


def reconstruct(channel, M, T):
    win = np.hanning(NFFT).astype(np.float32)
    win2 = win * win
    yp = np.pad(channel, (PAD, PAD))
    frames = sliding_window_view(yp, NFFT)[::HOP]   # view，不占内存
    ola = np.zeros(len(yp), np.float32)
    den = np.zeros(len(yp), np.float32)

    for s in range(0, T, CHUNK):
        e = min(s + CHUNK, T)
        fr = frames[s:e].astype(np.float32) * win[None, :]
        X = np.fft.rfft(fr, axis=1)
        X *= M[:, s:e].T
        rec = np.fft.irfft(X, n=NFFT, axis=1).astype(np.float32) * win[None, :]
        for i in range(e - s):
            st = (s + i) * HOP
            ola[st:st + NFFT] += rec[i]
            den[st:st + NFFT] += win2
        del fr, X, rec

    nz = den > 1e-6
    out = np.zeros_like(ola)
    out[nz] = ola[nz] / den[nz]
    return out[PAD:PAD + len(channel)]


def main():
    if len(sys.argv) < 3:
        print("用法: python extract_lead.py 输入.mp3 输出.mp3")
        sys.exit(1)
    src, dst = sys.argv[1], sys.argv[2]

    y = decode(src)
    n = len(y)
    print("解码: %.3fs, %d 帧, 48kHz 立体声" % (n / SR, n))

    mono = y.mean(axis=1).astype(np.float32)
    S, midi, f0, valid, T, F = estimate(mono)
    print("分析帧 %d, 有主音高 %d (%.0f%%), 音域 %d~%d MIDI"
          % (T, int(valid.sum()), 100 * valid.mean(), int(midi[valid].min()),
             int(midi[valid].max())))
    M = build_mask(f0, valid, T, F)
    del S, mono
    gc.collect()

    L = reconstruct(y[:, 0].astype(np.float32), M, T)
    R = reconstruct(y[:, 1].astype(np.float32), M, T)
    print("重建完成: %d 样本 (%.3fs)" % (len(L), len(L) / SR))

    # 不设响度、不做归一化、不做压限；仅防削波保护
    peak = max(float(np.max(np.abs(L))), float(np.max(np.abs(R))))
    print("峰值 %.4f (未做增益)" % peak)
    if peak > 0.99:
        L *= 0.99 / peak
        R *= 0.99 / peak

    pcm = (np.stack([L, R], axis=1) * 32767).astype(np.int16)
    tmp = dst + ".tmp.wav"
    wavfile.write(tmp, SR, pcm)
    title = os.path.splitext(os.path.basename(src))[0] + " (lead extract)"
    # 按输出后缀选编码：.flac/.wav 走无损（避免"提取后再压一次 mp3"），其余仍 320k
    lossless = dst.lower().endswith((".flac", ".wav"))
    codec = (["-c:a", "flac"] if lossless
             else ["-b:a", "320k", "-ac", "2"])
    try:
        subprocess.check_call([
            "ffmpeg", "-y", "-v", "error", "-i",
             tmp, *codec, "-ar", str(SR), "-metadata",
             "title=" + title, dst
        ])
    finally:
        subprocess.call(["rm", "-f", tmp])
    print("输出 ->", dst)


if __name__ == "__main__":
    main()
