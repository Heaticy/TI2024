//-----------------------------------------------------------------
// 程序描述:
//     FFT 调用函数（基 2 Cooley-Tukey FFT）
// 作    者: Heaticy
// 说明：
//     本函数实现对输入的实部与虚部信号进行快速傅里叶变换（FFT），
//     并计算输出频谱的幅值。
//     适用于单片机环境（浮点性能有限时需注意效率）。
//
// 参数说明：
//     pr[] : 输入信号的实部数组（长度 n）
//     pi[] : 输入信号的虚部数组（长度 n）
//     n    : FFT 点数（必须是 2 的 k 次方）
//     k    : FFT 阶数（k=log2(n)）
//     fr[] : FFT 计算后的实部输出
//     fi[] : FFT 计算后的虚部输出
//
// 注意事项：
//     1. FFT 输入信号需要先填充实部 pr[]，虚部 pi[]（无虚部则 pi[] 全部置零）。
//     2. n 必须是 2 的幂（如 64, 128, 256）。
//     3. 计算完成后，pr[] 将被重写为幅值谱（模值）。
//     4. 输出 fr[] 和 fi[] 为频域的实部和虚部。
//-----------------------------------------------------------------
#ifndef _FFT_H
#define _FFT_H
#include "math.h"
#define PI 3.1415926535

void kfft(float pr[], float pi[], int n, int k, float fr[], float fi[])
{
    int it, m, is, i, j, nv, l0;
    float p, q, s, vr, vi, poddr, poddi;

    // ---------- 1. 位反转置换（Bit-reversal） ----------
    // 将输入的 pr[], pi[] 重新排序，结果存入 fr[], fi[]
    for (it = 0; it <= n - 1; it++)
    {
        m = it;
        is = 0;
        for (i = 0; i <= k - 1; i++)
        {
            j = m / 2;
            is = 2 * is + (m - 2 * j); // 反转索引位
            m = j;
        }
        fr[it] = pr[is];
        fi[it] = pi[is];
    }

    // ---------- 2. 计算旋转因子 (Twiddle Factors) ----------
    pr[0] = 1.0;
    pi[0] = 0.0;
    p = 2.0 * PI / (1.0 * n);  // 基本旋转角度
    pr[1] = cos(p);
    pi[1] = -sin(p);

    // 通过递推方式计算后续的旋转因子
    for (i = 2; i <= n - 1; i++)
    {
        p = pr[i - 1] * pr[1];
        q = pi[i - 1] * pi[1];
        s = (pr[i - 1] + pi[i - 1]) * (pr[1] + pi[1]);
        pr[i] = p - q;
        pi[i] = s - p - q;
    }

    // ---------- 3. 第一层蝶形运算 ----------
    // 处理相邻点的加减（长度为 2 的子序列）
    for (it = 0; it <= n - 2; it = it + 2)
    {
        vr = fr[it];
        vi = fi[it];
        fr[it]     = vr + fr[it + 1];
        fi[it]     = vi + fi[it + 1];
        fr[it + 1] = vr - fr[it + 1];
        fi[it + 1] = vi - fi[it + 1];
    }

    // ---------- 4. 递归蝶形运算 ----------
    m = n / 2;
    nv = 2;
    for (l0 = k - 2; l0 >= 0; l0--)
    {
        m = m / 2;
        nv = 2 * nv;
        // 对每个小分段进行蝶形处理
        for (it = 0; it <= (m - 1) * nv; it = it + nv)
            for (j = 0; j <= (nv / 2) - 1; j++)
            {
                // 蝶形核心公式：
                p = pr[m * j] * fr[it + j + nv / 2];
                q = pi[m * j] * fi[it + j + nv / 2];
                s = pr[m * j] + pi[m * j];
                s = s * (fr[it + j + nv / 2] + fi[it + j + nv / 2]);

                poddr = p - q;       // 实部旋转结果
                poddi = s - p - q;   // 虚部旋转结果

                // 更新蝶形运算结果
                fr[it + j + nv / 2] = fr[it + j] - poddr;
                fi[it + j + nv / 2] = fi[it + j] - poddi;
                fr[it + j] = fr[it + j] + poddr;
                fi[it + j] = fi[it + j] + poddi;
            }
    }

    // ---------- 5. 计算幅值谱 ----------
    // 输出：pr[] 保存幅值，fr[] & fi[] 保持频域结果
    for (i = 0; i <= n - 1; i++)
    {
        pr[i] = sqrt(fr[i] * fr[i] + fi[i] * fi[i]);
    }
    return;
}
#endif
