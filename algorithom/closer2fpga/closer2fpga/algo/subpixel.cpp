#define _CRT_SECURE_NO_WARNINGS
#include "subpixel.h"

#include <cmath>
#include <vector>
#include <algorithm>

namespace {

    struct Vec2 {
        f32 x;
        f32 y;
    };

    static f32 clamp_f32(f32 v, f32 lo, f32 hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }

    static f32 dot(const Vec2& a, const Vec2& b) {
        return a.x * b.x + a.y * b.y;
    }

    static f32 norm2(const Vec2& a) {
        return a.x * a.x + a.y * a.y;
    }

    static f32 bilinear_sample(const GrayImage& img, f32 x, f32 y) {
        if (img.w <= 0 || img.h <= 0)
            return 0.0f;

        x = clamp_f32(x, 0.0f, (f32)(img.w - 1));
        y = clamp_f32(y, 0.0f, (f32)(img.h - 1));

        int x0 = (int)std::floor(x);
        int y0 = (int)std::floor(y);

        int x1 = x0 + 1;
        int y1 = y0 + 1;

        if (x1 >= img.w) x1 = img.w - 1;
        if (y1 >= img.h) y1 = img.h - 1;

        f32 dx = x - (f32)x0;
        f32 dy = y - (f32)y0;

        f32 p00 = (f32)img.get(x0, y0);
        f32 p10 = (f32)img.get(x1, y0);
        f32 p01 = (f32)img.get(x0, y1);
        f32 p11 = (f32)img.get(x1, y1);

        f32 a = p00 * (1.0f - dx) + p10 * dx;
        f32 b = p01 * (1.0f - dx) + p11 * dx;

        return a * (1.0f - dy) + b * dy;
    }

    /*
     * 使用中心差分计算局部梯度。
     *
     * 棋盘格角点附近实际上存在两组互相近似正交的边缘：
     *
     *        |\
     *        | \
     *        |  \
     *        +---\
     *
     * 因此不同方向采样得到的梯度，可以用来恢复两个边缘的交点。
     */
    static Vec2 gradient_at(const GrayImage& img, f32 x, f32 y) {
        const f32 h = 1.0f;

        f32 gx =
            bilinear_sample(img, x + h, y) -
            bilinear_sample(img, x - h, y);

        f32 gy =
            bilinear_sample(img, x, y + h) -
            bilinear_sample(img, x, y - h);

        return { gx * 0.5f, gy * 0.5f };
    }

    /*
     * 对一个候选角点建立局部结构张量：
     *
     *     G = sum w * [ gx*gx  gx*gy ]
     *                 [ gx*gy  gy*gy ]
     *
     * 最大特征向量表示主要边缘方向，
     * 最小特征向量表示另一组边缘方向。
     *
     * 棋盘格角点处两组方向同时存在，因此矩阵具有较好的二维约束。
     */
    static bool compute_structure_tensor(
        const GrayImage& img,
        f32 cx,
        f32 cy,
        int radius,
        f64& a,
        f64& b,
        f64& c)
    {
        a = 0.0;
        b = 0.0;
        c = 0.0;

        f64 weight_sum = 0.0;

        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dx = -radius; dx <= radius; ++dx) {

                f32 x = cx + (f32)dx;
                f32 y = cy + (f32)dy;

                if (x < 1.0f || x >= (f32)(img.w - 2) ||
                    y < 1.0f || y >= (f32)(img.h - 2))
                    continue;

                Vec2 g = gradient_at(img, x, y);

                f64 gx = (f64)g.x;
                f64 gy = (f64)g.y;

                f64 mag2 = gx * gx + gy * gy;

                if (mag2 < 1e-6)
                    continue;

                /*
                 * 高斯权重。
                 *
                 * 中心附近权重大，避免窗口边缘对角点位置产生过大影响。
                 */
                f64 rr = (f64)(dx * dx + dy * dy);
                f64 sigma = (f64)radius * 0.7 + 1.0;
                f64 w = std::exp(-rr / (2.0 * sigma * sigma));

                a += w * gx * gx;
                b += w * gx * gy;
                c += w * gy * gy;

                weight_sum += w;
            }
        }

        if (weight_sum < 1e-8)
            return false;

        a /= weight_sum;
        b /= weight_sum;
        c /= weight_sum;

        /*
         * 判断是否具有足够强的二维结构。
         */
        f64 det = a * c - b * b;
        f64 trace = a + c;

        if (trace < 1e-6)
            return false;

        f64 ratio = det / (trace * trace + 1e-12);

        /*
         * 对于纯直线，det 接近 0。
         * 对于真正的棋盘格角点，两方向都有梯度，因此 det 会明显增大。
         */
        if (ratio < 0.01)
            return false;

        return true;
    }

    /*
     * 求 2x2 对称矩阵
     *
     *     [ a b ]
     *     [ b c ]
     *
     * 的特征向量。
     */
    static bool principal_direction(
        f64 a,
        f64 b,
        f64 c,
        Vec2& v1,
        Vec2& v2)
    {
        f64 trace = a + c;
        f64 diff = a - c;

        f64 disc = std::sqrt(diff * diff + 4.0 * b * b);

        f64 lambda1 = 0.5 * (trace + disc);
        f64 lambda2 = 0.5 * (trace - disc);

        if (lambda1 < 1e-12 || lambda2 < 1e-12)
            return false;

        /*
         * 对最大特征值求特征向量。
         */
        f64 vx, vy;

        if (std::fabs(b) > std::fabs(diff) * 0.5) {
            vx = b;
            vy = lambda1 - a;
        }
        else {
            vx = lambda1 - c;
            vy = b;
        }

        f64 n = std::sqrt(vx * vx + vy * vy);

        if (n < 1e-12)
            return false;

        vx /= n;
        vy /= n;

        v1 = { (f32)vx, (f32)vy };

        /*
         * 与 v1 正交。
         */
        v2 = { (f32)(-vy), (f32)vx };

        return true;
    }

    /*
     * 给定一条方向，寻找沿该方向的灰度变化。
     *
     * 这里不是简单寻找最大梯度点，而是计算：
     *
     *     sum w(t) * t * gradient(t)
     *
     * 从而获得边缘中心的大致位置。
     */
    static bool estimate_edge_center(
        const GrayImage& img,
        f32 cx,
        f32 cy,
        const Vec2& direction,
        f32& offset)
    {
        const int radius = 7;

        f64 numerator = 0.0;
        f64 denominator = 0.0;

        for (int i = -radius; i <= radius; ++i) {

            f32 t = (f32)i;

            f32 x = cx + direction.x * t;
            f32 y = cy + direction.y * t;

            Vec2 g = gradient_at(img, x, y);

            /*
             * 取沿当前方向的梯度分量。
             */
            f32 grad = dot(g, direction);

            f64 ag = std::fabs((f64)grad);

            if (ag < 1e-4)
                continue;

            /*
             * 使用高斯权重。
             */
            f64 w = std::exp(
                -(f64)(i * i) /
                (2.0 * 4.0 * 4.0)
            );

            numerator += w * (f64)t * (f64)grad;
            denominator += w * (f64)grad;

            /*
             * 这里只用于累积局部边缘信息。
             */
            (void)ag;
        }

        if (std::fabs(denominator) < 1e-8)
            return false;

        /*
         * 一阶中心估计。
         */
        offset = (f32)(numerator / denominator);

        /*
         * 防止异常梯度导致跳飞。
         */
        if (offset < -3.0f || offset > 3.0f)
            return false;

        return true;
    }

    /*
     * 用局部灰度的一阶导数建立角点约束。
     *
     * 理想情况下：
     *
     *     edge1: n1 · (p - p0) = 0
     *     edge2: n2 · (p - p0) = 0
     *
     * 两条边的交点就是棋盘格角点。
     *
     * 实际图像中存在噪声，因此我们使用多个像素的
     * 加权最小二乘来求 p。
     */
    static bool refine_by_gradient_intersection(
        const GrayImage& img,
        const Point2f& initial,
        int radius,
        Point2f& result)
    {
        f64 A00 = 0.0;
        f64 A01 = 0.0;
        f64 A11 = 0.0;

        f64 B0 = 0.0;
        f64 B1 = 0.0;

        f64 total_weight = 0.0;

        /*
         * 梯度结构张量。
         */
        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dx = -radius; dx <= radius; ++dx) {

                f32 x = initial.x + (f32)dx;
                f32 y = initial.y + (f32)dy;

                if (x < 2.0f || x >= (f32)(img.w - 3) ||
                    y < 2.0f || y >= (f32)(img.h - 3))
                    continue;

                Vec2 g = gradient_at(img, x, y);

                f64 gx = g.x;
                f64 gy = g.y;

                f64 mag2 = gx * gx + gy * gy;

                if (mag2 < 1e-4)
                    continue;

                /*
                 * 对梯度做归一化。
                 *
                 * 这样强边缘不会完全压制弱边缘。
                 */
                f64 mag = std::sqrt(mag2);

                f64 nx = gx / mag;
                f64 ny = gy / mag;

                /*
                 * 从当前像素到候选点。
                 */
                f64 qx = (f64)x;
                f64 qy = (f64)y;

                /*
                 * 梯度方向上的直线：
                 *
                 *     n · (p - q) = 0
                 *
                 * 即：
                 *
                 *     n.x * px + n.y * py = n.x*q.x+n.y*q.y
                 */
                f64 rhs = nx * qx + ny * qy;

                /*
                 * 中心加权。
                 */
                f64 rr = (f64)(dx * dx + dy * dy);
                f64 sigma = (f64)radius * 0.7 + 1.0;

                f64 w = std::exp(
                    -rr / (2.0 * sigma * sigma)
                );

                /*
                 * 为了避免把所有方向完全一样的边缘
                 * 当成棋盘格角点，这里让强梯度稍微占优。
                 */
                w *= std::min(1.0, mag / 20.0);

                if (w < 1e-6)
                    continue;

                A00 += w * nx * nx;
                A01 += w * nx * ny;
                A11 += w * ny * ny;

                B0 += w * nx * rhs;
                B1 += w * ny * rhs;

                total_weight += w;
            }
        }

        if (total_weight < 1e-5)
            return false;

        f64 det = A00 * A11 - A01 * A01;

        /*
         * 如果所有梯度都来自同一条边，
         * 这个矩阵会接近奇异。
         */
        if (std::fabs(det) < 1e-8)
            return false;

        f64 px = (B0 * A11 - A01 * B1) / det;
        f64 py = (A00 * B1 - A01 * B0) / det;

        /*
         * 不能离原始 Shi-Tomasi 点太远。
         */
        f64 dx = px - (f64)initial.x;
        f64 dy = py - (f64)initial.y;

        f64 shift2 = dx * dx + dy * dy;

        if (shift2 > 16.0)
            return false;

        result.x = (f32)px;
        result.y = (f32)py;

        return true;
    }

    /*
     * 再做一次非常轻的二次局部优化。
     *
     * 这一步主要用于减少离散采样造成的亚像素偏差。
     */
    static bool final_local_adjust(
        const GrayImage& img,
        Point2f& p)
    {
        const int radius = 4;

        f64 sx = 0.0;
        f64 sy = 0.0;
        f64 sw = 0.0;

        /*
         * 对梯度能量做加权。
         *
         * 注意这里不是简单的亮暗质心。
         */
        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dx = -radius; dx <= radius; ++dx) {

                f32 x = p.x + (f32)dx;
                f32 y = p.y + (f32)dy;

                if (x < 2.0f || x >= (f32)(img.w - 3) ||
                    y < 2.0f || y >= (f32)(img.h - 3))
                    continue;

                Vec2 g = gradient_at(img, x, y);

                f64 e = (f64)g.x * g.x +
                    (f64)g.y * g.y;

                if (e < 1e-5)
                    continue;

                /*
                 * 中心权重。
                 */
                f64 rr = (f64)(dx * dx + dy * dy);

                f64 w = std::exp(
                    -rr / (2.0 * 2.5 * 2.5)
                );

                w *= std::sqrt(e);

                sx += w * x;
                sy += w * y;
                sw += w;
            }
        }

        if (sw < 1e-6)
            return false;

        f32 nx = (f32)(sx / sw);
        f32 ny = (f32)(sy / sw);

        /*
         * 只允许非常小的修正。
         */
        f32 dx = nx - p.x;
        f32 dy = ny - p.y;

        f32 len2 = dx * dx + dy * dy;

        if (len2 > 1.0f) {
            f32 len = std::sqrt(len2);

            dx /= len;
            dy /= len;

            dx *= 1.0f;
            dy *= 1.0f;
        }

        p.x += dx * 0.25f;
        p.y += dy * 0.25f;

        return true;
    }

} // namespace


void refine_subpixel(
    const GrayImage& img,
    std::vector<Point2f>& corners,
    int half_win)
{
    if (img.w <= 0 || img.h <= 0)
        return;

    if (corners.empty())
        return;

    /*
     * half_win 是原接口参数。
     *
     * 对当前棋盘格测试：
     *
     *     cell = 50 px
     *
     * 因此 half_win=7~9 比较合适。
     *
     * 这里限制一下范围，避免用户传入极端值。
     */
    int radius = half_win;

    if (radius < 3)
        radius = 3;

    if (radius > 15)
        radius = 15;

    for (auto& corner : corners) {

        /*
         * 保留 Shi-Tomasi 原始位置。
         */
        Point2f original = corner;

        /*
         * 第一步：
         * 检查这个位置是否真的具有二维角点结构。
         */
        f64 a, b, c;

        if (!compute_structure_tensor(
            img,
            original.x,
            original.y,
            radius,
            a, b, c))
        {
            /*
             * 如果局部结构不可靠，
             * 保留原始检测结果，而不是乱移动。
             */
            continue;
        }

        /*
         * 第二步：
         * 求局部主要方向。
         *
         * 这一步主要用于确认这里确实存在两组不同方向的边缘。
         */
        Vec2 dir1, dir2;

        if (!principal_direction(
            a, b, c,
            dir1, dir2))
        {
            continue;
        }

        /*
         * 检查两方向是否足够不同。
         *
         * 对棋盘格角点，两组边缘通常近似正交。
         */
        f32 d = std::fabs(dot(dir1, dir2));

        if (d > 0.85f)
            continue;

        /*
         * 第三步：
         * 使用所有局部梯度方向做交点拟合。
         */
        Point2f refined;

        bool ok = refine_by_gradient_intersection(
            img,
            original,
            radius,
            refined);

        if (!ok) {
            /*
             * 如果梯度交点拟合失败，
             * 不使用不可靠结果。
             */
            continue;
        }

        /*
         * 第四步：
         * 做一次非常小的局部调整。
         */
        final_local_adjust(img, refined);

        /*
         * 最终限制移动范围。
         *
         * Shi-Tomasi 已经给出了比较好的整数像素位置，
         * subpixel 阶段原则上只能做局部修正。
         */
        f32 dx = refined.x - original.x;
        f32 dy = refined.y - original.y;

        f32 shift2 = dx * dx + dy * dy;

        if (shift2 > 9.0f) {
            /*
             * 最大移动 3 px。
             */
            f32 len = std::sqrt(shift2);

            refined.x =
                original.x + dx * (3.0f / len);

            refined.y =
                original.y + dy * (3.0f / len);
        }

        /*
         * 边界保护。
         */
        refined.x = clamp_f32(
            refined.x,
            0.0f,
            (f32)(img.w - 1));

        refined.y = clamp_f32(
            refined.y,
            0.0f,
            (f32)(img.h - 1));

        corner = refined;
    }
}