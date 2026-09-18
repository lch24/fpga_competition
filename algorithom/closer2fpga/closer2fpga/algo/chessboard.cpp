#define _CRT_SECURE_NO_WARNINGS
#include "chessboard.h"
#include "shi_tomasi.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

// Merge multiple Shi-Tomasi responses that belong to the same physical corner.
static void cluster_corners(std::vector<Point2f>& pts, f32 radius)
{
    if (pts.empty())
        return;

    std::vector<bool> used(pts.size(), false);
    std::vector<Point2f> clustered;

    const f32 r2 = radius * radius;

    for (int i = 0; i < (int)pts.size(); ++i)
    {
        if (used[i])
            continue;

        f32 sx = pts[i].x;
        f32 sy = pts[i].y;
        int count = 1;
        used[i] = true;

        for (int j = i + 1; j < (int)pts.size(); ++j)
        {
            if (used[j])
                continue;

            f32 dx = pts[j].x - pts[i].x;
            f32 dy = pts[j].y - pts[i].y;

            if (dx * dx + dy * dy < r2)
            {
                sx += pts[j].x;
                sy += pts[j].y;
                ++count;
                used[j] = true;
            }
        }

        clustered.push_back(
            Point2f(sx / (f32)count, sy / (f32)count)
        );
    }

    pts = std::move(clustered);
}

// Compute 3x3 Sobel gradient (Gx, Gy) at pixel (x,y).
static void sobel3(
    const GrayImage& gray, int x, int y,
    f32& gx, f32& gy)
{
    int w = gray.w;
    int h = gray.h;

    if (x <= 0 || y <= 0 || x >= w - 1 || y >= h - 1)
    {
        gx = 0.0f;
        gy = 0.0f;
        return;
    }

    const uint8_t I[3][3] = {
        {gray.get(x - 1, y - 1), gray.get(x, y - 1), gray.get(x + 1, y - 1)},
        {gray.get(x - 1, y),     gray.get(x, y),     gray.get(x + 1, y)},
        {gray.get(x - 1, y + 1), gray.get(x, y + 1), gray.get(x + 1, y + 1)},
    };

    gx =
        -I[0][0] - 2.0f * I[0][1] - I[0][2]
        + I[2][0] + 2.0f * I[2][1] + I[2][2];

    gy =
        -I[0][0] - 2.0f * I[1][0] - I[2][0]
        + I[0][2] + 2.0f * I[1][2] + I[2][2];
}

// Keep only chessboard INTERIOR corners.
//
// An interior chessboard corner sits at the intersection of FOUR alternating
// black/white cells.  Divide a small window around the candidate into four
// quadrants (NE, NW, SW, SE) and check that opposite quadrants agree in
// brightness while adjacent quadrants differ:
//     QNE ~= QSW  (diagonal same color)
//     QNW ~= QSE  (diagonal same color)
//     |QNE - QNW| is large (adjacent different color)
//
// This directly validates the "4-cell junction" geometry and is immune
// to lens distortion, perspective skew, and background clutter.
static std::vector<Point2f> filter_inner_corners(
    const GrayImage& gray,
    const std::vector<Point2f>& pts)
{
    std::vector<Point2f> kept;

    const int win_r = 10;

    for (const auto& p : pts)
    {
        int cx = (int)(p.x + 0.5f);
        int cy = (int)(p.y + 0.5f);

        if (cx <= win_r || cy <= win_r ||
            cx >= gray.w - win_r || cy >= gray.h - win_r)
            continue;

        f64 sumNE = 0.0, sumNW = 0.0, sumSW = 0.0, sumSE = 0.0;
        int cntNE = 0, cntNW = 0, cntSW = 0, cntSE = 0;

        for (int dy = -win_r; dy <= win_r; ++dy)
        {
            for (int dx = -win_r; dx <= win_r; ++dx)
            {
                if (dx == 0 && dy == 0)
                    continue;

                uint8_t v = gray.get(cx + dx, cy + dy);

                if (dx > 0 && dy < 0)      { sumNE += v; cntNE++; }
                else if (dx < 0 && dy < 0) { sumNW += v; cntNW++; }
                else if (dx < 0 && dy > 0) { sumSW += v; cntSW++; }
                else if (dx > 0 && dy > 0) { sumSE += v; cntSE++; }
            }
        }

        if (cntNE == 0 || cntNW == 0 || cntSW == 0 || cntSE == 0)
            continue;

        f64 mNE = sumNE / cntNE;
        f64 mNW = sumNW / cntNW;
        f64 mSW = sumSW / cntSW;
        f64 mSE = sumSE / cntSE;

        f64 bright = std::max({mNE, mNW, mSW, mSE});
        f64 dark   = std::min({mNE, mNW, mSW, mSE});

        if (bright - dark < 40.0)
            continue;

        f64 diag1_diff = std::fabs(mNE - mSW);
        f64 diag2_diff = std::fabs(mNW - mSE);
        f64 adj_diff_a = std::fabs(mNE - mNW);
        f64 adj_diff_b = std::fabs(mNE - mSE);

        if (diag1_diff > (bright - dark) * 0.45)
            continue;
        if (diag2_diff > (bright - dark) * 0.45)
            continue;

        if (adj_diff_a < (bright - dark) * 0.30)
            continue;
        if (adj_diff_b < (bright - dark) * 0.30)
            continue;

        kept.push_back(p);
    }

    return kept;
}

// Use RANSAC to repeatedly fit rows as straight lines.  Perspective skew
// means rows are not horizontal (constant y), but they remain roughly
// colinear.  We extract row_count lines one by one.
static bool group_rows_by_y(
    const std::vector<Point2f>& pts,
    int row_count,
    int col_count,
    std::vector<std::vector<Point2f>>& rows)
{
    rows.clear();

    if ((int)pts.size() < row_count * (col_count - 1))
        return false;

    std::vector<Point2f> remaining = pts;

    for (int r = 0; r < row_count; ++r)
    {
        if ((int)remaining.size() < col_count - 1)
            return false;

        std::vector<Point2f> best_inliers;
        f32 best_err = 1e9f;

        for (int trial = 0; trial < 400; ++trial)
        {
            if ((int)remaining.size() < 2) break;

            int i = rand() % remaining.size();
            int j = rand() % remaining.size();
            if (i == j) continue;

            Point2f a = remaining[i];
            Point2f b = remaining[j];
            f32 dx = b.x - a.x;
            f32 dy = b.y - a.y;
            f32 len2 = dx * dx + dy * dy;
            if (len2 < 4.0f) continue;

            f32 nrm = std::sqrt(len2);
            f32 nx = dx / nrm;
            f32 ny = dy / nrm;

            std::vector<Point2f> inliers;
            f32 err_sum = 0.0f;

            for (auto& p : remaining)
            {
                f32 px = p.x - a.x;
                f32 py = p.y - a.y;
                f32 dist = std::fabs(px * ny - py * nx);

                if (dist < 18.0f)
                {
                    inliers.push_back(p);
                    err_sum += dist;
                }
            }

            int n = (int)inliers.size();
            if (n < col_count - 2 || n > col_count + 2)
                continue;

            f32 err = err_sum / n;
            if (err < best_err)
            {
                best_err = err;
                best_inliers = inliers;
            }
        }

        if (best_inliers.empty())
            return false;

        printf("  [chessboard] row %d has %zu points", r, best_inliers.size());
        for (auto& p : best_inliers)
            printf(" (%.0f,%.0f)", p.x, p.y);
        printf("\n");

        for (auto& p : best_inliers)
        {
            auto it = std::find_if(remaining.begin(), remaining.end(),
                [&](const Point2f& q) {
                    return std::fabs(q.x - p.x) < 0.1f &&
                           std::fabs(q.y - p.y) < 0.1f;
                });
            if (it != remaining.end())
                remaining.erase(it);
        }

        rows.push_back(std::move(best_inliers));
    }

    std::sort(rows.begin(), rows.end(),
        [](const std::vector<Point2f>& a, const std::vector<Point2f>& b)
        {
            f32 ay = 0.0f, by = 0.0f;
            for (auto& p : a) ay += p.y;
            ay /= a.size();
            for (auto& p : b) by += p.y;
            by /= b.size();
            return ay < by;
        });

    for (int r = 0; r < row_count; ++r)
    {
        std::sort(rows[r].begin(), rows[r].end(),
            [](const Point2f& a, const Point2f& b)
            {
                return a.x < b.x;
            });
    }

    return true;
}

// Public API:
//
// rows / cols mean INTERNAL corner counts.
//
// Example:
//   6x8 squares -> 5x7 internal corners
//   detect_chessboard(gray, 5, 7)
ChessboardInfo detect_chessboard(
    const GrayImage& gray,
    int rows,
    int cols)
{
    ChessboardInfo info = {};
    info.rows = rows;
    info.cols = cols;
    info.valid = false;

    if (rows <= 0 || cols <= 0)
        return info;

    const int expected_inner = rows * cols;

    std::vector<Point2f> candidates;

    shi_tomasi_detect(
        gray,
        candidates,
        0.15f,
        3
    );

    printf("  [chessboard] shi_tomasi raw: %zu\n", candidates.size());

    if ((int)candidates.size() < expected_inner)
    {
        printf("  [chessboard] not enough raw candidates\n");
        return info;
    }

    cluster_corners(candidates, 18.0f);

    printf("  [chessboard] after cluster: %zu\n", candidates.size());

    info.all_candidates = candidates;

    candidates = filter_inner_corners(gray, candidates);

    printf("  [chessboard] after inner filter: %zu\n", candidates.size());

    if ((int)candidates.size() < expected_inner)
    {
        printf("  [chessboard] not enough inner candidates\n");
        return info;
    }

    std::vector<std::vector<Point2f>> grid_rows;

    if (!group_rows_by_y(candidates, rows, cols, grid_rows))
    {
        printf("  [chessboard] row grouping FAILED\n");
        return info;
    }

    printf("  [chessboard] rows:");
    for (int r = 0; r < rows; ++r)
        printf(" %d", (int)grid_rows[r].size());
    printf("\n");

    for (int r = 0; r < rows; ++r)
    {
        if ((int)grid_rows[r].size() != cols)
        {
            printf("  [chessboard] row %d has %zu points, need %d\n",
                r, grid_rows[r].size(), cols);
            return info;
        }
    }

    std::vector<Point2f> inner;
    inner.reserve(expected_inner);

    for (int r = 0; r < rows; ++r)
    {
        std::vector<Point2f> row = grid_rows[r];
        std::sort(
            row.begin(), row.end(),
            [](const Point2f& a, const Point2f& b)
            {
                return a.x < b.x;
            });

        for (auto& p : row)
            inner.push_back(p);
    }

    for (int r = 0; r < rows; ++r)
    {
        for (int c = 1; c < cols; ++c)
        {
            const Point2f& a = inner[r * cols + c - 1];
            const Point2f& b = inner[r * cols + c];

            if (b.x <= a.x)
            {
                printf("  [chessboard] invalid x order at (%d,%d)\n", r, c);
                return info;
            }
        }

        if (r > 0)
        {
            const Point2f& above = inner[(r - 1) * cols];
            const Point2f& below = inner[r * cols];

            if (below.y <= above.y)
            {
                printf("  [chessboard] invalid y order at row %d\n", r);
                return info;
            }
        }
    }

    info.corners = std::move(inner);
    info.valid = true;

    printf("  [chessboard] internal corners: %zu\n", info.corners.size());
    printf("  [chessboard] valid: YES\n");

    return info;
}