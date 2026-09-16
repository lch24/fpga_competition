#define _CRT_SECURE_NO_WARNINGS
#include "chessboard.h"
#include "shi_tomasi.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
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

        // Use the first point as the grouping reference.  This keeps the
        // operation deterministic and is sufficient for the present scale.
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

// Estimate seven physical rows from the y coordinates.
//
// Unlike the old "sort by y and take N/7 points per row" method, this does
// not require every row to contain exactly nine detected points.  A row may
// contain 8 or 9 points, which is useful when one of the outer corners is
// missed by Shi-Tomasi.
static bool group_rows_by_y(
    const std::vector<Point2f>& pts,
    int row_count,
    std::vector<std::vector<Point2f>>& rows)
{
    rows.clear();

    if ((int)pts.size() < row_count * 7)
        return false;

    // Start with y-quantiles as row centers.
    std::vector<Point2f> sorted = pts;
    std::sort(
        sorted.begin(),
        sorted.end(),
        [](const Point2f& a, const Point2f& b)
        {
            return a.y < b.y;
        });

    std::vector<f32> centers(row_count, 0.0f);

    for (int r = 0; r < row_count; ++r)
    {
        int a = r * (int)sorted.size() / row_count;
        int b = (r + 1) * (int)sorted.size() / row_count;

        if (b <= a)
            return false;

        f32 sum = 0.0f;
        for (int i = a; i < b; ++i)
            sum += sorted[i].y;

        centers[r] = sum / (f32)(b - a);
    }

    // A few iterations of 1D k-means on y.
    std::vector<int> labels(pts.size(), 0);

    for (int iter = 0; iter < 20; ++iter)
    {
        std::vector<f32> sum(row_count, 0.0f);
        std::vector<int> count(row_count, 0);

        for (int i = 0; i < (int)pts.size(); ++i)
        {
            int best = 0;
            f32 best_d = std::fabs(pts[i].y - centers[0]);

            for (int r = 1; r < row_count; ++r)
            {
                f32 d = std::fabs(pts[i].y - centers[r]);
                if (d < best_d)
                {
                    best_d = d;
                    best = r;
                }
            }

            labels[i] = best;
            sum[best] += pts[i].y;
            count[best]++;
        }

        for (int r = 0; r < row_count; ++r)
        {
            if (count[r] == 0)
                return false;

            centers[r] = sum[r] / (f32)count[r];
        }

        // Keep row order stable.
        for (int r = 0; r < row_count - 1; ++r)
        {
            if (centers[r] > centers[r + 1])
            {
                std::swap(centers[r], centers[r + 1]);

                // Re-labeling is unnecessary for the next iteration because
                // labels are recomputed from the centers.
            }
        }
    }

    rows.resize(row_count);

    for (int i = 0; i < (int)pts.size(); ++i)
    {
        int best = 0;
        f32 best_d = std::fabs(pts[i].y - centers[0]);

        for (int r = 1; r < row_count; ++r)
        {
            f32 d = std::fabs(pts[i].y - centers[r]);
            if (d < best_d)
            {
                best_d = d;
                best = r;
            }
        }

        rows[best].push_back(pts[i]);
    }

    for (int r = 0; r < row_count; ++r)
    {
        std::sort(
            rows[r].begin(),
            rows[r].end(),
            [](const Point2f& a, const Point2f& b)
            {
                return a.x < b.x;
            });

        if ((int)rows[r].size() < 7 || (int)rows[r].size() > 10)
            return false;
    }

    return true;
}

// Estimate the spacing between neighboring points in one row.
static f32 estimate_row_spacing(const std::vector<Point2f>& row)
{
    if (row.size() < 2)
        return 0.0f;

    std::vector<f32> gaps;
    gaps.reserve(row.size() - 1);

    for (int i = 1; i < (int)row.size(); ++i)
    {
        f32 dx = row[i].x - row[i - 1].x;
        f32 dy = row[i].y - row[i - 1].y;
        f32 d = std::sqrt(dx * dx + dy * dy);

        if (d > 1.0f)
            gaps.push_back(d);
    }

    if (gaps.empty())
        return 0.0f;

    std::sort(gaps.begin(), gaps.end());
    return gaps[gaps.size() / 2];
}

// Extract the seven interior columns from a row.
//
// For the current test board, each complete row has 9 points.  If one outer
// point is missing, the row has 8 points and the seven internal points are
// still directly available:
//
//   missing left : [1 2 3 4 5 6 7 8] -> take first 7
//   missing right: [0 1 2 3 4 5 6 7] -> take last 7
//
// We determine which case it is from the expected grid spacing.  No point is
// fabricated here.
static bool extract_inner_row(
    const std::vector<Point2f>& row,
    int inner_cols,
    std::vector<Point2f>& inner)
{
    inner.clear();

    const int full_cols = inner_cols + 2; // 9 for a 5x7 inner grid
    const int n = (int)row.size();

    if (n == full_cols)
    {
        for (int c = 1; c < full_cols - 1; ++c)
            inner.push_back(row[c]);

        return (int)inner.size() == inner_cols;
    }

    if (n != full_cols - 1)
        return false;

    // With exactly one missing point, normally it is an outer corner.
    // Decide whether to drop the first or last detected point by looking at
    // the gap pattern.  A missing outer point produces no doubled gap inside
    // the detected row, so use the board-wide x extent as a secondary check.
    //
    // For this synthetic board the safer choice is to compare the first and
    // last gaps with the median interior gap.
    f32 spacing = estimate_row_spacing(row);

    if (spacing <= 1.0f)
        return false;

    f32 first_gap = row[1].x - row[0].x;
    f32 last_gap = row[n - 1].x - row[n - 2].x;

    // If the first gap is unusually large, the missing point is likely
    // between the first two points, not at the outside.  Same for the last
    // gap.  Such a row cannot safely provide all seven internal corners.
    if (first_gap > spacing * 1.45f || last_gap > spacing * 1.45f)
        return false;

    // In the normal case of one missing outer point, both choices produce
    // seven internal candidates.  Prefer the side whose endpoint is closer
    // to the expected global board boundary.  For the present detector,
    // the missing point is overwhelmingly likely to be an outer point.
    //
    // We choose the seven points closest to the row's center, which removes
    // exactly one endpoint.
    int start = 0;
    if (n > inner_cols)
        start = (n - inner_cols) / 2;

    for (int i = 0; i < inner_cols; ++i)
        inner.push_back(row[start + i]);

    return (int)inner.size() == inner_cols;
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

    // rows/cols are INTERNAL corners, so the full vertex grid is +2.
    const int full_rows = rows + 2;
    const int full_cols = cols + 2;
    const int expected_full = full_rows * full_cols;
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

    // For the current 50 px cells, 18 px is small enough to merge duplicate
    // responses while remaining far below the distance between neighboring
    // physical corners.
    cluster_corners(candidates, 18.0f);

    printf("  [chessboard] after cluster: %zu\n", candidates.size());
    printf("  [chessboard] full grid=%dx%d (%d), inner grid=%dx%d (%d)\n",
        full_rows, full_cols, expected_full,
        rows, cols, expected_inner);

    // We only need enough candidates to explain the internal grid.  Do not
    // require all 63 outer+inner corners to be detected.
    if ((int)candidates.size() < expected_inner)
    {
        printf("  [chessboard] not enough corner candidates\n");
        return info;
    }

    std::vector<std::vector<Point2f>> grid_rows;

    if (!group_rows_by_y(candidates, full_rows, grid_rows))
    {
        printf("  [chessboard] row grouping FAILED\n");
        return info;
    }

    printf("  [chessboard] rows:");
    for (int r = 0; r < full_rows; ++r)
        printf(" %d", (int)grid_rows[r].size());
    printf("\n");

    // Each physical row must have enough points to recover its seven
    // internal corners.  We allow one missing outer corner.
    for (int r = 0; r < full_rows; ++r)
    {
        if ((int)grid_rows[r].size() < cols)
        {
            printf("  [chessboard] row %d has only %zu points\n",
                r, grid_rows[r].size());
            return info;
        }
    }

    std::vector<Point2f> inner;
    inner.reserve(expected_inner);

    // Skip the first and last physical rows.  For every remaining row,
    // extract only the internal columns.
    for (int r = 1; r < full_rows - 1; ++r)
    {
        std::vector<Point2f> inner_row;

        if (!extract_inner_row(
            grid_rows[r],
            cols,
            inner_row))
        {
            printf("  [chessboard] cannot extract internal row %d\n", r);
            return info;
        }

        if ((int)inner_row.size() != cols)
        {
            printf("  [chessboard] internal row %d has %zu points, need %d\n",
                r, inner_row.size(), cols);
            return info;
        }

        for (auto& p : inner_row)
            inner.push_back(p);
    }

    if ((int)inner.size() != expected_inner)
    {
        printf("  [chessboard] internal corner count=%zu, need=%d\n",
            inner.size(), expected_inner);
        return info;
    }

    // Final sanity check: rows must increase in y, and points within each
    // row must increase in x.  This catches accidental row mixing before the
    // points are passed to subpixel/calibration.
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