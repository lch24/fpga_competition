#define _CRT_SECURE_NO_WARNINGS
#include "chessboard.h"
#include "shi_tomasi.h"
#include <algorithm>
#include <vector>
#include <cmath>

static void cluster_corners(std::vector<Point2f>& pts, f32 radius) {
    std::vector<bool> used(pts.size(), false);
    std::vector<Point2f> clustered;

    for (int i = 0; i < (int)pts.size(); ++i) {
        if (used[i]) continue;
        std::vector<int> group;
        group.push_back(i);
        used[i] = true;
        for (int j = i + 1; j < (int)pts.size(); ++j) {
            if (used[j]) continue;
            f32 dx = pts[j].x - pts[i].x;
            f32 dy = pts[j].y - pts[i].y;
            if (dx * dx + dy * dy < radius * radius) {
                group.push_back(j);
                used[j] = true;
            }
        }
        f32 sx = 0, sy = 0;
        for (int idx : group) { sx += pts[idx].x; sy += pts[idx].y; }
        clustered.push_back(Point2f(sx / group.size(), sy / group.size()));
    }
    pts = std::move(clustered);
}

static bool sort_corners_grid(std::vector<Point2f>& corners, int rows, int cols) {
    int row_count = rows + 1;
    int col_count = cols + 1;
    int need = row_count * col_count;

    if ((int)corners.size() < std::max(need - 4, 8)) return false;

    std::sort(corners.begin(), corners.end(),
        [](const Point2f& a, const Point2f& b) { return a.y < b.y; });

    std::vector<std::vector<Point2f>> row_groups(row_count);
    int N = (int)corners.size();
    int per_row = N / row_count;
    int remainder = N % row_count;
    int idx = 0;
    for (int r = 0; r < row_count; ++r) {
        int cnt = per_row + (r < remainder ? 1 : 0);
        for (int i = 0; i < cnt && idx < N; ++i, ++idx)
            row_groups[r].push_back(corners[idx]);
    }

    std::vector<Point2f> sorted;
    sorted.reserve(row_count * col_count);
    for (auto& row : row_groups) {
        std::sort(row.begin(), row.end(),
            [](const Point2f& a, const Point2f& b) { return a.x < b.x; });
        int take = std::min((int)row.size(), col_count);
        for (int i = 0; i < take; ++i)
            sorted.push_back(row[i]);
    }

    if ((int)sorted.size() < need - 4) return false;
    while ((int)sorted.size() < need) {
        Point2f last = sorted.empty() ? Point2f{0,0} : sorted.back();
        sorted.push_back(last);
    }

    corners = std::move(sorted);
    return true;
}

ChessboardInfo detect_chessboard(const GrayImage& gray, int rows, int cols) {
    ChessboardInfo info = {};
    info.rows = rows;
    info.cols = cols;
    info.valid = false;

    std::vector<Point2f> candidates;
    shi_tomasi_detect(gray, candidates, 0.15f, 3);
    printf("  [chessboard] shi_tomasi raw: %zu\n", candidates.size());

    cluster_corners(candidates, 18.0f);
    printf("  [chessboard] after cluster: %zu\n", candidates.size());

    if ((int)candidates.size() < 4) return info;

    int row_count = rows + 1;
    int col_count = cols + 1;
    int need = row_count * col_count;
    printf("  [chessboard] need=%d, have=%d, min_allowed=%d\n",
        need, (int)candidates.size(), std::max(need - 4, 8));

    if (!sort_corners_grid(candidates, rows, cols)) {
        printf("  [chessboard] sort_corners_grid FAILED\n");
        return info;
    }
    printf("  [chessboard] after sort: %zu\n", candidates.size());

    info.corners = std::move(candidates);
    info.valid = true;
    return info;
}