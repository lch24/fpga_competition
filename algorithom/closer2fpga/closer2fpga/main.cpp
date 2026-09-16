#define _CRT_SECURE_NO_WARNINGS
#include <cstdio>
#include <cmath>
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "common/types.h"
#include "common/image.h"
#include "algo/threshold.h"
#include "algo/chessboard.h"
#include "algo/shi_tomasi.h"
#include "algo/subpixel.h"
#include "algo/undistort.h"
#include "algo/calibrate.h"

static GrayImage make_checkerboard(int cell, int rows, int cols, int pad = 10) {
    int h = cell * rows + pad * 2;
    int w = cell * cols + pad * 2;
    GrayImage img(w, h);
    for (int i = 0; i < w * h; ++i) img.data[i] = 0;
    for (int y = pad; y < pad + cell * rows; ++y) {
        int ry = (y - pad) / cell;
        for (int x = pad; x < pad + cell * cols; ++x) {
            int rx = (x - pad) / cell;
            img.set(x, y, ((rx + ry) % 2 == 0) ? 255 : 0);
        }
    }
    return img;
}

static void print_points(const char* tag, const std::vector<Point2f>& pts) {
    if (pts.empty()) { printf("  [%s] EMPTY\n", tag); return; }
    f32 x_min=1e9, x_max=-1e9, y_min=1e9, y_max=-1e9;
    f32 sx=0, sy=0;
    for (auto& p : pts) {
        x_min=std::min(x_min,p.x); x_max=std::max(x_max,p.x);
        y_min=std::min(y_min,p.y); y_max=std::max(y_max,p.y);
        sx+=p.x; sy+=p.y;
    }
    printf("  [%s] n=%zu  x=[%.0f..%.0f] mean_x=%.1f  y=[%.0f..%.0f] mean_y=%.1f\n",
        tag, pts.size(), x_min, x_max, sx/pts.size(), y_min, y_max, sy/pts.size());
}

int main() {
    int pad = 60, cell = 50, rows = 6, cols = 8;
    int row_count = rows + 1;
    int col_count = cols + 1;
    int need = row_count * col_count;

    printf("=== Diagnostics ===\n\n");

    GrayImage ideal_img = make_checkerboard(cell, rows, cols, pad);
    printf("Ideal image: %dx%d\n", ideal_img.w, ideal_img.h);

    CameraParams cam = {};
    cam.fx = ideal_img.w * 0.85f;
    cam.fy = ideal_img.h * 0.85f;
    cam.cx = ideal_img.w * 0.5f;
    cam.cy = ideal_img.h * 0.5f;
    cam.k1 = -0.3f;
    cam.k2 =  0.15f;
    cam.p1 =  0.02f;
    cam.p2 = -0.01f;
    cam.k3 =  0.0f;
    cam.valid = true;

    GrayImage dist_img;
    make_distorted_image(ideal_img, cam, dist_img);
    printf("Distorted: %dx%d\n\n", dist_img.w, dist_img.h);

    printf("--- Step 1: Detect on IDEAL (no distortion) ---\n");
    {
        std::vector<Point2f> raw;
        shi_tomasi_detect(ideal_img, raw, 0.15f, 3);
        print_points("ideal shi_tomasi on GRAY", raw);

        ChessboardInfo info = detect_chessboard(ideal_img, rows, cols);
        if (info.valid) {
            print_points("ideal detected", info.corners);
            refine_subpixel(ideal_img, info.corners, 7);
            print_points("ideal+subpixel", info.corners);

            f32 mean_e = 0, max_e = 0;
            for (int i = 0; i < need; ++i) {
                f32 tx = pad + (i % col_count) * cell;
                f32 ty = pad + (i / col_count) * cell;
                f32 dx = info.corners[i].x - tx;
                f32 dy = info.corners[i].y - ty;
                f32 e = std::sqrt(dx*dx + dy*dy);
                mean_e += e;
                if (e > max_e) max_e = e;
            }
            mean_e /= need;
            printf("  IDEAL accuracy: mean=%.3f max=%.3f px\n\n", mean_e, max_e);
        } else {
            printf("  ideal detect FAILED\n\n");
        }
    }

    printf("--- Step 2: Detect on DISTORTED (gray) ---\n");
    {
        std::vector<Point2f> raw;
        shi_tomasi_detect(dist_img, raw, 0.15f, 3);
        print_points("dist shi_tomasi on GRAY", raw);

        ChessboardInfo info = detect_chessboard(dist_img, rows, cols);
        if (!info.valid) {
            printf("FAIL: detect_chessboard valid=false\n");
            cv::Mat m(dist_img.h, dist_img.w, CV_8UC1, dist_img.data);
            cv::imshow("Distorted Gray", m);
            cv::waitKey(0);
            return 1;
        }
        print_points("dist detected", info.corners);

        refine_subpixel(dist_img, info.corners, 7);
        print_points("dist detected (+sub)", info.corners);

        std::vector<Point2f> ideal_pts, truth_dist;
        for (int r = 0; r <= rows; ++r)
            for (int c = 0; c <= cols; ++c)
                ideal_pts.push_back({(f32)(pad + c * cell), (f32)(pad + r * cell)});
        for (auto& ip : ideal_pts) {
            f32 nx = (ip.x - cam.cx) / cam.fx;
            f32 ny = (ip.y - cam.cy) / cam.fy;
            f32 xd, yd;
            forward_distort_norm(nx, ny, cam, xd, yd);
            truth_dist.push_back({cam.fx * xd + cam.cx, cam.fy * yd + cam.cy});
        }

        f32 mean_err = 0, max_err = 0;
        for (int i = 0; i < (int)info.corners.size(); ++i) {
            f32 dx = info.corners[i].x - truth_dist[i].x;
            f32 dy = info.corners[i].y - truth_dist[i].y;
            f32 e = std::sqrt(dx*dx + dy*dy);
            mean_err += e;
            if (e > max_err) max_err = e;
        }
        mean_err /= info.corners.size();
        printf("\nDistorted accuracy: mean=%.3f max=%.3f px\n", mean_err, max_err);

        printf("\n  Per-corner errors (first 10):\n");
        for (int i = 0; i < std::min(10, (int)info.corners.size()); ++i) {
            f32 dx = info.corners[i].x - truth_dist[i].x;
            f32 dy = info.corners[i].y - truth_dist[i].y;
            f32 e = std::sqrt(dx*dx + dy*dy);
            printf("    [%d] det=(%.1f,%.1f) truth=(%.1f,%.1f)  err=%.2f\n",
                i, info.corners[i].x, info.corners[i].y,
                truth_dist[i].x, truth_dist[i].y, e);
        }

        cv::Mat dist_mat(dist_img.h, dist_img.w, CV_8UC1, dist_img.data);
        cv::Mat disp;
        cv::cvtColor(dist_mat, disp, cv::COLOR_GRAY2BGR);
        for (auto& c : info.corners)
            cv::circle(disp, cv::Point((int)c.x, (int)c.y), 5, cv::Scalar(0,0,255), -1);
        for (auto& t : truth_dist)
            cv::circle(disp, cv::Point((int)t.x, (int)t.y), 4, cv::Scalar(0,255,0), 1);
        int scale = 2;
        cv::Mat big;
        cv::resize(disp, big, cv::Size(disp.cols*scale, disp.rows*scale), 0, 0, cv::INTER_NEAREST);
        cv::putText(big, "Red=detected  Green=ground truth",
            cv::Point(10*scale, 30*scale), cv::FONT_HERSHEY_SIMPLEX,
            0.6*scale, cv::Scalar(0,255,255), 2*scale);
        cv::imshow("Distorted + Corners", big);
        printf("\nPress any key...\n");
        cv::waitKey(0);
    }
    return 0;
}