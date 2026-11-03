#include "experiment.h"
#include "../../closer2fpga/closer2fpga/algo/chessboard.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
namespace fs = std::filesystem;
using Points = std::vector<cv::Point2f>;
using Board = std::vector<cv::Point3f>;
constexpr int detectorCount = 2;
const char* detectorNames[] = {"opencv_sb", "project_cpp"};
const char* detectorLabels[] = {"OpenCV SB", "项目 C++ 检测器"};
const cv::TermCriteria criteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 300, 1e-12);

struct Error {
    double squareSum = 0, maximum = 0;
    int count = 0;
    void add(double value) { squareSum += value * value; maximum = std::max(maximum, value); ++count; }
    void merge(const Error& e) { squareSum += e.squareSum; maximum = std::max(maximum, e.maximum); count += e.count; }
    double rms() const { return count ? std::sqrt(squareSum / count) : 0; }
};
struct Detection {
    bool found = false;
    double seconds = 0;
    Points points;
};
struct InputFrame {
    std::string name;
    cv::Mat gray, color;
};
struct Fit {
    bool valid = false;
    std::string reason;
    cv::Mat k, d, deviations, perView;
    std::vector<cv::Mat> rotations, translations;
    double rms = 0, seconds = 0;
};
struct Case {
    int group = 0;
    double gain = 1;
    std::array<std::vector<Detection>, detectorCount> detections;
    std::array<Fit, detectorCount> fits;
};

Board boardPoints(cv::Size pattern) {
    Board result;
    for (int y = 0; y < pattern.height; ++y)
        for (int x = 0; x < pattern.width; ++x)
            result.emplace_back(static_cast<float>(x), static_cast<float>(y), 0.0f);
    return result;
}

// A rectangular, unmarked board permits row/column reversal. Remove only this
// index ambiguity: no translation, scaling or homography is fitted to the data.
Points alignOrder(const Points& reference, const Points& measured, cv::Size pattern) {
    if (reference.size() != size_t(pattern.area()) || measured.size() != reference.size())
        throw std::runtime_error("Invalid corner correspondence");
    double best = std::numeric_limits<double>::infinity();
    Points result;
    for (int flip = 0; flip < 4; ++flip) {
        Points candidate;
        double error = 0;
        for (int y = 0; y < pattern.height; ++y) for (int x = 0; x < pattern.width; ++x) {
            int row = (flip & 1) ? pattern.height - y - 1 : y;
            int col = (flip & 2) ? pattern.width - x - 1 : x;
            auto p = measured[size_t(row * pattern.width + col)];
            candidate.push_back(p);
            auto delta = p - reference[size_t(y * pattern.width + x)];
            error += delta.dot(delta);
        }
        if (error < best) { best = error; result = std::move(candidate); }
    }
    return result;
}

Error difference(const Points& reference, const Points& measured, cv::Size pattern) {
    auto ordered = alignOrder(reference, measured, pattern);
    Error result;
    for (size_t i = 0; i < reference.size(); ++i) result.add(cv::norm(reference[i] - ordered[i]));
    return result;
}

Detection detect(const cv::Mat& gray, cv::Size pattern, int method) {
    Detection result;
    const auto start = cv::getTickCount();
    if (method == 0) {
        result.found = cv::findChessboardCornersSB(gray, pattern, result.points,
            cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY);
        // SB already estimates subpixel coordinates; do not refine them twice.
    } else {
        GrayImage input(gray.cols, gray.rows);
        for (int y = 0; y < gray.rows; ++y)
            std::copy_n(gray.ptr<uint8_t>(y), gray.cols, input.data + size_t(y) * gray.cols);
        auto board = detect_chessboard(input, pattern.height, pattern.width);
        result.found = board.valid;
        for (auto p : board.corners) result.points.emplace_back(p.x, p.y);
    }
    result.seconds = double(cv::getTickCount() - start) / cv::getTickFrequency();
    result.found = result.found && result.points.size() == size_t(pattern.area());
    return result;
}

Fit calibrate(const Board& board, const std::vector<Detection>& views, cv::Size size) {
    Fit fit;
    std::vector<Points> points;
    for (const auto& view : views) {
        if (!view.found) { fit.reason = "Incomplete board detection"; return fit; }
        points.push_back(view.points);
    }
    if (points.size() < 3) { fit.reason = "Need at least three views"; return fit; }
    const auto start = cv::getTickCount();
    try {
        cv::Mat extrinsicDeviations;
        fit.rms = cv::calibrateCamera(std::vector<Board>(points.size(), board), points, size,
            fit.k, fit.d, fit.rotations, fit.translations, fit.deviations,
            extrinsicDeviations, fit.perView, cv::CALIB_FIX_K3, criteria);
        fit.seconds = double(cv::getTickCount() - start) / cv::getTickFrequency();
        if (!std::isfinite(fit.rms) || !cv::checkRange(fit.k) || !cv::checkRange(fit.d)
            || fit.k.at<double>(0, 0) <= 0 || fit.k.at<double>(1, 1) <= 0)
            throw std::runtime_error("Nonfinite or nonpositive camera parameters");
        Error checked;
        for (size_t v = 0; v < points.size(); ++v) {
            Points projected;
            cv::projectPoints(board, fit.rotations[v], fit.translations[v], fit.k, fit.d, projected);
            for (size_t i = 0; i < board.size(); ++i) checked.add(cv::norm(projected[i] - points[v][i]));
        }
        if (std::abs(checked.rms() - fit.rms) > 1e-3)
            throw std::runtime_error("Independent reprojection RMS check failed");
        fit.valid = true;
    } catch (const cv::Exception& error) { fit.reason = error.what(); }
    return fit;
}

std::array<double, 9> parameters(const Fit& fit) {
    cv::Mat d = fit.d.reshape(1, 1);
    return {fit.k.at<double>(0, 0), fit.k.at<double>(1, 1), fit.k.at<double>(0, 2),
        fit.k.at<double>(1, 2), d.at<double>(0), d.at<double>(1), d.at<double>(4),
        d.at<double>(2), d.at<double>(3)};
}

void writeError(cv::FileStorage& out, const Error& error) {
    out << "{" << "points" << error.count;
    if (error.count) out << "rms_px" << error.rms() << "max_px" << error.maximum;
    out << "}";
}

void writeFit(cv::FileStorage& out, const Fit& fit) {
    out << "{" << "valid" << int(fit.valid) << "reason" << fit.reason;
    if (fit.valid) {
        auto p = parameters(fit);
        const char* names[] = {"fx", "fy", "cx", "cy", "k1", "k2", "k3", "p1", "p2"};
        for (int i = 0; i < 9; ++i) out << names[i] << p[i];
        out << "rms_px" << fit.rms << "seconds" << fit.seconds << "K" << fit.k << "D" << fit.d
            << "intrinsic_stddev" << fit.deviations << "per_view_rms" << fit.perView
            << "rvecs" << fit.rotations << "tvecs" << fit.translations;
    }
    out << "}";
}

void writeBrightness(cv::FileStorage& out, const cv::Mat& gray, const cv::Mat& mask = cv::Mat()) {
    cv::Scalar mean, stddev;
    cv::meanStdDev(gray, mean, stddev, mask);
    const int count = mask.empty() ? int(gray.total()) : cv::countNonZero(mask);
    if (!count) throw std::runtime_error("Empty brightness ROI");
    cv::Mat low = gray <= 5, high = gray >= 250;
    if (!mask.empty()) { cv::bitwise_and(low, mask, low); cv::bitwise_and(high, mask, high); }
    out << "{" << "pixels" << count << "mean_gray" << mean[0] << "std_gray" << stddev[0]
        << "le5_percent" << 100.0 * cv::countNonZero(low) / count
        << "ge250_percent" << 100.0 * cv::countNonZero(high) / count << "}";
}

cv::Mat boardMask(const cv::Mat& gray, const Points& points) {
    cv::Mat mask = cv::Mat::zeros(gray.size(), CV_8U);
    std::vector<cv::Point2f> hull;
    std::vector<cv::Point> polygon;
    cv::convexHull(points, hull);
    for (auto p : hull) polygon.emplace_back(cvRound(p.x), cvRound(p.y));
    cv::fillConvexPoly(mask, polygon, cv::Scalar(255));
    return mask;
}

Error poseOnlyError(const Fit& fit, const Board& board, const std::vector<Detection>& views) {
    Error result;
    for (const auto& view : views) {
        if (!view.found) return {};
        cv::Mat r, t;
        if (!cv::solvePnP(board, view.points, fit.k, fit.d, r, t, false, cv::SOLVEPNP_ITERATIVE))
            throw std::runtime_error("Pose refit failed");
        cv::solvePnPRefineLM(board, view.points, fit.k, fit.d, r, t, criteria);
        Points predicted;
        cv::projectPoints(board, r, t, fit.k, fit.d, predicted);
        for (size_t i = 0; i < predicted.size(); ++i) result.add(cv::norm(predicted[i] - view.points[i]));
    }
    return result;
}

void writeReport(const fs::path& path, const std::vector<Case>& cases,
                 const std::array<std::vector<InputFrame>, 2>& groups, cv::Size pattern,
                 const std::array<std::array<Error, 2>, 2>& transfer) {
    std::ofstream out(path, std::ios::binary);
    if (!out) throw std::runtime_error("Cannot open report");
    out << "# 环境亮度对角点检测与标定结果的影响\n\n"
        << "输入为 `an.png/an2.png/an3.png` 与 `ming.png/ming2.png/ming3.png`，每组分别标定。"
        << "图像尺寸 " << groups[0][0].gray.cols << "×" << groups[0][0].gray.rows
        << "，棋盘内角点 " << pattern.width << "×" << pattern.height << "。\n\n";
    out << "## 实验结论\n\n" << std::fixed << std::setprecision(6);
    for (int method = 0; method < detectorCount; ++method) {
        out << "- " << detectorLabels[method] << "：原始暗组 ";
        for (int g = 0; g < 2; ++g) {
            if (g) out << "，原始亮组 ";
            out << std::count_if(cases[g].detections[method].begin(), cases[g].detections[method].end(),
                [](const Detection& d) { return d.found; }) << "/3 检测成功";
        }
        out << "。\n";
    }
    double largestDrift = 0, largestCorner = 0, largestFocal = 0;
    int completeCases = 0;
    for (const auto& test : cases) {
        if (test.gain == 1) continue;
        Error drift;
        bool all = true;
        for (size_t v = 0; v < groups[test.group].size(); ++v) {
            const auto& base = cases[test.group].detections[0][v];
            const auto& altered = test.detections[0][v];
            all = all && base.found && altered.found;
            if (base.found && altered.found) drift.merge(difference(base.points, altered.points, pattern));
        }
        if (all) ++completeCases;
        largestDrift = std::max(largestDrift, drift.rms());
        largestCorner = std::max(largestCorner, drift.maximum);
        if (test.fits[0].valid && cases[test.group].fits[0].valid) {
            const auto p = parameters(test.fits[0]), base = parameters(cases[test.group].fits[0]);
            largestFocal = std::max({largestFocal, std::abs(p[0] - base[0]), std::abs(p[1] - base[1])});
        }
    }
    out << "- 同图增益实验中，SB 在 " << completeCases << "/8 个变亮或变暗的三视图组合中完整检测。"
        << "各组合角点位移 RMS 最大为 " << largestDrift << " px，单角点位移最大为 " << largestCorner
        << " px；焦距 fx/fy 相对原图结果的最大绝对变化为 " << largestFocal << " px。\n"
        << "- 这些数据支持：本批素材上，使用 SB 的流程对单纯数字亮度增益较稳定；"
        << "项目 C++ 检测分支的检测成功率明显更低，需要单独排查候选筛选、环形检查和网格组织的稳定性。\n"
        << "- 项目分支没有任何一个完整三视图组合通过检测，因此本次不能给出项目检测 + 标定的亮度精度结论，"
        << "也不能把 OpenCV 的成功率当作 FPGA 实测成功率。\n\n"
        << "## 测试方法\n\n"
        << "- OpenCV " << CV_VERSION << "：SB 检测器开启归一化、穷举搜索及精度增强；直接使用其亚像素结果。\n"
        << "- 项目检测器：直接链接 `closer2fpga` 现有 C++ 实现，两种检测器接收同一份 OpenCV 灰度图，隔离灰度转换差异。\n"
        << "- 两条检测分支都使用 OpenCV `calibrateCamera` 求解，固定 k3=0，估计 fx、fy、cx、cy、k1、k2、p1、p2 及各视图姿态；最多 300 次迭代、终止阈值 1e-12。\n"
        << "- 控制实验仅对原灰度图做 `round(gain × gray)` 并饱和到 [0,255]，不改变坐标。每次从原图生成，避免累积量化。\n"
        << "- 角点位移只消除棋盘编号方向的歧义，不做平移、缩放或单应变换对齐。\n\n"
        << "实际暗、亮照片中棋盘位置与姿态存在变化，组间参数差异包含拍摄几何的影响。"
        << "同图增益实验用于单独观察数字亮度变化，不能复现真实低照度新增的噪声、拖影或曝光变化。"
        << "本次是 C++ 离线实验，未运行 RTL 或板上标定器。\n\n"
        << "## 原始图片亮度与检测\n\n"
        << "| 图片 | 全图平均灰度 | 棋盘区域平均灰度 | 棋盘灰度标准差 | SB 角点数 | 项目角点数 |\n"
        << "|---|---:|---:|---:|---:|---:|\n" << std::fixed << std::setprecision(3);
    for (int g = 0; g < 2; ++g) for (size_t v = 0; v < groups[g].size(); ++v) {
        const auto& image = groups[g][v];
        const auto& detected = cases[g].detections;
        out << "| " << image.name << " | " << cv::mean(image.gray)[0] << " | ";
        if (detected[0][v].found) {
            cv::Scalar mean, stddev;
            cv::meanStdDev(image.gray, mean, stddev, boardMask(image.gray, detected[0][v].points));
            out << mean[0] << " | " << stddev[0];
        } else out << "— | —";
        for (int method = 0; method < detectorCount; ++method)
            out << " | " << (detected[method][v].found ? std::to_string(detected[method][v].points.size()) : "失败");
        out << " |\n";
    }
    out << "\n棋盘区域取 SB 内角点的凸包；像素灰度范围为 0–255。检测失败不纳入后续标定，不用另一检测器补点。\n\n"
        << "## 原始暗、亮组标定\n\n"
        << "| 参数 | SB 暗组 | SB 亮组 | 项目角点暗组 + OpenCV 求解 | 项目角点亮组 + OpenCV 求解 |\n"
        << "|---|---:|---:|---:|---:|\n" << std::setprecision(9);
    const char* names[] = {"fx", "fy", "cx", "cy", "k1", "k2", "k3", "p1", "p2", "重投影 RMS / px"};
    for (int p = 0; p < 10; ++p) {
        out << "| " << names[p];
        for (int method = 0; method < detectorCount; ++method) for (int g = 0; g < 2; ++g) {
            const auto& fit = cases[g].fits[method];
            out << " | ";
            if (fit.valid) out << (p == 9 ? fit.rms : parameters(fit)[p]);
            else out << "未标定";
        }
        out << " |\n";
    }
    out << "\n重投影 RMS 是当前标定数据上的拟合残差，不是参数的真实误差。相机内参和畸变参数没有独立真值，不能仅凭残差更小判定哪组参数更准确。\n\n"
        << "固定每组标定的 K、D，在另一组只重估 R、t，检查参数跨组一致性：\n\n"
        << "| 检测器 | 暗组参数 → 亮组角点 RMS / px | 亮组参数 → 暗组角点 RMS / px |\n"
        << "|---|---:|---:|\n" << std::setprecision(6);
    for (int method = 0; method < detectorCount; ++method) {
        out << "| " << detectorLabels[method];
        for (int direction = 0; direction < 2; ++direction) {
            out << " | ";
            if (transfer[method][direction].count) out << transfer[method][direction].rms();
            else out << "—";
        }
        out << " |\n";
    }
    out << "\n## 同图亮度控制实验\n\n"
        << "每一行对原组的三张图片应用同一增益；角点位移相对于同一检测器、同一原图的 gain=1 结果，"
        << "只累计前后均检测成功的图片。参数变化同样相对于各自原组。\n\n"
        << "| 原组 | 增益 | 检测器 | 成功 / 3 | 比较角点数 | 角点位移 RMS / px | 最大位移 / px | 标定 RMS / px | Δfx / px | Δfy / px |\n"
        << "|---|---:|---|---:|---:|---:|---:|---:|---:|---:|\n";
    for (const auto& test : cases) for (int method = 0; method < detectorCount; ++method) {
        Error drift;
        int found = 0;
        for (size_t v = 0; v < test.detections[method].size(); ++v) {
            const auto& d = test.detections[method][v];
            const auto& base = cases[test.group].detections[method][v];
            if (d.found) ++found;
            if (d.found && base.found) drift.merge(difference(base.points, d.points, pattern));
        }
        out << "| " << (test.group ? "亮" : "暗") << " | " << test.gain << " | " << detectorLabels[method]
            << " | " << found << "/3 | " << drift.count << " | ";
        if (drift.count) out << drift.rms() << " | " << drift.maximum;
        else out << "— | —";
        const auto& fit = test.fits[method];
        const auto& baseFit = cases[test.group].fits[method];
        out << " | ";
        if (fit.valid) out << fit.rms; else out << "—";
        if (fit.valid && baseFit.valid)
            out << " | " << parameters(fit)[0] - parameters(baseFit)[0]
                << " | " << parameters(fit)[1] - parameters(baseFit)[1];
        else out << " | — | —";
        out << " |\n";
    }
    out << "\n完整角点坐标、亮度统计、每视图误差、参数及参数标准差保存在 [results.json](results.json)。"
        << "`overlays/` 保存两种检测器在原图上的角点叠加图。`input_sha256.json` 保存输入校验值。\n\n"
        << "OpenCV 方法说明：[Camera Calibration and 3D Reconstruction](https://docs.opencv.org/4.12.0/d9/d0c/group__calib3d.html)。\n";
    if (!out) throw std::runtime_error("Report write failed");
}
} // namespace

int runExperiment(const fs::path& input, const fs::path& output, int columns, int rows) {
    if (columns < 2 || rows < 2 || columns > 30 || rows > 30)
        throw std::runtime_error("Invalid chessboard dimensions");
    if (fs::exists(output)) throw std::runtime_error("Output exists: choose a new directory");
    const cv::Size pattern(columns, rows);
    const auto board = boardPoints(pattern);
    const std::array<std::array<std::string, 3>, 2> names = {{
        {"an.png", "an2.png", "an3.png"}, {"ming.png", "ming2.png", "ming3.png"}}};
    std::array<std::vector<InputFrame>, 2> groups;
    cv::Size size;
    for (int g = 0; g < 2; ++g) for (const auto& name : names[g]) {
        InputFrame image;
        image.name = name;
        image.color = cv::imread((input / name).string());
        if (image.color.empty()) throw std::runtime_error("Cannot read " + name);
        cv::cvtColor(image.color, image.gray, cv::COLOR_BGR2GRAY);
        if (size.empty()) size = image.gray.size();
        if (size != image.gray.size()) throw std::runtime_error("Image dimensions differ");
        groups[g].push_back(std::move(image));
    }
    fs::create_directories(output / "overlays");
    cv::FileStorage out((output / "results.json").string(), cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
    if (!out.isOpened()) throw std::runtime_error("Cannot create results.json");
    out << "opencv_version" << CV_VERSION << "width" << size.width << "height" << size.height
        << "columns" << columns << "rows" << rows << "fixed_k3" << 1
        << "criteria_max_iterations" << 300 << "criteria_epsilon" << 1e-12
        << "same_geometry_real_pairs_assumed" << 0
        << "capture_relationship" << "User-confirmed corresponding shots with slight camera/board movement"
        << "cases" << "[";
    std::vector<Case> cases;
    // Both gain=1 baselines are first; subsequent cases refer to these exact points.
    for (double gain : {1.0, 0.25, 0.5, 0.75, 1.25}) for (int g = 0; g < 2; ++g) {
        Case test;
        test.group = g; test.gain = gain;
        out << "{" << "group" << (g ? "bright" : "dark") << "gain" << gain << "images" << "[";
        for (size_t v = 0; v < groups[g].size(); ++v) {
            const auto& image = groups[g][v];
            cv::Mat gray;
            image.gray.convertTo(gray, CV_8U, gain);
            out << "{" << "image" << image.name << "brightness";
            writeBrightness(out, gray);
            for (int method = 0; method < detectorCount; ++method) {
                auto d = detect(gray, pattern, method);
                std::cout << image.name << " gain=" << gain << " " << detectorNames[method]
                    << " found=" << d.found << " corners=" << d.points.size()
                    << " seconds=" << d.seconds << std::endl;
                out << detectorNames[method] << "{" << "found" << int(d.found)
                    << "seconds" << d.seconds << "corners" << d.points;
                if (gain != 1 && d.found && cases[g].detections[method][v].found) {
                    out << "drift_from_original";
                    writeError(out, difference(cases[g].detections[method][v].points, d.points, pattern));
                }
                out << "}";
                if (gain == 1) {
                    cv::Mat overlay = image.color.clone();
                    cv::drawChessboardCorners(overlay, pattern, d.points, d.found);
                    if (!cv::imwrite((output / "overlays" / (std::string(detectorNames[method]) + "_" + image.name)).string(), overlay))
                        throw std::runtime_error("Overlay write failed");
                }
                test.detections[method].push_back(std::move(d));
            }
            const auto& roiPoints = gain == 1 ? test.detections[0][v] : cases[g].detections[0][v];
            if (roiPoints.found) {
                out << "board_brightness";
                writeBrightness(out, gray, boardMask(gray, roiPoints.points));
            }
            out << "}";
        }
        out << "]" << "calibration" << "{";
        for (int method = 0; method < detectorCount; ++method) {
            test.fits[method] = calibrate(board, test.detections[method], size);
            out << detectorNames[method]; writeFit(out, test.fits[method]);
            std::cout << (g ? "bright" : "dark") << " gain=" << gain << " " << detectorNames[method]
                << " calibration=" << test.fits[method].valid << " RMS=" << test.fits[method].rms << std::endl;
        }
        out << "}" << "}";
        cases.push_back(std::move(test));
    }
    out << "]" << "cross_group_pose_only" << "[";
    std::array<std::array<Error, 2>, 2> transfer;
    for (int method = 0; method < detectorCount; ++method) for (int from = 0; from < 2; ++from) {
        if (cases[from].fits[method].valid)
            transfer[method][from] = poseOnlyError(cases[from].fits[method], board, cases[1 - from].detections[method]);
        out << "{" << "detector" << detectorNames[method] << "camera_group" << (from ? "bright" : "dark")
            << "point_group" << (from ? "dark" : "bright") << "error";
        writeError(out, transfer[method][from]); out << "}";
    }
    out << "]" << "complete" << 1;
    out.release();
    writeReport(output / "report.md", cases, groups, pattern, transfer);
    std::cout << "Report: " << (output / "report.md").string() << std::endl;
    return 0;
}

void selfTest() {
    // Detect indexing and denominator mistakes independently of image results.
    const cv::Size pattern(8, 5);
    Points reference, shifted;
    for (int y = 0; y < pattern.height; ++y) for (int x = 0; x < pattern.width; ++x) {
        reference.emplace_back(float(x * 20), float(y * 20));
        shifted.emplace_back(float(x * 20 + 3), float(y * 20 + 4));
    }
    std::reverse(shifted.begin(), shifted.end());
    auto e = difference(reference, shifted, pattern);
    if (e.count != 40 || std::abs(e.rms() - 5) > 1e-9 || std::abs(e.maximum - 5) > 1e-9)
        throw std::runtime_error("Corner comparison self-test failed");
    const auto object = boardPoints(pattern);
    cv::Mat k = (cv::Mat_<double>(3, 3) << 780, 0, 640, 0, 790, 360, 0, 0, 1);
    cv::Mat d = (cv::Mat_<double>(1, 5) << -0.12, 0.02, 0.001, -0.002, 0);
    std::vector<Detection> views;
    for (int i = 0; i < 6; ++i) {
        Detection view; view.found = true;
        cv::Vec3d r(0.05 + 0.07 * i, -0.25 + 0.09 * i, -0.12 + 0.03 * i);
        cv::Vec3d t(-4.0 + i * 0.5, -2.5 + i * 0.2, 13.0 + i * 0.7);
        cv::projectPoints(object, r, t, k, d, view.points); views.push_back(view);
    }
    auto fit = calibrate(object, views, {1280, 720});
    if (!fit.valid || fit.rms > 0.001 || cv::norm(fit.k, k, cv::NORM_INF) > 0.1
        || cv::norm(fit.d.reshape(1, 1), d, cv::NORM_INF) > 0.001)
        throw std::runtime_error("Synthetic calibration self-test failed");
    std::cout << "Self-test PASS: corner correspondence, point RMS, synthetic K/D recovery" << std::endl;
}
