#include <opencv2/opencv.hpp>
#include <cstdio>
#include <vector>
#include <cmath>
#include "../closer2fpga/algo/chessboard.h"
#include "../closer2fpga/algo/subpixel.h"

GrayImage convert(const cv::Mat& in) {
    GrayImage img(in.cols, in.rows);
    for (int y = 0; y < in.rows; ++y) for (int x = 0; x < in.cols; ++x) img.set(x, y, in.at<unsigned char>(y, x));
    return img;
}
// Only whole-grid flips are allowed. Nearest-neighbor matching would hide
// duplicate corners or a wrongly ordered lattice.
double error(const std::vector<Point2f>& a, const std::vector<cv::Point2f>& b) {
    if (a.size() != 40 || b.size() != 40) return 1e9;
    double best = 1e9;
    for (int flip = 0; flip < 4; ++flip) {
        double sum = 0;
        for (int r = 0; r < 5; ++r) for (int c = 0; c < 8; ++c) {
            auto p = a[r * 8 + c];
            auto q = b[(flip & 2 ? 4-r : r)*8 + (flip & 1 ? 7-c : c)];
            sum += (p.x-q.x)*(p.x-q.x) + (p.y-q.y)*(p.y-q.y);
        }
        best = std::min(best, std::sqrt(sum / 40));
    }
    return best;
}
int main(int argc, char** argv) {
    std::string root = argc > 1 ? argv[1] : "E:/fpga/algorithom";
    int failures = 0, count = 0;
    for (int i = 0; i < 3; ++i) {
        auto base = cv::imread(root + "/test" + std::to_string(i) + ".jpg", cv::IMREAD_GRAYSCALE);
        if (base.empty()) return 2;
        std::vector<cv::Point2f> reference;
        bool ref_ok = cv::findChessboardCornersSB(base, {8,5}, reference, cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY);
        auto board = detect_chessboard(convert(base), 5, 8);
        double e = error(board.corners, reference);
        std::printf("test%d base valid=%d ref=%d grid_rms=%.4f\n", i, board.valid, ref_ok, e);
        ++count; if (!board.valid || !ref_ok || e > 1.0) ++failures;
        for (double angle : {0., 15., 35., 60., 90., 135.}) {
            auto m = cv::getRotationMatrix2D({base.cols/2.f, base.rows/2.f}, angle, 0.65);
            m.at<double>(0,2) += 100; m.at<double>(1,2) += 200;
            cv::Mat transformed;
            cv::warpAffine(base, transformed, m, {base.cols+200,base.rows+400}, cv::INTER_LINEAR, cv::BORDER_CONSTANT, {127});
            std::vector<cv::Point2f> truth;
            cv::transform(reference, truth, m);
            auto detected = detect_chessboard(convert(transformed), 5, 8);
            e = error(detected.corners, truth);
            std::printf("test%d rotate=%g scale=.65 valid=%d grid_rms=%.4f\n", i, angle, detected.valid, e);
            ++count; if (!detected.valid || !ref_ok || e > 1.2) ++failures;
        }
        for (double scale : {0.35, 0.5, 1.0}) {
            cv::Mat reduced;
            cv::resize(base, reduced, {}, scale, scale, cv::INTER_AREA);
            cv::GaussianBlur(reduced, reduced, {5,5}, 0.8);
            reduced.convertTo(reduced, CV_8U, 0.6, 35);
            std::vector<cv::Point2f> truth = reference;
            double sx = double(reduced.cols)/base.cols, sy = double(reduced.rows)/base.rows;
            for (auto& p : truth) { p.x = float((p.x+.5)*sx-.5); p.y = float((p.y+.5)*sy-.5); }
            auto detected = detect_chessboard(convert(reduced), 5, 8);
            e = error(detected.corners, truth);
            std::printf("test%d scale=%g blur+dim valid=%d grid_rms=%.4f\n", i, scale, detected.valid, e);
            ++count; if (!detected.valid || !ref_ok || e > 1.2) ++failures;
        }
    }
    // Analytic ideal chessboard: pixel boundaries are at n - 0.5.
    cv::Mat synthetic(400, 520, CV_8U, cv::Scalar(127));
    for (int r = 0; r < 6; ++r) for (int c = 0; c < 9; ++c)
        cv::rectangle(synthetic, {80+c*40,80+r*40,40,40}, cv::Scalar((r+c)%2 ? 235 : 20), cv::FILLED);
    std::vector<cv::Point2f> truth;
    for (int r = 0; r < 5; ++r) for (int c = 0; c < 8; ++c) truth.push_back({119.5f+c*40,119.5f+r*40});
    cv::GaussianBlur(synthetic, synthetic, {5,5}, 0.8);
    auto detected = detect_chessboard(convert(synthetic), 5, 8);
    double e = error(detected.corners, truth);
    std::printf("analytic grid valid=%d rms=%.5f\n", detected.valid, e);
    ++count; if (!detected.valid || e > .1) ++failures;
    std::vector<Point2f> perturbed;
    for (auto p : truth) perturbed.push_back({p.x+1.2f,p.y-.8f});
    refine_subpixel(convert(synthetic), perturbed, 6);
    e = error(perturbed, truth);
    std::printf("subpixel from perturbed positions rms=%.5f\n", e);
    ++count; if (e > .05) ++failures;
    for (int tilt = 0; tilt < 2; ++tilt) {
        std::vector<cv::Point2f> src{{0,0},{519,0},{519,399},{0,399}};
        std::vector<cv::Point2f> dst = tilt == 0
            ? std::vector<cv::Point2f>{{85,30},{480,65},{500,360},{20,370}}
            : std::vector<cv::Point2f>{{35,80},{490,10},{445,375},{90,330}};
        auto h = cv::getPerspectiveTransform(src, dst);
        cv::Mat warped;
        cv::warpPerspective(synthetic, warped, h, synthetic.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, {127});
        std::vector<cv::Point2f> projected;
        cv::perspectiveTransform(truth, projected, h);
        auto found = detect_chessboard(convert(warped),5,8);
        e = error(found.corners, projected);
        std::printf("perspective%d valid=%d rms=%.5f\n", tilt, found.valid, e);
        ++count; if (!found.valid || e > .3) ++failures;
    }
    cv::Mat missing = synthetic.clone();
    cv::rectangle(missing, {223,183,35,35}, cv::Scalar(127), cv::FILLED);
    ++count; if (detect_chessboard(convert(missing),5,8).valid) ++failures;
    cv::Mat noise(240,320,CV_8U);
    cv::RNG rng(12345);
    rng.fill(noise, cv::RNG::UNIFORM, 0, 256);
    ++count; if (detect_chessboard(convert(noise),5,8).valid) ++failures;
    for (int value : {0,127,255}) {
        auto blank = convert(cv::Mat(180,240,CV_8U,cv::Scalar(value)));
        ++count; if (detect_chessboard(blank,5,8).valid) ++failures;
    }
    GrayImage empty;
    ++count; if (detect_chessboard(empty,5,8).valid) ++failures;
    auto small = convert(cv::Mat(8,8,CV_8U,cv::Scalar(0)));
    ++count; if (detect_chessboard(small,5,8).valid) ++failures;
    ++count; if (detect_chessboard(convert(synthetic),0,8).valid) ++failures;
    std::printf("RESULT: %d/%d passed\n", count-failures, count);
    return failures ? 1 : 0;
}
