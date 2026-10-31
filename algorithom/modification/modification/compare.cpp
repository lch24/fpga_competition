#include "compare.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
namespace fs = std::filesystem;
using Points = std::vector<cv::Point2f>;
using Board = std::vector<cv::Point3f>;
const cv::TermCriteria criteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 300, 1e-12);

struct Camera {
    cv::Mat k, d;
};
struct Fit {
    Camera camera;
    double rms = 0, seconds = 0;
    std::vector<cv::Mat> rotations, translations;
    cv::Mat deviations, perView;
};
struct Experiment {
    std::string name;
    std::vector<Points> points;
    Camera fpga;
    double reportedRms = 0;
    Fit fixed, full, seeded;
};

Camera readCamera(const cv::FileNode& node) {
    std::vector<double> p;
    node >> p;
    if (p.size() != 9) throw std::runtime_error("Expected fx,fy,cx,cy,k1,k2,k3,p1,p2");
    return {(cv::Mat_<double>(3, 3) << p[0], 0, p[2], 0, p[1], p[3], 0, 0, 1),
            (cv::Mat_<double>(1, 5) << p[4], p[5], p[7], p[8], p[6])};
}

void writeCamera(cv::FileStorage& out, const Camera& camera) {
    const cv::Mat d = camera.d.reshape(1, 1);
    out << "fx" << camera.k.at<double>(0, 0) << "fy" << camera.k.at<double>(1, 1)
        << "cx" << camera.k.at<double>(0, 2) << "cy" << camera.k.at<double>(1, 2)
        << "k1" << d.at<double>(0) << "k2" << d.at<double>(1)
        << "k3" << d.at<double>(4) << "p1" << d.at<double>(2) << "p2" << d.at<double>(3);
}

Fit calibrate(const Board& object, const std::vector<Points>& points,
              cv::Size size, bool freeK3, const Camera* seed = nullptr) {
    Fit fit;
    int flags = freeK3 ? 0 : cv::CALIB_FIX_K3;
    if (seed) {
        fit.camera = {seed->k.clone(), seed->d.clone()};
        flags |= cv::CALIB_USE_INTRINSIC_GUESS;
    }
    cv::Mat extrinsicDeviations;
    const int64 start = cv::getTickCount();
    fit.rms = cv::calibrateCamera(std::vector<Board>(points.size(), object), points,
        size, fit.camera.k, fit.camera.d, fit.rotations, fit.translations,
        fit.deviations, extrinsicDeviations, fit.perView, flags, criteria);
    fit.seconds = static_cast<double>(cv::getTickCount() - start) / cv::getTickFrequency();
    if (!std::isfinite(fit.rms) || !cv::checkRange(fit.camera.k) || !cv::checkRange(fit.camera.d))
        throw std::runtime_error("Nonfinite calibration result");
    // Independently recompute the library RMS to catch ordering/unit mistakes.
    double sse = 0;
    for (size_t i = 0; i < points.size(); ++i) {
        Points predicted;
        cv::projectPoints(object, fit.rotations[i], fit.translations[i], fit.camera.k, fit.camera.d, predicted);
        const double norm = cv::norm(points[i], predicted, cv::NORM_L2);
        sse += norm * norm;
    }
    const double checked = std::sqrt(sse / static_cast<double>(points.size() * object.size()));
    if (std::abs(checked - fit.rms) > 1e-3) throw std::runtime_error("RMS consistency check failed");
    return fit;
}

void writeFit(cv::FileStorage& out, const std::string& name, const Fit& fit) {
    out << name << "{";
    writeCamera(out, fit.camera);
    out << "rms" << fit.rms << "seconds" << fit.seconds
        << "intrinsic_stddev" << fit.deviations << "per_view_rms" << fit.perView
        << "rvecs" << fit.rotations << "tvecs" << fit.translations << "}";
}

// Refit ONLY six pose variables for each image. Intrinsics and distortion stay
// exactly fixed. This puts FPGA and OpenCV on the same detected corner set.
void evaluateCamera(cv::FileStorage& out, const std::string& label, const Camera& camera,
                    const Board& object, const std::vector<Points>& points) {
    double total = 0, maxError = 0;
    out << label << "{" << "per_view_rms" << "[";
    for (const auto& measured : points) {
        cv::Mat r, t;
        if (!cv::solvePnP(object, measured, camera.k, camera.d, r, t, false, cv::SOLVEPNP_ITERATIVE))
            throw std::runtime_error("Pose fitting failed");
        cv::solvePnPRefineLM(object, measured, camera.k, camera.d, r, t, criteria);
        Points predicted;
        cv::projectPoints(object, r, t, camera.k, camera.d, predicted);
        double sum = 0;
        for (size_t i = 0; i < measured.size(); ++i) {
            const double e = cv::norm(predicted[i] - measured[i]);
            sum += e * e;
            maxError = std::max(maxError, e);
        }
        out << std::sqrt(sum / static_cast<double>(object.size()));
        total += sum;
    }
    out << "]" << "rms" << std::sqrt(total / static_cast<double>(points.size() * object.size()))
        << "max_corner_error" << maxError << "}";
}

struct Statistics {
    double sum2 = 0, maximum = 0;
    size_t count = 0;
    void add(double value) { sum2 += value * value; maximum = std::max(maximum, value); ++count; }
    void write(cv::FileStorage& out, const std::string& name) const {
        out << name << "{" << "pixels" << static_cast<int>(count)
            << "rms" << std::sqrt(sum2 / static_cast<double>(count)) << "max" << maximum << "}";
    }
};

// Hold the virtual output camera fixed: differences then describe which source
// pixel is sampled, rather than mixing a zoom/principal-point change into it.
void compareMaps(cv::FileStorage& out, const std::string& name, const Camera& a,
                 const Camera& b, const cv::Mat& outputK, cv::Size size) {
    cv::Mat ax, ay, bx, by;
    cv::initUndistortRectifyMap(a.k, a.d, cv::Mat(), outputK, size, CV_32FC1, ax, ay);
    cv::initUndistortRectifyMap(b.k, b.d, cv::Mat(), outputK, size, CV_32FC1, bx, by);
    Statistics all, center, edge;
    for (int y = 0; y < size.height; ++y) for (int x = 0; x < size.width; ++x) {
        const double distance = std::hypot(ax.at<float>(y, x) - bx.at<float>(y, x),
                                           ay.at<float>(y, x) - by.at<float>(y, x));
        all.add(distance);
        const bool isCenter = x >= size.width / 4 && x < size.width * 3 / 4
                           && y >= size.height / 4 && y < size.height * 3 / 4;
        (isCenter ? center : edge).add(distance);
    }
    out << name << "{";
    all.write(out, "all"); center.write(out, "central_half"); edge.write(out, "outside_central_half");
    out << "}";
}

void writeCorrected(const fs::path& file, const cv::Mat& nativeImage, const cv::Vec4d& transform,
                    const Camera& camera, const cv::Mat& outputK, cv::Size size) {
    cv::Mat mx, my, corrected;
    cv::initUndistortRectifyMap(camera.k, camera.d, cv::Mat(), outputK, size, CV_32FC1, mx, my);
    // Original PNG coordinates -> sensor coordinates: sx*x+tx, sy*y+ty.
    // Sample the original PNG directly, avoiding a resize before remapping.
    mx = (mx - transform[2]) / transform[0];
    my = (my - transform[3]) / transform[1];
    cv::remap(nativeImage, corrected, mx, my, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    if (!cv::imwrite(file.string(), corrected)) throw std::runtime_error("Image write failed");
}
} // namespace

int compareExperiments(const fs::path& manifest, const fs::path& output) {
    cv::FileStorage input(manifest.string(), cv::FileStorage::READ);
    if (!input.isOpened()) throw std::runtime_error("Cannot open manifest");
    if (std::string(input["coordinate_status"]).rfind("INVALID", 0) == 0)
        throw std::runtime_error("Invalid image coordinate transform: supply actual crop offsets and display scale before calibration");
    if (fs::exists(output)) throw std::runtime_error("Output directory already exists");
    fs::create_directories(output);
    const cv::Size size{int(input["width"]), int(input["height"])};
    const cv::Size pattern{int(input["columns"]), int(input["rows"])};
    if (size.width < 1 || size.height < 1 || pattern.width < 2 || pattern.height < 2)
        throw std::runtime_error("Invalid image or board size");
    Board object;
    for (int row = 0; row < pattern.height; ++row)
        for (int col = 0; col < pattern.width; ++col)
            object.emplace_back(static_cast<float>(col), static_cast<float>(row), 0.0f);
    cv::FileStorage out((output / "comparison.json").string(),
        cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
    out << "opencv_version" << CV_VERSION << "width" << size.width << "height" << size.height
        << "columns" << pattern.width << "rows" << pattern.height
        << "coordinate_status" << std::string(input["coordinate_status"])
        << "coordinate_note" << std::string(input["coordinate_note"]);
    std::vector<Experiment> experiments;
    out << "groups" << "[";
    for (const auto& group : input["groups"]) {
        Experiment experiment;
        experiment.name = std::string(group["name"]);
        experiment.fpga = readCamera(group["fpga_params"]);
        experiment.reportedRms = double(group["fpga_reported_rms"]);
        std::vector<cv::Mat> images;
        std::vector<cv::Vec4d> transforms;
        std::vector<std::string> filenames;
        out << "{" << "name" << experiment.name << "views" << "[";
        for (const auto& view : group["views"]) {
            const fs::path file = manifest.parent_path() / std::string(view["image"]);
            cv::Mat image = cv::imread(file.string()), gray;
            if (image.empty()) throw std::runtime_error("Cannot read " + file.string());
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
            Points native, canonical;
            const int64 start = cv::getTickCount();
            if (!cv::findChessboardCornersSB(gray, pattern, native,
                    cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY)
                || native.size() != object.size())
                throw std::runtime_error("Chessboard detection failed: " + file.string());
            const double seconds = static_cast<double>(cv::getTickCount() - start) / cv::getTickFrequency();
            std::vector<double> affine;
            view["png_to_sensor"] >> affine;
            if (affine.size() != 4 || affine[0] <= 0 || affine[1] <= 0)
                throw std::runtime_error("Each image needs [sx,sy,tx,ty] png_to_sensor");
            const cv::Vec4d transform(affine[0], affine[1], affine[2], affine[3]);
            for (const auto& point : native)
                canonical.emplace_back(static_cast<float>(point.x * transform[0] + transform[2]),
                                       static_cast<float>(point.y * transform[1] + transform[3]));
            cv::Mat overlay = image.clone();
            cv::drawChessboardCorners(overlay, pattern, native, true);
            const std::string filename = file.filename().string();
            if (!cv::imwrite((output / ("corners_" + filename)).string(), overlay))
                throw std::runtime_error("Overlay write failed");
            std::vector<cv::Point2f> hull;
            cv::convexHull(canonical, hull);
            out << "{" << "image" << filename << "width" << image.cols << "height" << image.rows
                << "png_to_sensor" << affine << "native_corners" << native << "sensor_corners" << canonical
                << "detection_seconds" << seconds
                << "corner_hull_fraction" << cv::contourArea(hull) / size.area() << "}";
            experiment.points.push_back(canonical);
            images.push_back(image); transforms.push_back(transform); filenames.push_back(filename);
        }
        out << "]";
        if (experiment.points.size() < 3) throw std::runtime_error("At least three views are required");
        experiment.fixed = calibrate(object, experiment.points, size, false);
        experiment.full = calibrate(object, experiment.points, size, true);
        experiment.seeded = calibrate(object, experiment.points, size, false, &experiment.fpga);
        out << "fpga" << "{";
        writeCamera(out, experiment.fpga);
        out << "reported_rms" << experiment.reportedRms << "}";
        writeFit(out, "opencv_fixed_k3", experiment.fixed);
        writeFit(out, "opencv_free_k3", experiment.full);
        writeFit(out, "opencv_fpga_seed_fixed_k3", experiment.seeded);
        evaluateCamera(out, "fpga_on_opencv_corners", experiment.fpga, object, experiment.points);
        compareMaps(out, "fpga_vs_opencv_fixed_map", experiment.fpga, experiment.fixed.camera,
                    experiment.fixed.camera.k, size);
        compareMaps(out, "opencv_free_vs_fixed_map", experiment.full.camera, experiment.fixed.camera,
                    experiment.fixed.camera.k, size);
        for (size_t i = 0; i < images.size(); ++i) {
            writeCorrected(output / ("fpga_" + filenames[i]), images[i], transforms[i],
                           experiment.fpga, experiment.fixed.camera.k, size);
            writeCorrected(output / ("opencv_fixed_" + filenames[i]), images[i], transforms[i],
                           experiment.fixed.camera, experiment.fixed.camera.k, size);
            writeCorrected(output / ("opencv_free_" + filenames[i]), images[i], transforms[i],
                           experiment.full.camera, experiment.fixed.camera.k, size);
        }
        out << "}";
        std::cout << experiment.name << " fixed RMS=" << experiment.fixed.rms
                  << " free RMS=" << experiment.full.rms
                  << " FPGA-seed RMS=" << experiment.seeded.rms << std::endl;
        experiments.push_back(experiment);
    }
    out << "]" << "cross_group" << "[";
    for (size_t source = 0; source < experiments.size(); ++source)
        for (size_t target = 0; target < experiments.size(); ++target) {
            if (source == target) continue;
            const auto& a = experiments[source];
            const auto& b = experiments[target];
            out << "{" << "trained_on" << a.name << "evaluated_on" << b.name;
            evaluateCamera(out, "fpga", a.fpga, object, b.points);
            evaluateCamera(out, "opencv_fixed_k3", a.fixed.camera, object, b.points);
            evaluateCamera(out, "opencv_free_k3", a.full.camera, object, b.points);
            out << "}";
        }
    out << "]";
    out << "complete" << 1;
    return 0;
}

int checkCalibrationMath() {
    Board object;
    for (int row = 0; row < 5; ++row) for (int col = 0; col < 8; ++col)
        object.emplace_back(static_cast<float>(col), static_cast<float>(row), 0.0f);
    const std::vector<cv::Vec3d> rotations{{.25,-.3,.05},{-.3,.2,-.12},{.15,.4,.18},
        {-.2,-.4,.3},{.4,.05,-.3},{-.1,.25,.2}};
    const std::vector<cv::Vec3d> translations{{-3.5,-2,12},{-3,-2,14},{-4,-2,13},
        {-3.2,-1.8,12.5},{-3,-2,13.5},{-3.8,-1.5,11.5}};
    for (bool freeK3 : {false, true}) {
        const Camera truth{(cv::Mat_<double>(3,3) << 600,0,630,0,610,350,0,0,1),
                           (cv::Mat_<double>(1,5) << -.16,.02,.001,-.002,freeK3 ? .012 : 0)};
        std::vector<Points> points;
        for (size_t i = 0; i < rotations.size(); ++i) {
            Points projected;
            cv::projectPoints(object, rotations[i], translations[i], truth.k, truth.d, projected);
            points.push_back(projected);
        }
        const Fit fit = calibrate(object, points, {1280,720}, freeK3);
        const double kError = cv::norm(fit.camera.k, truth.k, cv::NORM_INF);
        const double dError = cv::norm(fit.camera.d.reshape(1,1), truth.d, cv::NORM_INF);
        if (fit.rms > .001 || kError > .1 || dError > .002)
            throw std::runtime_error("Synthetic known-camera recovery failed");
        cv::FileStorage memory("", cv::FileStorage::WRITE | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_JSON);
        evaluateCamera(memory, "evaluation", truth, object, points);
        cv::FileStorage recovered(memory.releaseAndGetString(), cv::FileStorage::READ | cv::FileStorage::MEMORY);
        const double poseRms = double(recovered["evaluation"]["rms"]);
        if (poseRms > .001) throw std::runtime_error("Fixed-camera pose evaluation failed");
        std::cout << "PASS synthetic " << (freeK3 ? "free" : "fixed") << " k3: RMS=" << fit.rms
                  << " max K error=" << kError << " max D error=" << dError
                  << " pose RMS=" << poseRms << std::endl;
    }
    return 0;
}
