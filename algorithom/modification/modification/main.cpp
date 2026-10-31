#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include "compare.h"

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

// SB already returns subpixel positions; do not run a second estimator over them.
int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--self-test")
            return checkCalibrationMath();
        if (argc == 4 && std::string(argv[1]) == "--compare")
            return compareExperiments(argv[2], argv[3]);
        if (argc != 3) {
            std::cerr << "Usage: modification IMAGE_DIRECTORY NEW_OUTPUT_DIRECTORY\n"
                      << "   or: modification --compare MANIFEST.json NEW_OUTPUT_DIRECTORY\n";
            return 2;
        }
        const fs::path input = argv[1], output = argv[2];
        if (fs::exists(output)) throw std::runtime_error("Output directory already exists");
        fs::create_directories(output);
        cv::FileStorage result((output / "detection.json").string(),
            cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
        result << "opencv_version" << CV_VERSION << "columns" << 8 << "rows" << 5;
        result << "views" << "[";
        bool allFound = true;
        for (int i = 1; i <= 6; ++i) {
            const std::string name = "ftest" + std::to_string(i) + ".png";
            cv::Mat image = cv::imread((input / name).string()), gray;
            if (image.empty()) throw std::runtime_error("Cannot read " + name);
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
            std::vector<cv::Point2f> corners;
            const int64 start = cv::getTickCount();
            const bool found = cv::findChessboardCornersSB(gray, {8, 5}, corners,
                cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY);
            const double seconds = static_cast<double>(cv::getTickCount() - start) / cv::getTickFrequency();
            std::cout << name << " " << image.cols << "x" << image.rows
                      << " found=" << found << " corners=" << corners.size()
                      << " seconds=" << seconds << std::endl;
            result << "{" << "image" << name << "width" << image.cols << "height" << image.rows
                   << "found" << int(found) << "seconds" << seconds << "corners" << corners << "}";
            cv::drawChessboardCorners(image, {8, 5}, corners, found);
            if (!cv::imwrite((output / ("corners_" + name)).string(), image))
                throw std::runtime_error("Cannot write corner overlay");
            allFound = allFound && found && corners.size() == 40;
        }
        result << "]";
        return allFound ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << error.what() << std::endl;
        return 1;
    }
}
