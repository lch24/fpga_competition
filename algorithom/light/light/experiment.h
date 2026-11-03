#pragma once
#include <filesystem>

// Compare two real lighting groups and controlled gain changes of the same frames.
// OpenCV supplies the reference detector/calibrator; the project detector is linked
// from closer2fpga without copying or modifying its implementation.
int runExperiment(const std::filesystem::path& input,
                  const std::filesystem::path& output, int columns, int rows);
void selfTest();
