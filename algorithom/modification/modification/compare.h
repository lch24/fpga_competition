#pragma once
#include <filesystem>

// Read explicit image-to-sensor coordinate transforms from a JSON manifest.
// All calibration, pose fitting and remapping use OpenCV's public API.
int compareExperiments(const std::filesystem::path& manifest,
                       const std::filesystem::path& output);
int checkCalibrationMath();
