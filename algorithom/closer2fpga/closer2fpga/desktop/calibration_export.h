#pragma once
// Desktop file I/O only; the calibration/detection kernels are unchanged.
#include "../algo/calibrate.h"
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace desktop {
struct CalibrationExportView {
    std::string path;
    int width = 0, height = 0;
    bool image_loaded = false, board_valid = false;
};

namespace export_detail {
inline std::string quote(const std::string& s) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << '"';
    for (unsigned char c : s) {
        if (c == '"' || c == '\\') out << '\\' << c;
        else if (c < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << unsigned(c);
        else out << c;
    }
    out << '"';
    return out.str();
}
inline std::string bits(float x) {
    static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
    uint32_t u; std::memcpy(&u, &x, sizeof(u));
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::hex << std::setfill('0') << std::setw(8) << u;
    return out.str();
}
inline std::string bits(double x) {
    static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559);
    uint64_t u; std::memcpy(&u, &x, sizeof(u));
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::hex << std::setfill('0') << std::setw(16) << u;
    return out.str();
}
inline void number(std::ostream& out, double x) {
    if (std::isfinite(x)) out << x; else out << "null";
}
template<class Range> inline void numbers(std::ostream& out, const Range& a, bool hex) {
    out << '[';
    bool first = true;
    for (auto x : a) {
        if (!first) out << ", "; first = false;
        if (hex) out << quote(bits(x)); else number(out, x);
    }
    out << ']';
}
inline std::ofstream file(const std::filesystem::path& path) {
    std::ofstream out;
    out.exceptions(std::ios::failbit | std::ios::badbit);
    out.open(path, std::ios::out | std::ios::trunc);
    out.imbue(std::locale::classic());
    out << std::setprecision(std::numeric_limits<double>::max_digits10) << std::boolalpha;
    return out;
}
} // namespace export_detail

// One immutable directory per run. COMPLETE.txt is written last; absence means
// an incomplete export, which must not be used as an FPGA comparison fixture.
// A null result means calibration was skipped, not an all-zero valid camera.
inline std::filesystem::path export_calibration_run(
    const std::filesystem::path& root,
    const std::vector<CalibrationExportView>& views,
    const std::vector<std::vector<Point2f>>& points,
    int width, int height, int rows, int cols, double square_size,
    const CameraCalibrationResult* result) {
    using namespace export_detail;
    if (views.size() != points.size()) throw std::invalid_argument("export view count mismatch");
    std::filesystem::create_directories(root);
    const auto stamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    std::filesystem::path dir;
    for (unsigned suffix = 0;; ++suffix) {
        dir = root / ("run_" + std::to_string(stamp) + "_" + std::to_string(suffix));
        if (std::filesystem::create_directory(dir)) break;
    }
    auto csv = file(dir / "corners.csv");
    csv << "view_id,point_index,x,y,x_fp32_hex,y_fp32_hex\n";
    bool rtl_ready = result && views.size() == 3 && rows == 5 && cols == 8 &&
        width >= 2 && height >= 2 && width <= 65535 && height <= 65535 &&
        std::isfinite(square_size) && square_size > 0;
    for (size_t v = 0; v < points.size(); ++v) {
        rtl_ready = rtl_ready && views[v].image_loaded && views[v].board_valid &&
            views[v].width == width && views[v].height == height && points[v].size() == 40;
        for (size_t i = 0; i < points[v].size(); ++i) {
            const auto p = points[v][i];
            rtl_ready = rtl_ready && std::isfinite(p.x) && std::isfinite(p.y);
            csv << v << ',' << i << ',';
            number(csv, p.x); csv << ','; number(csv, p.y);
            csv << ',' << bits(p.x) << ',' << bits(p.y) << '\n';
        }
    }
    csv.close();
    auto json = file(dir / "calibration.json");
    json << "{\n  \"format\": \"closer2fpga.calibration.v1\",\n"
         << "  \"algorithm\": \"single_seed_schur_hybrid_lm_v2\", \"rtl_algorithm_matches\": false,\n"
         << "  \"corner_file\": \"corners.csv\",\n"
         << "  \"width\": " << width << ", \"height\": " << height
         << ", \"rows\": " << rows << ", \"cols\": " << cols << ",\n"
         << "  \"square_size\": "; number(json, square_size);
    json << ", \"square_size_fp64_hex\": " << quote(bits(square_size)) << ",\n"
         << "  \"max_iterations_per_stage\": " << 60
         << ", \"estimate_k3\": " << false << ",\n"
         << "  \"calibration_attempted\": " << (result != nullptr)
         << ", \"rtl_input_ready\": " << rtl_ready << ",\n  \"views\": [\n";
    for (size_t v = 0; v < views.size(); ++v) {
        const auto& view = views[v];
        json << "    {\"view_id\": " << v << ", \"path\": " << quote(view.path)
             << ", \"width\": " << view.width << ", \"height\": " << view.height
             << ", \"image_loaded\": " << view.image_loaded << ", \"board_valid\": " << view.board_valid
             << ", \"corner_count\": " << points[v].size() << '}' << (v+1<views.size()?",\n":"\n");
    }
    json << "  ],\n  \"result\": ";
    if (!result) json << "null\n";
    else {
        const auto& r = *result;
        const auto& k = r.camera;
        const char* names[] = {"fx","fy","cx","cy","k1","k2","k3","p1","p2"};
        const float camera[] = {k.fx,k.fy,k.cx,k.cy,k.k1,k.k2,k.k3,k.p1,k.p2};
        json << "{\n    \"camera_valid\": " << k.valid << ", \"converged\": " << r.converged
             << ", \"weak_geometry\": " << r.weak_geometry << ", \"accepted_steps\": " << r.iterations
             << ",\n    \"metrics_present\": " << (r.poses.size()==views.size() && r.per_view_rms.size()==views.size())
             << ",\n    \"message\": " << quote(r.message);
        for (int hex = 0; hex < 2; ++hex) {
            json << ",\n    " << quote(hex?"camera_fp32_hex":"camera") << ": {";
            for (int i = 0; i < 9; ++i) {
                if (i) json << ", "; json << quote(names[i]) << ": ";
                if (hex) json << quote(bits(camera[i])); else number(json, camera[i]);
            }
            json << '}';
        }
        json << ",\n    \"rms\": "; number(json, r.rms);
        json << ", \"rms_fp64_hex\": " << quote(bits(r.rms))
             << ",\n    \"max_error\": "; number(json, r.max_error);
        json << ", \"max_error_fp64_hex\": " << quote(bits(r.max_error))
             << ",\n    \"per_view_rms\": "; numbers(json, r.per_view_rms, false);
        json << ",\n    \"per_view_rms_fp64_hex\": "; numbers(json, r.per_view_rms, true);
        json << ",\n    \"poses\": [\n";
        for (size_t v = 0; v < r.poses.size(); ++v) {
            const auto& pose = r.poses[v];
            json << "      {\"view_id\": " << v << ", \"rotation_row_major\": "; numbers(json, pose.rotation, false);
            json << ", \"rotation_fp64_hex\": "; numbers(json, pose.rotation, true);
            json << ", \"translation\": "; numbers(json, pose.translation, false);
            json << ", \"translation_fp64_hex\": "; numbers(json, pose.translation, true);
            json << '}' << (v+1<r.poses.size()?",\n":"\n");
        }
        json << "    ]\n  }\n";
    }
    json << "}\n";
    json.close();
    auto complete = file(dir / "COMPLETE.tmp");
    complete << "closer2fpga.calibration.v1\nExport completed; use corners.csv and calibration.json from this same directory.\n";
    complete.close();
    std::filesystem::rename(dir / "COMPLETE.tmp", dir / "COMPLETE.txt");
    return std::filesystem::absolute(dir);
}
} // namespace desktop
