#include "chessboard.h"
#include "chessboard/internal.h"
#include <algorithm>
using namespace chessboard;
ChessboardInfo detect_chessboard(const GrayImage& gray, int rows, int cols) {
    if (gray.data && gray.w >= 32 && gray.h >= 32 && std::max(gray.w, gray.h) > 960) {
        // Coarse-to-fine detection suppresses multiple responses around broad
        // printed edges. Only the grid search is downsampled; final localization
        // always uses the original image. Fall back to full resolution for small
        // or distant boards whose corners would disappear in the pyramid.
        GrayImage half(gray.w / 2, gray.h / 2);
        for (int y = 0; y < half.h; ++y)
            for (int x = 0; x < half.w; ++x)
                half.set(x, y,
                         uint8_t((int(gray.get(2 * x, 2 * y)) + gray.get(2 * x + 1, 2 * y) +
                                  gray.get(2 * x, 2 * y + 1) + gray.get(2 * x + 1, 2 * y + 1) + 2) /
                                 4));
        auto coarse = detect_chessboard(half, rows, cols);
        if (coarse.valid) {
            for (auto& p : coarse.corners) {
                p.x = 2 * p.x + 0.5f;
                p.y = 2 * p.y + 0.5f;
            }
            if (refine_grid(gray, coarse))
                return coarse;
        }
    }
    return detect_native(gray, rows, cols);
}
