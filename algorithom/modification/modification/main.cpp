#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

int main()
{
    // =========================================================
    // 1. 读取图片
    // =========================================================

    std::string path =
        "E:/fpga/algorithom/exsample1.png";

    cv::Mat image = cv::imread(path);

    if (image.empty())
    {
        std::cerr << "Image load failed!"
            << std::endl;
        return -1;
    }

    std::cout << "Image size: "
        << image.cols
        << " x "
        << image.rows
        << std::endl;


    // =========================================================
    // 2. 灰度化
    // =========================================================

    cv::Mat gray;

    cv::cvtColor(
        image,
        gray,
        cv::COLOR_BGR2GRAY
    );


    // =========================================================
    // 3. 棋盘格参数
    //
    // 你的棋盘：
    // 8 × 6 个内角点
    // =========================================================

    cv::Size patternSize(8, 6);


    // =========================================================
    // 4. 检测棋盘角点
    // =========================================================

    std::vector<cv::Point2f> imagePoints;

    bool found =
        cv::findChessboardCornersSB(
            gray,
            patternSize,
            imagePoints
        );

    std::cout << "Chessboard found: "
        << (found ? "true" : "false")
        << std::endl;

    std::cout << "Corner count: "
        << imagePoints.size()
        << std::endl;


    if (!found)
    {
        std::cerr
            << "Chessboard corners not found!"
            << std::endl;

        cv::imshow("image", image);

        cv::waitKey(0);

        return -1;
    }


    // =========================================================
    // 5. 亚像素角点优化
    // =========================================================

    cv::TermCriteria criteria(
        cv::TermCriteria::EPS |
        cv::TermCriteria::MAX_ITER,
        30,
        0.001
    );

    cv::cornerSubPix(
        gray,
        imagePoints,
        cv::Size(5, 5),
        cv::Size(-1, -1),
        criteria
    );


    // =========================================================
    // 6. 显示检测到的角点
    // =========================================================

    cv::Mat cornerImage =
        image.clone();

    cv::drawChessboardCorners(
        cornerImage,
        patternSize,
        imagePoints,
        true
    );

    cv::imshow(
        "Chessboard Corners",
        cornerImage
    );

    cv::waitKey(500);


    // =========================================================
    // 7. 构造棋盘格的三维世界坐标
    //
    // 假设每个格子大小 = 1
    //
    // Z 全部为 0，因为棋盘是一个平面
    // =========================================================

    std::vector<cv::Point3f> objectPoints;

    for (int row = 0;
        row < patternSize.height;
        row++)
    {
        for (int col = 0;
            col < patternSize.width;
            col++)
        {
            objectPoints.emplace_back(
                static_cast<float>(col),
                static_cast<float>(row),
                0.0f
            );
        }
    }


    std::cout << "3D point count: "
        << objectPoints.size()
        << std::endl;


    // =========================================================
    // 8. calibrateCamera() 要求：
    //
    // vector<vector<Point3f>>
    // vector<vector<Point2f>>
    //
    // 所以即使只有一张图片，也要再套一层 vector
    // =========================================================

    std::vector<std::vector<cv::Point3f>>
        allObjectPoints;

    std::vector<std::vector<cv::Point2f>>
        allImagePoints;

    allObjectPoints.push_back(
        objectPoints
    );

    allImagePoints.push_back(
        imagePoints
    );


    // =========================================================
    // 9. 相机内参、畸变参数
    // =========================================================

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;


    // =========================================================
    // 10. 相机标定
    // =========================================================

    double rms =
        cv::calibrateCamera(
            allObjectPoints,
            allImagePoints,
            gray.size(),
            cameraMatrix,
            distCoeffs,
            rvecs,
            tvecs
        );


    // =========================================================
    // 11. 输出标定结果
    // =========================================================

    std::cout << std::endl;
    std::cout << "============================"
        << std::endl;

    std::cout << "RMS = "
        << rms
        << std::endl;

    std::cout << std::endl;

    std::cout << "Camera Matrix:"
        << std::endl;

    std::cout << cameraMatrix
        << std::endl;

    std::cout << std::endl;

    std::cout << "Distortion Coefficients:"
        << std::endl;

    std::cout << distCoeffs
        << std::endl;


    // =========================================================
    // 12. 输出旋转和平移
    // =========================================================

    std::cout << std::endl;

    std::cout << "Rotation Vector:"
        << std::endl;

    std::cout << rvecs[0]
        << std::endl;

    std::cout << std::endl;

    std::cout << "Translation Vector:"
        << std::endl;

    std::cout << tvecs[0]
        << std::endl;


    // =========================================================
    // 13. 去畸变
    // =========================================================

    cv::Mat undistorted;

    cv::undistort(
        image,
        undistorted,
        cameraMatrix,
        distCoeffs
    );


    // =========================================================
    // 14. 显示原图和去畸变图
    // =========================================================

    cv::imshow(
        "Original",
        image
    );

    cv::imshow(
        "Undistorted",
        undistorted
    );

    cv::waitKey(0);

    return 0;
}

