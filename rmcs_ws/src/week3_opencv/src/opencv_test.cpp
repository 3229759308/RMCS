#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iostream>
#include <string>
#include <vector>

int main()
{
    // 从 ROS2 包的安装目录读取彩色图片。
    const std::string path =
        ament_index_cpp::get_package_share_directory("week3_opencv")
        + "/assets/images/test.jpg";

    const cv::Mat img = cv::imread(path, cv::IMREAD_COLOR);

    if (img.empty()) {
        std::cerr << "Image load failed: " << path << '\n';
        return 1;
    }

    // 1. 灰度化：将三个颜色通道转换为一个亮度通道。
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    // 2. 高斯滤波：减少细碎噪声对边缘检测的影响。
    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(3, 3), 0);

    // 3. Canny 边缘检测：得到黑底白色边缘图。
    cv::Mat edges;
    cv::Canny(blurred, edges, 20, 80);

    // 4. 提取所有轮廓，包括人物内部细节。
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(
        edges,
        contours,
        cv::RETR_LIST,
        cv::CHAIN_APPROX_SIMPLE
    );

    std::cout << "Contours: " << contours.size() << '\n';

    // 5. 在原图副本上用红色绘制全部轮廓。
    cv::Mat result = img.clone();
    cv::drawContours(
        result,
        contours,
        -1,
        cv::Scalar(0, 0, 255),
        1,
        cv::LINE_AA
    );
    // 直接使用 gray，先观察不经过高斯滤波的细节效果。
    cv::Mat grad_x, grad_y;

    cv::Sobel(gray, grad_x, CV_32F, 1, 0, 3);
    cv::Sobel(gray, grad_y, CV_32F, 0, 1, 3);

    // 计算梯度强度，综合横向和纵向的灰度变化。
    cv::Mat magnitude;
    cv::magnitude(grad_x, grad_y, magnitude);

    // 将梯度强度转换为黑色线条的深度。
    const float strength = 1.0f;
    const float noise_floor = 10.0f;  // 原来是 0.0f

    cv::Mat sketch(gray.size(), CV_8UC1);

    for (int y = 0; y < gray.rows; ++y) {
        for (int x = 0; x < gray.cols; ++x) {
            float value = magnitude.at<float>(y, x) - noise_floor;

            if (value < 0.0f) {
                value = 0.0f;
            }

            sketch.at<uchar>(y, x) =
                cv::saturate_cast<uchar>(255.0f - strength * value);
        }
    }

    // 转成三通道，兼容你后面已有的渐变着色代码。
    cv::Mat line_art;
    cv::cvtColor(sketch, line_art, cv::COLOR_GRAY2BGR);

    cv::imshow("Sobel Sketch", line_art);
    // 渐变的颜色节点，注意 OpenCV 使用 BGR 顺序。
    const std::vector<cv::Vec3d> colors = {
        {40, 165, 255},  // 橙色
        {80,  70, 255},  // 红色
        {210, 60, 190},  // 紫色
        {220, 180, 40}   // 青色
    };

    cv::Mat gradient_art = line_art.clone();

    for (int y = 0; y < line_art.rows; ++y) {
        for (int x = 0; x < line_art.cols; ++x) {
            // 将坐标归一化到 0~1，避免图片尺寸影响渐变。
            const double nx =
                static_cast<double>(x) / (line_art.cols > 1 ? line_art.cols - 1 : 1);
            const double ny =
                static_cast<double>(y) / (line_art.rows > 1 ? line_art.rows - 1 : 1);

            // 主要从上往下渐变，同时带一点从左往右的变化。
            const double t = 0.25 * nx + 0.75 * ny;

            // 找到当前所在的颜色区间。
            const double position = t * (colors.size() - 1);
            int index = static_cast<int>(position);

            if (index >= static_cast<int>(colors.size()) - 1) {
                index = static_cast<int>(colors.size()) - 2;
            }

            const double ratio = position - index;

            // 在相邻两种颜色之间做线性插值。
            const cv::Vec3d color =
                colors[index] * (1.0 - ratio)
                + colors[index + 1] * ratio;

            // 黑色线条完全着色，灰色边缘部分着色，白底不着色。
            // 这样可以保留 LINE_AA 产生的平滑边缘。
            const double alpha =
                1.0 - line_art.at<cv::Vec3b>(y, x)[0] / 255.0;

            cv::Vec3b& pixel = gradient_art.at<cv::Vec3b>(y, x);

            for (int c = 0; c < 3; ++c) {
                pixel[c] = cv::saturate_cast<uchar>(
                    255.0 * (1.0 - alpha) + color[c] * alpha
                );
            }
        }
    }


    // 6. 显示原图、边缘图和轮廓叠加结果。
    cv::imshow("Original", img);
    cv::imshow("Gradient Art", gradient_art);
    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}