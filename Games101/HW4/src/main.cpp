#include <chrono>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata)
{
    // if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4)
    if (event == cv::EVENT_LBUTTONDOWN) //  && control_points.size() < 4)
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
                  << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }

    if (event == cv::EVENT_RBUTTONUP && !control_points.empty())
    {
        control_points.pop_back();
    }

    auto point_size = control_points.size();
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window)
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                     3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

#if 0
cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    if(control_points.size() == 1) return control_points.back();

    std::vector<cv::Point2f> newControlPoint;
    for(auto i = 0; i+1 < control_points.size(); i++)
    {
        auto newPoint = control_points[i] + t * (control_points[i +1] - control_points[i]);
        newControlPoint.emplace_back(std::move(newPoint));
    }

    return recursive_bezier(newControlPoint, t);
}
#else

/**
 * @brief 计算阶乘的函数
 * @param n 计算阶乘的数据
 * @return decltype(n) 计算的结果
 * @note 递归的计算方式
 */
auto factorial(int n) -> decltype(n)
{
    if (n == 0)
    {
        return 1;
    }

    return n * factorial(n - 1);
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t)
{
    auto n = control_points.size() - 1;
    cv::Point2f newPoint{0.0f, 0.0f};
    for (auto i = 0; i <= n; i++)
    {
        newPoint += control_points[i] * factorial(n) / static_cast<float>(factorial(n - i) * factorial(i)) *
                   std::pow(t, i) * std::pow(1 - t, n - i);
    }

    return newPoint;
}
#endif

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window)
{
    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        auto point = recursive_bezier(control_points, t);

        /// BGR
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;

#if 0
        /// do AA
        /// pixel center
        auto x = static_cast<uint32_t>(point.x);
        auto y = static_cast<uint32_t>(point.y);

        float centerX = x + 0.5f;
        float centerY = y + 0.5f;

        auto center = cv::Point2f(centerX, centerY);
        // dir
        cv::Point2f dir = center - point;
        auto length = std::sqrt(dir.dot(dir));

        dir = dir / length; /// normalized

        /// pixel in the  way
        auto nearPixel = cv::Point2f(centerX, centerY) + cv::Point2f(1.0, 1.0);
        cv::Vec3b color = cv::Vec3b(0, 255, 0) * length + cv::Vec3b(0, 0, 0) * (1 - length);

        window.at<cv::Vec3b>(nearPixel.y, nearPixel.x) = color;

        /// project to x
        if (dir.x > 0.0f)
        {
            auto xDir = cv::Point2f{dir.x, 0.0f};
            length = dir.dot(xDir);

            xDir = xDir / std::abs(dir.x);
            auto nearPixel_x = center + xDir;

            color = cv::Vec3b(0, 255, 0) * length + cv::Vec3b(0, 0, 0) * (1.0 - length);
            window.at<cv::Vec3b>(nearPixel_x.y, nearPixel_x.x) = color;
        }
        /// project to y
        if (dir.y > 0.0f)
        {
            auto yDir = cv::Point2f{0.0, dir.y};
            length = dir.dot(yDir);

            yDir = yDir / std::abs(dir.y);
            auto nearPixel_y = center + yDir;

            color = cv::Vec3b(0, 255, 0) * length + cv::Vec3b(0, 0, 0) * (1.0 - length);
            window.at<cv::Vec3b>(nearPixel_y.y, nearPixel_y.x) = color;
        }
#endif
    }
}

int main()
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) /// < Esc
    {
        window.setTo(cv::Scalar(0));
        for (auto &point : control_points)
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() >= 4)
        {
            naive_bezier(control_points, window);
        }

        if (control_points.size() > 2)
        {
            bezier(control_points, window);

            // cv::imshow("Bezier Curve", window);
            // cv::imwrite("my_bezier_curve.png", window);
            // key = cv::waitKey(0);

            // return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20); ///< wait key press
    }

    return 0;
}
