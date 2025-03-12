#include <chrono>
#include <iostream>
#include <vector>
#include <cmath>
#include <numeric>
#include <opencv2/opencv.hpp>
#include "Combination.hpp"

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

/**
 * @brief De Casteljau’s Algorithm 递归实现
 * @param control_points 控制点
 * @param t
 * @return cv::Point2f t对应的插值的点
 */
cv::Point2f recursive_bezier_recursive(const std::vector<cv::Point2f> &control_points, float t)
{
    if (control_points.size() == 1)
    {
        return control_points.back();
    }

    std::vector<cv::Point2f> newControlPoint;
    for (auto i = 0; i + 1 < control_points.size(); i++)
    {
        auto newPoint = control_points[i] + t * (control_points[i + 1] - control_points[i]);
        newControlPoint.emplace_back(std::move(newPoint));
    }

    return recursive_bezier_recursive(newControlPoint, t);
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t)
{
    auto n = control_points.size() - 1;
    cv::Point2f newPoint{0.0f, 0.0f};
    for (auto i = 0; i <= n; i++)
    {
        newPoint += control_points[i] * Combination()(n, i) *
                    std::pow(t, i) * std::pow(1 - t, n - i);
    }

    return newPoint;
}

/**
 * @brief De Casteljau’s Algorithm 实现
 *
 * @param control_points
 * @param t
 * @return cv::Point2f
 */
cv::Point2f recursive_bezier_loop(const std::vector<cv::Point2f> &control_points, float t)
{
    auto temp = std::vector<cv::Point2f>{control_points};

    for (auto i = 1; i < temp.size(); i++)
    {
        for (auto j = 0; j < temp.size() - 1; j++)
        {
            temp[j] = temp[j] + t * (temp[j + 1] - temp[j]);
        }
    }
    return temp[0];
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window)
{
    auto width = window.cols;
    auto height = window.rows;

    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        auto point = recursive_bezier_loop(control_points, t);

        /// 计算最近的像素
        auto x = std::trunc(point.x);
        auto y = std::trunc(point.y);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255; /// 颜色使用 BGR的顺序

        cv::Point2f center(x + 0.5, y + 0.5);
        float d_0 = std::sqrt(std::pow(center.x - x, 2) + std::pow(center.y - y, 2));

        /// 获取周围的3*3的9个像素
        std::vector<cv::Point2f> pixels;
        std::vector<float> weight;
        for (int dy = 0; dy <= 1; dy++)
        {
            for (int dx = 0; dx <= 1; dx++)
            {
                auto px = x + dx;
                auto py = y + dy;

                if (px < 0 || px > width || py < 0 || py >= height)
                {
                    continue;
                }

                float offsetx = point.x - (px + 0.5f);
                float offsety = point.y - (py + 0.5f);
                auto disPoint = cv::Point2f(offsetx, offsety);
                auto dis = std::sqrt(disPoint.dot(disPoint));

                weight.emplace_back(dis);
                pixels.emplace_back(cv::Point2f{px, py});
            }
        }

        float sum_w = std::accumulate(weight.begin(), weight.end(), 0.0f) / d_0;
        for (auto i = 0; i < pixels.size(); i++)
        {
            cv::Vec3b &pixel = window.at<cv::Vec3b>(pixels[i].y, pixels[i].x);
            auto wi = weight[i] / d_0 / sum_w;
            float color = pixel[1];
            pixel[1] = (1 - wi) * color + 255.0f * wi;
        }

        /// TODO:: 或许可以在线条完全生成完之在考虑抗锯齿
        /// 根据实际的效果来看, FXAA是一个更好的思路与方法，上述抗锯齿方案无法实际考虑生成的线的梯度变化，只能单纯的对周围的像素进行着色：
        /// 1. 在线比较弯曲的情况，这种会产生较好的效果。
        /// 2. 在线导数为0的时候，并不是我们期待的效果。
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
