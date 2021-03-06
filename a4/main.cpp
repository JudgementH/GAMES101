#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4)
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }
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

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t)
{
    // TODO: Implement de Casteljau's algorithm
    std::vector<cv::Point2f> result_points = control_points;
    std::vector<cv::Point2f> temp_points;
    while (result_points.size() > 1)
    {
        for (int i = 0; i < result_points.size() - 1; i++)
        {
            temp_points.emplace_back((1 - t) * result_points[i] + t * result_points[i + 1]);
        }
        result_points = temp_points;
        temp_points.clear();
    }
    return result_points[0];
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window)
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's
    // recursive Bezier algorithm.

    for (double t = 0.0; t < 1.0; t += 0.001)
    {
        auto point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;

        std::vector<cv::Point2f> points(4);
        points[0] = cv::Point2f(std::floorf(point.x - 0.5f), std::floorf(point.y - 0.5f));
        points[1] = cv::Point2f(std::ceilf(point.x - 0.5f), std::floorf(point.y - 0.5f));
        points[2] = cv::Point2f(std::ceilf(point.x - 0.5f), std::ceilf(point.y - 0.5f));
        points[3] = cv::Point2f(std::floorf(point.x - 0.5f), std::ceilf(point.y - 0.5f));

        float x_near = std::floorf(point.x);
        float y_near = std::floorf(point.y);

        float d_near = sqrtf((point.x - x_near) * (point.x - x_near) + (point.y - y_near) * (point.y - y_near));

        for (auto &p : points)
        {
            float d_p = sqrtf((point.x - p.x) * (point.x - p.x) + (point.y - p.y) * (point.y - p.y));
            float temp = window.at<cv::Vec3b>(p.y, p.x)[1];
            window.at<cv::Vec3b>(p.y, p.x)[1] = std::max(temp, d_near / d_p * 255);
        }
    }
}

int main()
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27)
    {
        for (auto &point : control_points)
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4)
        {
            naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

    return 0;
}
