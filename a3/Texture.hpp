//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <iostream>
class Texture
{
private:
    cv::Mat image_data;

public:
    Texture(const std::string &name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        float u_img = u * width;
        float v_img = (1 - v) * height;

        int u_low = std::floor(u_img);
        int u_high = std::ceil(u_img);
        int v_low = std::floor(v_img);
        int v_high = std::ceil(v_img);

        float u_lerp_t = u_img - u_low;
        float v_lerp_t = v_img - v_low;

        cv::Vec3b u0_color_lerp = (1 - u_lerp_t) * image_data.at<cv::Vec3b>(v_low, u_low) +
                                  u_lerp_t * image_data.at<cv::Vec3b>(v_low, u_high);

        cv::Vec3b u1_color_lerp = (1 - u_lerp_t) * image_data.at<cv::Vec3b>(v_high, u_low) +
                                  u_lerp_t * image_data.at<cv::Vec3b>(v_high, u_high);

        cv::Vec3b color = (1 - v_lerp_t) * u0_color_lerp + v_lerp_t * u1_color_lerp;

        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
};
#endif // RASTERIZER_TEXTURE_H
