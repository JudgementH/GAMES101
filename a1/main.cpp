#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos) {
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
                 0, 1, 0, -eye_pos[1],
                 0, 0, 1, -eye_pos[2],
                 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle) {
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    double radian = rotation_angle / 180.0 * MY_PI;
    model << cos(radian), -sin(radian), 0, 0,
            sin(radian), cos(radian), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar) {
    zNear = -zNear;
    zFar = -zFar;
    // Students will implement this function
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    double height = 2.0 * abs(zNear) * tan(eye_fov / 2.0);
    double weight = aspect_ratio * height;
    projection << (2.0 * zNear) / weight, 0, 0, 0,
            0, (2.0 * zNear) / height, 0, 0,
            0, 0, (zNear + zFar) / (zNear - zFar), (2.0 * zNear * zFar) / (zNear - zFar),
            0, 0, 1, 0;
    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {
    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
    //绕着任意轴旋转
    // angle 是 角度值， CPP的cos函数需要弧度制
    double radian = angle / 180.0 * MY_PI;
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f N;
    N << 0, -axis[2], axis[1],
            axis[2], 0, -axis[0],
            axis[1], axis[0], 0;
    auto temp = cos(radian) * I + (1 - cos(radian)) * axis * axis.transpose() + sin(radian) * N;
    rotation << temp(0, 0), temp(0, 1), temp(0, 2), 0,
            temp(1, 0), temp(1, 1), temp(1, 2), 0,
            temp(2, 0), temp(2, 1), temp(2, 2), 0,
            0, 0, 0, 1;
    return rotation;
}


int main(int argc, const char **argv) {
   float angle = 0;
   bool command_line = false;
   std::string filename = "output.png";

   if (argc >= 3) {
       command_line = true;
       angle = std::stof(argv[2]); // -r by default
       if (argc == 4) {
           filename = std::string(argv[3]);
       } else
           return 0;
   }

   rst::rasterizer r(700, 700);

   Eigen::Vector3f eye_pos = {0, 0, 5};

   std::vector<Eigen::Vector3f> pos{{2,  0, -2},
                                    {0,  2, -2},
                                    {-2, 0, -2}};

   std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

   auto pos_id = r.load_positions(pos);
   auto ind_id = r.load_indices(ind);

   int key = 0;
   int frame_count = 0;

   if (command_line) {
       r.clear(rst::Buffers::Color | rst::Buffers::Depth);

       r.set_model(get_model_matrix(angle));
       r.set_view(get_view_matrix(eye_pos));
       r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

       r.draw(pos_id, ind_id, rst::Primitive::Triangle);
       cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
       image.convertTo(image, CV_8UC3, 1.0f);

       cv::imwrite(filename, image);

       return 0;
   }

   Eigen::Vector3f axis(1.0f, 1.0f, 1.0f);

   while (key != 27) {
       r.clear(rst::Buffers::Color | rst::Buffers::Depth);

//        r.set_model(get_model_matrix(angle));
       r.set_model(get_rotation(axis, angle));
       r.set_view(get_view_matrix(eye_pos));
       r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

       r.draw(pos_id, ind_id, rst::Primitive::Triangle);

       cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
       image.convertTo(image, CV_8UC3, 1.0f);
       cv::imshow("image", image);
       key = cv::waitKey(10);

       std::cout << "frame count: " << frame_count++ << '\n';

       if (key == 'a') {
           angle += 10;
       } else if (key == 'd') {
           angle -= 10;
       }
   }

    return 0;
}
