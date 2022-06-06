// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions) {
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices) {
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols) {
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f &v3, float w = 1.0f) {
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f *_v) {
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f p(x, y, 0);
    Vector3f ap = p - _v[0];
    Vector3f bp = p - _v[1];
    Vector3f cp = p - _v[2];
    float sign1 = ap.cross(_v[1] - _v[0]).z();
    float sign2 = bp.cross(_v[2] - _v[1]).z();
    float sign3 = cp.cross(_v[0] - _v[2]).z();
    return (sign1 > 0 && sign2 > 0 && sign3 > 0) || (sign1 < 0 && sign2 < 0 && sign3 < 0);
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f *v) {
    float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) /
               (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() -
                v[2].x() * v[1].y());
    float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) /
               (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() -
                v[0].x() * v[2].y());
    float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) /
               (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() -
                v[1].x() * v[0].y());
    return {c1, c2, c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type) {
    auto &buf = pos_buf[pos_buffer.pos_id];
    auto &ind = ind_buf[ind_buffer.ind_id];
    auto &col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto &i : ind) {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto &vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto &vert : v) {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i) {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);
        rasterize_triangle(t);
    }

    //down sampling
    if (ssaa != nullptr) {
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {

                Eigen::Vector3f color = {0, 0, 0};
                for (int j = y * ssaa->times; j < (y + 1) * ssaa->times; j++) {
                    for (int i = x * ssaa->times; i < (x + 1) * ssaa->times; i++) {
                        int index_ssaa = get_ssaa_index(i, j);
                        color += ssaa->frame_buf[index_ssaa];
                    }
                }

                set_pixel(Eigen::Vector3f(x, y, 0), color / (ssaa->times * ssaa->times));

            }
        }
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle &t) {
    if (ssaa != nullptr) rasterize_triangle_ssaa(t);
    else if(msaa != nullptr)
    rasterize_triangle_msaa(t);
    else rasterize_triangle_no_aa(t);
}

void rst::rasterizer::set_model(const Eigen::Matrix4f &m) {
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f &v) {
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f &p) {
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff) {
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color) {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        if (ssaa != nullptr) {
            std::fill(ssaa->frame_buf.begin(), ssaa->frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        }
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth) {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        if (ssaa != nullptr) {
            std::fill(ssaa->depth_buf.begin(), ssaa->depth_buf.end(), std::numeric_limits<float>::infinity());
        }
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h) {
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

rst::rasterizer::rasterizer(int w, int h, rst::SSAA *aa) : width(w), height(h), ssaa(aa) {
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    assert(ssaa->times > 0);
    ssaa->frame_buf.resize(w * h * ssaa->times * ssaa->times);
    ssaa->depth_buf.resize(w * h * ssaa->times * ssaa->times);
}

rst::rasterizer::rasterizer(int w, int h, rst::MSAA *aa) : width(w), height(h), msaa(aa) {
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    assert(msaa->times > 0);
}

int rst::rasterizer::get_index(int x, int y) {
    return (height - 1 - y) * width + x;
}

int rst::rasterizer::get_ssaa_index(int x, int y) {
    return (height * ssaa->times - 1 - y) * width * ssaa->times + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f &point, const Eigen::Vector3f &color) {
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height - 1 - point.y()) * width + point.x();
    frame_buf[ind] = color;

}

//no Anti-Aliasing
void rst::rasterizer::rasterize_triangle_no_aa(const Triangle &t) {
    auto v = t.toVector4();

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    float min_x = std::min(v[0].x(), std::min(v[1].x(), v[2].x()));
    float max_x = std::max(v[0].x(), std::max(v[1].x(), v[2].x()));
    float min_y = std::min(v[0].y(), std::min(v[1].y(), v[2].y()));
    float max_y = std::max(v[0].y(), std::max(v[1].y(), v[2].y()));

    for (int y = (int) min_y; y <= (int) max_y; y++) {
        for (int x = (int) min_x; x <= (int) max_x; x++) {
            if (insideTriangle(x, y, t.v)) {
                // If so, use the following code to get the interpolated z value.
                //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                //z_interpolated *= w_reciprocal;
                auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated =
                        alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                int index = get_index(x, y);

                // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                if (z_interpolated < depth_buf[index]) {
                    depth_buf[index] = z_interpolated;
                    set_pixel(Vector3f(x, y, z_interpolated), t.getColor());
                }
            }
        }
    }

}

//ssaa
void rst::rasterizer::rasterize_triangle_ssaa(const Triangle &t) {
    auto v = t.toVector4();

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    float min_x = std::min(v[0].x(), std::min(v[1].x(), v[2].x()));
    float max_x = std::max(v[0].x(), std::max(v[1].x(), v[2].x()));
    float min_y = std::min(v[0].y(), std::min(v[1].y(), v[2].y()));
    float max_y = std::max(v[0].y(), std::max(v[1].y(), v[2].y()));

    for (int y = (int) min_y; y < (int) max_y; y++) {
        for (int x = (int) min_x; x < (int) max_x; x++) {

            //in one pixel
            for (int j = y * ssaa->times; j < (y + 1) * ssaa->times; j++) {
                for (int i = x * ssaa->times; i < (x + 1) * ssaa->times; i++) {
                    float x_ = (float) i / ssaa->times;
                    float y_ = (float) j / ssaa->times;
                    if (insideTriangle(x_, y_, t.v)) {
                        // If so, use the following code to get the interpolated z value.
                        //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                        //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        //z_interpolated *= w_reciprocal;
                        auto[alpha, beta, gamma] = computeBarycentric2D(x_, y_, t.v);
                        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated =
                                alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;
                        int index = get_ssaa_index(i, j);

                        // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                        if (z_interpolated < ssaa->depth_buf[index]) {
                            ssaa->depth_buf[index] = z_interpolated;
                            ssaa->frame_buf[index] = t.getColor();
                        }

                    }
                }

            }
        }
    }
}

//msaa
void rst::rasterizer::rasterize_triangle_msaa(const Triangle &t) {
    auto v = t.toVector4();

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    float min_x = std::min(v[0].x(), std::min(v[1].x(), v[2].x()));
    float max_x = std::max(v[0].x(), std::max(v[1].x(), v[2].x()));
    float min_y = std::min(v[0].y(), std::min(v[1].y(), v[2].y()));
    float max_y = std::max(v[0].y(), std::max(v[1].y(), v[2].y()));

    for (int y = (int) min_y; y < (int) max_y; y++) {
        for (int x = (int) min_x; x < (int) max_x; x++) {

            //in one pixel
            float count = 0.0;
            for (int j = y * msaa->times; j < (y + 1) * msaa->times; j++) {
                for (int i = x * msaa->times; i < (x + 1) * msaa->times; i++) {
                    float x_ = (float) i / msaa->times;
                    float y_ = (float) j / msaa->times;
                    if (insideTriangle(x_, y_, t.v)) {
                        count += 1;
                    }
                }
            }

            if (count > 0) {
                // If so, use the following code to get the interpolated z value.
                //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                //z_interpolated *= w_reciprocal;
                auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated =
                        alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                int index = get_index(x, y);

                // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                if (z_interpolated < depth_buf[index]) {
                    depth_buf[index] = z_interpolated;
                    Vector3f color = (count / (msaa->times * msaa->times)) * t.getColor();
                    set_pixel(Vector3f(x, y, z_interpolated), color);
                }
            }


        }
    }
}


// clang-format on