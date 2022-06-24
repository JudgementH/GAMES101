//
// Created by goksu on 2/25/20.
//
#include <thread>
#include <mutex>
#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"

inline float deg2rad(const float &deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;
std::mutex mtx;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene &scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int progress = 0;

    // change the spp value to change sample ammount
    int spp = 16;
    int m = 0;

    std::cout << "SPP: " << spp << "\n";
    constexpr int num_threads = 32;
    std::thread th[num_threads];
    int thread_height = scene.height / num_threads;
    auto renderRows = [&](uint32_t start_height, uint32_t end_height)
    {
        for (uint32_t j = start_height; j < end_height; ++j)
        {
            for (uint32_t i = 0; i < scene.width; ++i)
            {
                // generate primary ray direction
                float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                          imageAspectRatio * scale;
                float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

                Vector3f dir = normalize(Vector3f(-x, y, 1));
                for (int k = 0; k < spp; k++)
                {
                    framebuffer[static_cast<int>(j * scene.width + i)] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
                }
            }
            mtx.lock();
            progress++;
            UpdateProgress(progress / (float)scene.height);
            mtx.unlock();
        }
    };

    for (int t = 0; t < num_threads; ++t)
    {
        th[t] = std::thread(renderRows, t * thread_height, (t + 1) * thread_height);
    }
    for (int t = 0; t < num_threads; ++t)
    {
        th[t].join();
    }

    UpdateProgress(1.f);

    // save framebuffer to file
    FILE *fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i)
    {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);
}
