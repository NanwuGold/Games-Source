//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include <vector>

inline float deg2rad(const float &deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene &scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(-1, 5, 10);
    int m = 0;

    /// 相机坐标
    auto cameraRight = Vector3f(1.0, 0.0, 0.0);
    auto cameraUp = Vector3f(0.0, 1.0, 0.0);
    auto cameraDir = Vector3f(0.0, 0.0, 1.0);

    for (uint32_t j = 0; j < scene.height; ++j)
    {
        for (uint32_t i = 0; i < scene.width; ++i)
        {
            // generate primary ray direction
            float x = (2.0f * (i + 0.5f) / (float)scene.width - 1.0) * imageAspectRatio * scale;
            float y = (1.0f - 2.0f * (j + 0.5f) / (float)scene.height) * scale;

            /// 计算方向需要将相机空间的方向变换到世界空间中，因为场景使用的是世界空间的坐标
            /// auto dir = normalize(Vector3f{x * cameraRight + y * cameraUp + (-1.0) * cameraDir} - Vector3f(0));
            /// 由于相机空间与世界坐标的基向量朝向相同，只存在平移的差异，并且由于平移不对向量方向产生影响，为了简化计算，随使用下面计算方式
            auto dir = normalize(Vector3f{x, y, -1.0});

            auto color = scene.castRay({eye_pos, dir}, 0);
            framebuffer[m++] = color;
        }
        UpdateProgress(j / (float)scene.height);
    }
    UpdateProgress(1.f);

    /// save framebuffer to file
    /// PPM 格式按照行优先顺序写入，只要保证像素来源的数据是对齐的，PPM就可以正确解析像素数据
    /// PPM 格式以左上角为起点，从左向右写入数据
    FILE *fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i)
    {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].x));
        color[1] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].y));
        color[2] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].z));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);
}
