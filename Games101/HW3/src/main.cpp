#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float angle)
{
    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
                0, 1, 0, 0,
                -sin(angle), 0, cos(angle), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
              0, 2.5, 0, 0,
              0, 0, 2.5, 0,
              0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    float eye_fov_radian = eye_fov * MY_PI / 180;

    // get widget width & height 
    auto t = std::abs(zNear) * std::tan(eye_fov_radian * 0.5);
    auto b = -t;
    auto r = t * aspect_ratio;
    auto l = -r;

    Eigen::Matrix4f translation;
    translation << 1.0, 0.0, 0.0, -(r + l) / 2.0,
                   0.0, 1.0, 0.0, -(t + b) / 2.0,
                   0.0, 0.0, 1.0, -(zNear + zFar) / 2.0,
                   0.0, 0.0, 0.0, 1.0;

    Eigen::Matrix4f scale;
    scale << 2.0 / (r  - l), 0.0, 0.0, 0,
            0.0, 2.0 / (t - b), 0.0, 0,
            0.0, 0.0, 2.0 / (zFar - zNear), 0,
            0.0, 0.0, 0.0, 1.0;


    Eigen::Matrix4f persp2Ortho;
    persp2Ortho << zNear, 0.0, 0.0, 0,
                   0.0, zNear, 0.0, 0,
                   0.0, 0.0, (zNear + zFar), -zFar*zNear,
                   0.0, 0.0, 1.0, 0.0;  

    /// 你一定很好奇这里为什么要有一个反转矩阵
    /// 当然是因为zNear < zFar & > 0
    /// 推导的矩阵无法正确将数据变换
    Eigen::Matrix4f mt;
    mt << 1.0, 0.0, 0.0,  0,
            0.0, 1.0, 0.0,  0,
            0.0, 0.0, -1.0, 0,
            0.0, 0.0, 0.0,  1.0;  

    projection =  scale * translation * persp2Ortho * mt;
    
    return projection;
}

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        auto texCoord = payload.tex_coords;
        return_color = payload.texture->getColor(texCoord.x(), texCoord.y());
    }

    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        Eigen::Vector3f viewDir = (eye_pos - point).normalized();
        Eigen::Vector3f lightDir = (light.position - point).normalized();
        Eigen::Vector3f half = (viewDir + lightDir).normalized();

        auto R_2 = (point - light.position).dot(point - light.position);
        Eigen::Vector3f I_R2 = light.intensity / R_2;

        Eigen::Vector3f La = ka.cwiseProduct(amb_light_intensity);
        Eigen::Vector3f Ld = kd.cwiseProduct(I_R2) * std::max(0.0f, normal.dot(lightDir)); 
        Eigen::Vector3f Ls = ks.cwiseProduct(I_R2) * std::pow(std::max(0.0f, normal.dot(half)), p);

        result_color +=  (La + Ld + Ls);
    }

    return result_color * 255.f;
}

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;
    normal = normal.normalized();

    Eigen::Vector3f result_color = {0.0,0.0,0.0};
    for (auto& light : lights)
    {
        Eigen::Vector3f viewDir = (eye_pos - point).normalized();
        Eigen::Vector3f lightDir = (light.position - point).normalized();
        Eigen::Vector3f half = (viewDir + lightDir).normalized();

        auto R_2 = (point - light.position).dot(point - light.position);
        Eigen::Vector3f I_R2 = light.intensity / R_2;

        Eigen::Vector3f La = ka.cwiseProduct(amb_light_intensity);
        Eigen::Vector3f Ld = kd.cwiseProduct(I_R2) * std::max(0.0f, normal.dot(lightDir)); 
        Eigen::Vector3f Ls = ks.cwiseProduct(I_R2) * std::pow(std::max(0.0f, normal.dot(half)), p);

        result_color +=  (La + Ld + Ls);
    }

    return result_color * 255.f;
}

Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;

    /// 获得切线空间的法线  -- 从纹理采样获得
    /// 计算TBN矩阵
    /// 变换法线到世界空间中， 切线空间->世界空间

    auto x = normal.x();
    auto y = normal.y();
    auto z = normal.z();
    Eigen::Vector3f n = {x, y, z};
    Eigen::Vector3f t = {x * y / sqrt(x * x + z * z), sqrt(x * x + z * z), z * y / sqrt(x * x + z * z)};
    t = t.normalized();
    Eigen::Vector3f b = n.cross(t);
    b = b.normalized();
    Eigen::Matrix3f TBN ;
    TBN << t.x(), b.x(), n.x(),
            t.y(), b.y(), n.y(),
            t.z(), b.z(), n.z();

    auto texture = payload.texture;
    auto texCoord = payload.tex_coords;
    auto u = texCoord.x();
    auto v = texCoord.y();

    auto w = texture->width;
    auto h = texture->height;

    /// 这里的纹理虽然给的是一个法线贴图 但是 这里将纹理当作了高度图使用，所以此处的操作是为了将纹理的值转换为高度，然后使用高度图计算此处的切平面，然后通过cross的方式计算法线
    auto dU = kh * kn * (texture->getColor(u + 1.0f / w, v).norm() - texture->getColor(u, v).norm());
    auto dV = kh * kn * (texture->getColor(u, v + 1.0f /h).norm() - texture->getColor(u, v).norm());

    Eigen::Vector3f ln = { -dU, -dV, 1.0f};
    normal = (TBN * ln).normalized();

#if 0
    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto& light : lights)
    {
        Eigen::Vector3f viewDir = (eye_pos - point).normalized();
        Eigen::Vector3f lightDir = (light.position - point).normalized();
        Eigen::Vector3f half = (viewDir + lightDir).normalized();

        auto R_2 = (point - light.position).dot(point - light.position);
        Eigen::Vector3f I_R2 = light.intensity / R_2;

        Eigen::Vector3f La = ka.cwiseProduct(amb_light_intensity);
        Eigen::Vector3f Ld = kd.cwiseProduct(I_R2) * std::max(0.0f, normal.dot(lightDir));
        Eigen::Vector3f Ls = ks.cwiseProduct(I_R2) * std::pow(std::max(0.0f, normal.dot(half)), p);

        result_color +=  (La + Ld + Ls);
    }
#else
    Eigen::Vector3f result_color = {0, 0, 0};
    result_color = normal;
#endif
    return result_color * 255.f;

    // TODO: Implement bump mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)

    // Normal n = normalize(TBN * ln)
}

Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;

    auto x = normal.x();
    auto y = normal.y();
    auto z = normal.z();

    Eigen::Vector3f n = {x,y,z}; n = n.normalized();
    Eigen::Vector3f t = {x * y / sqrt(x * x + z * z), sqrt(x * x + z * z), z * y / sqrt( x * x + z * z)}; t = t.normalized();
    Eigen::Vector3f b = n.cross(t); b = b.normalized();
    Eigen::Matrix3f TBN;
    TBN << t.x(), b.x(), n.x(),
           t.y(), b.y(), n.y(),
           t.z(), b.z(), n.z();

    auto texture = payload.texture;
    auto u = payload.tex_coords.x();
    auto v = payload.tex_coords.y();
    auto w = texture->width;
    auto h = texture->height;

    auto dU = kh * kn * (texture->getColor(u + 1.0f / w, v).norm() - texture->getColor(u, v).norm());
    auto dV = kh * kn * (texture->getColor(u, v + 1.0f / h).norm() - texture->getColor(u, v).norm());

    auto ln = Eigen::Vector3f{-dU, -dV, 1.0f};

    /// 对顶点进行实际的偏移 -- 不使用计算得到的法线偏移顶点  而是使用原来的顶点法线偏移顶点
    point =  point + kn * normal * texture->getColor(u, v).norm();

    /// 更新法线 使用更新后的法线计算光照
    normal = (TBN * ln).normalized();

    // TODO: Implement displacement mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Position p = p + kn * n * h(u,v)
    // Normal n = normalize(TBN * ln)


    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        Eigen::Vector3f viewDir = (eye_pos - point).normalized();
        Eigen::Vector3f lightDir = (light.position - point).normalized();
        Eigen::Vector3f half = (viewDir + lightDir).normalized();

        auto R_2 = (point - light.position).dot(point - light.position);
        Eigen::Vector3f I_R2 = light.intensity / R_2;

        Eigen::Vector3f La = ka.cwiseProduct(amb_light_intensity);
        Eigen::Vector3f Ld = kd.cwiseProduct(I_R2) * std::max(0.0f, normal.dot(lightDir));
        Eigen::Vector3f Ls = ks.cwiseProduct(I_R2) * std::pow(std::max(0.0f, normal.dot(half)), p);

        result_color +=  (La + Ld + Ls);

    }
    return result_color * 255.f;
}


int main(int argc, const char** argv)
{
    std::vector<Triangle*> TriangleList;

    float angle = 140;
    bool command_line = false;

    std::string filename = "output.png";
    objl::Loader Loader;
    std::string obj_path = "../models/spot/";

    // Load .obj File
    bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
    for(auto mesh:Loader.LoadedMeshes)
    {
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            Triangle* t = new Triangle();
            for(int j=0;j<3;j++)
            {
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));
                t->setNormal(j,Vector3f(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }

    rst::rasterizer r(700, 700);

    auto texture_path = "hmap.jpg";
    r.set_texture(Texture(obj_path + texture_path));

    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;

    if (argc >= 2)
    {
        command_line = true;
        filename = std::string(argv[1]);

        if (argc == 3 && std::string(argv[2]) == "texture")
        {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            texture_path = "spot_texture.png";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (argc == 3 && std::string(argv[2]) == "normal")
        {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "phong")
        {
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "bump")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "displacement")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = displacement_fragment_shader;
        }
    }

    Eigen::Vector3f eye_pos = {0,0,10};

    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);
    // r.set_fragment_shader(normal_fragment_shader);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

        if (key == 'a' )
        {
            angle -= 0.1;
        }
        else if (key == 'd')
        {
            angle += 0.1;
        }

    }
    return 0;
}
