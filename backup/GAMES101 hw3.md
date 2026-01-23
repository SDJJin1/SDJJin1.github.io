hw3和hw2在rasterize_triangle函数上的区别是需要自己计算插值

``` c++
void rst::rasterizer::rasterize_triangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& view_pos) 
{
    auto v = t.toVector4();

    int minx = INT_MAX;
    int maxx = INT_MIN;
    int miny = INT_MAX;
    int maxy = INT_MIN;

    for(auto& i : v)
    {
        minx = i.x() < minx ? i.x() : minx;
        miny = i.y() < miny ? i.y() : miny;
        maxx = i.x() > maxx ? i.x() : maxx;
        maxy = i.y() > maxy ? i.y() : maxy;
    }

    for(int i = minx; i <= maxx; i++)
    {
        for(int j = miny; j <= maxy; j++)
        {
            float x = i + 0.5f;
            float y = j + 0.5f;

            if(insideTriangle(x, y, t.v))
            {
                auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float Z = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma/v[2].w());
    			float zp = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    		    zp *= Z;
                
                int index = get_index(i, j);
                if(zp < depth_buf[index])
                {
                    auto interpolated_color = interpolate(alpha, beta, gamma, t.color[0], t.color[1], t.color[2], 1);
                    auto interpolated_normal = interpolate(alpha, beta, gamma, t.normal[0], t.normal[1], t.normal[2], 1).normalized();
                    auto interpolated_texcoords = interpolate(alpha, beta, gamma, t.tex_coords[0], t.tex_coords[1], t.tex_coords[2], 1);
    			    auto interpolated_shadingcoords = interpolate(alpha, beta, gamma, view_pos[0], view_pos[1], view_pos[2], 1);
                    
                    fragment_shader_payload payload(interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
                    payload.view_pos = interpolated_shadingcoords;
			  depth_buf[index] = zp;
                    auto pixel_color = fragment_shader(payload);

                    set_pixel(Eigen::Vector2i(i, j), pixel_color);
                }
            }
        }
    }
}
```

完成后就可以得到

<img width="700" height="700" alt="Image" src="https://github.com/user-attachments/assets/708b1b64-2b6d-4cc7-995f-efd79ce57af9" />

Blinn-Phong模型的公式

<img width="1003" height="633" alt="Image" src="https://github.com/user-attachments/assets/a78e832d-0622-47b5-a728-3a2e3745d473" />

``` c++
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

    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto& light : lights)
    {
        Eigen::Vector3f light_direction = (light.position - point).normalized();
        Eigen::Vector3f view_direction = (eye_pos - point).normalized();
        Eigen::Vector3f half_vector = (light_direction + view_direction).normalized();

        float reflection_intensity = std::max(0.0f, half_vector.dot(normal));

        float light_intensity_attenuation = (light.position - point).dot(light.position - point);

        Eigen::Vector3f diffuse_reflection = kd.cwiseProduct(light.intensity / light_intensity_attenuation);
        diffuse_reflection *= reflection_intensity;

        Eigen::Vector3f high_lights = ks.cwiseProduct(light.intensity / light_intensity_attenuation);
        high_lights *= std::pow(reflection_intensity, p);

        result_color += (diffuse_reflection + high_lights);
        
    }

    Eigen::Vector3f ambient_light_reflection = ka.cwiseProduct(amb_light_intensity);

    result_color += ambient_light_reflection;

    return result_color * 255.f;
}
```
就可以得到

<img width="700" height="700" alt="Image" src="https://github.com/user-attachments/assets/ce183f1d-4fcf-4a16-9dae-bff3e108d667" />

按题目的要求，将纹理颜色视为公式中的kd
``` c++
Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
     return_color = payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y());

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
        Eigen::Vector3f light_direction = (light.position - point).normalized();
        Eigen::Vector3f view_direction = (eye_pos - point).normalized();
        Eigen::Vector3f half_vector = (light_direction + view_direction).normalized();

        float reflection_intensity = std::max(0.0f, half_vector.dot(normal));

        float light_intensity_attenuation = (light.position - point).dot(light.position - point);

        Eigen::Vector3f diffuse_reflection = kd.cwiseProduct(light.intensity / light_intensity_attenuation);
        diffuse_reflection *= reflection_intensity;

        Eigen::Vector3f high_lights = ks.cwiseProduct(light.intensity / light_intensity_attenuation);
        high_lights *= std::pow(reflection_intensity, p);

        result_color += (diffuse_reflection + high_lights);
    }
        Eigen::Vector3f ambient_light_reflection = ka.cwiseProduct(amb_light_intensity);

        result_color += ambient_light_reflection;

    return result_color * 255.f;
}
```

<img width="700" height="700" alt="Image" src="https://github.com/user-attachments/assets/b006f374-e3bd-4c85-84d8-f5c362f1c4e9" />

bump
``` c++
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

 
    float x = normal.x();
    float y = normal.y();
    float z = normal.z();
    Eigen::Vector3f t = {-x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),-z*y/sqrt(x*x+z*z)};
    Eigen::Vector3f b = normal.cross(t);
    Eigen::Matrix3f TBN;
    TBN << t.x(), b.x(), normal.x(), t.y(), b.y(), normal.y(), t.z(), b.z(), normal.z();
    float u = payload.tex_coords.x();
    float v = payload.tex_coords.y();
    float w = payload.texture->width;
    float h = payload.texture->height;
    float dU = kh * kn *(payload.texture->getColor(u + 1.0 / w, v).norm() - payload.texture->getColor(u, v).norm());
    float dV = kh * kn *(payload.texture->getColor(u, v + 1.0 / h).norm() - payload.texture->getColor(u, v).norm());

    Eigen::Vector3f ln = {-dU, -dV, 1};
    normal = (TBN * ln).normalized();

    Eigen::Vector3f result_color = {0, 0, 0};
    result_color = normal;

    return result_color * 255.f;
}
```
<img width="700" height="700" alt="Image" src="https://github.com/user-attachments/assets/cc0e7179-98de-41f4-bd74-f0cc7bac9d1c" />

displacement
``` c++
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
    
    float x = normal.x();
    float y = normal.y();
    float z = normal.z();
    Eigen::Vector3f t = {-x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),-z*y/sqrt(x*x+z*z)};
    Eigen::Vector3f b = normal.cross(t);
    Eigen::Matrix3f TBN;
    TBN << t.x(), b.x(), normal.x(), t.y(), b.y(), normal.y(), t.z(), b.z(), normal.z();
    float u = payload.tex_coords.x();
    float v = payload.tex_coords.y();
    float w = payload.texture->width;
    float h = payload.texture->height;
    float dU = kh * kn *(payload.texture->getColor(u + 1.0 / w, v).norm() - payload.texture->getColor(u, v).norm());
    float dV = kh * kn *(payload.texture->getColor(u, v + 1.0 / h).norm() - payload.texture->getColor(u, v).norm());

    Eigen::Vector3f ln = {-dU, -dV, 1};
    normal = (TBN * ln).normalized();


    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        Eigen::Vector3f light_direction = (light.position - point).normalized();
        Eigen::Vector3f view_direction = (eye_pos - point).normalized();
        Eigen::Vector3f half_vector = (light_direction + view_direction).normalized();

        float reflection_intensity = std::max(0.0f, half_vector.dot(normal));

        float light_intensity_attenuation = (light.position - point).dot(light.position - point);

        Eigen::Vector3f diffuse_reflection = kd.cwiseProduct(light.intensity / light_intensity_attenuation);
        diffuse_reflection *= reflection_intensity;

        Eigen::Vector3f high_lights = ks.cwiseProduct(light.intensity / light_intensity_attenuation);
        high_lights *= std::pow(reflection_intensity, p);

        result_color += (diffuse_reflection + high_lights);


    }

    Eigen::Vector3f ambient_light_reflection = ka.cwiseProduct(amb_light_intensity);

    result_color += ambient_light_reflection;
    return result_color * 255.f;
}
```

<img width="700" height="700" alt="Image" src="https://github.com/user-attachments/assets/88b7701e-5797-4bb2-aa3f-ee47fd00b165" />