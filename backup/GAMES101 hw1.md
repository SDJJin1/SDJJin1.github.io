根据课上讲的求模型矩阵的方法

<img width="999" height="654" alt="Image" src="https://github.com/user-attachments/assets/66c8a127-94f3-4214-9559-352d19a612f4" />

``` c++
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    float arc1 = rotation_angle / 180.0 * acos(-1);
    model << std::cos(arc1), -std::sin(arc1), 0, 0,
            std::sin(arc1), std::cos(arc1), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return model;
}
```

要求投影矩阵，就要先得到正交矩阵，然后挤压成投影矩阵

<img width="1252" height="819" alt="Image" src="https://github.com/user-attachments/assets/74ec068e-2a5c-485f-9915-67671bb788ea" />

求正交矩阵的方法

<img width="1132" height="751" alt="Image" src="https://github.com/user-attachments/assets/c9abdbb9-46a1-4245-930e-9a55006aa629" />

由正交矩阵挤压成投影矩阵

<img width="343" height="146" alt="Image" src="https://github.com/user-attachments/assets/ae19a871-24a7-46db-a9c3-335e8ae47138" />

再由fov求其他参数

<img width="971" height="548" alt="Image" src="https://github.com/user-attachments/assets/65d17868-4d03-4e7c-b506-da25c6fdf0ba" />

``` c++
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    float t = std::tan(eye_fov) * std::abs(zNear);
    float b = t * -1;
    float r = aspect_ratio * t;
    float l = r * -1;

    Eigen::Matrix4f ortho;
    ortho << 2/ (r - l), 0, 0, 0,
            0, 2/ (t - b), 0, 0,
            0, 0, 2/ (zNear - zFar), 0,
            0, 0, 0, 1;

    Eigen::Matrix4f x;
    x << 1, 0, 0, -((r + l) / 2),
        0, 1, 0, -((t + b) / 2),
        0, 0, 1, -((zNear + zFar)),
        0, 0, 0, 1;
    
    ortho = ortho * x;

    Eigen::Matrix4f persp;
    persp << zNear, 0, 0, 0,
            0, zNear, 0, 0,
            0, 0, zNear, -1 * zNear * zFar,
            0, 0, 1, 0;

    projection = ortho * persp;

    return projection;
}
```