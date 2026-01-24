```  render c++
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(-1, 5, 10);
    int m = 0;
    for (uint32_t j = 0; j < scene.height; ++j) {
        for (uint32_t i = 0; i < scene.width; ++i) {
            // generate primary ray direction
            float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                      imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;
            
            Vector3f dir = Vector3f(x, y, -1);
            dir = normalize(dir);
            framebuffer[m++] = scene.castRay(Ray(eye_pos, dir), 0);
        }
        UpdateProgress(j / (float)scene.height);
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].x));
        color[1] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].y));
        color[2] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].z));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
```

``` c++
inline Intersection Triangle::getIntersection(Ray ray)
{
    Intersection inter;

    if (dotProduct(ray.direction, normal) > 0)
        return inter;
    double u, v, t_tmp = 0;
    Vector3f pvec = crossProduct(ray.direction, e2);
    double det = dotProduct(e1, pvec);
    if (fabs(det) < EPSILON)
        return inter;

    double det_inv = 1. / det;
    Vector3f tvec = ray.origin - v0;
    u = dotProduct(tvec, pvec) * det_inv;
    if (u < 0 || u > 1)
        return inter;
    Vector3f qvec = crossProduct(tvec, e1);
    v = dotProduct(ray.direction, qvec) * det_inv;
    if (v < 0 || u + v > 1)
        return inter;
    t_tmp = dotProduct(e2, qvec) * det_inv;

    inter.happened = true;
    inter.coords = ray(t_tmp);
    inter.normal = normal;
    inter.distance = t_tmp;
    inter.obj = this;
    inter.m = m;

    return inter;
}
``` 

``` c++
inline bool Bounds3::IntersectP(const Ray& ray, const Vector3f& invDir,
                                const std::array<int, 3>& dirIsNeg) const
{
    Vector3f t_min = (pMin - ray.origin) * invDir;
    Vector3f t_max = (pMax - ray.origin) * invDir;

    float enter_x = dirIsNeg[0] ? t_min.x : t_max.x;
    float enter_y = dirIsNeg[1] ? t_min.y : t_max.y;
    float enter_z = dirIsNeg[2] ? t_min.z : t_max.z;
    float exit_x = dirIsNeg[0] ? t_max.x : t_min.x;
    float exit_y = dirIsNeg[1] ? t_max.y : t_min.y;
    float exit_z = dirIsNeg[2] ? t_max.z : t_min.z;
    
    float t_enter = fmax(enter_x, fmax(enter_y, enter_z));
    float t_exit = fmin(exit_x, fmin(exit_y, exit_z));

    return t_enter < t_exit && t_exit >= 0;
}
```


``` c++
Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    Vector3f invDir = ray.direction_inv;
    Vector3f rayDir = ray.direction;
    std::array<int, 3> dirIsNeg = {
        rayDir.x > 0 ? 1 : 0,
        rayDir.y > 0 ? 1 : 0,
        rayDir.z > 0 ? 1 : 0
    };

    if(!node->bounds.IntersectP(ray, invDir, dirIsNeg))
    {
        return {};
    }

    if(node->object)
    {
        return node->object->getIntersection(ray);
    }

    Intersection hit1 = getIntersection(node->left, ray);
    Intersection hit2 = getIntersection(node->right, ray);
    
    return hit1.distance < hit2.distance ? hit1 : hit2;
}
```