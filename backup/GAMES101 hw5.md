``` c++
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = std::tan(deg2rad(scene.fov * 0.5f));
    float imageAspectRatio = scene.width / (float)scene.height;

    // Use this variable as the eye position to start your rays.
    Vector3f eye_pos(0);
    int m = 0;
    for (int j = 0; j < scene.height; ++j)
    {
        for (int i = 0; i < scene.width; ++i)
        {
            // generate primary ray direction
            float x, world_scene_width;
            float y, world_scene_height;
            world_scene_width = 1 * scale * 2 * imageAspectRatio;
            world_scene_height = 1 * scale * 2;
            x = (i + 0.5) / (scene.width - 1);
            x = x * 2 - 1;
            x = x * world_scene_width / 2;
            y = (-2 * (j + 0.5) / (scene.height - 1) + 1) * world_scene_height / 2;       

            Vector3f dir = Vector3f(x, y, -1); // Don't forget to normalize this direction!
            framebuffer[m++] = castRay(eye_pos, dir, scene, 0);
        }
        UpdateProgress(j / (float)scene.height);
    }

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (char)(255 * clamp(0, 1, framebuffer[i].x));
        color[1] = (char)(255 * clamp(0, 1, framebuffer[i].y));
        color[2] = (char)(255 * clamp(0, 1, framebuffer[i].z));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
```

<img width="1247" height="866" alt="Image" src="https://github.com/user-attachments/assets/2520c58a-5c2b-4027-a353-ebf229cc71c0" />

``` c++
bool rayTriangleIntersect(const Vector3f& v0, const Vector3f& v1, const Vector3f& v2, const Vector3f& orig,
                          const Vector3f& dir, float& tnear, float& u, float& v)
{
    Vector3f E1 = v1 - v0,
    E2 = v2 - v0,
    S = orig - v0,
    S1 = crossProduct(dir, E2),
    S2 = crossProduct(S, E1);

    float S1E1 = dotProduct(S1, E1);
    tnear = dotProduct(S2, E2) / S1E1;
    u = dotProduct(S1, S) / S1E1;
    v = dotProduct(S2, dir) / S1E1;
    if(tnear < 0) return false;
    if((1 - u - v) > 0 && u > 0 && v > 0) return true;
    return false;
}
```