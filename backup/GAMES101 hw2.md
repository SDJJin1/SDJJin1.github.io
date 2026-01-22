首先要判断点在三角形内

<img width="762" height="480" alt="Image" src="https://github.com/user-attachments/assets/9396a6dd-1e88-4825-b61a-06609b5e890a" />

``` c++
static bool sameSide(Vector3f p, Vector3f a, Vector3f b, Vector3f c)
{
    Vector3f ab = b - a;
    Vector3f ac = c - a;
    Vector3f ap = p - a;
    return (ab.cross(ac)).dot(ab.cross(ap)) >= 0;
}


static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    Vector3f p = {x, y, 1};

    return sameSide(p, _v[0], _v[1], _v[2]) &&
           sameSide(p, _v[1], _v[2], _v[0]) &&
           sameSide(p, _v[2], _v[0], _v[1]);

}
```

<img width="780" height="478" alt="Image" src="https://github.com/user-attachments/assets/902607b6-b3cb-4ef1-9810-aad9c87a159c" />

``` c++
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
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
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                
                int index = get_index(i, j);
                if(z_interpolated < depth_buf[index])
                {
                    depth_buf[index] = z_interpolated;
                    set_pixel(Eigen::Vector3f(i, j, z_interpolated), t.getColor());
                }
            }
        }
    }
}
```