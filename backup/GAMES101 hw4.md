根据课上讲的

<img width="990" height="614" alt="Image" src="https://github.com/user-attachments/assets/86c183bf-26e9-449a-83d8-eab3299f9a12" />

``` c++
cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    if (control_points.size() == 2)
    {
        return (1 - t) * control_points[0] + t * control_points[1];
    }

    std::vector<cv::Point2f> new_control_points;

    for (int i = 0; i < control_points.size() - 1; i++)
    {
        new_control_points.push_back((1 - t) * control_points[i] + t * control_points[i+1]);
    }
    
    return recursive_bezier(new_control_points, t);

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    for (float t = 0; t <= 1; t += 0.001)
    {
        auto point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
    }
}
```

<img width="700" height="700" alt="Image" src="https://github.com/user-attachments/assets/08601d97-9ad4-4bda-9cf6-49450f29be63" />