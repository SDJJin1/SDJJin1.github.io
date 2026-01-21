照题目模拟即可
``` c++
#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

int main(){
    float arc1 = 45.0 / 180.0 * acos(-1);
    Eigen::Vector3f ans(2.0, 1.0, 1.0);
    Eigen::Matrix3f Rot;
    Rot << std::cos(arc1), std::sin(-arc1), 0,
            std::sin(arc1), std::cos(arc1), 0,
            0, 0, 1;
    ans = Rot * ans;
    Eigen::Matrix3f Trans;
    Trans << 1.0, 0.0, 1.0,
            0.0, 1.0, 2.0,
            0.0, 0.0, 1.0;
    ans = Trans * ans;
    std::cout << ans << std::endl;

    return 0;
}
```