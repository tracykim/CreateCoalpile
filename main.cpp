#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <boost/random.hpp>

using namespace std;
#define PI 3.1415926535
const int num_coalpile = 3; // 煤堆个数

int main()
{
    double R[num_coalpile]={50, 52, 48}; // 圆圈起始半径
    double t_center[][2] = {
        {0, 0},
        //{0.034, 0.259},
        //{0.134, 0.5},
        //{0.293, 0.707},
        //{0.5, 0.866},
        {0, 1.2}, //{0.741, 0.966},
        //{1, 1},
        {0, 3.0}, //{1.5, 0.866},
    }; // 每个煤堆中心偏移的距离
    double r_const;
    double boundary; // 在这之外的点剔除
    const double loop_diff =0.5; // 两个点之间相邻的度数
    const double r_diff = 0.8; // 圆圈半径缩小长度

    const double height[] = {25, 30, 20}; // 煤堆高度
    const int num_circle = 360 * 1/loop_diff; // 每一圈点的个数
    const double h_diff = 0.5; // 每层煤堆之间的高度
    const int num_layers = height[0]/h_diff + 1; // 煤堆层数
    int num_points = 0; // 统计点云中的点数

    // Fill in the cloud data
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = num_circle*num_layers*num_coalpile;
    cloud.height = 1;
    cloud.points.resize(cloud.width * cloud.height);
    cout<<"cloud points: "<<cloud.points.size()<<endl;

    //添加高斯噪声
    boost::mt19937 rng;
    rng.seed(static_cast<unsigned int>(time(0)));
    boost::normal_distribution<> nd(0, 0.1);
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(rng, nd);//产生随机数


    for(int i=0; i<num_coalpile; i++)
    {
        cout<<"num_coalpile: "<<i<<endl;
        double r = R[i];
        r_const = R[i];
        boundary = 0.5 * R[i];
        for(double z=0; z<height[i]; z = z+h_diff)
        {
            cout<<"z: "<<z<<",r: "<<r<<endl;
            // 第一个煤堆
            for(double loop=0;loop<360;loop=loop+loop_diff)
            {
                float hudu = loop * PI / 180;
                double x = r*cos(hudu) + static_cast<float> (var_nor());

                if(x < -boundary)
                {
                    //cout<<"x: "<<x<<endl;
                    continue;
                }
                cloud.points[num_points].x = x + r_const * t_center[i][0];
                cloud.points[num_points].y = r*sin(hudu) + (r_const*t_center[i][1]) + static_cast<float> (var_nor());
                cloud.points[num_points].z = z + static_cast<float> (var_nor());
                num_points++;
            }

            if (r > r_diff)
                r=r-r_diff;
        }
    }
    cout<<"num_points: "<<num_points<<endl;
    cloud.width = num_points;
    cloud.height = 1;
    cloud.points.resize(cloud.width * cloud.height);
    cout<<"cloud points: "<<cloud.points.size()<<endl;

    pcl::io::savePCDFile("cloud.pcd", cloud);
    return 0;
}
