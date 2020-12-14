#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <boost/random.hpp>

using namespace std;
#define PI 3.1415926535

int main()
{
    double r=50; // 圆圈起始半径
    const double r_const = r;
    const double boundary = 0.5 * r; // 在这之外的点剔除
    cout<<"boundary: "<<boundary<<endl;
    const double loop_diff =0.5; // 两个点之间相邻的度数
    const double r_diff = 0.8; // 圆圈半径缩小长度

    const double height = 25; // 煤堆高度
    const int num_circle = 360 * 1/loop_diff; // 每一圈点的个数
    const double h_diff = 0.5; // 每层煤堆之间的高度
    const int num_layers = height/h_diff + 1; // 煤堆层数
    const int num_coalpile = 3; // 煤堆个数
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

    double z = 0;
    while(z<height)
    {
        cout<<"z: "<<z<<",r: "<<r<<endl;

        for(double loop=0;loop<360;loop=loop+loop_diff)
        {
            //point_num=z*360 + loop;
            float hudu = loop * PI / 180;
//            if(hudu>2*PI/3 && hudu<4*PI/3)
//                continue;
            double x = r*cos(hudu) + static_cast<float> (var_nor());
            if(x < -boundary)
            {
                //cout<<"x: "<<x<<endl;
                continue;
            }
            cloud.points[num_points].x = x;
            cloud.points[num_points].y = r*sin(hudu) + static_cast<float> (var_nor());
            cloud.points[num_points].z = z + static_cast<float> (var_nor());
            num_points++;

            // 第二个圆堆
            cloud.points[num_points].x = r*cos(hudu) + static_cast<float> (var_nor());
            cloud.points[num_points].y = r*sin(hudu) + (1.6*r_const) + static_cast<float> (var_nor());
            cloud.points[num_points].z = (z) + static_cast<float> (var_nor());
            num_points++;

            // 第三个
            cloud.points[num_points].x = r*cos(hudu)+ static_cast<float> (var_nor());
            cloud.points[num_points].y = r*sin(hudu) - (1.6*r_const)  + static_cast<float> (var_nor());
            cloud.points[num_points].z = (z) + static_cast<float> (var_nor());
            num_points++;
        }
        if (r > r_diff)
            r=r-r_diff;
        z = z+h_diff;
    }
    cout<<"num_points: "<<num_points<<endl;
    cloud.width = num_points;
    cloud.height = 1;
    cloud.points.resize(cloud.width * cloud.height);
    cout<<"cloud points: "<<cloud.points.size()<<endl;

    pcl::io::savePCDFile("cloud.pcd", cloud);
    return 0;
}
