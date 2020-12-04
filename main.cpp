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
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Fill in the cloud data
    cloud.width = 360;
    cloud.height = 16;
    cloud.points.resize(cloud.width * cloud.height);

    long point_num;
    double r=10; // 半径
    cout<<"cloud.height: "<<cloud.height<<endl;

    //添加高斯噪声
    boost::mt19937 rng;
    rng.seed(static_cast<unsigned int>(time(0)));
    boost::normal_distribution<> nd(0, 0.5);
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(rng, nd);//产生随机/数

    for(int z=0;z<cloud.height;++z)
    {
        cout<<"r: "<<r<<endl;

        for(int loop=0;loop<360;++loop)
        {
            point_num=z*360 + loop;
            float hudu = loop * PI / 180;
            cloud.points[point_num].x = r*cos(hudu);// + static_cast<float> (var_nor());
            cloud.points[point_num].y = r*sin(hudu);// + static_cast<float> (var_nor());
            cloud.points[point_num].z = 0.5*(z+1.0f);// + static_cast<float> (var_nor());
            //或者为0.3*static_cast<float>(z);目地在于int转换为float

            // 第二个圆堆
            cloud.points[point_num].x = r*cos(hudu);// + static_cast<float> (var_nor());
            cloud.points[point_num].y = r*sin(hudu);// + static_cast<float> (var_nor());
            cloud.points[point_num].z = 0.5*(z+1.0f);// + static_cast<float> (var_nor());
        }
        if (r > 0.6)
            r=r-0.6;
    }
    pcl::io::savePCDFile("cloud.pcd", cloud);
    return 0;
}
