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
    int pile_size = 55; // z轴高度
    cloud.width = 360;
    cloud.height = pile_size*3;
    cloud.points.resize(cloud.width * cloud.height);

    int point_num;
    double r=40; // 半径
    double r_const = r;
    cout<<"cloud.height: "<<cloud.height<<endl;

    //添加高斯噪声
    boost::mt19937 rng;
    rng.seed(static_cast<unsigned int>(time(0)));
    boost::normal_distribution<> nd(0, 0.5);
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(rng, nd);//产生随机/数

    for(int z=0;z<pile_size;++z)
    {
        cout<<"z: "<<z<<",r: "<<r<<endl;

        for(int loop=0;loop<360;++loop)
        {
            point_num=z*360 + loop;
            float hudu = loop * PI / 180;
            cloud.points[point_num].x = r*cos(hudu) + static_cast<float> (var_nor());
            cloud.points[point_num].y = r*sin(hudu) + static_cast<float> (var_nor());
            cloud.points[point_num].z = 0.5*(z+1.0f) + static_cast<float> (var_nor());
            //或者为0.3*static_cast<float>(z);目地在于int转换为float

            // 第二个圆堆
//            if(hudu>2*PI/3 && hudu<4*PI/3) //hudu<3/PI || hudu>5*PI/3
//                continue;
            cloud.points[cloud.width*pile_size + point_num].x = r*cos(hudu) + (r_const+15) + static_cast<float> (var_nor());
            cloud.points[cloud.width*pile_size + point_num].y = r*sin(hudu) + 20 + static_cast<float> (var_nor());
            cloud.points[cloud.width*pile_size + point_num].z = 0.45*(z+3.0f) + static_cast<float> (var_nor());

            // 第三个
            cloud.points[cloud.width*pile_size*2 + point_num].x = r*cos(hudu) + (2*r_const+20); + static_cast<float> (var_nor());
            cloud.points[cloud.width*pile_size*2 + point_num].y = r*sin(hudu) - 25 + static_cast<float> (var_nor());
            cloud.points[cloud.width*pile_size*2 + point_num].z = 0.6*(z+2.0f) + static_cast<float> (var_nor());
        }
        if (r > 0.6)
            r=r-0.7;
    }
    pcl::io::savePCDFile("cloud.pcd", cloud);
    return 0;
}
