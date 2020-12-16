#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <boost/random.hpp>

using namespace std;
#define PI 3.1415926535
const int num_coalpile = 2; // 煤堆个数

int main()
{
    double R[]={50, 50, 50}; // 圆圈起始半径
    double HUDU[][2]={ {112, 248}, {65, 188} }; // 圆圈起始弧度
    double t_center[][2] = {
        {-1.5, 0},
        {-0.75, 1.29}
        //{0.75, 1.29}
    }; // 每个煤堆中心偏移的距离
    double r_const;
    const double loop_diff =0.5; // 两个点之间相邻的度数
    const double r_diff = 0.75; // 圆上下层的差
    double init_hudu = 0.5; // 初始弧度数
    const double delta_hudu = 0.06;

    const double height[] = {22, 22, 22}; // 煤堆高度
    const int num_circle = 360 * 1/loop_diff; // 每一圈点的个数
    const double h_diff = 0.5; // 每层煤堆之间的高度
    const int num_layers = height[0]/h_diff + 1; // 煤堆层数
    int num_points = 0; // 统计点云中的点数

    // Fill in the cloud data
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = num_circle*num_layers*num_coalpile*3;
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
        double hudu1 = HUDU[i][0], hudu2 = HUDU[i][1];
        init_hudu = 0.5;
        for(double h=0; h<height[i]; h = h+h_diff)
        {
            cout<<"h: "<<h<<",r: "<<r<<endl;

            for(double loop=0;loop<360;loop=loop+loop_diff)
            {
                if(loop > hudu1 && loop < hudu2 && h < 16)
                    continue;
                float hudu = loop * PI / 180;
                double x = r*cos(hudu) + (r_const * t_center[i][0]) + static_cast<float> (var_nor());
                double y = r*sin(hudu) + (r_const * t_center[i][1]) + static_cast<float> (var_nor());
                double z = h + static_cast<float> (var_nor());
                cloud.points[num_points].x = x;
                cloud.points[num_points].y = y;
                cloud.points[num_points].z = z;
                num_points++;

                cloud.points[num_points].x = -x;
                cloud.points[num_points].y = y;
                cloud.points[num_points].z = z;
                num_points++;
            }


            if (r > r_diff)
                r=r-r_diff;
            hudu1 += init_hudu; // 上层的圆圈画的大一些
            hudu2 -= init_hudu;
            init_hudu += delta_hudu;

            // 把最上层的圆填满
            if(h+h_diff >= height[i])
            {
                cout<<"到达最上层圆, r: "<<r<<endl;
                for(double r_above = r; r_above>0;r_above=r_above-r_diff)
                {
                    for(double loop=0;loop<360;loop=loop+loop_diff)
                    {
                        float hudu = loop * PI / 180;
                        double x = r_above*cos(hudu) + (r_const * t_center[i][0]) + static_cast<float> (var_nor());
                        double y = r_above*sin(hudu) + (r_const * t_center[i][1]) + static_cast<float> (var_nor());
                        double z = h + static_cast<float> (var_nor());
                        cloud.points[num_points].x = x;
                        cloud.points[num_points].y = y;
                        cloud.points[num_points].z = z;
                        num_points++;

                        cloud.points[num_points].x = -x;
                        cloud.points[num_points].y = y;
                        cloud.points[num_points].z = z;
                        num_points++;
                    }
                }
            }

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
