#include <iostream>
#include <cstdlib>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ctime>
#include <cstdlib>

#include "render.h"

using namespace std;
using namespace pcl;
using namespace Eigen;

#define PT PointXYZRGB

int main (int argc, char** argv)
{
    string cloud_name;
    ifstream file(argv[1]);

    if ( !file.is_open() )
        return false;

    string str;

    getline( file, str );
    cloud_name = str;

    // width
    getline( file, str );
    int w = atoi(str.c_str());
    // height
    getline( file, str );
    int h = atoi(str.c_str());
    // resolution
    getline( file, str );
    float r = atof(str.c_str());
    // f (distance from pinhole)
    getline( file, str );
    float f = atof(str.c_str());
    // theta
    getline( file, str );
    float theta = atof(str.c_str());

    file.close();

    PointCloud<PT>::Ptr cloud (new pcl::PointCloud<PT>);

    io::loadPCDFile<PT>(cloud_name, *cloud);

    Eigen::Affine3f tf = Eigen::Affine3f::Identity();

    //tf.translation() << 2.5, 0.0, 0.0;
    tf.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitY()));
    pcl::transformPointCloud (*cloud, *cloud, tf);

    Mat result = render::makeImagefromSize(cloud, w, h, r, f);

    imwrite("img.jpg", result);

    namedWindow("Render");
    imshow("Render", result);
    waitKey();

    visualization::PCLVisualizer v;

    v.addPointCloud(cloud, "cloud");
    v.spin();

    return 0;
}

