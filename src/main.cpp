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

#include "PC_Transformer.h"

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace PC_Transform;
// This is the main function
int main (int argc, char** argv)
{
	string cloud_name;
	Vector3f rotate;
	Vector3f trans;

	ifstream file(argv[1]);

    if ( !file.is_open() )
        return false;

    string str;
    getline( file, str);
    cloud_name = str;

    for(int i = 0; i < 3 && !file.eof(); i++)
    {
    	getline( file, str);
    	trans(i) = atof(str.c_str());
    }

    for(int i = 0; i < 3 && !file.eof(); i++)
    {
    	getline( file, str);
    	rotate(i) = atof(str.c_str());
    }
    file.close();

    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::io::loadPLYFile (cloud_name, *source_cloud) ;

    PC_Roll_Pitch_Yaw_Translate(source_cloud, transformed_cloud, rotate, trans);

	// Visualization
	printf("\nPoint cloud colors :	black	= original point cloud\n			red	= transformed point cloud\n");
	pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 0, 0, 0); // Where 255,255,255 are R,G,B colors
	viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");	// We add the point cloud to the viewer and pass the color handler

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20);
	viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

	viewer.addCoordinateSystem (1.0, 0);
	viewer.setBackgroundColor(0.95, 0.95, 0.95, 0); // Setting background to a dark grey
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
	viewer.setPosition(800, 400); // Setting visualiser window position

	while (!viewer.wasStopped ()) { // Display the visualiser untill 'q' key is pressed
		viewer.spinOnce ();
	}

	return 0;
}

