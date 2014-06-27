#include <iostream>
#include <cstdlib>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ctime>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "cvaux.h"

using namespace pcl;
using namespace cv;

namespace render
{

    #define PT PointXYZRGB
    Mat makeImagefromSize(PointCloud<PT>::Ptr scloud, const int width, const int height)
    {
        PointCloud cloud;
        pcl::fromROSMsg(cloud_transformed, cloud);
        
        float resolution = 0.02f;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
        octree.setInputCloud(cloud.makeShared());
        octree.addPointsFromInputCloud();
        
        if(!((pose->pose.position.x == 0) && (pose->pose.position.y) == 0 && (pose->pose.position.z == 0))) {
                ROS_WARN("Expecting point to be the origin of its coordinate frame, but it isn't");
                //TODO broadcast the frame corresponding to this pose?
        }
        Eigen::Vector3f    origin(pose->pose.position.x,        pose->pose.position.y, pose->pose.position.z);
        Eigen::Vector3f direction(pose->pose.position.x + RAND_MAX, pose->pose.position.y + 0.0f, pose->pose.position.z + 0.0f);
        //AlignedPointTVector voxelCenterList;
        std::vector<int> k_indices;
        
        //int nPoints = octree.getIntersectedVoxelCenters(origin, direction, voxelCenterList);
        int nPoints = octree.getIntersectedVoxelIndices(origin, direction, k_indices);
        ROS_INFO("vector intersected %d voxels", nPoints);
	}

}