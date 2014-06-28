#include <iostream>
#include <cstdlib>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ctime>


#include "render.h"

using namespace pcl;

namespace PC_Transform
{
	#define PT PointXYZRGB
    Mat makeImagefromSize(PointCloud<PT>::Ptr cloud, 
                const int width = 500, const int height = 500, float resolution = 0.02f)
    {     
        octree::OctreePointCloudSearch<PT> octree (resolution);
        octree.setInputCloud(cloud);
        octree.defineBoundingBox();
        octree.addPointsFromInputCloud();
        
        Eigen::Vector3f    origin (0, 0.0f, 0.0f);
        Eigen::Vector3f direction (1, 0.0f, 0.0f);
        octree::AlignedPointTVector voxelCenterList;
        std::vector<int> k_indices;
        
        int nPoints = octree.getIntersectedVoxelCenters(origin, direction, voxelCenterList);
        // int nPoints = octree.getIntersectedVoxelIndices(origin, direction, k_indices);
        cout << "vector intersected " << nPoints << " voxels\n";

        for (int i = 0; i < nPoints; i++)
        {
            cout << "Point Found: (" << voxelCenterList[i].x << ", " << voxelCenterList[i].y << ", " << voxelCenterList[i].z << ")\n";
        }
        return NULL;
	}



}