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

	PC_Translate(PointCloud<PointXYZ>Ptr source_cloud, PointCloud<PointXYZ>Ptr dest_cloud, Vector3f translation)
	{
		Matrix4f transformation_matrix = Matrix4fIdentity();

		for(int i = 0; i < 3; i++)
			transformation_matrix(i, 3) = translation(i, 0);

		transformPointCloud (*source_cloud, *dest_cloud, transformation_matrix);
	}

	PC_Yaw(PointCloud<PointXYZ>Ptr source_cloud, PointCloud<PointXYZ>Ptr dest_cloud, double theta_rad)
	{
		Matrix4f transformation_matrix = Matrix4fIdentity();

		//Defining a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
		transformation_matrix (0,0) = cos(theta);
		transformation_matrix (0,1) = -sin(theta);
		transformation_matrix (1,0) = sin(theta);
		transformation_matrix (1,1) = cos(theta)

		transformPointCloud (*source_cloud, *dest_cloud, transformation_matrix);

	}

		PC_Pitch(PointCloud<PointXYZ>Ptr source_cloud, PointCloud<PointXYZ>Ptr dest_cloud, double theta_rad)
	{
		Matrix4f transformation_matrix = Matrix4fIdentity();

		//Defining a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
		transformation_matrix (0,0) = cos(theta);
		transformation_matrix (0,2) = sin(theta);
		transformation_matrix (2,0) = -sin(theta);
		transformation_matrix (2,2) = cos(theta);

		transformPointCloud (*source_cloud, *dest_cloud, transformation_matrix);

	}

		PC_Roll(PointCloud<PointXYZ>Ptr source_cloud, PointCloud<PointXYZ>Ptr dest_cloud, double theta_rad)
	{
		Matrix4f transformation_matrix = Matrix4fIdentity();

		//Defining a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
		transformation_matrix (1,1) = cos(theta);
		transformation_matrix (1,2) = -sin(theta);
		transformation_matrix (2,1) = sin(theta);
		transformation_matrix (2,2) = cos(theta);

		transformPointCloud (*source_cloud, *dest_cloud, transformation_matrix);
	}

		PC_Roll_Pitch_Yaw_Translate(PointCloud<PointXYZ>Ptr source_cloud, PointCloud<PointXYZ>Ptr dest_cloud, Vector3f rotations, Vector3f translations)
	{
		Matrix4f transformation_matrix = Matrix4fIdentity();
		double a = rotations(0);
		double b = rotations(1);
		double c = rotations(2);

		transformation_matrix (0,0) = cos(a)*cos(b);
		transformation_matrix (0,1) = cos(a)*sin(b)*sin(c)-sin(a)*cos(c);
		transformation_matrix (0,2) = cos(a)*sin(b)*cos(c)+sin(a)*sin(c);
		transformation_matrix (1,0) = sin(a)*cos(b);
		transformation_matrix (1,1) = sin(a)*sin(b)*sin(c)+cos(a)*cos(c);
		transformation_matrix (1,2) = sin(a)*sin(b)*cos(c)-cos(a)*sin(c);
		transformation_matrix (2,0) = -sin(b);
		transformation_matrix (2,1) = cos(b)*sin(c);
		transformation_matrix (2,2) = cos(b)*cos(c);

		transformPointCloud (*source_cloud, *dest_cloud, transformation_matrix);

		PC_Transform::PC_Translate(dest_cloud, dest_cloud, translations);

		//transformPointCloud (*source_cloud, *dest_cloud, transformation_matrix);

	}




}