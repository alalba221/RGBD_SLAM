# pragma once

#include<fstream>
#include<vector>
using namespace std;

/// Opencv
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

/// PCL
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>

/// Define our own type
// Type of the points in cloud
typedef pcl::PointXYZRGBA PointT;
//Type of our own cloud
typedef pcl::PointCloud<PointT> PointCloud;


/// Structure for the inner parameter of camera

struct CAMERA_INTRINSTIC_PARAMETERS{
	double cx,cy,fx,fy,scale;
};


/// Function interface
// Convert rgb image to point-cloud
PointCloud::Ptr image2PointCloud(cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSTIC_PARAMETERS& camera);

// Convert 2d point back to 3d point
//input: Point3f (u,v,d)
cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA_INTRINSTIC_PARAMETERS& camera );
