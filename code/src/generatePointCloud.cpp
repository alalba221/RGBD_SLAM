
#include <iostream>
#include <string>
using namespace std;
//using namespace cv;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//Define the format of Points Cloud, here we use XYZRGB
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud; 

// cammera inner parameter
const double camera_factor = 1000;
const double camera_cx = 325.5;
const double camera_cy = 253.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;

 
int main( int argc, char** argv )
{
    // Read ./data/rgb.png and ./data/depth.png，then convert them to Point Clouds

    
    cv::Mat rgb, depth;
    
    rgb = cv::imread( "./data/rgb.png" );
    // rgb image's format is 8UC3
    // depth's format is 16UC1，set flags -1
    depth = cv::imread( "./data/depth.png", -1 );


    // build a new Point Cloud
    
    //  An enhanced relative of scoped_ptr with reference counted copy semantics.
    //  The object pointed to is deleted when the last shared_ptr pointing to it
    //  is destroyed or reset.
//
    PointCloud::Ptr cloud ( new PointCloud );
    // Traverse Depth Image
    for (int m = 0; m < depth.rows; m++)
        for (int n=0; n < depth.cols; n++)
        {
            // Get the depth value at [m][n]
            unsigned short d = depth.ptr<unsigned short>(m)[n];
            // no depth value at this position == no point here, skip over
            if (d == 0)
                continue;
            // add a Point to the cloud
            PointT p;

            // Get the spcace location of this point
            p.z = double(d) / camera_factor;
            p.x = (n - camera_cx) * p.z / camera_fx;
            p.y = (m - camera_cy) * p.z / camera_fy;
            
            
            // get the color for corresponding rgb image
            p.b = rgb.ptr<unsigned char>(m)[n*3];
            p.g = rgb.ptr<unsigned char>(m)[n*3+1];
            p.r = rgb.ptr<unsigned char>(m)[n*3+2];

            // add p into the cloud
            cloud->points.push_back( p );
        }
    // set and save tte cloud
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cout<<"point cloud size = "<<cloud->points.size()<<endl;
    cloud->is_dense = false;
    pcl::io::savePCDFile( "./pointcloud.pcd", *cloud );
    // clear data
    cloud->points.clear();
    cout<<"Point cloud saved."<<endl;
    return 0;
}