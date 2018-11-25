# pragma once

#include <fstream>
#include <vector>
#include <map>
using namespace std;

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

/// Opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>


/// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
/// Define our own type
// Type of the points in cloud
typedef pcl::PointXYZRGBA PointT;
//Type of our own cloud
typedef pcl::PointCloud<PointT> PointCloud;


/// Structure for the inner parameter of camera

struct CAMERA_INTRINSTIC_PARAMETERS{
	double cx,cy,fx,fy,scale;
};

/// Frame structure

struct FRAME
{
	cv::Mat rgb, depth; //
	cv::Mat desp;		// descroptor for each frame
	vector<cv::KeyPoint> kp; // key points
};

/// PnP
struct  RESULT_OF_PNP
{
	cv::Mat rvec,tvec;
	int inliers;
};

/// Function interface
// Convert rgb image to point-cloud
PointCloud::Ptr image2PointCloud(cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSTIC_PARAMETERS& camera);

// Convert 2d point back to 3d point
//input: Point3f (u,v,d)
cv::Point3f point2dTo3d( cv::Point3f& point, CAMERA_INTRINSTIC_PARAMETERS& camera );

// Extract keypoints and descriptor
void computeKeyPointsAndDesp(FRAME& frame, string detector, string descriptor);

// Calculate the cam's motion between 2 frames
RESULT_OF_PNP estimateMotion(FRAME& frame1,FRAME& frame2, CAMERA_INTRINSTIC_PARAMETERS& camera);

// cvMat2Eigen
Eigen::Isometry3d cvMat2Eigen( cv::Mat& rvec, cv::Mat& tvec );

// joinPointCloud 
PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSTIC_PARAMETERS& camera ) ;

class ParameterReader
{
public:
    ParameterReader( string filename="../parameters.txt" )
    {
        ifstream fin( filename.c_str() );
        if (!fin)
        {
            cerr<<"parameter file does not exist."<<endl;
            return;
        }
        while(!fin.eof())
        {
            string str;
            getline( fin, str );
            if (str[0] == '#')
            {
                // commnet started with "#"
                continue;
            }

            int pos = str.find("=");
            if (pos == -1)
                continue;
            string key = str.substr( 0, pos );
            string value = str.substr( pos+1, str.length() );
            data[key] = value;

            if ( !fin.good() )
                break;
        }
    }
    string getData( string key )
    {
        map<string, string>::iterator iter = data.find(key);
        if (iter == data.end())
        {
            cerr<<"Parameter name "<<key<<" not found!"<<endl;
			return string("NOT_FOUND");
        }
        //cout<<data["detector"]<<endl;
        return iter->second;
    }
public:
    map<string, string> data;
};

inline static CAMERA_INTRINSTIC_PARAMETERS getDefaultCamera()
{
    ParameterReader pd;
    CAMERA_INTRINSTIC_PARAMETERS camera;
    camera.fx = atof( pd.getData( "camera.fx" ).c_str());
    camera.fy = atof( pd.getData( "camera.fy" ).c_str());
    camera.cx = atof( pd.getData( "camera.cx" ).c_str());
    camera.cy = atof( pd.getData( "camera.cy" ).c_str());
    camera.scale = atof( pd.getData( "camera.scale" ).c_str() );
    return camera;
}