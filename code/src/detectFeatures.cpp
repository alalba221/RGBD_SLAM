#include <iostream>
#include "slamBase.h"

using namespace std;
using namespace cv;
//OpenCV Feature detect module
#include <opencv2/features2d/features2d.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include<opencv2/highgui/highgui.hpp>

int main(int argc,char**argv){
  
  // Read rgb and depth images
  Mat rgb1 = imread("../data/rgb1.png");
  Mat rgb2 = imread("../data/rgb2.png");
  Mat depth1 = imread("../data/depth1.png",-1);
  Mat depth2 = imread("../data/depth2.png",-1);
  
  //Declare the FeatureDetector and DescriptorExtractor and Matchers
  Ptr<FeatureDetector> detector = ORB::create();
  Ptr<DescriptorExtractor> descriptor = ORB::create();
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
  
  //KeyPoints
  vector<KeyPoint> kp1,kp2;
  
  /// 1. Detect the positions of Oriented Fast Corner KeyPoints
  detector->detect(rgb1,kp1);
  detector->detect(rgb2,kp2);
  
  cout<<"Key points of two images: "<<kp1.size()<<", "<<kp2.size()<<endl;
  
  //visualized,show key points
  
  Mat imgShow;
  drawKeypoints(rgb1,kp1,imgShow,Scalar::all(-1), DrawMatchesFlags::DEFAULT);
  imshow("keypoints",imgShow);
  waitKey(0);
  
  /// 2. Calculate the BRIEF descriptor, according to the keypoint position
  Mat desp1, desp2;
  descriptor->compute(rgb1, kp1, desp1);
  descriptor->compute(rgb2, kp2, desp2);
  
  /// 3. Match the BRIEF descriptor of two images, using HAMMING distance
  vector<DMatch> matches;
  matcher->match(desp1, desp2, matches);
  cout<<"Find total "<<matches.size()<<" matches."<<endl;
  
  // Show feature matching result
  Mat imgMatches;
  drawMatches(rgb1,kp1, rgb2, kp2, matches, imgMatches);
  imshow("matches", imgMatches);
  waitKey(0);
  
  /// 4. Sift through the matching points
  vector<DMatch> goodMatches;
  double minDis = 9999;
  double maxDis = 0;
  
  for(size_t i=0;i<matches.size();i++ ){
    if(matches[i].distance < minDis){
      minDis = matches[i].distance;
    }
    if(matches[i].distance > maxDis){
      maxDis = matches[i].distance;
    }
  }
  
  cout<<"min Distance = "<<minDis<<", max Distance = "<<maxDis<<endl;
  
  for(size_t i=0;i<matches.size();i++){
    
    if(matches[i].distance <= max(2*minDis,30.0)){
      goodMatches.push_back(matches[i]);
    }
  }
  
  // show good match points
  drawMatches(rgb1,kp1, rgb2, kp2, goodMatches, imgMatches);
  imshow("good matches", imgMatches);
  waitKey(0);
  
/*****************************PNP***********************************************************/ 
  /// Caculate the Rotation and Transaction Matrix for camera
  /// Call the PnP function in OpenCV
  
  // 3D point for the first frame
  vector<Point3f> pts_obj;
  // 2D point for the second frame
  vector<Point2f> pts_img;
  
  // intrisic parameter of the camera
  
   CAMERA_INTRINSTIC_PARAMETERS C ;
   C.cx = 325,5;
   C.cy = 253.5;
   C.fx = 518.0;
   C.fy = 519.0;
   C.scale = 1000.0;
  
   for(size_t i=0; i<goodMatches.size();i++){
    Point2f p = kp1[ goodMatches[i].queryIdx].pt;
    
    ushort d = depth1.ptr<ushort>( int(p.y) )[ int(p.x) ];
    
    if(d==0)
      continue;
    
    pts_img.push_back( Point2f( kp2[goodMatches[i].trainIdx].pt ) );
    
    cv::Point3f pt ( p.x, p.y, d );
    cv::Point3f pd = point2dTo3d( pt, C );
    pts_obj.push_back( pd );
  }
  
  double camera_matrix_data[3][3] = {
    {C.fx, 0, C.cx},
    {0, C.fy, C.cy},
    {0, 0, 1}
  };
  // Build intrisic matrix
   Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data );
   Mat rvec, tvec, inliers;
  
   //solvePnP ( pts_obj, pts_img, cameraMatrix, Mat(), rvec, tvec,false);
  cv::solvePnPRansac( pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 0.99, inliers );
   cout<<"inliers: "<<inliers.rows<<endl;
   cout<<"R="<<rvec<<endl;
   cout<<"t="<<tvec<<endl;
   vector< cv::DMatch > matchesShow;
    for (size_t i=0; i<inliers.rows; i++)
    {
        matchesShow.push_back( goodMatches[inliers.ptr<int>(i)[0]] );    
    }
    cv::drawMatches( rgb1, kp1, rgb2, kp2, matchesShow, imgMatches );
    cv::imshow( "inlier matches", imgMatches );
    //cv::imwrite( "./data/inliers.png", imgMatches );
    cv::waitKey( 0 );
}






























