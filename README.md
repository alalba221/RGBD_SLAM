# Proposal
## Visual Odometry based on RGB-D camera
## Yue Zhang
## Summary
  * ### Description 
    In this project I propose a mapping system that generates 3D maps using the actual environment and objects as input, which can take
    advantage of the dense color and depth images provided by RGB-D cameras. By tracking feature points and their relation in space, and
    inferring the camera position, a 3D map can be generated.
    
    
  * ### Importance 
    – Why is this problem important/interesting? 
    
    This approach can work when we don't have the freedom of placing beacons at known locations, for example most mobile Augmented
    Reality usually doesn’t have the luxury of known beacons. For other applications, GPS might be good enough, neither is GPS accurate
    enough – especially indoors. This approach does not require any further sensors or odometry. With the availability of low-cost and
    light-weight RGB-D sensors such as the Microsoft Kinect, it is an attractive alternative for 3D map generating and Object
    localization. 
  * ### Proposal 
    - Extract visual key points from the color images and use the corresponding 
    depth images to localize them in 3D.
    - Estimate the most probability transformations between associated key points using optimization algorithm (RANSAC, Least Square) and 
    optimize the pose graph using non-linear optimization.
    
      [ What is Fundamental and Essential Matrix ](https://www.youtube.com/watch?v=6oMC_3iyeIM&list=PLgnQpQtFTOGRsi5vzy9PiQpNWHjq-bKN1&index=33)
      
      [ What is RANSAC ](https://www.youtube.com/watch?v=oT9c_LlFBqs&index=40&list=PLgnQpQtFTOGRsi5vzy9PiQpNWHjq-bKN1/)
    - Generate a volumetric 3D map of the environment that can 
    be used for robot localization, navigation, and path planning.
  * ### Relationship to Computer Animation (optional) 
    – If it is not obvious how
    your topic relates to computer animation, you should explain it here. 
    
    For Augmented Reality Games, the device has to know its 3D position in the world. This project provides a method of calculating this
    through the spatial relationship between itself and multiple key points. This is an important aspect to implement computer
    animation, virtual reality and augmented reality.
## Goals 
  – You should give a list of final goals, specifying what you hope to 
  accomplish by the end of the semester. Your goals should be as specific as 
  possible
  - Build a map of an `unknown space`. 
  - Locate the `uncontrolled device` within that space.
  - Real-time.
  - Drift-free. 

## Work Breakdown 


- [X] Build programming enviroment on Ubuntu 16.04 
    - [X] OpenCV 
    - [x] PCL: 
      `sudo apt-get install libpcl-dev libpcl-tools`
      Head files of plc will be installed in `/usr/include/pcl-1.7` and Lib files are in `/usr/lib`
      
- [X] Implement a conversion program from 2D images to 3D point clouds
- [ ] Extract and match the features in one frame with another, and use these features to estimate the motion of the camera by Ransac algorithm.
- [ ] Joint combination of point cloud. 
- [ ] Add ability to cope with a video stream.  
- [ ] Implement the graph optimization backend to reduces the drift in the trajectory estimate using G2O library.   ( __2 Weeks__ )
- [ ] Loop closure.   ( __2 Weeks__ )
- [ ] Map Representation
