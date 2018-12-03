ADD_EXECUTABLE(main main.cpp)


# opencv 
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# pcl 
find_package( PCL REQUIRED COMPONENT common io visualization filters )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
add_executable( generate_pointcloud generatePointCloud.cpp )
target_link_libraries( generate_pointcloud ${OpenCV_LIBS} ${PCL_LIBRARIES} )

# g2o
# 因为g2o不是常用库，要添加它的findg2o.cmake文件
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
SET( G2O_ROOT /usr/local/include/g2o )
FIND_PACKAGE( G2O REQUIRED )
# CSparse
FIND_PACKAGE( CSparse REQUIRED )
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )



ADD_LIBRARY( slambase slamBase.cpp )
TARGET_LINK_LIBRARIES( slambase
    ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} )
    
ADD_EXECUTABLE( detectFeatures detectFeatures.cpp )
TARGET_LINK_LIBRARIES( detectFeatures 
    slambase
    ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} )

ADD_EXECUTABLE( joinPointCloud joinPointCloud.cpp)
TARGET_LINK_LIBRARIES( joinPointCloud
    slambase
    ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} )
    
ADD_EXECUTABLE( visualOdometry visualOdometry.cpp)
TARGET_LINK_LIBRARIES( visualOdometry
    slambase
    ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} )
    
ADD_EXECUTABLE( slamEnd slamEnd.cpp )
TARGET_LINK_LIBRARIES( slamEnd
    slambase
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY})
    
ADD_EXECUTABLE( slam slam.cpp )
TARGET_LINK_LIBRARIES( slam
    slambase
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY})