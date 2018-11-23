ADD_EXECUTABLE(main main.cpp)


# opencv 
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# pcl 
find_package( PCL REQUIRED COMPONENT common io )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )


list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
add_executable( generate_pointcloud generatePointCloud.cpp )
target_link_libraries( generate_pointcloud ${OpenCV_LIBS} ${PCL_LIBRARIES} )

ADD_LIBRARY( slambase slamBase.cpp )
TARGET_LINK_LIBRARIES( slambase
    ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} )
    
ADD_EXECUTABLE( detectFeatures detectFeatures.cpp )
TARGET_LINK_LIBRARIES( detectFeatures 
    slambase
    ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} )