cmake_minimum_required(VERSION 2.8.3)
project(pcd_image_save)

add_compile_options(-std=c++11)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)
find_package(catkin
    REQUIRED COMPONENTS
            roscpp
            sensor_msgs
	    std_msgs
            nav_msgs
            geometry_msgs
	    cv_bridge
	    pcl_ros
            visualization_msgs
)
find_package(OpenCV REQUIRED)

#add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS
            include
    CATKIN_DEPENDS
            roscpp
            sensor_msgs
            nav_msgs
            geometry_msgs
            visualization_msgs
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS} ${EIGEN3_LIBRARY_DIR})

add_executable(pcd_image_save src/main.cpp)
add_dependencies(pcd_image_save ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PCL_LIBRARIES} ${EIGEN3_LIBRARY_DIR})
target_link_libraries(pcd_image_save ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBRARY_DIR} ${OpenCV_LIBRARIES})
