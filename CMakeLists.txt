cmake_minimum_required(VERSION 2.8.3)
project(rabbit)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  eigen_conversions
  pcl_ros
  pcl_conversions
  tf
)

find_package(PCL REQUIRED QUIET)
add_definitions(${PCL_DEFINITIONS})
find_package(OpenCV REQUIRED QUIET)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp  pcl_conversions
)

include_directories(
	include
  3rdparty
	${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)


file(GLOB_RECURSE SRC
"src/*.cpp"
"src/*.cc"
"3rdparty/*.cpp"
)

link_directories(
	${OpenCV_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_LIBRARY_DIRS}
)

add_library(librabbit SHARED ${SRC})

add_executable(IOTest app/UnitTest/IOTest.cpp)
add_dependencies(IOTest ${catkin_EXPORTED_TARGETS} )
target_link_libraries(IOTest librabbit ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})

add_executable(ReadPCD app/ReadPCD.cpp)
add_dependencies(ReadPCD ${catkin_EXPORTED_TARGETS} )
target_link_libraries(ReadPCD librabbit ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})

add_executable(PCD2PLY app/PCD2PLY.cpp)
add_dependencies(PCD2PLY ${catkin_EXPORTED_TARGETS} )
target_link_libraries(PCD2PLY librabbit ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})

add_executable(FrameTest app/UnitTest/FrameTest.cpp)
add_dependencies(FrameTest ${catkin_EXPORTED_TARGETS} )
target_link_libraries(FrameTest librabbit ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})

add_executable(OdometryTest app/UnitTest/OdometryTest.cpp)
add_dependencies(OdometryTest ${catkin_EXPORTED_TARGETS} )
target_link_libraries(OdometryTest librabbit ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})

add_executable(IMUTest app/UnitTest/IMUTest.cpp)
add_dependencies(IMUTest ${catkin_EXPORTED_TARGETS} )
target_link_libraries(IMUTest librabbit ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})

add_executable(RabbitRun app/RUN/Run.cpp)
add_dependencies(RabbitRun ${catkin_EXPORTED_TARGETS} )
target_link_libraries(RabbitRun librabbit ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})