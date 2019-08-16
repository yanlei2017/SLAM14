cmake_minimum_required(VERSION 3.0.0)
project(Slam14 VERSION 0.1.0)
set(CMAKE_CXX_STANDARD 11 )

include_directories("/usr/include/eigen3")

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
message("Sophus_INCLUDE_DIRS :" ${Sophus_INCLUDE_DIRS})
message("Sophus_LBRARIES :" ${Sophus_LIBRARIES})

set(OpenCV_DIR /usr/local/opencv347/share/OpenCV)#comment to using opencv 2.x
find_package(OpenCV 3.0 REQUIRED)#using opencv 3.x,comment to using opencv 2.x
#find_package(OpenCV REQUIRED)#uncomment to using opencv 2.x
include_directories(${OpenCV_INCLUDE_DIRS})
message("OpenCV_DIR : " ${OpenCV_DIR})
message("OpenCV_INCLUDE_DIRS : " ${OpenCV_INCLUDE_DIRS})
#message("OpenCV_LIBS : " ${OpenCV_LIBS} )

find_package(PCL REQUIRED COMPONENT common io)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(g2o REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

message("CMAKE_ROOT: " ${CMAKE_ROOT})
message("CMAKE_HOME_DIRECTORY: " ${CMAKE_HOME_DIRECTORY})
message("CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})
message("CMAKE_PREFIX_PATH: " ${CMAKE_PREFIX_PATH})

add_subdirectory(CH2_CmakePractice CH2_CmakePractice/bin)
add_subdirectory(CH3_Rotation CH3_Rotation/bin)
add_subdirectory(CH4_SE3&SO3 CH4_SE3&SO3/bin)
add_subdirectory(CH5_CameraAndImage CH5_CameraAndImage/bin)
add_subdirectory(CH6_NonlinearOptimization CH6_NonlinearOptimization/bin)
add_subdirectory(CH7_VisualOdometer_1 CH7_VisualOdometer_1/bin)

