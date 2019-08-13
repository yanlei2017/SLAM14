
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
set(CMAKE_CXX_STANDARD 11)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(g2o REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

include_directories("/usr/include/eigen3")

add_executable(ceresDemo ceresDemo.cpp)
add_executable(g2oDemo g2oDemo.cpp)
target_link_libraries(ceresDemo ${CERES_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(g2oDemo ${OpenCV_LIBS}  g2o_core  g2o_stuff)

