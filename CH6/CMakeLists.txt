
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

add_executable(main main.cpp)
target_link_libraries(main ${CERES_LIBRARIES} ${OpenCV_LIBS})
