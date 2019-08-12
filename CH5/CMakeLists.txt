find_package(OpenCV REQUIRED)
<<<<<<< HEAD
set(CMAKE_CXX_STANDARD 11 )
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(imageBasics imageBasics.cpp)
target_link_libraries(imageBasics ${OpenCV_LIBS})
=======
set(CMAKE_CXX_STANDARD 11)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")

find_package(PCL REQUIRED COMPONENT common io)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(joinmap_acer joinmap_acer.cpp)
target_link_libraries(joinmap_acer ${OpenCV_LIBS} ${PCL_LIBRARIES})

>>>>>>> 57342546f4de485a313951a128a513abe2754109
