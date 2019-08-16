add_library(featureExtraction_static  lib/libfeatureExtraction.cpp)
add_library(featureExtraction_shared  SHARED  lib/libfeatureExtraction.cpp)
add_library(libTriangulation_static  lib/libTriangulation.cpp)
add_library(libTriangulation_shared  SHARED lib/libTriangulation.cpp)

add_executable(featureExtraction featureExtraction.cpp)
target_link_libraries(featureExtraction  ${OpenCV_LIBS})

add_executable(pose_estimation_2d2d pose_estimation_2d2d.cpp)
target_link_libraries(pose_estimation_2d2d ${OpenCV_LIBS} featureExtraction_shared)

add_executable(triangulation triangulation.cpp)
target_link_libraries(triangulation ${OpenCV_LIBS} libTriangulation_shared featureExtraction_shared)


