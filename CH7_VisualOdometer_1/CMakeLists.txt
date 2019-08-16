add_library(featureExtraction_static libfeatureExtraction.cpp)
add_library(featureExtraction_shared SHARED  libfeatureExtraction.cpp)
add_executable(featureExtraction featureExtraction.cpp)
add_executable(pose_estimation_2d2d pose_estimation_2d2d.cpp)

target_link_libraries(featureExtraction  ${OpenCV_LIBS})
target_link_libraries(pose_estimation_2d2d ${OpenCV_LIBS} featureExtraction_shared)


