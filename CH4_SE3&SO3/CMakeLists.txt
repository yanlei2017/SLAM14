add_executable(someprove someprove.cpp)
add_executable(useSophus useSophus.cpp)
target_link_libraries(someprove ${Sophus_LIBRARIES})
target_link_libraries(useSophus ${Sophus_LIBRARIES})

