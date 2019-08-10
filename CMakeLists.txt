cmake_minimum_required(VERSION 3.0.0)
project(Slam14 VERSION 0.1.0)

include(CTest)
enable_testing()
include_directories("/usr/include/eigen3")
add_subdirectory(CH2 CH2/bin)
add_subdirectory(CH3 CH3/bin)
add_subdirectory(CH4 CH4/bin)


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
