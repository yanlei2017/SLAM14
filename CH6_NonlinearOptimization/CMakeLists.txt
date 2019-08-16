add_executable(ceresDemo ceresDemo.cpp)
add_executable(g2oDemo g2oDemo.cpp)
target_link_libraries(ceresDemo ${CERES_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(g2oDemo ${OpenCV_LIBS}  g2o_core  g2o_stuff)

