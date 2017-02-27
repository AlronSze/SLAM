set( build_libs
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    SLAM
    ${PROJECT_SOURCE_DIR}/lib/libDBoW2.so
    g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension
)

add_executable( slam_main main.cpp )
target_link_libraries( slam_main
    ${build_libs}
)

