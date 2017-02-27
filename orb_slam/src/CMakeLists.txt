add_library( SLAM
    draw_image.cpp frame.cpp loop_closing.cpp map.cpp map_point.cpp optimizer.cpp orb_feature.cpp parameter.cpp tracking.cpp
)

target_link_libraries( SLAM
    ${OpenCV_LIBRARIES}
    g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sim3 g2o_types_sba
    )

