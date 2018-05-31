add_library( myslam
    ViewerHandler.cpp
    frame.cpp
    lidar_odometry.cpp
)

target_link_libraries( myslam
    ${THIRD_PARTY_LIBS}
)