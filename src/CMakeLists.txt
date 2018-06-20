add_library( myslam
    ViewerHandler.cpp
    frame.cpp
    lidar_odometry.cpp
    keypoint.cpp
    mymap.cpp
    preprocess.cpp
)

target_link_libraries( myslam
    ${THIRD_PARTY_LIBS}
)