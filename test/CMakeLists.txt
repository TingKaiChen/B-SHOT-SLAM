add_executable( ptpicking pointpicking.cpp )
target_link_libraries( ptpicking myslam )

add_executable( showPC ShowPointCloud.cpp )
target_link_libraries( showPC myslam )

add_executable( odometry_test odometry_test.cpp )
target_link_libraries( odometry_test myslam )

add_executable( eigen_test test.cpp )
target_link_libraries( eigen_test myslam )

add_executable( pointcloud_preprocessing pointcloud_preprocessing.cpp )
target_link_libraries( pointcloud_preprocessing myslam )
