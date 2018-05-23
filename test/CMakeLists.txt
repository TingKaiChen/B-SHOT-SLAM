add_executable( ptpicking pointpicking.cpp )
target_link_libraries( ptpicking myslam )

add_executable( showPC ShowPointCloud.cpp )
target_link_libraries( showPC myslam )