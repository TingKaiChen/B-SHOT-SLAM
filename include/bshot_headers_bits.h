#ifndef headers_bshot_bits
#define headers_bshot_bits

#include <iostream>
#include <bitset>
#include <vector>
#include <ctime>
#include <chrono>

#include <dirent.h> // for looping over the files in the directory

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/geometry.h>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot_omp.h>
#include <pcl/kdtree/kdtree_flann.h>

// #include <pcl/geometry>

// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/visualization/point_cloud_handlers.h>


#include <string>
#include <fstream>
#include <string>


using namespace std;
using namespace std::chrono;
using namespace Eigen;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

#define PI 3.14159265

#endif // headers_bshot_bits


