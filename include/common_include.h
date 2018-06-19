#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

// define the commonly included file to avoid a long include list
// for Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector3f;
using Eigen::Matrix3d;
using Eigen::Matrix3f;
using Eigen::Matrix4f;

// for Sophus
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
// using Sophus::SO3;
// using Sophus::SE3;
typedef Sophus::SO3<float> SO3;
typedef Sophus::SE3<float> SE3;

// for cv
#include <opencv2/core/core.hpp>
using cv::Mat;

// std 
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <set>
#include <unordered_map>
#include <map>
#include <bitset>
#include <ctime>
#include <cmath>

using namespace std; 
#endif