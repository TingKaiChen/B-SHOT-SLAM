#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include <common_include.h>
#include "bshot_bits.h"
#include <frame.h>

namespace myslam{
	class LidarOdometry{
		public:
			LidarOdometry();
			~LidarOdometry();

			void setRefFrame(Frame::Ptr ref);
			void setSrcFrame(Frame::Ptr src);
			void extractKeypoints();
			void computeDescriptors();
			void featureMatching();
			void poseEstimation();
			void passSrc2Ref();
			bool isInitial(){return ref_ == nullptr;};
			Frame::PCPtr getKeypoints(){return test_;};
		private:
			Frame::Ptr ref_;	// Reference frame
			Frame::Ptr src_;	// Source frame
			Frame::PCPtr test_;	// test frame
			pcl::PointCloud<pcl::PointXYZ> ref_pcl_;	// PCL pointcloud type of reference frame
			pcl::PointCloud<pcl::PointXYZ> src_pcl_;	// PCL pointcloud type of source frame
	};
}
#endif