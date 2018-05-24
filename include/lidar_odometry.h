#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include <common_include.h>
#include <frame.h>

namespace myslam{
	class LidarOdometry{
		public:
			LidarOdometry();
			~LidarOdometry();

			void setRefFrame(Frame::Ptr ref){ref_ = ref;};
			void setSrcFrame(Frame::Ptr src){src_ = src;};
			void extractKeypoints();
			void computeDescriptors();
			void featureMatching();
			void poseEstimation();
		private:
			Frame::Ptr ref_;	// Reference frame
			Frame::Ptr src_;	// Source frame
	};
}
#endif