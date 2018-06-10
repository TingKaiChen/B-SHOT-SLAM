#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "common_include.h"
#include "bshot_bits.h"
#include "frame.h"
// #include "map.h"
#include "mymap.h"

namespace myslam{
	class LidarOdometry{
		public:
			LidarOdometry();
			~LidarOdometry();

			enum STATUS{INITIAL, RUN};

			void setRefFrame(Frame::Ptr ref);
			void setSrcFrame(Frame::Ptr src);
			void extractKeypoints();
			void computeDescriptors();
			void featureMatching();
			void poseEstimation();
			void passSrc2Ref();
			bool isInitial(){return status_ == INITIAL;};
			Frame::Ptr getRefFrame(){return ref_;};
			Frame::Ptr getSrcFrame(){return src_;};
			Frame::PCPtr getKeypoints();
			Frame::PCPtr getSrcKeypoints();
			Frame::PCPtr getRefKeypoints();
			pcl::PointCloud<pcl::PointXYZ> eigen2pcl(Frame::PCPtr pcptr);
		private:
			Frame::Ptr ref_;	// Reference frame
			Frame::Ptr src_;	// Source frame
			Frame::PCPtr test_;	// test frame
			pcl::PointCloud<pcl::PointXYZ> ref_pcl_;	// PCL pointcloud type of reference frame
			pcl::PointCloud<pcl::PointXYZ> src_pcl_;	// PCL pointcloud type of source frame
		    bshot cb;
		    pcl::registration::CorrespondenceRejectorSampleConsensus< pcl::PointXYZ > Ransac_based_Rejection;
			STATUS status_;		    
			vector<float> seg_ratios_;
		    Map globalMap_;
	};
}
#endif