#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "common_include.h"
#include "bshot_bits.h"
#include "frame.h"
// #include "map.h"
#include "mymap.h"
#include <pcl/registration/icp.h>
#include <pcl/keypoints/iss_3d.h>

namespace myslam{
	class LidarOdometry{
		public:
			LidarOdometry();
			~LidarOdometry();

			enum STATUS{INITIAL, RUN};

			void setRefFrame(Frame::Ptr ref);
			void setSrcFrame(Frame::Ptr src);
			void extractKeypoints();
			// void icp();
			void computeDescriptors();
			void featureMatching();
			void poseEstimation();
			void evaluateEstimation();
			void updateMap();
			void updateCorrespondence();
			void kpEvaluation();
			pcl::PointCloud<pcl::PointXYZ> issKpDetection(pcl::PointCloud<pcl::PointXYZ> kps);
			void passSrc2Ref();
			bool isInitial(){return status_ == INITIAL;};
			Frame::Ptr getRefFrame(){return ref_;};
			Frame::Ptr getSrcFrame(){return src_;};
			Frame::PCPtr getKeypoints();
			Frame::PCPtr getSrcKeypoints();
			Frame::PCPtr getRefKeypoints();
			Frame::PCPtr getISSKeypoints();
			typedef vector<Vector3f> PC;
			vector<PC> getBlockKeypoints();
			// SE3 getTransformationDiff(){return (src_->getPose()*ref_->getPose().inverse());};
			Matrix4f getTransformationDiff(){return (src_->getPose()*ref_->getPose().inverse());};
			vector<pair<Vector3f,Vector3f> > getCorrespondences(){return corrs;};
			pcl::PointCloud<pcl::PointXYZ> eigen2pcl(Frame::PCPtr pcptr);
			vector<bshot_descriptor> eigen2dc(Frame::DCPPtr pcptr);
			void setSRType(string sr_type){ sr_type_ = sr_type; };
			void setEvaluateCorr(bool eval_corr){ evaluate_corr_ = eval_corr; };
			void setEvaluateICP(bool eval_icp){ evaluate_icp_ = eval_icp; };
			void setRunICP(bool run_icp){ run_icp_ = run_icp; };
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
		    pcl::Correspondences corr;
		    vector<pair<Vector3f,Vector3f> > corrs;
		    Matrix4f T_best_;
		    bool shouldUpdateMap;
		    string sr_type_;
		    bool evaluate_corr_;
		    bool evaluate_icp_;
		    bool run_icp_;

		    pcl::PointCloud<pcl::PointXYZ> isskps_src;
		    pcl::PointCloud<pcl::PointXYZ> isskps_ref;
	};
}
#endif