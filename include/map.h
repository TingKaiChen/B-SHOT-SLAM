#ifndef MAP_H
#define MAP_H

#include "common_include.h"
#include "keypoint.h"
#include "bshot_bits.h"

namespace myslam{
	class Map{
		public:
			typedef shared_ptr<Map> Ptr;
			Map(){};

			void addKeypoint(Keypoint::Ptr keypoint);
			void getKeypoints(
				Vector3f pos, 
				float range, 
				pcl::PointCloud<pcl::PointXYZ>& kpts_pos, 
				vector<bshot_descriptor>& descriptors);
			void getAllKeypoints(vector<Vector3f>& vec);
			inline pcl::PointXYZ eigenPt2PclPt(Vector3f pt){
				return pcl::PointXYZ(pt[0], pt[1], pt[2]);
			};

		private:
			unordered_map<unsigned long, Keypoint::Ptr> keypoints_;
	};
}


#endif