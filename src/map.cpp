#include "map.h"

namespace myslam{
	void Map::addKeypoint(Keypoint::Ptr keypoint){
		// The keypoint does not exist
		if(keypoints_.find(keypoint->getId()) == keypoints_.end()){
			keypoints_.insert(make_pair(keypoint->getId(), keypoint));
		}
		// The keypoint already exist
		else{
			keypoints_[keypoint->getId()] = keypoint;
		}
	}
	void Map::getKeypoints(
			Vector3f pos, 
			float range, 
			pcl::PointCloud<pcl::PointXYZ>& kpts_pos, 
			vector<bshot_descriptor>& descriptors){
		// TODO: find all descriptors in the range to the pos
	}

}