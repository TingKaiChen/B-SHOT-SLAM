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
		kpts_pos.clear();
		descriptors.clear();
		int count = 0;
		for(auto& element:keypoints_){
			float distance = (element.second->getPosition()-pos).norm();
			if(distance <= range){
				kpts_pos.points.push_back(eigenPt2PclPt(element.second->getPosition()));
				descriptors.push_back(element.second->getDescriptor());
			}
			cout<<"\rRef pc loading... "<<(count++)*100/keypoints_.size()<<"%";
		}
		cout<<endl;
		kpts_pos.width = kpts_pos.points.size();
		kpts_pos.height = 1;

		cout<<"ref pc size:\t"<<kpts_pos.points.size()<<endl;
	}

	void Map::getAllKeypoints(vector<Vector3f>& vec){
		vec.clear();
		for(auto& element:keypoints_){
			vec.push_back(element.second->getPosition());
		}
	}

}