#include "mymap.h"

namespace myslam{
	void Map::addKeypoint(Keypoint::Ptr keypoint){
		// The keypoint does not exist
		if(keypoints_.find(keypoint->getPosition()) == keypoints_.end()){
			keypoints_.insert(make_pair(keypoint->getPosition(), keypoint));
		}
		// The keypoint already exist
		else{
			keypoints_[keypoint->getPosition()] = keypoint;
		}
	}

	void Map::getKeypoints(
			Vector3f pos, 
			float range, 
			pcl::PointCloud<pcl::PointXYZ>& kpts_pos, 
			vector<bshot_descriptor>& descriptors){
		// Clear the containers of keypoint and descriptor
		kpts_pos.clear();
		descriptors.clear();
		int count = 0;
		// Search range
		int x_min = int(trunc((pos[0]-range)/prec))*prec;
		int x_max = int(trunc((pos[0]+range)/prec))*prec;
		int y_min = int(trunc((pos[1]-range)/prec))*prec;
		int y_max = int(trunc((pos[1]+range)/prec))*prec;
		int z_min = int(trunc((pos[2]-range)/prec))*prec;
		int z_max = int(trunc((pos[2]+range)/prec))*prec;
		// Vecmap::hasher fn = keypoints_.hash_function();
		for(int x = x_min; x <= x_max; x += prec){
			for(int y = y_min; y <= y_max; y += prec){
				for(int z = z_min; z <= z_max; z += prec){
					// unsigned long id = fn(Vector3f(x, y, z));
					unsigned long bk_id = keypoints_.bucket(Vector3f(x, y, z));
					kpts_pos.points.reserve(kpts_pos.points.size()+keypoints_.bucket_size(bk_id));
					descriptors.reserve(descriptors.size()+keypoints_.bucket_size(bk_id));
					for(auto it = keypoints_.begin(bk_id); it !=keypoints_.end(bk_id); it++){
						kpts_pos.points.push_back(eigenPt2PclPt(it->second->getPosition()));
						descriptors.push_back(it->second->getDescriptor());
					}
					cout<<"\rcount:\t"<<(count++);
				}
			}
		}
		cout<<endl;
		kpts_pos.width = kpts_pos.points.size();
		kpts_pos.height = 1;
	}

	void Map::getAllKeypoints(vector<Vector3f>& vec){
		vec.clear();
		for(auto& element:keypoints_){
			vec.push_back(element.second->getPosition());
		}
	}

}