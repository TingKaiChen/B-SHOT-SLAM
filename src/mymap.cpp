#include "mymap.h"

namespace myslam{
	void Map::addKeypoint(Keypoint::Ptr keypoint){
		// The keypoint block does not exist
		BlockMap::hasher fn = keypoints_.hash_function();
		unsigned long block_id = getBlockID(keypoint->getPosition());
		if(keypoints_.find(block_id) == keypoints_.end()){
			Block kp_block;
			kp_block.insert(make_pair(keypoint->getPosition(), keypoint));
			keypoints_.insert(make_pair(block_id, kp_block));
		}
		// The keypoint block already exist
		else{
			vector<Vector3f> del_points;
			auto it = keypoints_[block_id].begin();
			while(it != keypoints_[block_id].end()){
			// for(auto& kp: keypoints_[block_id]){
				if((keypoint->getPosition()-it->first).norm() < 800 &&
					keypoint->getSegRatio() > it->second->getSegRatio()){
					// keypoints_[keypoint->getPosition()].insert(make_pair(keypoint->getPosition(), keypoint));
					// del_points.push_back(kp.first);
					keypoints_[block_id][keypoint->getPosition()] = keypoint;
					it = keypoints_[block_id].erase(it);
				}
				else if((keypoint->getPosition()-it->first).norm() > 800){
					// keypoints_[keypoint->getPosition()].insert(make_pair(keypoint->getPosition(), keypoint));
					keypoints_[block_id][keypoint->getPosition()] = keypoint;
					it++;
				}
				else{
					it++;
				}
				
			}
			// for(auto& kp_erase: del_points){
			// 	keypoints_[block_id].erase(kp_erase);
			// }
		}


		// The keypoint does not exist
		// if(keypoints_.find(keypoint->getPosition()) == keypoints_.end()){
		// 	unsigned long bk_id = keypoints_.bucket(keypoint->getPosition());
		// 	if(keypoints_.bucket_size(bk_id) == 0){
		// 		keypoints_.insert(make_pair(keypoint->getPosition(), keypoint));
		// 	}
		// 	else{
		// 		vector<Vector3f> del_points;
		// 		for(auto it = keypoints_.begin(bk_id); it != keypoints_.end(bk_id); it++){
		// 			if((keypoint->getPosition()-it->second->getPosition()).norm() < 300 || 
		// 				keypoint->getSegRatio() > it->second->getSegRatio()){
		// 				// keypoints_.erase(it->first);
		// 				del_points.push_back(it->first);
		// 				keypoints_.insert(make_pair(keypoint->getPosition(), keypoint));
		// 			}
		// 			else if((keypoint->getPosition()-it->second->getPosition()).norm() > 300){
		// 				keypoints_.insert(make_pair(keypoint->getPosition(), keypoint));
		// 			}
		// 		}
		// 		for(auto kp: del_points){
		// 			keypoints_.erase(kp);
		// 		}
		// 	}
			
		// 	// keypoints_.insert(make_pair(keypoint->getPosition(), keypoint));
		// }
		// // The keypoint already exist
		// else{
		// 	keypoints_[keypoint->getPosition()] = keypoint;
		// }
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
		BlockMap::hasher fn = keypoints_.hash_function();
		unsigned long block_id;
		// Search range
		int x_min = int(trunc((pos[0]-range)/prec))*prec;
		int x_max = int(trunc((pos[0]+range)/prec))*prec;
		int y_min = int(trunc((pos[1]-range)/prec))*prec;
		int y_max = int(trunc((pos[1]+range)/prec))*prec;
		int z_min = int(trunc((pos[2]-range)/prec))*prec;
		int z_max = int(trunc((pos[2]+range)/prec))*prec;
		for(int x = x_min; x <= x_max; x += prec){
			for(int y = y_min; y <= y_max; y += prec){
				for(int z = z_min; z <= z_max; z += prec){
					block_id = getBlockID(Vector3f(x, y, z));
					// unsigned long bk_id = keypoints_.bucket(Vector3f(x, y, z));
					int bucket_sz = keypoints_[block_id].size();
					// if(bucket_sz != 0)
						// cout<<"bk size:\t"<<bucket_sz<<endl;
					kpts_pos.points.reserve(kpts_pos.points.size()+bucket_sz);
					// kpts_pos.points.reserve(kpts_pos.points.size()+keypoints_.bucket_size(bk_id));
					descriptors.reserve(descriptors.size()+bucket_sz);
					// descriptors.reserve(descriptors.size()+keypoints_.bucket_size(bk_id));
					for(auto& kp: keypoints_[block_id]){
					// for(auto it = keypoints_.begin(bk_id); it !=keypoints_.end(bk_id); it++){
						kpts_pos.points.push_back(eigenPt2PclPt(kp.first));
						// kpts_pos.points.push_back(eigenPt2PclPt(it->second->getPosition()));
						descriptors.push_back(kp.second->getDescriptor());
						// descriptors.push_back(it->second->getDescriptor());
					// cout<<"\rcount:\t"<<(count++);
					}
				}
			}
		}
		cout<<endl;
		kpts_pos.width = kpts_pos.points.size();
		kpts_pos.height = 1;
	}

	void Map::getAllKeypoints(vector<Vector3f>& vec){
		vec.clear();
		for(auto& block:keypoints_){
			for(auto& kp: block.second){
				vec.push_back(kp.first);				
			}
		}
		cout<<"All kps:\t"<<vec.size()<<endl;
	}

	int Map::size(){
		int count = 0;
		for(auto& block: keypoints_)
			for(auto& kp: block.second)
				count++;
		// cout<<"add total:\t"<<count<<endl;
		return count;
	}

	unsigned long Map::getBlockID(Vector3f pos){
		Vector3f grid_p(
			int(trunc(pos[0]/prec))*prec, 
			int(trunc(pos[1]/prec))*prec, 
			int(trunc(pos[2]/prec))*prec);
		// 64bits keyID = 1bit+21bits(x)+21bits(y)+21bits(z)
		bitset<64> i = ((bitset<64>(grid_p[0])<<42) & bitset<64>(0x1FFFFF)<<42);
		bitset<64> j = ((bitset<64>(grid_p[1])<<21) & bitset<64>(0x1FFFFF)<<21);
		bitset<64> k = (bitset<64>(grid_p[2]) & bitset<64>(0x1FFFFF));
		return ((i|j|k).to_ulong());
	}

}