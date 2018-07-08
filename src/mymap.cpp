#include "mymap.h"

namespace myslam{
	void Map::addKeypoint(Keypoint::Ptr keypoint){
		// The keypoint block does not exist
		unsigned long block_id = getBlockID(keypoint->getPosition());
		if(keypoints_.find(block_id) == keypoints_.end()){
			Block kp_block;
			kp_block.insert(make_pair(keypoint->getPosition(), keypoint));
			keypoints_.insert(make_pair(block_id, kp_block));
		}
		// The keypoint block already exist
		else{
			vector<Vector3f> del_points;
			bool isCandidate = true;
			for(auto& kp:keypoints_[block_id]){
				if((keypoint->getPosition()-kp.first).norm() < 800 &&
					keypoint->getSegRatio() <= kp.second->getSegRatio()){
					isCandidate = false;
				}
			}
			if(isCandidate){
				keypoints_[block_id][keypoint->getPosition()] = keypoint;
			}
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
		unsigned long block_id;
		// Search range
		int x_min = int(round((pos[0]-range)/prec))*prec;
		int x_max = int(round((pos[0]+range)/prec))*prec;
		int y_min = int(round((pos[1]-range)/prec))*prec;
		int y_max = int(round((pos[1]+range)/prec))*prec;
		int z_min = int(round((pos[2]-range)/prec))*prec;
		int z_max = int(round((pos[2]+range)/prec))*prec;
		for(int x = x_min; x <= x_max; x += prec){
			for(int y = y_min; y <= y_max; y += prec){
				for(int z = z_min; z <= z_max; z += prec){
					block_id = getBlockID(Vector3f(x, y, z));
					// unsigned long bk_id = keypoints_.bucket(Vector3f(x, y, z));
					if(keypoints_.find(block_id) == keypoints_.end())
						continue;
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
		// cout<<"All kps:\t"<<vec.size()<<endl;
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
			int(round(pos[0]/prec))*prec, 
			int(round(pos[1]/prec))*prec, 
			int(round(pos[2]/prec))*prec);
		// 64bits keyID = 1bit+21bits(x)+21bits(y)+21bits(z)
		bitset<64> i = ((bitset<64>(int(grid_p[0]))<<42) & bitset<64>(0x1FFFFF)<<42);
		bitset<64> j = ((bitset<64>(int(grid_p[1]))<<21) & bitset<64>(0x1FFFFF)<<21);
		bitset<64> k = (bitset<64>(int(grid_p[2])) & bitset<64>(0x1FFFFF));
		return ((i|j|k).to_ulong());
	}

	void Map::getBlockKeypoints(vector<KPointCloud>& kpc){
		for(auto& block: keypoints_){
			KPointCloud temp;
			temp.reserve(block.second.size());
			for(auto& kp: block.second){
				temp.push_back(kp.first);
			}
			kpc.push_back(temp);
		}
	}


}