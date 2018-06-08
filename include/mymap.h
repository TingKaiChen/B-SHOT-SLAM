#ifndef MYMAP_H
#define MYMAP_H

#include "common_include.h"
#include "keypoint.h"
#include "bshot_bits.h"

namespace myslam{
	class Map{
		public:
			struct MapHasher{
				unsigned long operator()(const Vector3f& p) const{
					int prec = 10000;	// Map's grid size (mm)
					Vector3f grid_p(
						int(trunc(p[0]/prec))*prec, 
						int(trunc(p[1]/prec))*prec, 
						int(trunc(p[2]/prec))*prec);
					// 64bits keyID = 1bit+21bits(x)+21bits(y)+21bits(z)
					bitset<64> i = ((bitset<64>(grid_p[0])<<42) & bitset<64>(0x1FFFFF)<<42);
					bitset<64> j = ((bitset<64>(grid_p[1])<<21) & bitset<64>(0x1FFFFF)<<21);
					bitset<64> k = (bitset<64>(grid_p[2]) & bitset<64>(0x1FFFFF));
					return ((i|j|k).to_ulong());
				}
			};
			typedef shared_ptr<Map> Ptr;
			typedef unordered_map<Vector3f, Keypoint::Ptr, MapHasher> Vecmap;
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
			// unordered_map<unsigned long, Keypoint::Ptr> keypoints_;
			unordered_map<Vector3f, Keypoint::Ptr, MapHasher> keypoints_;
			int prec = 10000;	// Map's grid size (mm)
	};
}


#endif