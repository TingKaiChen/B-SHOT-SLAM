#include "lidar_odometry.h"

namespace myslam{
	LidarOdometry::LidarOdometry(): ref_(nullptr), src_(nullptr), test_(nullptr){

	}

	LidarOdometry::~LidarOdometry(){

	}

	void LidarOdometry::setRefFrame(Frame::Ptr ref){
		ref_ = ref;
		Frame::PCPtr refptr = ref_->getPointCloud();
		ref_pcl_.clear();
		ref_pcl_.width = refptr->size();
		ref_pcl_.height = 1;
		ref_pcl_.points.resize(refptr->size());
		int i = 0;
		for(vector<Vector3d>::iterator it = refptr->begin(); it != refptr->end(); it++){
			// ref_pcl_.push_back(pcl::PointXYZ((*it)[0], (*it)[1], (*it)[2]));
			ref_pcl_.points[i++] = pcl::PointXYZ((*it)[0], (*it)[1], (*it)[2]);
		}
		// cout<<"size of ref. PCL: "<<ref_pcl_.size()<<endl;
	}

	void LidarOdometry::setSrcFrame(Frame::Ptr src){
		src_ = src;
		Frame::PCPtr srcptr = src_->getPointCloud();
		src_pcl_.clear();
		src_pcl_.width = srcptr->size();
		src_pcl_.height = 1;
		src_pcl_.points.resize(srcptr->size());
		int i = 0;
		for(vector<Vector3d>::iterator it = srcptr->begin(); it != srcptr->end(); it++){
			// src_pcl_.push_back(pcl::PointXYZ((*it)[0], (*it)[1], (*it)[2]));
			src_pcl_.points[i++] = pcl::PointXYZ((*it)[0], (*it)[1], (*it)[2]);
		}
		// cout<<"size of src. PCL: "<<src_pcl_.size()<<endl;
	}

	void LidarOdometry::passSrc2Ref(){
		ref_ = src_;
		ref_pcl_ = src_pcl_;
		// cout<<"Pass src. data to ref., ref size: "<<ref_pcl_.size()<<endl;
	}

    typedef pair<int, float> IdxRatioPair;
    bool comparator(const IdxRatioPair& l, const IdxRatioPair& r){return l.second < r.second;}
	void LidarOdometry::extractKeypoints(){
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud (src_pcl_.makeShared());
        pcl::PointXYZ searchPoint; 
        vector<IdxRatioPair> SegRatio;
        for(int i = 0; i < src_pcl_.points.size(); i++){
        	// Skip the origin
        	if(src_pcl_.points[i].x == 0 && src_pcl_.points[i].y == 0 && src_pcl_.points[i].z == 0)
        		continue;
        	searchPoint = src_pcl_.points[i];
        	vector<int> ptIdxRadSearch;
        	vector<float> ptRadSqarDistance;
        	float radius = 3000.0f;
        	if(kdtree.radiusSearch(searchPoint, radius, ptIdxRadSearch, ptRadSqarDistance) > 0){
    			// Calculate gecmetric centroid and centroid vector
    			pcl::PointXYZ centroid;
    			pcl::computeCentroid(pcl::PointCloud<pcl::PointXYZ>(src_pcl_, ptIdxRadSearch), centroid);
    			Vector3f ct(centroid.x, centroid.y, centroid.z);
    			Vector3f sp(searchPoint.x, searchPoint.y, searchPoint.z);
    			Vector3f ctvec = sp-ct;
    			// Calculate segmentation ratio
    			struct VecNum{
    				float positive = 0.0f;
    				float negative = 0.0f;
    			} vecnum;
        		for(size_t j = 0; j < ptIdxRadSearch.size(); j++){
        			Vector3f pt(src_pcl_.points[j].x, src_pcl_.points[j].y, src_pcl_.points[j].z);
        			if(ctvec.dot(pt-sp) > 0)
        				vecnum.positive += 1;
        			else if(ctvec.dot(pt-sp) < 0)
        				vecnum.negative += 1;
        		}
        		float seg_ratio = 1-min(vecnum.positive, vecnum.negative)/max(vecnum.positive, vecnum.negative);
        		if(isnan(seg_ratio))
        			continue;
        		SegRatio.push_back(IdxRatioPair(i, seg_ratio));
        	}
        	cout<<"\rprocess: "<<float(i)/src_pcl_.points.size()*100;
        }
        cout<<endl;
        cout<<"Sorting...";
        sort(SegRatio.begin(), SegRatio.end(), comparator);
        cout<<"done"<<endl;

        test_ = make_shared<vector<Vector3d>>();
        for(int i=0; i < SegRatio.size(); i++){
        	test_->push_back(Vector3d(src_pcl_.points[SegRatio[i].first].x, src_pcl_.points[SegRatio[i].first].y, src_pcl_.points[SegRatio[i].first].z));       	
        }

        // for(int i=0; i<SegRatio.size(); i++){
        // 	cout<<SegRatio[i].second<<" ";
        // }
        // cout<<endl;



  //       for(int i = 0; i<src_pcl_.points.size(); i++)
  //       	if(src_pcl_.points[i].x != 0 || src_pcl_.points[i].y != 0 || src_pcl_.points[i].z != 0){
		//         searchPoint = src_pcl_.points[i];
		//         break;
  //       	}
  //       vector<int> pointIdxRadiusSearch;
		// vector<float> pointRadiusSquaredDistance;
		// float radius = 3000.0f;
		// std::cout << "Neighbors within radius search at (" << searchPoint.x 
  //           << " " << searchPoint.y 
  //           << " " << searchPoint.z
  //           << ") with radius=" << radius << std::endl;
  //       if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
	 //        int num = 0;
		//     for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i, ++num){
		//       	std::cout << "    "  <<   src_pcl_.points[ pointIdxRadiusSearch[i] ].x 
	 //                << " " << src_pcl_.points[ pointIdxRadiusSearch[i] ].z 
	 //                << " " << src_pcl_.points[ pointIdxRadiusSearch[i] ].y 
	 //                << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
		//     }
		// }
		// float dist = pcl::geometry::distance(searchPoint, src_pcl_.points[ pointIdxRadiusSearch.back() ]);
		// cout<<"distance: "<<dist<<endl;
	}

	void LidarOdometry::computeDescriptors(){

	}

	void LidarOdometry::featureMatching(){

	}

	void LidarOdometry::poseEstimation(){

	}
}