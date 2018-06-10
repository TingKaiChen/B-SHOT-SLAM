#include "lidar_odometry.h"

namespace myslam{
	LidarOdometry::LidarOdometry(): ref_(nullptr), src_(nullptr), test_(nullptr), status_(INITIAL){

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
		for(vector<Vector3f>::iterator it = refptr->begin(); it != refptr->end(); it++){
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
		for(vector<Vector3f>::iterator it = srcptr->begin(); it != srcptr->end(); it++){
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
        	// float radius = 3000.0f*Vector3f(searchPoint.x, searchPoint.y, searchPoint.z).norm()/50000.0f;
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
        			Vector3f pt(src_pcl_.points[ptIdxRadSearch[j]].x, src_pcl_.points[ptIdxRadSearch[j]].y, src_pcl_.points[ptIdxRadSearch[j]].z);
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

        test_ = make_shared<vector<Vector3f>>();
        for(int i=0; i < SegRatio.size(); i++){
        	test_->push_back(Vector3f(src_pcl_.points[SegRatio[i].first].x, src_pcl_.points[SegRatio[i].first].y, src_pcl_.points[SegRatio[i].first].z));       	
        }
        src_->setKeypoints(make_shared<vector<Vector3f>>(test_->begin()+0.95*test_->size(), test_->end()));
        seg_ratios_.clear();
        seg_ratios_.reserve(0.01*test_->size());
        for(auto it = SegRatio.begin()+0.99*test_->size(); it != SegRatio.end(); it++){
        	seg_ratios_.push_back(it->second);
        }
        if(isInitial()){
        	passSrc2Ref();
	        ref_->setKeypoints(src_->getKeypoints());
        }
		cb.cloud1 = src_pcl_;
        cb.cloud2 = ref_pcl_;
		cb.cloud1_keypoints = eigen2pcl(src_->getKeypoints());
		cb.cloud2_keypoints = eigen2pcl(ref_->getKeypoints());
	}

	void LidarOdometry::computeDescriptors(){
	    cb.calculate_normals (3000);
	    cb.calculate_SHOT (3000);
	    cb.compute_bshot();
	}

	void LidarOdometry::featureMatching(){
		if(isInitial()){
			// Reference frame initialization
			passSrc2Ref();
	        ref_->setKeypoints(src_->getKeypoints());
	        cb.cloud2 = ref_pcl_;
			cb.cloud2_keypoints = eigen2pcl(ref_->getKeypoints());
		}
		else{
			// Find possible keypoints in global map
            Vector3f pos(ref_->getPose().matrix().topRightCorner<3,1>());
            float range = 80000;
			globalMap_.getKeypoints(pos, range, cb.cloud2_keypoints, cb.cloud2_bshot);
		}

		cout<<"Src size:\t"<<cb.cloud1_keypoints.size()<<endl;
		cout<<"Ref size:\t"<<cb.cloud2_keypoints.size()<<endl;
		cout<<"add size:\t"<<globalMap_.size()<<endl;

		pcl::Correspondences corresp;

	    int *dist = new int[std::max(cb.cloud1_bshot.size(), cb.cloud2_bshot.size())];
	    int *left_nn = new int[cb.cloud1_bshot.size()];
	    int *right_nn = new int[cb.cloud2_bshot.size()];

	    int min_ix;
	    for (int i=0; i<(int)cb.cloud1_bshot.size(); ++i)
	    {
	        for (int k=0; k<(int)cb.cloud2_bshot.size(); ++k)
	        {
	            dist[k] = (int)( cb.cloud1_bshot[i].bits ^ cb.cloud2_bshot[k].bits).count();
	        }
	        minVect(dist, (int)cb.cloud2_bshot.size(), &min_ix);
	        left_nn[i] = min_ix;

	        // pcl::Correspondence corr;
         //    corr.index_query = i;
         //    corr.index_match = left_nn[i];
         //    corresp.push_back(corr);
	    }
	    for (int i=0; i<(int)cb.cloud2_bshot.size(); ++i)
	    {
	        for (int k=0; k<(int)cb.cloud1_bshot.size(); ++k)
	            dist[k] = (int)(cb.cloud2_bshot[i].bits ^ cb.cloud1_bshot[k].bits).count();
	        minVect(dist, (int)cb.cloud1_bshot.size(), &min_ix);
	        right_nn[i] = min_ix;
	    }

	    for (int i=0; i<(int)cb.cloud1_bshot.size(); ++i){
	        if (right_nn[left_nn[i]] == i)
	        {
	            pcl::Correspondence corr;
	            corr.index_query = i;
	            corr.index_match = left_nn[i];
	            corresp.push_back(corr);
	        }
	    }
		cout<<"Cor size:\t"<<corresp.size()<<endl;


	    delete [] dist;
	    delete [] left_nn;
	    delete [] right_nn;

	    /// RANSAC BASED Correspondence Rejection
	    pcl::CorrespondencesConstPtr correspond = boost::make_shared< pcl::Correspondences >(corresp);

	    pcl::Correspondences corr;
	    Ransac_based_Rejection.setInputSource(cb.cloud1_keypoints.makeShared());
	    Ransac_based_Rejection.setInputTarget(cb.cloud2_keypoints.makeShared());
	    double sac_threshold = 1000;// default PCL value..can be changed and may slightly affect the number of correspondences
	    Ransac_based_Rejection.setInlierThreshold(sac_threshold);
	    Ransac_based_Rejection.setInputCorrespondences(correspond);
	    Ransac_based_Rejection.getCorrespondences(corr);

	    Matrix4f mat = Ransac_based_Rejection.getBestTransformation();
	    cout<<"Estimation done"<<endl;
	    // mat = mat*ref_->getPose().matrix();	// If the pattern is frame-to-frame
        Matrix3f R = mat.block<3,3>(0, 0);
        Vector3f T = mat.topRightCorner<3,1>();
	    for(int i = 0, idx = 0; i < cb.cloud1_bshot.size(); i++){
	    	if(i == corr[idx].index_query && !isInitial()){	// Skip the inliers
	    		idx++;
	    		continue;
	    	}
	    	Vector3f kp_pos = src_->getKeypoints()->at(i);
	    	kp_pos = R*kp_pos+T;
	    	Keypoint::Ptr kp = Keypoint::createKeypoint(
        		kp_pos, seg_ratios_[i], cb.cloud1_bshot[i]);
        	globalMap_.addKeypoint(kp);
	    }

		cout<<"inlier size:\t"<<corr.size()<<endl;	    

	}

	void LidarOdometry::poseEstimation(){
	    Eigen::Matrix4f mat = Ransac_based_Rejection.getBestTransformation();
	    // cout << "Mat : \n" << mat << endl;
	    // src_->setPose(SE3(mat*ref_->getPose().matrix()));	// If the pattern is frame-to-frame
	    src_->setPose(SE3(mat));	// If the pattern is frame-to-localmap
        Vector3f pos(src_->getPose().matrix().topRightCorner<3,1>());
        cout<<"curr pos:\t"<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<endl;

	    status_ = RUN;
	}

	Frame::PCPtr LidarOdometry::getKeypoints(){
		Frame::PCPtr kps = make_shared<vector<Vector3f>>();
		globalMap_.getAllKeypoints(*kps);
		return kps;
	}

	Frame::PCPtr LidarOdometry::getSrcKeypoints(){
		Frame::PCPtr srckps = make_shared<vector<Vector3f>>();
		for(auto& pt: cb.cloud1_keypoints.points){
			srckps->push_back(Vector3f(pt.x, pt.y, pt.z));
		}
		return srckps;
	}

	Frame::PCPtr LidarOdometry::getRefKeypoints(){
		Frame::PCPtr refkps = make_shared<vector<Vector3f>>();
		for(auto& pt: cb.cloud2_keypoints.points){
			refkps->push_back(Vector3f(pt.x, pt.y, pt.z));
		}
		return refkps;
	}

	pcl::PointCloud<pcl::PointXYZ> LidarOdometry::eigen2pcl(Frame::PCPtr pcptr){
		pcl::PointCloud<pcl::PointXYZ> pclpc;
		pclpc.clear();
		pclpc.width = pcptr->size();
		pclpc.height = 1;
		pclpc.points.resize(pcptr->size());
		int i = 0;
		for(vector<Vector3f>::iterator it = pcptr->begin(); it != pcptr->end(); it++){
			pclpc.points[i++] = pcl::PointXYZ((*it)[0], (*it)[1], (*it)[2]);
		}
		return pclpc;
	}

}