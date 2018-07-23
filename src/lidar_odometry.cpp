#include "lidar_odometry.h"

namespace myslam{
	LidarOdometry::LidarOdometry(): 
		ref_(nullptr), src_(nullptr), test_(nullptr), status_(INITIAL), shouldUpdateMap(true){

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
	}

	void LidarOdometry::passSrc2Ref(){
		ref_ = src_;
		ref_pcl_ = src_pcl_;
	}

    typedef pair<int, float> IdxRatioPair;
    bool comparator(const IdxRatioPair& l, const IdxRatioPair& r){return l.second < r.second;}
	void LidarOdometry::extractKeypoints(){
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud (src_pcl_.makeShared());
        pcl::PointXYZ searchPoint; 
        vector<IdxRatioPair> SegRatio;

        float neigh_num = 0;
        float neigh_cnt = 0;
        for(int i = 0; i < src_pcl_.points.size(); i++){
        	// Skip the origin
        	if(src_pcl_.points[i].x == 0 && src_pcl_.points[i].y == 0 && src_pcl_.points[i].z == 0)
        		continue;
        	searchPoint = src_pcl_.points[i];
        	vector<int> ptIdxRadSearch;
        	vector<float> ptRadSqarDistance;
        	float radius = 3000.0f;
        	// float radius = 3000.0f*Vector3f(searchPoint.x, searchPoint.y, searchPoint.z).norm()/50000.0f;
        	if(kdtree.radiusSearch(searchPoint, radius, ptIdxRadSearch, ptRadSqarDistance, 300) > 0){
        		neigh_num += ptIdxRadSearch.size();
        		if(ptIdxRadSearch.size() != 0)
	        		neigh_cnt ++;
    			// Calculate gecmetric centroid and centroid vector
    			pcl::PointXYZ centroid;
    			pcl::computeCentroid(pcl::PointCloud<pcl::PointXYZ>(src_pcl_, ptIdxRadSearch), centroid);
    			Vector3f ct(centroid.x, centroid.y, centroid.z);
    			Vector3f sp(searchPoint.x, searchPoint.y, searchPoint.z);
    			Vector3f ctvec = sp-ct;
    			// // Calculate segmentation ratio: vector version
    			// struct VecNum{
    			// 	float positive = 0.0f;
    			// 	float negative = 0.0f;
    			// } vecnum;
       //  		for(size_t j = 0; j < ptIdxRadSearch.size(); j++){
       //  			Vector3f pt(src_pcl_.points[ptIdxRadSearch[j]].x, src_pcl_.points[ptIdxRadSearch[j]].y, src_pcl_.points[ptIdxRadSearch[j]].z);
       //  			if(ctvec.dot(pt-sp) > 0)
       //  				vecnum.positive += 1;
       //  			else if(ctvec.dot(pt-sp) < 0)
       //  				vecnum.negative += 1;
       //  		}
       //  		float seg_ratio = 1-min(vecnum.positive, vecnum.negative)/max(vecnum.positive, vecnum.negative);

    			// Calculate segmentation ratio: simplified vector version
        		float sum = 0;
        		for(size_t j = 0; j < ptIdxRadSearch.size(); j++){
        			Vector3f pt(src_pcl_.points[ptIdxRadSearch[j]].x, src_pcl_.points[ptIdxRadSearch[j]].y, src_pcl_.points[ptIdxRadSearch[j]].z);
        			if(ctvec.norm() == 0 || (pt-sp).norm() == 0)
        				continue;
        			// sum += ctvec.dot(pt-sp)/(ctvec.norm()*(pt-sp).norm());	// TODO: normalization?
        			sum += ctvec.dot(pt-sp);	// TODO: normalization?
        		}
        		float seg_ratio = abs(sum)/ptIdxRadSearch.size();

        		// // Calculate segmentation ratio: simplified matrix version (slow)
        		// float sum = 0;
        		// MatrixXf neigh_pc(3, ptIdxRadSearch.size());
        		// for(size_t j = 0; j < ptIdxRadSearch.size(); j++){
        		// 	neigh_pc.col(j) = Vector3f(src_pcl_.points[ptIdxRadSearch[j]].x, 
        		// 							   src_pcl_.points[ptIdxRadSearch[j]].y, 
        		// 							   src_pcl_.points[ptIdxRadSearch[j]].z).normalized();
        		// }
        		// float seg_ratio = abs((neigh_pc.transpose()*ctvec).sum())/ptIdxRadSearch.size();

        		if(isnan(seg_ratio))
        			continue;
        		SegRatio.push_back(IdxRatioPair(i, seg_ratio));
        	}
        	cout<<"\rprocess: "<<float(i)/src_pcl_.points.size()*100;
        }
        cout<<endl;
        // cout<<"AVG neigh number:\t"<<neigh_num/neigh_cnt<<endl;
        cout<<"Sorting...";
        sort(SegRatio.begin(), SegRatio.end(), comparator);
        cout<<"done"<<endl;

        test_ = make_shared<vector<Vector3f>>();
        for(int i=0; i < SegRatio.size(); i++){
        	test_->push_back(Vector3f(src_pcl_.points[SegRatio[i].first].x, src_pcl_.points[SegRatio[i].first].y, src_pcl_.points[SegRatio[i].first].z));       	
        }
        if(test_->size() >= 600){
	        src_->setKeypoints(make_shared<vector<Vector3f>>(test_->end()-600, test_->end()));
	        seg_ratios_.clear();
	        seg_ratios_.reserve(600);
	        for(auto it = SegRatio.end()-600; it != SegRatio.end(); it++){
	        	seg_ratios_.push_back(it->second);
	        }
        }
	    else{
	        src_->setKeypoints(make_shared<vector<Vector3f>>(test_->begin(), test_->end()));
	        seg_ratios_.clear();
	        seg_ratios_.reserve(SegRatio.size());
	        for(auto it = SegRatio.begin(); it != SegRatio.end(); it++){
	        	seg_ratios_.push_back(it->second);
	        }
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

        std::shared_ptr<vector<std::bitset<352>>> descriptor(new vector<std::bitset<352>>);
        descriptor->reserve(cb.cloud1_bshot.size());
        for(auto it: cb.cloud1_bshot){
        	descriptor->push_back(it.bits);
        }
        src_->setDescriptors(descriptor);
	}

	void LidarOdometry::featureMatching(){
		if(isInitial()){
			// Reference frame initialization
			passSrc2Ref();
	        ref_->setKeypoints(src_->getKeypoints());
	        cb.cloud2 = ref_pcl_;
	        cb.cloud2_bshot = cb.cloud1_bshot;
			cb.cloud2_keypoints = eigen2pcl(ref_->getKeypoints());
		}
		else{
			// Find possible keypoints in global map
            Vector3f pos(ref_->getPose().topRightCorner<3,1>());
            float range = 100000;
			globalMap_.getKeypoints(pos, range, cb.cloud2_keypoints, cb.cloud2_bshot);

			pcl::PointCloud<pcl::PointXYZ> kp_temp;
			pcl::transformPointCloud(eigen2pcl(ref_->getKeypoints()), kp_temp, ref_->getPose());
			cb.cloud2_keypoints += kp_temp;
			vector<bshot_descriptor> bshot_temp = eigen2dc(ref_->getDescriptors());
			cb.cloud2_bshot.reserve(cb.cloud2_bshot.size()+bshot_temp.size());
			cb.cloud2_bshot.insert(cb.cloud2_bshot.end(), bshot_temp.begin(), bshot_temp.end());
			// Check if ref_keypoint is transformed
		}

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

	    corr.clear();
	    Ransac_based_Rejection.setMaximumIterations(2000);
	    Ransac_based_Rejection.setInputSource(cb.cloud1_keypoints.makeShared());
	    Ransac_based_Rejection.setInputTarget(cb.cloud2_keypoints.makeShared());
	    double sac_threshold = 1500;// default PCL value..can be changed and may slightly affect the number of correspondences
	    Ransac_based_Rejection.setInlierThreshold(sac_threshold);
	    Ransac_based_Rejection.setInputCorrespondences(correspond);
	    Ransac_based_Rejection.getCorrespondences(corr);

	    cout<<"Estimation done"<<endl;

	}

	void LidarOdometry::evaluateEstimation(){
		Matrix4f T_j = Ransac_based_Rejection.getBestTransformation();
		Matrix4f T_i = ref_->getPose();
		Matrix4f T_ij = T_i.inverse()*T_j;
		// Use heading vector and their angle
		Vector3f heading(0, 1, 0);
		float h_diff = acos(heading.transpose()*T_ij.block<3,3>(0, 0)*heading);
		cout<<"------"<<endl;
		cout<<"H_diff (rad):\t"<<h_diff<<endl;
		cout<<"H_diff (ang):\t"<<h_diff*180/M_PI<<endl;
		Vector3f t_diff(T_ij.topRightCorner<3,1>());
		cout<<"T_diff:\t"<<t_diff.norm()<<endl;
		cout<<"------"<<endl;

		// ICP refinement
		Matrix4f T_est;
		if(h_diff*180/M_PI > 10 || t_diff.norm() > 1200 || corr.size()<15 ){
		    T_est = ref_->getPose();
		    shouldUpdateMap = false;
		}
		else{
		    T_est = Ransac_based_Rejection.getBestTransformation();
		    shouldUpdateMap = true;
		}
		pcl::PointCloud<PointXYZ> icp_cloud;
	    pcl::transformPointCloud(cb.cloud1_keypoints, icp_cloud, T_est);
	    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	    icp.setInputSource(icp_cloud.makeShared());
	    icp.setInputTarget(cb.cloud2_keypoints.makeShared());
	    pcl::PointCloud<PointXYZ> icp_out;
	    icp.align(icp_out);
	    T_best_ = icp.getFinalTransformation()*T_est;

	    // T_best_ = Ransac_based_Rejection.getBestTransformation();

	 //    // Correspondence average distance
		// cout<<"Corr num:\t"<<corr.size()<<endl;
		// pcl::PointCloud<PointXYZ> corr_cloud;
	 //    // pcl::transformPointCloud(cb.cloud1_keypoints, corr_cloud, T_j);
	 //    pcl::transformPointCloud(cb.cloud1_keypoints, corr_cloud, T_best_);
		// float dist_avg = 0;
		// for(auto c: corr){
		// 	dist_avg += pcl::geometry::distance(corr_cloud[c.index_query], cb.cloud2_keypoints[c.index_match]);
		// }
		// dist_avg = dist_avg/corr.size();
		// cout<<"Corr avg dist:\t"<<dist_avg<<endl;
		// vector<float> dist_vec;
		// dist_vec.reserve(corr.size());
		// float dist_sd = 0;
		// for(auto c: corr){
		// 	float dist_corr = pcl::geometry::distance(corr_cloud[c.index_query], cb.cloud2_keypoints[c.index_match]);
		// 	// cout<<dist_corr<<endl;
		// 	dist_sd += (dist_corr - dist_avg) * (dist_corr - dist_avg);			
		// 	dist_vec.push_back(dist_corr);
		// }
		// dist_sd = sqrt(dist_sd/corr.size());
		// cout<<"Corr SD dist:\t"<<dist_sd<<endl;
		// sort(dist_vec.begin(), dist_vec.end());
		// cout<<"Corr med:\t"<<dist_vec[dist_vec.size()/2]<<endl;
	}

	void LidarOdometry::poseEstimation(){
	    Eigen::Matrix4f mat = Ransac_based_Rejection.getBestTransformation();
	    // src_->setPose(SE3(mat*ref_->getPose().matrix()));	// If the pattern is frame-to-frame
	    // src_->setPose(SE3(mat));	// If the pattern is frame-to-localmap
	    // src_->setPose(SE3(T_best_));	// If the pattern is frame-to-localmap
	    src_->setPose(T_best_);	// If the pattern is frame-to-localmap
        Vector3f pos(src_->getPose().topRightCorner<3,1>());
        cout<<"curr pos:\t"<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<endl;

	}

	void LidarOdometry::updateMap(){
		// if(shouldUpdateMap){
			Matrix4f mat = T_best_;
		    Matrix3f R = mat.block<3,3>(0, 0);
	        Vector3f T = mat.topRightCorner<3,1>();
		    for(int i = 0, idx = 0; i < cb.cloud1_bshot.size(); i++){
		    	// if(i == corr[idx].index_query && !isInitial()){	// Skip the inliers
		    	// 	idx++;
		    	// 	continue;
		    	// }
		    	Vector3f kp_pos = src_->getKeypoints()->at(i);
		    	kp_pos = R*kp_pos+T;
		    	Keypoint::Ptr kp = Keypoint::createKeypoint(
	        		kp_pos, seg_ratios_[i], cb.cloud1_bshot[i]);
	        	globalMap_.addKeypoint(kp);

	      //   	if(i == corr[idx].index_query || isInitial()){	// Only add inliers
	      //   		Vector3f kp_pos = src_->getKeypoints()->at(i);
			    // 	kp_pos = R*kp_pos+T;
			    // 	Keypoint::Ptr kp = Keypoint::createKeypoint(
		     //    		kp_pos, seg_ratios_[i], cb.cloud1_bshot[i]);
		     //    	globalMap_.addKeypoint(kp);
		    	// 	idx++;
		    	// }
		    	
		    }
		    cout<<"Map updated"<<endl;

		// }
		

	    status_ = RUN;
	}

	void LidarOdometry::updateCorrespondence(){
		corrs.clear();
	    corrs.reserve(corr.size());
	    for(auto& co: corr){
	    	Vector3f p1(cb.cloud1_keypoints[co.index_query].x, cb.cloud1_keypoints[co.index_query].y, cb.cloud1_keypoints[co.index_query].z);
	    	Vector3f p2(cb.cloud2_keypoints[co.index_match].x, cb.cloud2_keypoints[co.index_match].y, cb.cloud2_keypoints[co.index_match].z);
	    	// p1 = R*p1+T;
	    	corrs.push_back(make_pair(p1, p2));
	    }

		cout<<"inlier size:\t"<<corr.size()<<endl;	  

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

	vector<bshot_descriptor> LidarOdometry::eigen2dc(Frame::DCPPtr pcptr){
		vector<bshot_descriptor> dcpc;
		dcpc.reserve(pcptr->size());
		for(auto it = pcptr->begin(); it != pcptr->end(); it++){
			bshot_descriptor bs;
			bs.bits = *it;
			dcpc.push_back(bs);
		}
		return dcpc;
	}


			typedef vector<Vector3f> PC;
	vector<PC> LidarOdometry::getBlockKeypoints(){
		vector<PC> kpblock;
		globalMap_.getBlockKeypoints(kpblock);
		return kpblock;
	}


}