#include "preprocess.h"

namespace myslam{
	Preprocessor::Preprocessor(){

	}

	Preprocessor::Preprocessor(vector<velodyne::Laser>& lasers, vector<double>& vertAngle, shared_ptr<vector<Vector3f>> pc){
		lasers_ = lasers;
		pc_ = pc;
		vertAngle_ = vertAngle;  // Degree
	    sort(vertAngle_.begin(), vertAngle_.end());
	}

	void Preprocessor::setLasers(vector<velodyne::Laser>& lasers){
		lasers_ = lasers;
	}

	void Preprocessor::setVerticalAngles(vector<double>& vertAngle){
		vertAngle_ = vertAngle;  // Degree
	    sort(vertAngle_.begin(), vertAngle_.end());
	}

	void Preprocessor::setPointCloud(shared_ptr<vector<Vector3f>> pc){
		pc_ = pc;
	}

	bool Preprocessor::readFrame(){
        if( lasers_.empty() ){
        	return false;
        }
        for( const velodyne::Laser& laser : lasers_ ){
            // Distance unit: mm?
            const double distance = static_cast<double>( laser.distance )*2;
            const double azimuth  = laser.azimuth  * CV_PI / 180.0;
            const double vertical = laser.vertical * CV_PI / 180.0;

            double x = static_cast<double>( ( distance * std::cos( vertical ) ) * std::sin( azimuth ) );
            double y = static_cast<double>( ( distance * std::cos( vertical ) ) * std::cos( azimuth ) );
            double z = static_cast<double>( ( distance * std::sin( vertical ) ) );

            rimg[azimuth][vertical] = distance;
            rimg[azimuth][-0.6] = 4340;	// The smallest vertical radian = -30.67*pi/180 = -0.53
            rmmap[azimuth][vertical] = 0;
            rmmap[azimuth][-0.6] = 1;
        }
        return true;
	}

	void Preprocessor::removeGround(){
		for(auto& col: rimg){
	        bool lost_pt = false;
	        bool set_th_pt = false;
	        bool prev_is_ground = true;
	        double vert_prev = -0.6;	
	        double x_0 = (-2450/tan(vert_prev))*cos(col.first);
	        double y_0 = (-2450/tan(vert_prev))*sin(col.first);
	        double z_0 = -2450;
	        Vector3f p_prev(x_0, y_0, z_0);
	        Vector3f p_th(x_0, y_0, z_0);

	        for(auto& vert: col.second){
	            double x = vert.second*cos(vert.first)*sin(col.first);
	            double y = vert.second*cos(vert.first)*cos(col.first);
	            double z = vert.second*sin(vert.first);
	            Vector3f p_curr(x, y, z);

	            double grad = asin((p_curr[2]-p_prev[2])/(p_curr-p_prev).norm())*180/CV_PI;

	            // Set threshold point
	            if(prev_is_ground && 
	                (grad > grad_th || vert.second == 0 || vert.second < p_prev.norm())){
	                set_th_pt = true;
	                p_th = p_prev;
	            }

	            if(prev_is_ground){    // Previous point is ground point
	                if(grad < grad_th && !lost_pt){
	                    rmmap[col.first][vert.first] = 1;
	                    prev_is_ground = true;
	                }
	                else{   // Previous point is a threshold point
	                    rmmap[col.first][vert.first] = 0;
	                    prev_is_ground = false;
	                }
	            }
	            // Lower ground condition
	            else if(!prev_is_ground && p_curr[2] < lowpt_th && grad < grad_th){
	                rmmap[col.first][vert.first] = 1;
	                prev_is_ground = true;
	                set_th_pt = false;
	            }

	            if(vert.second == 0){   // Current point is a lost point
	                rmmap[col.first][vert.first] = 1;
	                lost_pt = true;
	                prev_is_ground = false;
	            }
	            else{
	                lost_pt = false;
	            }

	            if(vert.second < p_prev.norm() && vert.second != 0){
	                rmmap[col.first][vert.first] = 0;
	                prev_is_ground = false;
	            }

	            // Set start point
	            if(set_th_pt && (p_curr[2]-p_th[2]) < height_th && p_curr[2] < p_prev[2]){
	                set_th_pt = false;
	                rmmap[col.first][vert.first] = 1;
	                prev_is_ground = true;
	            }

	            // Remove self-car
	            if(x <= 820 && x >= -820 && 
	                y <= 1300 && y >= -1800 && 
	                z <= 100 && z >= -2000){
	                rmmap[col.first][vert.first] = 2;
	                // continue;
	            }

	            p_prev = p_curr;
	            vert_prev = vert.first;
	        }
	    }
	}

	void Preprocessor::removeOccluded(){
		// Occluded edge points
        for(auto& vert: vertAngle_){
            double v = vert*CV_PI/180.0;
            double prev_hor;
            bool isFirst = true;
            for(auto& col: rimg){
                if(isFirst){
                    prev_hor = col.first;
                    isFirst = false;
                }
                else if(rimg[col.first][v] == 0){   // Lost point
                    continue;
                }
                else{
                    double d_dist = rimg[col.first][v] - rimg[prev_hor][v];
                    double d_hor = col.first - prev_hor;
                    if(abs(d_dist) > dist_th && abs(d_hor) < angdiff_th){   // is a occluded point
                        if(d_dist > 0){ // Current point is background pt
                            if(rmmap[col.first][v] == 0)
                                rmmap[col.first][v] = 3;
                        }
                        else{   // Previous point is background pt
                            if(rmmap[prev_hor][v] == 0)
                                rmmap[prev_hor][v] = 3;
                        }
                    }
                    prev_hor = col.first;
                }
            }
        }
	}

	void Preprocessor::writePointCloud(){
		for(auto& col: rimg){
            for(auto& vert: col.second){
                if(vert.second == 0 || vert.first == -0.6)
                    continue;
                double x = vert.second*cos(vert.first)*sin(col.first);
                double y = vert.second*cos(vert.first)*cos(col.first);
                double z = vert.second*sin(vert.first);
                Vector3f pt(x, y, z);
                if(rmmap[col.first][vert.first] == 0){
                	pc_->push_back(pt);
                }
            }
        }	
	}

	void Preprocessor::run(){
		pc_->clear();
		rimg.clear();
		rmmap.clear();
		while(!readFrame()){};
		removeGround();
		removeOccluded();
		writePointCloud();
	}
}