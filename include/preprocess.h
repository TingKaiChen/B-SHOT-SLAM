#ifndef PREPROCESS_H
#define PREPROCESS_H

#include "common_include.h"
#include "VelodyneCapture.h"

namespace myslam{
	class Preprocessor{
		public:
			typedef shared_ptr<Preprocessor> Ptr;
			typedef map<double, double> RangeImgCol;
			typedef map<double, RangeImgCol> RangeImg;
			typedef map<double, int> RemoveCol;
			typedef map<double, RemoveCol> RemoveMap;


			Preprocessor();
			Preprocessor(vector<velodyne::Laser>& lasers, vector<double>& vertAngle, shared_ptr<vector<Vector3f>> pc);

			void setLasers(vector<velodyne::Laser>& lasers);
			void setVerticalAngles(vector<double>& vertAngle);
			void setPointCloud(shared_ptr<vector<Vector3f>> pc);
			bool readFrame();		// Read in a frame
			void removeGround();	// Remove ground points
			void removeOccluded();	// Remove occluded points
			void writePointCloud(); 
			void run();
			inline RangeImg getRangeImage(){return rimg;};
			inline RemoveMap getRemoveMap(){return rmmap;};

		private:
			vector<double> vertAngle_;
            double grad_th = 45;	// Degree
            double height_th = 250;
            double dist_th = 1500;
            double angdiff_th = 1.0*CV_PI/180.0;	// Radian

            RangeImg rimg;
            RemoveMap rmmap;

            shared_ptr<vector<Vector3f>> pc_;
            vector<velodyne::Laser> lasers_;
	};
}

#endif