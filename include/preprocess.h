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
			typedef map<double, bool> SelCol;
			typedef map<double, SelCol> SelMap;


			Preprocessor();
			Preprocessor(vector<velodyne::Laser>& lasers, vector<double>& vertAngle, shared_ptr<vector<Vector3f>> pc);

			void setLasers(vector<velodyne::Laser>& lasers);
			void setSelectedPoints(vector<int>& selptlist);
			void saveSelectPoints(bool savesel){save_sel_ = savesel;};
			void haveSelectList(bool havesellist){have_sel_list_ = havesellist;};
			void setVerticalAngles(vector<double>& vertAngle);
			void setVerticalInitial(double vertinit){vert_init_ = vertinit;};
			void setLowPtThreshold(double lowptth){lowpt_th = lowptth;};
			void setPointCloud(shared_ptr<vector<Vector3f>> pc);
			bool readFrame();		// Read in a frame
			void removeGround();	// Remove ground points
			void removeOccluded();	// Remove occluded points
			void writePointCloud(); 
			void run();
			inline RangeImg getRangeImage(){return rimg;};
			inline RemoveMap getRemoveMap(){return rmmap;};
			inline SelMap getSelMap(){return selmap;};

		private:
			vector<double> vertAngle_;
			double vert_init_;	// Radian
            double grad_th = 45;	// Degree
            double lowpt_th = -2000;	
            double height_th = 500;
            double dist_th = 3000;
            double angdiff_th = 1.0*CV_PI/180.0;	// Radian

            RangeImg rimg;
            RemoveMap rmmap;
            SelMap selmap;

            shared_ptr<vector<Vector3f>> pc_;
            vector<velodyne::Laser> lasers_;
            vector<int> selpts_;
            bool save_sel_;
            bool have_sel_list_;
	};
}

#endif