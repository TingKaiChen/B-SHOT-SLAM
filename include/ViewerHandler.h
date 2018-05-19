#ifndef VIEWERHANDLER_H
#define VIEWERHANDLER_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

typedef cv::viz::Viz3d VizViewer;
typedef cv::Affine3d ATrans;
typedef cv::Affine3d::Mat3 AMat3;
typedef cv::Affine3d::Mat4 AMat4;
typedef cv::Affine3d::Vec3 AVec3;

enum MODE{STOP, NEXT, SELECT1, SELECT2};

class InputHandler{
public:
    InputHandler(VizViewer* v, MODE m):
        viewer(v), mode(m), pov(TOP), isFirstSelection(true),
        max_z(0), min_z(0), hasTrimSelected(false), isFirstSave(true){
        	saveIdxs.clear();
        	outFilename = "test.txt";
        };
    void setZRange(double zmin, double zmax){min_z = zmin; max_z = zmax;};
    void setPointClouds(std::vector<cv::Vec3f>* s, std::vector<cv::Vec3f>* t, 
    	std::vector<cv::Vec3f>* save){
    	source = s;
    	trim = t;
    	savedpc = save;
    };
    void setMode(MODE m){mode = m;};
    MODE getMode(){return mode;};
    void setFirstSelection(bool b){isFirstSelection = b;};
    bool getFisrtSelection(){return isFirstSelection;};
    bool isSelected(){return !sPoints.empty();};
    bool rangeDefined(){return cube_vertices.size() >=2;};
    bool isExist(int idx);
    void seperateByRange();
    bool trimEmpty(){return trim->empty();};
    void saveTrimPC();
    std::vector<cv::Vec3f>* getSavedPC(){return savedpc;};
    void saveSelectedPT(){
    	if(isFirstSave){
	    	saveIdxs.push_back(framePtIdx);
	    	isFirstSave = false;
	    }
	    else{
	    	saveIdxs.back() = framePtIdx;
	    }
    };
    void resetSelectedPT(){
    	if(!framePtIdx.empty()){
	        viewer->removeWidget("Cloud_saved");
	    }
	    savedpc->clear();
    	tempIdx.clear(); 
    	framePtIdx.clear();
    	isFirstSave = true;
    };
    void saveIdxOutFilename(std::string fn){outFilename = fn;};
    void savePtIdxs();
    void printProjMat(){
        cv::Matx44d m;
        viewer->getCamera().computeProjectionMatrix(m);
        std::cout<<m<<std::endl;
    };
    void setClickPt(cv::Point p);
    void printSelectingRange(cv::Point p);
    void clearSelPoints();
    void closeWindow(){viewer->close();};
    void printViewerPose();
    enum PoV{TOP, LEFT, BACK, RIGHT, FRONT};
    void rotateViewerPose();
    void resetViewerPose(){
        viewer->setViewerPose(AMat4(1,0,0,0,0,-1,0,0,0,0,-1,220000,0,0,0,1));
        pov = TOP;
    };
    void spinOnce(){viewer->spinOnce();};
private:
    VizViewer* viewer;
    MODE mode;
    PoV pov;
    bool isFirstSelection;
    bool hasTrimSelected;
    bool isFirstSave;
    std::vector<cv::Point3d> sPoints;
    std::vector<cv::Point3d> cube_vertices;	// (min, max, min, max)
    double max_z;
    double min_z;
    std::vector<cv::Vec3f> *source, *trim, *savedpc;
    std::ifstream ifs;
    std::ofstream ofs;
    std::stringstream ss;
    std::string outFilename;
    std::vector<std::vector<int>> saveIdxs;
    std::vector<int> framePtIdx;
    std::vector<int> tempIdx;
};

#endif