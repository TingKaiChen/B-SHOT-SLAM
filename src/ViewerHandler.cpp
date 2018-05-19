#include "ViewerHandler.h"
bool InputHandler::isExist(int idx){
    if( std::binary_search(framePtIdx.begin(), framePtIdx.end(), idx) ||
        std::binary_search(tempIdx.begin(), tempIdx.end(), idx))
        return true;
    else
        return false;
};

void InputHandler::seperateByRange(){
    int id_min, id_max;
    if(cube_vertices.size() == 4){
        id_min = 2;
        id_max = 3;
    }
    else if(cube_vertices.size() >=2 && cube_vertices.size() != 4){
        id_min = 0;
        id_max = 1;
    }
    tempIdx.clear();
    trim->clear();
    for(int i = 0; i < source->size(); i++){
        if( (*source)[i][0] > cube_vertices[id_min].x &&
            (*source)[i][0] < cube_vertices[id_max].x &&
            (*source)[i][1] > cube_vertices[id_min].y &&
            (*source)[i][1] < cube_vertices[id_max].y &&
            (*source)[i][2] > cube_vertices[id_min].z &&
            (*source)[i][2] < cube_vertices[id_max].z && !isExist(i)){
            trim->push_back((*source)[i]);
            tempIdx.push_back(i);
        }
    }
    std::sort(tempIdx.begin(), tempIdx.end());
};

void InputHandler::saveTrimPC(){
    savedpc->reserve(savedpc->size()+trim->size());
    savedpc->insert(savedpc->end(), trim->begin(), trim->end());
    if(!savedpc->empty()){
        cv::Mat savedMat = cv::Mat( static_cast<int>( 
            savedpc->size() ), 1, CV_32FC3, &(*savedpc)[0] );
        cv::viz::WCloud cloudSaved( savedMat, cv::viz::Color::yellow());
        cloudSaved.setRenderingProperty(cv::viz::POINT_SIZE, 4);
        viewer->showWidget( "Cloud_saved", cloudSaved );

        // Save selected index to framePtIdx
        framePtIdx.reserve(framePtIdx.size()+tempIdx.size());
        framePtIdx.insert(framePtIdx.end(), tempIdx.begin(), tempIdx.end());
    }
    std::sort(framePtIdx.begin(), framePtIdx.end());
};

void InputHandler::savePtIdxs(){
    ofs.open(outFilename);
    for(int i = 0; i < saveIdxs.size(); i++){
        for(int j = 0; j < saveIdxs[i].size(); j++){
            ofs<<saveIdxs[i][j]<<' ';
        }
        ofs<<std::endl;
    }
    ofs.close();
};


void InputHandler::setClickPt(cv::Point p){
    cv::viz::Camera cam = viewer->getCamera();
    cv::Matx33d InMat(
        cam.getFocalLength()[0], 0, cam.getPrincipalPoint()[0],
        0, -cam.getFocalLength()[1], cam.getPrincipalPoint()[1],
        0, 0, 1);
    cv::Matx44d exMat = viewer->getViewerPose().matrix;
    double scale;
    if(sPoints.size() < 2 )
        scale = viewer->getViewerPose().translation()[2];
    else if (sPoints.size() >= 2){
        cv::Point3d exCenter = cv::Point3d(
            (sPoints[0].x+sPoints[1].x)/2, 
            (sPoints[0].y+sPoints[1].y)/2, 
            (sPoints[0].z+sPoints[1].z)/2);
        // std::cout<<exCenter<<std::endl;
        if(pov == LEFT || pov == RIGHT)
            scale = abs(viewer->getViewerPose().translation()[0]-exCenter.x);
        else if (pov == BACK || pov == FRONT)
            scale = abs(viewer->getViewerPose().translation()[1]-exCenter.y);
    }

    cv::Vec3d pt_c = scale*InMat.inv()*cv::Vec3d(p.x, p.y, 1);
    cv::Vec4d pt_w = exMat*cv::Vec4d(pt_c[0], pt_c[1], pt_c[2], 1);
    cv::Point3d center(pt_w[0], pt_w[1], pt_w[2]);
    cv::viz::WSphere VP(center, 10, 10, cv::viz::Color::red());
    // Normal vector & new y-axis
    cv::Vec3d nVec, yVec;
    if(pov == TOP){
        nVec = cv::Vec3d(0, 0, 1);
        yVec = cv::Vec3d(0, 1, 0);
    }
    else if (pov == LEFT || pov == RIGHT){
        nVec = cv::Vec3d(1, 0, 0);
        yVec = cv::Vec3d(0, 0, 1);
    }
    else if(pov == BACK || pov == FRONT){
        nVec = cv::Vec3d(0, 1, 0);
        yVec = cv::Vec3d(0, 0, 1);
    }

    if(isFirstSelection){
        isFirstSelection = false;
    }
    else{
        int selectIdx;
        if(mode == SELECT1)
            selectIdx = 0;
        else if(mode == SELECT2)
            selectIdx = 2;
        cv::Point3d ct(
            (center.x+sPoints[selectIdx].x)/2, 
            (center.y+sPoints[selectIdx].y)/2, 
            (center.z+sPoints[selectIdx].z)/2);
        // Calculate the size of selecting range
        cv::Size2d sz;
        if(pov == TOP){
            sz = cv::Size2d(abs(center.x-sPoints[selectIdx].x), abs(center.y-sPoints[selectIdx].y));
        }
        else if (pov == LEFT || pov == RIGHT){
            sz = cv::Size2d(abs(center.y-sPoints[selectIdx].y), abs(center.z-sPoints[selectIdx].z));
        }
        else if(pov == BACK || pov == FRONT){
            sz = cv::Size2d(abs(center.x-sPoints[selectIdx].x), abs(center.z-sPoints[selectIdx].z));
        }
        // Prevent size from 0 value
        if(sz.width == 0)
            sz.width = 0.01;
        if(sz.height == 0)
            sz.height = 0.01;
        if(mode == SELECT1){
        	cv::Point3d min_point(
        		std::min(center.x, sPoints[selectIdx].x), 
        		std::min(center.y, sPoints[selectIdx].y), min_z);
        	cv::Point3d max_point(
        		std::max(center.x, sPoints[selectIdx].x), 
        		std::max(center.y, sPoints[selectIdx].y), max_z);
        	cube_vertices.push_back(min_point);
        	cube_vertices.push_back(max_point);
        	cv::viz::WCube cube1(min_point, max_point, false, cv::viz::Color::green());
            cube1.setRenderingProperty(cv::viz::OPACITY, 0.3);
            viewer->showWidget("Cube1", cube1);
            viewer->removeWidget("CPL1");
            setMode(SELECT2);
            std::cout<<"Mode: SELECT2"<<std::endl;
        }
        else if(mode == SELECT2){
        	cv::Point3d min_point, max_point;
        	if(pov == LEFT || pov == RIGHT){
        		min_point = cv::Point3d(
        		cube_vertices[0].x, 
        		std::max(std::min(center.y, sPoints[selectIdx].y), cube_vertices[0].y),
        		std::min(center.z, sPoints[selectIdx].z));
            	max_point = cv::Point3d(
        		cube_vertices[1].x, 
        		std::min(std::max(center.y, sPoints[selectIdx].y), cube_vertices[1].y),
        		std::max(center.z, sPoints[selectIdx].z));
        	}
        	else if(pov == FRONT || pov == BACK){
        		min_point = cv::Point3d(
        		std::max(std::min(center.x, sPoints[selectIdx].x), cube_vertices[0].x),
        		cube_vertices[0].y, 
        		std::min(center.z, sPoints[selectIdx].z));
            	max_point = cv::Point3d(
        		std::min(std::max(center.x, sPoints[selectIdx].x), cube_vertices[1].x),
        		cube_vertices[1].y, 
        		std::max(center.z, sPoints[selectIdx].z));
        	}
        	cube_vertices.push_back(min_point);
        	cube_vertices.push_back(max_point);
        	cv::viz::WCube cube1(min_point, max_point, false, cv::viz::Color::green());
            cube1.setRenderingProperty(cv::viz::OPACITY, 0.3);
            viewer->showWidget("Cube1", cube1);
            viewer->removeWidget("CPL2");
            setMode(STOP);
            std::cout<<"Mode: STOP"<<std::endl;
        }
        isFirstSelection = true;
        seperateByRange();
        if(hasTrimSelected){
            viewer->removeWidget("Cloud_trim");
            hasTrimSelected = false;
        }
        if(!trimEmpty()){
            cv::Mat trimMat = cv::Mat( static_cast<int>( 
                trim->size() ), 1, CV_32FC3, &(*trim)[0] );
            cv::viz::WCloud cloudTrim( trimMat, cv::viz::Color::red());
            cloudTrim.setRenderingProperty(cv::viz::POINT_SIZE, 4);
            viewer->showWidget( "Cloud_trim", cloudTrim );
            hasTrimSelected = true;
        }
    }
    sPoints.push_back(center);
}

void InputHandler::printSelectingRange(cv::Point p){
    cv::viz::Camera cam = viewer->getCamera();
    cv::Matx33d InMat(
        cam.getFocalLength()[0], 0, cam.getPrincipalPoint()[0],
        0, -cam.getFocalLength()[1], cam.getPrincipalPoint()[1],
        0, 0, 1);
    cv::Matx44d exMat = viewer->getViewerPose().matrix;
    double scale;
    if(sPoints.size() < 2 )
        scale = viewer->getViewerPose().translation()[2];
    else if (sPoints.size() >= 2){
        cv::Point3d exCenter = cv::Point3d(
            (sPoints[0].x+sPoints[1].x)/2, 
            (sPoints[0].y+sPoints[1].y)/2, 
            (sPoints[0].z+sPoints[1].z)/2);
        // std::cout<<exCenter<<std::endl;
        if(pov == LEFT || pov == RIGHT)
            scale = abs(viewer->getViewerPose().translation()[0]-exCenter.x);
        else if (pov == BACK || pov == FRONT)
            scale = abs(viewer->getViewerPose().translation()[1]-exCenter.y);
    }
    cv::Vec3d pt_c = scale*InMat.inv()*cv::Vec3d(p.x, p.y, 1);
    cv::Vec4d pt_w = exMat*cv::Vec4d(pt_c[0], pt_c[1], pt_c[2], 1);
    cv::Point3d center(pt_w[0], pt_w[1], pt_w[2]);
    cv::viz::WSphere VP(center, 10, 10, cv::viz::Color::red());
    int selectIdx;
    if(mode == SELECT1)
        selectIdx = 0;
    else if(mode == SELECT2)
        selectIdx = 2;
    cv::Point3d ct(
        (center.x+sPoints[selectIdx].x)/2, 
        (center.y+sPoints[selectIdx].y)/2, 
        (center.z+sPoints[selectIdx].z)/2);
    // Calculate the size of selecting range
    cv::Size2d sz;
    if(pov == TOP){
        sz = cv::Size2d(abs(center.x-sPoints[selectIdx].x), abs(center.y-sPoints[selectIdx].y));
    }
    else if (pov == LEFT || pov == RIGHT){
        sz = cv::Size2d(abs(center.y-sPoints[selectIdx].y), abs(center.z-sPoints[selectIdx].z));
    }
    else if(pov == BACK || pov == FRONT){
        sz = cv::Size2d(abs(center.x-sPoints[selectIdx].x), abs(center.z-sPoints[selectIdx].z));
    }
    // Prevent size from 0 value
    if(sz.width == 0)
        sz.width = 0.01;
    if(sz.height == 0)
        sz.height = 0.01;
    // Normal vector & new y-axis
    cv::Vec3d nVec, yVec;
    if(pov == TOP){
        nVec = cv::Vec3d(0, 0, 1);
        yVec = cv::Vec3d(0, 1, 0);
    }
    else if (pov == LEFT || pov == RIGHT){
        nVec = cv::Vec3d(1, 0, 0);
        yVec = cv::Vec3d(0, 0, 1);
    }
    else if(pov == BACK || pov == FRONT){
        nVec = cv::Vec3d(0, 1, 0);
        yVec = cv::Vec3d(0, 0, 1);
    }
    cv::viz::WPlane VPlane(ct, nVec, yVec, sz, cv::viz::Color::red());          
    VPlane.setRenderingProperty(cv::viz::OPACITY, 0.3);
    if(mode == SELECT1)
        viewer->showWidget("CPL1", VPlane);
    else if(mode == SELECT2)
        viewer->showWidget("CPL2", VPlane);
}

void InputHandler::clearSelPoints(){
	if(sPoints.size() > 2 && !isFirstSelection)
        viewer->removeWidget("CPL2");
    else if(sPoints.size() > 0 && !isFirstSelection)
        viewer->removeWidget("CPL1");
    if(cube_vertices.size() >= 2)
    	viewer->removeWidget("Cube1");
    if(!trimEmpty()){
        trim->clear();
        viewer->removeWidget("Cloud_trim");
        hasTrimSelected = false;
    }
    sPoints.clear();
    cube_vertices.clear();
    tempIdx.clear();
};

void InputHandler::printViewerPose(){
    std::cout<<(viewer->getViewerPose().matrix)<<std::endl;
    std::cout<<"Clip: "<<viewer->getCamera().getClip()<<std::endl;
    std::cout<<"Focal length: "<<viewer->getCamera().getFocalLength()<<std::endl;
    std::cout<<"Fov: "<<viewer->getCamera().getFov()<<std::endl;
    std::cout<<"Principal point: "<<viewer->getCamera().getPrincipalPoint()<<std::endl;
    std::cout<<"Window size: "<<viewer->getCamera().getWindowSize()<<std::endl;
};

void InputHandler::rotateViewerPose(){
    AVec3 T(0, 0, 0);
    if(mode == SELECT2){
        T = cv::Vec3d(
            (sPoints[0].x+sPoints[1].x)/2, 
            (sPoints[0].y+sPoints[1].y)/2, 
            (sPoints[0].z+sPoints[1].z)/2);
    }
    switch(pov){
        case TOP:{
            AMat3 R(0,0,1,-1,0,0,0,-1,0);
            viewer->setViewerPose(ATrans(R, T));
            std::cout<<"PoV: Left"<<std::endl;
            pov = PoV((pov+1)%5);
            break;
        }
        case LEFT:{
            AMat3 R(1,0,0,0,0,1,0,-1,0);
            viewer->setViewerPose(ATrans(R, T));
            std::cout<<"PoV: Back"<<std::endl;
            pov = PoV((pov+1)%5);
            break;    
        }
        case BACK:{
            AMat3 R(0,0,-1,1,0,0,0,-1,0);
            viewer->setViewerPose(ATrans(R, T));
            std::cout<<"PoV: Right"<<std::endl;
            pov = PoV((pov+1)%5);
            break;
        }
        case RIGHT:{
            AMat3 R(-1,0,0,0,0,-1,0,-1,0);
            viewer->setViewerPose(ATrans(R, T));
            std::cout<<"PoV: Front"<<std::endl;
            pov = PoV((pov+1)%5);
            break;
        }
        case FRONT:{
            AMat3 R(1,0,0,0,-1,0,0,0,-1);
            viewer->setViewerPose(ATrans(R, T));
            std::cout<<"PoV: Top"<<std::endl;
            pov = PoV((pov+1)%5);
            break;
        }
    }
}
