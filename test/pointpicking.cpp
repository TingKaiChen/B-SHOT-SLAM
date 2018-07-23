#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

// Include VelodyneCapture Header
#include "VelodyneCapture.h"
#include "ViewerHandler.h"

typedef cv::viz::Viz3d VizViewer;
typedef cv::Affine3d ATrans;
typedef cv::Affine3d::Mat3 AMat3;
typedef cv::Affine3d::Mat4 AMat4;
typedef cv::Affine3d::Vec3 AVec3;

void KeyboardCallback(const cv::viz::KeyboardEvent&, void*);
void MouseCallback(const cv::viz::MouseEvent&, void*);

// ./ptpicking source.pcap outputIDX.txt inputIDX.txt
int main( int argc, char* argv[] )
{
    // Parameters
    int Start_Frame = 280;
    int frame_id = Start_Frame;
    bool hasSelectedPTs = false;

    std::ifstream ifs;
    if(argc >= 4)
        ifs.open(argv[3]);

    // Open VelodyneCapture that retrieve from PCAP
    velodyne::HDL32ECapture capture( argv[1], Start_Frame );

    if( !capture.isOpen() ){
        std::cerr << "Can't open VelodyneCapture." << std::endl;
        return -1;
    }

    // Create Viewer
    VizViewer viewer( "Velodyne" );

    // Register Callback
    InputHandler handler(&viewer, NEXT);
    if(argc >= 3)
        handler.saveIdxOutFilename(argv[2]);
    viewer.registerKeyboardCallback(KeyboardCallback, &handler);
    viewer.registerMouseCallback(MouseCallback, &handler);

    std::vector<cv::Vec3f> buffer, trimpoints, savedPC;
    handler.setPointClouds(&buffer, &trimpoints, &savedPC);
    while( capture.isRun() && !viewer.wasStopped() ){
        if(handler.getMode() == NEXT){
            buffer.clear();
            // Capture One Rotation Data
            std::vector<velodyne::Laser> lasers;
            capture >> lasers;
            if( lasers.empty() ){
                continue;
            }

            double max_z = 0;
            double min_z = 0;
            // Convert to 3-dimention Coordinates
            buffer.reserve( lasers.size() );
            for( const velodyne::Laser& laser : lasers ){
                const double distance = static_cast<double>( laser.distance );
                const double azimuth  = laser.azimuth  * CV_PI / 180.0;
                const double vertical = laser.vertical * CV_PI / 180.0;

                float x = static_cast<float>( ( distance * std::cos( vertical ) ) * std::sin( azimuth ) );
                float y = static_cast<float>( ( distance * std::cos( vertical ) ) * std::cos( azimuth ) );
                float z = static_cast<float>( ( distance * std::sin( vertical ) ) );

                if( x == 0.0f && y == 0.0f && z == 0.0f ){
                    x = std::numeric_limits<float>::quiet_NaN();
                    y = std::numeric_limits<float>::quiet_NaN();
                    z = std::numeric_limits<float>::quiet_NaN();
                }

                buffer.push_back( cv::Vec3f( x, y, z ) );
                max_z = std::max(max_z, double(z));
                min_z = std::min(min_z, double(z));
            }
            handler.setZRange(min_z, max_z);

            // Create Widget
            cv::Mat cloudMat = cv::Mat( static_cast<int>( buffer.size() ), 1, CV_32FC3, &buffer[0] );
            cv::viz::WCloud cloud( cloudMat );
            // Show Point Cloud
            viewer.showWidget( "Cloud", cloud );
            handler.setMode(STOP);

            // Clear previous loaded selected points
            if(hasSelectedPTs){
                viewer.removeWidget("Cloudidx");
                hasSelectedPTs = false;
            }
            // Show selected points
            std::string line;
            int idx;
            if(ifs.is_open() && std::getline(ifs, line)){
                std::stringstream ss(line);
                std::vector<cv::Vec3f> buffer2;
                while(ss>>idx){
                    buffer2.push_back(buffer[idx]);
                }
                // Create Widget
                if(!buffer2.empty()){
                    cv::Mat idxMat = cv::Mat( static_cast<int>( buffer2.size() ), 1, CV_32FC3, &buffer2[0] );
                    cv::viz::WCloud cloudidx( idxMat, cv::viz::Color::yellow() );
                    cloudidx.setRenderingProperty(cv::viz::POINT_SIZE, 4);

                    // Show Point Cloud
                    viewer.showWidget( "Cloudidx", cloudidx );
                    hasSelectedPTs = true;
                }
                else
                    hasSelectedPTs = false;
            }

            std::cout<<"Frame:\t#"<<(frame_id++)<<std::endl;
        }
        viewer.spinOnce();
    }

    // Close All Viewers
    cv::viz::unregisterAllWindows();

    return 0;
}

void KeyboardCallback(const cv::viz::KeyboardEvent& event, void* h){
    InputHandler* handler = (InputHandler*) h;
    // Next frame
    if( event.code == 'n' && event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN ){
        handler->saveSelectedPT();
        handler->setMode(NEXT);
        handler->clearSelPoints();
        handler->resetSelectedPT();
        // handler->resetViewerPose();
    }
    // Enter/Exist selecting mode
    else if(event.code == 's' && event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN){
        if(handler->getMode() == STOP){
            handler->setMode(SELECT1);
            if(handler->isSelected())
                handler->clearSelPoints();
            handler->setFirstSelection(true);
            std::cout<<"Mode: SELECT1"<<std::endl;
        }
        else if(handler->getMode() != STOP){
            handler->setMode(STOP);
            std::cout<<"Mode: STOP"<<std::endl;
        }
    }
    // Save point index file
    else if(event.code == 'S' && event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN){
        handler->saveSelectedPT();
        handler->savePtIdxs();
        std::cout<<"Successfully saved"<<std::endl;
    }
    // Show informaion
    else if(event.code == 'i' && event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN){
        handler->printViewerPose();
        // handler->printProjMat();
    }
    // Rotate point of view
    else if(event.code == 'v' && event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN){
        handler->rotateViewerPose();
    }
    // Press "Enter": save point clouds
    else if(event.code == char(13) && event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN){
        handler->saveTrimPC();
    }
    // Save &close Viewer
    else if( event.code == 'e' && event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN ){
        handler->saveSelectedPT();
        handler->savePtIdxs();
        std::cout<<"Successfully saved"<<std::endl;
        handler->closeWindow();
    }
    // Close Viewer
    else if( event.code == 'q' && event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN ){
        handler->closeWindow();
    }
};

void MouseCallback(const cv::viz::MouseEvent& event, void* h){
    InputHandler* handler = (InputHandler*) h;
    // Next frame
    if( event.button == cv::viz::MouseEvent::LeftButton && 
        event.type == cv::viz::MouseEvent::MouseButtonPress &&
        event.modifiers == cv::viz::KeyboardEvent::ALT+cv::viz::KeyboardEvent::CTRL){
        std::cout<<event.pointer<<std::endl;
        std::cout<<event.modifiers<<std::endl;
        // handler->setClickPt(event.pointer);
    }
    else if( event.type == cv::viz::MouseEvent::MouseMove &&
        (handler->getMode() == SELECT1 || handler->getMode() == SELECT2) &&
        handler->getFisrtSelection() == false){
        handler->printSelectingRange(event.pointer);
    }
    else if( event.button == cv::viz::MouseEvent::LeftButton && 
        event.type == cv::viz::MouseEvent::MouseButtonPress &&
        !event.modifiers &&
        (handler->getMode() == SELECT1 || handler->getMode() == SELECT2)){
            handler->setClickPt(event.pointer);
    }
};
