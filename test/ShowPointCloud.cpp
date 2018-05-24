// Include MySLAM
#include "common_include.h"
#include "VelodyneCapture.h"
#include "frame.h"

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/core/eigen.hpp>


typedef cv::viz::Viz3d VizViewer;

void KeyboardCallback(const cv::viz::KeyboardEvent&, void*);

bool isStop = false;

// ./ptpicking source.pcap outputIDX.txt inputIDX.txt
int main( int argc, char* argv[] )
{
    // Open VelodyneCapture that retrieve from PCAP
    velodyne::HDL32ECapture capture( argv[1], 0 );

    if( !capture.isOpen() ){
        std::cerr << "Can't open VelodyneCapture." << std::endl;
        return -1;
    }

    // Create Viewer
    VizViewer viewer( "Velodyne" );

    // Register Callback
    viewer.registerKeyboardCallback(KeyboardCallback, &viewer);

    // Frame sequence
    vector<myslam::Frame::Ptr> FrameSequence;

    while( capture.isRun() && !viewer.wasStopped() ){
        if(!isStop){
            // Create a pointcloud smart pointer
            myslam::Frame::PCPtr pc = make_shared<vector<Vector3d>>();
            // Capture One Rotation Data
            std::vector<velodyne::Laser> lasers;
            capture >> lasers;
            if( lasers.empty() ){
                continue;
            }

            // Convert to 3-dimention Coordinates
            std::vector<cv::Vec3d> buffer;
            buffer.resize( lasers.size() );
            pc->resize( lasers.size() );
            for( const velodyne::Laser& laser : lasers ){
                const double distance = static_cast<double>( laser.distance );
                const double azimuth  = laser.azimuth  * CV_PI / 180.0;
                const double vertical = laser.vertical * CV_PI / 180.0;

                double x = static_cast<double>( ( distance * std::cos( vertical ) ) * std::sin( azimuth ) );
                double y = static_cast<double>( ( distance * std::cos( vertical ) ) * std::cos( azimuth ) );
                double z = static_cast<double>( ( distance * std::sin( vertical ) ) );

                if( x == 0.0 && y == 0.0 && z == 0.0 ){
                    x = std::numeric_limits<double>::quiet_NaN();
                    y = std::numeric_limits<double>::quiet_NaN();
                    z = std::numeric_limits<double>::quiet_NaN();
                }

                // Add a point into the point cloud
                pc->push_back( Vector3d( x, y, z ) );
                // Convert a point from Eigen type to OpenCV type
                Vector3d vs = pc->back(); 
                cv::Mat v;
                cv::eigen2cv(Vector3d( x, y, z ), v);
                buffer.push_back( v );
            }

            // Create a frame
            myslam::Frame::Ptr fptr = myslam::Frame::createFrame();
            fptr->setPointCloud(pc);
            FrameSequence.push_back(fptr);
            // Create Widget
            cv::Mat cloudMat = cv::Mat( static_cast<int>( buffer.size() ), 1, CV_64FC3, &buffer[0] );
            // cv::Mat cloudMat = cv::Mat( static_cast<int>( pc->size() ), 1, CV_64FC3, &(pc->at(0)) );
            // cv::Mat cloudMat = cv::Mat( static_cast<int>( pc->size() ), 1, CV_64FC3, &((FrameSequence.back()->pointcloud_)->at(0)) );
            // cout<<cloudMat.size[0]<<" "<<pc->size()<<endl;
            cv::viz::WCloud cloud( cloudMat );
            // Show Point Cloud
            viewer.showWidget( "Cloud", cloud );
        // isStop = true;
        }
        viewer.spinOnce();
    }

    // Close All Viewers
    cv::viz::unregisterAllWindows();

    return 0;
}

void KeyboardCallback(const cv::viz::KeyboardEvent& event, void* v){
    VizViewer* viewer = (VizViewer*) v;
    // Stop/continue
    if(event.code == 's' && event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN){
        if(isStop){
            isStop = false;
            std::cout<<"Mode: Run"<<std::endl;
        }
        else if(!isStop){
            isStop = true;
            std::cout<<"Mode: STOP"<<std::endl;
        }
    }
    // Close Viewer
    else if( event.code == 'q' && event.action == cv::viz::KeyboardEvent::Action::KEY_DOWN ){
        viewer->close();
    }
};
