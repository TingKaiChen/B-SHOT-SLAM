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

    std::vector<cv::Vec3f> buffer;
    while( capture.isRun() && !viewer.wasStopped() ){
        if(!isStop){
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
            buffer.resize( lasers.size() );
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

            // Create Widget
            cv::Mat cloudMat = cv::Mat( static_cast<int>( buffer.size() ), 1, CV_32FC3, &buffer[0] );
            cv::viz::WCloud cloud( cloudMat );
            // Show Point Cloud
            viewer.showWidget( "Cloud", cloud );
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
