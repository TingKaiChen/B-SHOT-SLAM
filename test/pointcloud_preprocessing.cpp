// Include MySLAM
#include "common_include.h"
#include "VelodyneCapture.h"
#include "frame.h"
#include "lidar_odometry.h"
#include "preprocess.h"

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
typedef unordered_map<unsigned long, vector<Vector3f> > Grid;
typedef unordered_map<unsigned long, Grid> GridMap;
typedef unordered_map<unsigned long, int> ColGroundVal;
typedef map<double, double> RangeImgCol;
typedef map<double, RangeImgCol> RangeImg;
typedef map<double, int> RemoveCol;
typedef map<double, RemoveCol> RemoveMap;

void KeyboardCallback(const cv::viz::KeyboardEvent&, void*);

bool isStop = false;

// ./ptpicking source.pcap outputIDX.txt inputIDX.txt
int main( int argc, char* argv[] )
{
    if(argc < 2){
        cerr<<"./odometry_test pcap_data [Save_File] [Load_File]"<<endl;
        return -1;
    }

    // Open VelodyneCapture that retrieve from PCAP
    velodyne::HDL32ECapture capture( argv[1], 11 );

    if( !capture.isOpen() ){
        std::cerr << "Can't open VelodyneCapture." << std::endl;
        return -1;
    }

    // Create Viewer
    VizViewer viewer( "Velodyne" );
    VizViewer viewer2( "Velodyne2" );
    VizViewer viewer3( "Velodyne3" );

    // Register Callback
    viewer.registerKeyboardCallback(KeyboardCallback, &viewer);

    viewer.setBackgroundColor(cv::viz::Color::gray());
    viewer2.setBackgroundColor(cv::viz::Color::gray());
    viewer3.setBackgroundColor(cv::viz::Color::gray());

    // Frame sequence
    vector<myslam::Frame::Ptr> FrameSequence;

    int frame_num = 519;
    int frame_id = 0;
    double max_dist = 0;
    vector<double> vertAngle = capture.getVerticalAngle();  // Degree
    sort(vertAngle.begin(), vertAngle.end());

    myslam::Preprocessor preprocessor;
    preprocessor.setVerticalAngles(vertAngle);

    while( capture.isRun() && !viewer.wasStopped() ){
        if(!isStop){
            // Create a pointcloud smart pointer
            myslam::Frame::PCPtr pc = make_shared<vector<Vector3f>>();
            preprocessor.setPointCloud(pc);
            // Capture One Rotation Data
            std::vector<velodyne::Laser> lasers;
            capture >> lasers;
            if( lasers.empty() ){
                continue;
            }
            preprocessor.setLasers(lasers);
            preprocessor.run();

            RangeImg rimg = preprocessor.getRangeImage();
            RemoveMap rmmap = preprocessor.getRemoveMap();

            std::vector<cv::Vec3d> buffer;
            std::vector<cv::Vec3d> buffer_g;
            std::vector<cv::Vec3d> buffer_car;
            std::vector<cv::Vec3d> buffer_ocbg;
            std::vector<cv::Vec3d> buffer_preproc;

            for(auto& col: rimg){
                for(auto& vert: col.second){
                    if(vert.second == 0 || vert.first == -0.6)
                    // if(vert.second == 0 || vert.first == -1.25)
                        continue;
                    double x = vert.second*cos(vert.first)*sin(col.first);
                    double y = vert.second*cos(vert.first)*cos(col.first);
                    double z = vert.second*sin(vert.first);
                    Vector3f pt(x, y, z);
                    cv::Mat v;
                    cv::eigen2cv(pt, v);
                    switch(rmmap[col.first][vert.first]){
                        case 0: // Remaining points
                            buffer.push_back( v );
                            break;
                        case 1: // Ground points
                            buffer_g.push_back( v );
                            break;
                        case 2: // Self-Car points
                            buffer_car.push_back(v);
                            break;
                        case 3: // Occluded edge points
                            buffer_ocbg.push_back(v);
                            break;
                    }
                }
            }


            // // Create a frame
            // myslam::Frame::Ptr fptr = myslam::Frame::createFrame();
            // fptr->setPointCloud(pc);
            // FrameSequence.push_back(fptr);
            // cout<<"size of pointcloud: "<<pc->size()<<endl;

            // Create Widget: current point cloud
            cv::Mat cloudMat = cv::Mat( static_cast<int>( buffer.size() ), 1, CV_64FC3, &buffer[0] );
            cv::viz::WCloud cloud( cloudMat, cv::viz::Color::red() );
            cloud.setRenderingProperty(cv::viz::POINT_SIZE, 4);
            // Show Point Cloud
            viewer.showWidget( "Cloud", cloud );
            viewer2.showWidget( "Cloud", cloud );

            // Create Widget: ground point cloud
            cv::Mat cloudMat_g = cv::Mat( static_cast<int>( buffer_g.size() ), 1, CV_64FC3, &buffer_g[0] );
            cv::viz::WCloud cloud_g( cloudMat_g, cv::viz::Color::yellow() );
            cloud_g.setRenderingProperty(cv::viz::POINT_SIZE, 4);
            // Show Point Cloud
            viewer.showWidget( "Cloud_g", cloud_g );

            // Create Widget: car point cloud
            cv::Mat cloudMat_car = cv::Mat( static_cast<int>( buffer_car.size() ), 1, CV_64FC3, &buffer_car[0] );
            cv::viz::WCloud cloud_car( cloudMat_car, cv::viz::Color::green() );
            cloud_car.setRenderingProperty(cv::viz::POINT_SIZE, 4);
            // Show Point Cloud
            viewer.showWidget( "Cloud_car", cloud_car );

            // Create Widget: car point cloud
            cv::Mat cloudMat_ocbg = cv::Mat( static_cast<int>( buffer_ocbg.size() ), 1, CV_64FC3, &buffer_ocbg[0] );
            cv::viz::WCloud cloud_ocbg( cloudMat_ocbg, cv::viz::Color::cyan() );
            cloud_ocbg.setRenderingProperty(cv::viz::POINT_SIZE, 4);
            // Show Point Cloud
            viewer.showWidget( "Cloud_ocbg", cloud_ocbg );

            // Create Widget: current point cloud by preprocessor
            for(auto it = pc->begin(); it != pc->end(); it++){
                cv::Mat v;
                cv::eigen2cv(*it, v);
                buffer_preproc.push_back(v);
            }
            cv::Mat cloudMat_pre = cv::Mat( static_cast<int>( buffer_preproc.size() ), 1, CV_64FC3, &buffer_preproc[0] );
            cv::viz::WCloud cloud_pre( cloudMat_pre, cv::viz::Color::red() );
            cloud_pre.setRenderingProperty(cv::viz::POINT_SIZE, 4);
            // Show Point Cloud
            viewer3.showWidget( "Cloud_pre", cloud_pre );


            cout<<"pc sz:\t"<<buffer.size()<<endl;
            cout<<"pc_pre sz:\t"<<buffer_preproc.size()<<endl;
            cout<<"ground sz:\t"<<buffer_g.size()<<endl;
            cout<<"car sz:\t"<<buffer_car.size()<<endl;
            cout<<"ocpt sz:\t"<<buffer_ocbg.size()<<endl;
            cout<<"Frame:\t#"<<(frame_id++)<<endl;
        isStop = true;
        }
        viewer.spinOnce();
        viewer2.spinOnce();
        viewer3.spinOnce();
    }
        cout<<endl;

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
