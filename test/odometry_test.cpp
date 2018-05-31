// Include MySLAM
#include "common_include.h"
#include "VelodyneCapture.h"
#include "frame.h"
#include "lidar_odometry.h"

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
    velodyne::HDL32ECapture capture( argv[1], 11 );

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
    vector<Vector3f> Keypoints;
    vector<cv::Vec3d> Trajectory;
    myslam::LidarOdometry lo;

    while( capture.isRun() && !viewer.wasStopped() ){
        if(!isStop){
            // Create a pointcloud smart pointer
            myslam::Frame::PCPtr pc = make_shared<vector<Vector3f>>();
            // Capture One Rotation Data
            std::vector<velodyne::Laser> lasers;
            capture >> lasers;
            if( lasers.empty() ){
                continue;
            }

            // Convert to 3-dimention Coordinates
            std::vector<cv::Vec3d> buffer;
            // buffer.resize( lasers.size() );
            // pc->resize( lasers.size() );

            for( const velodyne::Laser& laser : lasers ){
                // Distance unit: mm
                const double distance = static_cast<double>( laser.distance );
                const double azimuth  = laser.azimuth  * CV_PI / 180.0;
                const double vertical = laser.vertical * CV_PI / 180.0;

                double x = static_cast<double>( ( distance * std::cos( vertical ) ) * std::sin( azimuth ) );
                double y = static_cast<double>( ( distance * std::cos( vertical ) ) * std::cos( azimuth ) );
                double z = static_cast<double>( ( distance * std::sin( vertical ) ) );

                // Remove ground points
                if(z<-500) continue;
                // Remove origin
                if( x == 0.0 && y == 0.0 && z == 0.0 ){
                    x = std::numeric_limits<double>::quiet_NaN();
                    y = std::numeric_limits<double>::quiet_NaN();
                    z = std::numeric_limits<double>::quiet_NaN();
                    continue;
                }
                // Remove self-car
                if(x <= 820 && x >= -820 && y <= 1300 && y >= -1800 && z <= 100 && z >= -2000)
                    continue;

                // Add a point into the point cloud
                pc->push_back( Vector3f( x, y, z ) );
                // Convert a point from Eigen type to OpenCV type
                Vector3f vs = pc->back(); 
                cv::Mat v;
                cv::eigen2cv(Vector3f( x, y, z ), v);
                buffer.push_back( v );
            }

            // Create a frame
            myslam::Frame::Ptr fptr = myslam::Frame::createFrame();
            fptr->setPointCloud(pc);
            FrameSequence.push_back(fptr);
            cout<<"size of pointcloud: "<<pc->size()<<endl;
            // Test lidar odometry
            if(lo.isInitial()){
                lo.setSrcFrame(fptr);                
                lo.extractKeypoints();
                lo.computeDescriptors();
                lo.featureMatching();
                lo.poseEstimation();
            }
            else{
                lo.passSrc2Ref();
                lo.setSrcFrame(fptr);
                lo.extractKeypoints();
                lo.computeDescriptors();
                lo.featureMatching();
                lo.poseEstimation();
            }
            cv::Mat v;
            Vector3f position(fptr->getPose().matrix().topRightCorner<3,1>());
            cv::eigen2cv(position, v);
            Trajectory.push_back(v);

            // Point cloud transformation
            Matrix3f R = fptr->getPose().matrix().block<3,3>(0, 0);
            Vector3f T = position;
            int i=0;
            for(vector<Vector3f>::iterator it = pc->begin(); it != pc->end(); it++){
                *it = R*(*it)+T;
                cv::Mat v;
                cv::eigen2cv(*it, v);
                buffer[i] = v;
                i++;
            }


            // Save every frame into the map
            // TODO: implement Map class

            // Create Widget: current point cloud
            cv::Mat cloudMat = cv::Mat( static_cast<int>( buffer.size() ), 1, CV_64FC3, &buffer[0] );
            cv::viz::WCloud cloud( cloudMat );
            // Show Point Cloud
            viewer.showWidget( "Cloud", cloud );

            // myslam::Frame::PCPtr kpsptr = lo.getKeypoints();
            myslam::Frame::PCPtr kpsptr = fptr->getKeypoints();
            for(vector<Vector3f>::iterator it = kpsptr->begin(); it != kpsptr->end(); it++){
                Keypoints.push_back(R*(*it)+T);
            }
            vector<cv::Vec3d> buffer2;
            // buffer2.resize( int(Keypoints.size()) );
            buffer2.resize( int(kpsptr->size()) );
            // for(int i = 0; i < Keypoints.size(); i++){
            for(int i = 0; i < kpsptr->size(); i++){
                cv::Mat v;
                // cv::eigen2cv(Keypoints[i], v);
                Vector3f kp = R*(*kpsptr)[i]+T;
                cv::eigen2cv(kp, v);
                buffer2[i] = v;
            }
            // Create Widget: keypoints
            cv::Mat cloudMat2 = cv::Mat( static_cast<int>( buffer2.size() ), 1, CV_64FC3, &buffer2[0] );
            cv::viz::WCloud cloud2( cloudMat2, cv::viz::Color::yellow() );
            cloud2.setRenderingProperty(cv::viz::POINT_SIZE, 4);
            // Show Point Cloud
            viewer.showWidget( "Cloud2", cloud2 );

            // Create Widget: trajectory
            cv::Mat trajMat = cv::Mat( static_cast<int>( Trajectory.size() ), 1, CV_64FC3, &Trajectory[0] );
            cv::viz::WCloud traj( trajMat, cv::viz::Color::green() );
            traj.setRenderingProperty(cv::viz::POINT_SIZE, 4);
            viewer.showWidget( "Trajectory", traj );

            // Create current position
            cv::viz::WCloud cur_pos(cv::Mat(1, 1, CV_64FC3, &(Trajectory.back())), cv::viz::Color::red());
            cur_pos.setRenderingProperty(cv::viz::POINT_SIZE, 4);
            viewer.showWidget( "Current position", cur_pos );


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
