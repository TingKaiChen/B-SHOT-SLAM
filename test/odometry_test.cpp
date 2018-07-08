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
#include <pcl/correspondence.h>

typedef cv::viz::Viz3d VizViewer;

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
    // velodyne::HDL32ECapture capture( argv[1], 1850 );// low computation time
    // velodyne::HDL32ECapture capture( argv[1], 2000 );   // ICP deviation

    if( !capture.isOpen() ){
        std::cerr << "Can't open VelodyneCapture." << std::endl;
        return -1;
    }

    vector<cv::Vec3d> LoadTrajectory;
    // Load trajectories
    if(argc >= 4){
        ifstream ifs;
        ifs.open(argv[3]);
        string line;
        cv::Vec3d vec;
        while(ifs.is_open() && getline(ifs, line)){
            stringstream ss(line);
            float num;
            int i=0;
            while(ss>>num){
                vec[i++] = num;
            }
            LoadTrajectory.push_back(vec);
        }
    }

    // Create Viewer
    VizViewer viewer( "Velodyne" );
    VizViewer corrviewer("Correspondence");

    // Register Callback
    viewer.registerKeyboardCallback(KeyboardCallback, &viewer);

    // Frame sequence
    vector<myslam::Frame::Ptr> FrameSequence;
    vector<Vector3f> Keypoints;
    vector<cv::Vec3d> Trajectory;
    myslam::LidarOdometry lo;

    vector<double> vertAngle = capture.getVerticalAngle();  // Degree
    sort(vertAngle.begin(), vertAngle.end());

    myslam::Preprocessor preprocessor;
    preprocessor.setVerticalAngles(vertAngle);

    int frame_num = 0;
    int frame_id = 0;

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

            // if((frame_num--) == 0 || frame_id == 326){
            //     isStop = true;
            // }
            // if(frame_id%3 != 0){
            //     frame_id++;
            //     continue;
            // }

            // Point cloud preprocessing
            preprocessor.setLasers(lasers);
            preprocessor.run();

            // Convert to 3-dimention Coordinates
            std::vector<cv::Vec3d> buffer;

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
                lo.evaluateEstimation();
                lo.poseEstimation();
                lo.updateMap();
                lo.updateCorrespondence();
            }
            else{
                lo.passSrc2Ref();
                lo.setSrcFrame(fptr);
                lo.extractKeypoints();
                lo.computeDescriptors();
                lo.featureMatching();
                lo.evaluateEstimation();
                lo.poseEstimation();
                lo.updateMap();
                lo.updateCorrespondence();
            }
            cv::Mat v;
            Vector3f position(fptr->getPose().topRightCorner<3,1>());
            cv::eigen2cv(position, v);
            Trajectory.push_back(v);

            // Point cloud transformation
            Matrix3f R = fptr->getPose().block<3,3>(0, 0);
            Vector3f T = position;
            int i=0;
            buffer.reserve(pc->size());
            for(vector<Vector3f>::iterator it = pc->begin(); it != pc->end(); it++){
                *it = R*(*it)+T;
                cv::Mat v;
                cv::eigen2cv(*it, v);
                buffer.push_back(v);
                // buffer[i] = v;
                i++;
            }

            corrviewer.removeAllWidgets();

            // Create Widget: current point cloud
            cv::Mat cloudMat = cv::Mat( static_cast<int>( buffer.size() ), 1, CV_64FC3, &buffer[0] );
            cv::viz::WCloud cloud( cloudMat );
            // Show Point Cloud
            viewer.showWidget( "Cloud", cloud );

            Keypoints.clear();
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

            // Create Widget: trajectory_load
            if(argc >= 4){
                cv::Mat loadtrajMat = cv::Mat( static_cast<int>( LoadTrajectory.size() ), 1, CV_64FC3, &LoadTrajectory[0] );
                cv::viz::WCloud loadtraj( loadtrajMat, cv::viz::Color::gold() );
                loadtraj.setRenderingProperty(cv::viz::POINT_SIZE, 4);
                viewer.showWidget( "LoadTrajectory", loadtraj );
            }

            // Create current position
            cv::viz::WCloud cur_pos(cv::Mat(1, 1, CV_64FC3, &(Trajectory.back())), cv::viz::Color::red());
            cur_pos.setRenderingProperty(cv::viz::POINT_SIZE, 4);
            viewer.showWidget( "Current position", cur_pos );

            //// Correspondence visualization
            myslam::Frame::PCPtr refkpsptr = lo.getKeypoints();
            // myslam::Frame::PCPtr refkpsptr = lo.getRefKeypoints();
            vector<cv::Vec3d> refkps;
            refkps.resize( int(refkpsptr->size()) );
            for(int i = 0; i < refkpsptr->size(); i++){
                cv::Mat v;
                cv::eigen2cv((*refkpsptr)[i], v);
                refkps[i] = v;
            }
            // Create Widget: reference keypoints
            cv::Mat cloudMat3 = cv::Mat( static_cast<int>( refkps.size() ), 1, CV_64FC3, &refkps[0] );
            cv::viz::WCloud cloud3( cloudMat3, cv::viz::Color::red() );
            cloud3.setRenderingProperty(cv::viz::POINT_SIZE, 4);
            corrviewer.showWidget( "Cloud3", cloud3 );

            // Create Widget: current keypoints in corresponding viewer
            vector<cv::Vec3d> buffer3(buffer2);
            // vector<cv::Vec3d> buffer3 = buffer2;
            cv::Mat cloudMat_ref = cv::Mat( static_cast<int>( buffer3.size() ), 1, CV_64FC3, &buffer3[0] );
            cv::viz::WCloud cloud_ref( cloudMat_ref, cv::viz::Color::yellow() );
            cloud_ref.setRenderingProperty(cv::viz::POINT_SIZE, 4);

            cv::Affine3d ref_T = cv::Affine3d::Identity();
            ref_T.translation(cv::Vec3d(180000, 0, 0));
            cloud_ref.applyTransform(ref_T);
            corrviewer.showWidget( "Cloud_ref", cloud_ref );

            vector<cv::Vec3d> buffer4(buffer);
            cv::Mat cloudMat_ref_all = cv::Mat( static_cast<int>( buffer4.size() ), 1, CV_64FC3, &buffer4[0] );
            cv::viz::WCloud cloud_ref_all( cloudMat_ref_all, cv::viz::Color::white() );
            cloud_ref_all.applyTransform(ref_T);
            corrviewer.showWidget( "Cloud_ref_all", cloud_ref_all );

            // Create Widget: correspondences
            vector<pair<Vector3f,Vector3f> > corrs = lo.getCorrespondences();
            int linenum = 0;
            for(auto& corr: corrs){
                Vector3f pt1_eigen = R*corr.first+T;
                cv::Point3d pt1(pt1_eigen[0]+180000, pt1_eigen[1], pt1_eigen[2]);
                cv::Point3d pt2(corr.second[0], corr.second[1], corr.second[2]);
                cv::viz::WLine corrline(pt1, pt2, cv::viz::Color::pink());
                // corrline.setRenderingProperty(cv::viz::LINE_WIDTH, 4);
                corrviewer.showWidget("Correspond"+to_string(linenum++), corrline);
            } 

            if(corrs.size()<15){
                cout<<"Limited correspondence: "<<corrs.size()<<endl;
                // isStop = true;
            }

            // // Create Widget: range
            // cv::Point3d cent = Trajectory.back();
            // cv::Point3d minpt(cent.x-80000, cent.y-80000, cent.z-80000);
            // cv::Point3d maxpt(cent.x+80000, cent.y+80000, cent.z+80000);
            // cv::viz::WCube range_cube(minpt, maxpt, false, cv::viz::Color::green());
            // range_cube.setRenderingProperty(cv::viz::OPACITY, 0.3);
            // corrviewer.showWidget( "Range_cube", range_cube );


            cout<<"Frame:\t#"<<(frame_id++)<<endl;

            if(frame_id == 2681){
                isStop = true;
            }

        // isStop = true;
        }
        viewer.spinOnce();
        corrviewer.spinOnce();
    }

    // Save trajectories
    if(argc >= 3){
        ofstream ofs;
        ofs.open(argv[2]);
        for(auto& pos: Trajectory){
            Vector3f pos_;
            cv::cv2eigen(cv::Mat(pos), pos_);
            ofs<<pos_[0]<<" "<<pos_[1]<<" "<<pos_[2]<<endl;;
        }
        ofs<<endl;
        ofs.close();
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
