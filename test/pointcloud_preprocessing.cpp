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
    velodyne::HDL32ECapture capture( argv[1], 0 );

    if( !capture.isOpen() ){
        std::cerr << "Can't open VelodyneCapture." << std::endl;
        return -1;
    }

    // Create Viewer
    VizViewer viewer( "Velodyne" );
    VizViewer viewer2( "Velodyne2" );

    // Register Callback
    viewer.registerKeyboardCallback(KeyboardCallback, &viewer);

    // Frame sequence
    vector<myslam::Frame::Ptr> FrameSequence;

    int frame_num = 519;
    int frame_id = 0;
    double max_dist = 0;
    vector<double> vertAngle = capture.getVerticalAngle();  // Degree
    sort(vertAngle.begin(), vertAngle.end());

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


            GridMap gridmap;
            ColGroundVal groundval;

            RangeImg rimg;
            RemoveMap rmmap;

            // Convert to 3-dimention Coordinates
            std::vector<cv::Vec3d> buffer;
            std::vector<cv::Vec3d> buffer_g;
            std::vector<cv::Vec3d> buffer_car;
            std::vector<cv::Vec3d> buffer_ocbg;
            for( const velodyne::Laser& laser : lasers ){
                // Distance unit: mm
                const double distance = static_cast<double>( laser.distance );
                const double azimuth  = laser.azimuth  * CV_PI / 180.0;
                const double vertical = laser.vertical * CV_PI / 180.0;

                double x = static_cast<double>( ( distance * std::cos( vertical ) ) * std::sin( azimuth ) );
                double y = static_cast<double>( ( distance * std::cos( vertical ) ) * std::cos( azimuth ) );
                double z = static_cast<double>( ( distance * std::sin( vertical ) ) );

                rimg[azimuth][vertical] = distance;
                rimg[azimuth][-0.6] = 1200;
                rmmap[azimuth][vertical] = 0;
                rmmap[azimuth][-0.6] = 1;


                // Add a point into the point cloud
                pc->push_back( Vector3f( x, y, z ) );
                // Convert a point from Eigen type to OpenCV type
                Vector3f vs = pc->back(); 
            }


            double grad_th = 45;
            double height_th = 250;

            for(auto& col: rimg){
                bool lost_pt = false;
                bool set_th_pt = false;
                bool prev_is_ground = true;
                double vert_prev = -0.6;
                double x_0 = (-1200/tan(vert_prev))*cos(col.first);
                double y_0 = (-1200/tan(vert_prev))*sin(col.first);
                double z_0 = -1200;
                Vector3f p_prev(x_0, y_0, z_0);
                Vector3f p_th(x_0, y_0, z_0);

                for(auto& vert: col.second){
                    double x = vert.second*cos(vert.first)*sin(col.first);
                    double y = vert.second*cos(vert.first)*cos(col.first);
                    double z = vert.second*sin(vert.first);
                    Vector3f p_curr(x, y, z);

                    double grad = asin((p_curr[2]-p_prev[2])/(p_curr-p_prev).norm())*180/CV_PI;

                    // Set threshold point
                    if(prev_is_ground && 
                        (grad > grad_th || vert.second == 0 || vert.second < p_prev.norm())){
                        set_th_pt = true;
                        p_th = p_prev;
                    }

                    if(prev_is_ground){    // Previous point is ground point
                        if(grad < grad_th && !lost_pt){
                            rmmap[col.first][vert.first] = 1;
                            prev_is_ground = true;
                        }
                        else{   // Previous point is a threshold point
                            rmmap[col.first][vert.first] = 0;
                            prev_is_ground = false;
                        }
                    }
                    // Lower ground condition
                    else if(!prev_is_ground && p_curr[2] < -1000 && grad < grad_th){
                        rmmap[col.first][vert.first] = 1;
                        prev_is_ground = true;
                        set_th_pt = false;
                    }

                    if(vert.second == 0){   // Current point is a lost point
                        rmmap[col.first][vert.first] = 1;
                        lost_pt = true;
                        prev_is_ground = false;
                    }
                    else{
                        lost_pt = false;
                    }

                    if(vert.second < p_prev.norm() && vert.second != 0){
                        rmmap[col.first][vert.first] = 0;
                        prev_is_ground = false;
                    }

                    // Set start point
                    if(set_th_pt && (p_curr[2]-p_th[2]) < height_th && p_curr[2] < p_prev[2]){
                        set_th_pt = false;
                        rmmap[col.first][vert.first] = 1;
                        prev_is_ground = true;
                    }

                    // Remove self-car
                    if(x <= 820 && x >= -820 && 
                        y <= 1300 && y >= -1800 && 
                        z <= 100 && z >= -2000){
                        rmmap[col.first][vert.first] = 2;
                        // continue;
                    }

                    p_prev = p_curr;
                    vert_prev = vert.first;
                }
            }

            // Occluded edge points
            double dist_th = 1500;
            double angdiff_th = 1.0*CV_PI/180.0;
            for(auto& vert: vertAngle){
                double v = vert*CV_PI/180.0;
                double prev_hor;
                bool isFirst = true;
                for(auto& col: rimg){
                    if(isFirst){
                        prev_hor = col.first;
                        isFirst = false;
                    }
                    else if(rimg[col.first][v] == 0){   // Lost point
                        continue;
                    }
                    else{
                        double d_dist = rimg[col.first][v] - rimg[prev_hor][v];
                        double d_hor = col.first - prev_hor;
                        if(abs(d_dist) > dist_th && abs(d_hor) < angdiff_th){   // is a occluded point
                            if(d_dist > 0){ // Current point is background pt
                                if(rmmap[col.first][v] == 0)
                                    rmmap[col.first][v] = 3;
                            }
                            else{   // Previous point is background pt
                                if(rmmap[prev_hor][v] == 0)
                                    rmmap[prev_hor][v] = 3;
                            }
                        }
                        prev_hor = col.first;
                    }
                }
            }

            for(auto& col: rimg){
                Vector3f p_prev(0, 0, -1200);
                for(auto& vert: col.second){
                    if(vert.second == 0 || vert.first == -0.6)
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



            cout<<"Frame:\t#"<<(frame_id++)<<endl;
        // isStop = true;
        }
        viewer.spinOnce();
        viewer2.spinOnce();
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
