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
typedef map<double, bool> RemoveCol;
typedef map<double, RemoveCol> RemoveMap;

void KeyboardCallback(const cv::viz::KeyboardEvent&, void*);

bool isStop = false;

int prec = 500;
unsigned long getBlockID(Vector3f pos, bool is2D){
    Vector3f grid_p(
        int(round(pos[0]/prec))*prec, 
        int(round(pos[1]/prec))*prec, 
        int(round(pos[2]/prec))*prec);
    // 64bits keyID = 1bit+21bits(x)+21bits(y)+21bits(z)
    bitset<64> i = ((bitset<64>(int(grid_p[0]))<<42) & bitset<64>(0x1FFFFF)<<42);
    bitset<64> j = ((bitset<64>(int(grid_p[1]))<<21) & bitset<64>(0x1FFFFF)<<21);
    bitset<64> k = (bitset<64>(int(grid_p[2])) & bitset<64>(0x1FFFFF));
    if(is2D){
        return ((i|j).to_ulong());
    }
    else{
        return ((i|j|k).to_ulong());
    }
}

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

    // Register Callback
    viewer.registerKeyboardCallback(KeyboardCallback, &viewer);

    // Frame sequence
    vector<myslam::Frame::Ptr> FrameSequence;

    int frame_num = 519;
    int frame_id = 0;
    double max_dist = 0;

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

            // if((frame_num--) == 0 || frame_id == 291){
            //     isStop = true;
            //     // continue;
            // }
            // if(frame_id%3 != 0){
            //     frame_id++;
            //     continue;
            // }

            GridMap gridmap;
            ColGroundVal groundval;

            RangeImg rimg;
            RemoveMap rmmap;

            // Convert to 3-dimention Coordinates
            std::vector<cv::Vec3d> buffer;
            std::vector<cv::Vec3d> buffer_g;
            for( const velodyne::Laser& laser : lasers ){
                // Distance unit: mm
                const double distance = static_cast<double>( laser.distance );
                const double azimuth  = laser.azimuth  * CV_PI / 180.0;
                const double vertical = laser.vertical * CV_PI / 180.0;

                double x = static_cast<double>( ( distance * std::cos( vertical ) ) * std::sin( azimuth ) );
                double y = static_cast<double>( ( distance * std::cos( vertical ) ) * std::cos( azimuth ) );
                double z = static_cast<double>( ( distance * std::sin( vertical ) ) );

                rimg[azimuth][vertical] = distance;
                rimg[azimuth][-90*CV_PI/180.0] = 1200;
                rmmap[azimuth][vertical] = false;
                rmmap[azimuth][-90*CV_PI/180.0] = true;


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
                double vert_prev = -90*CV_PI/180.0;
                Vector3f p_prev(0, 0, -1200);
                Vector3f p_th(0, 0, -1200);
                double z_max = -1200;

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
                        if(p_th[2] > z_max){
                            z_max = p_th[2];
                        }
                    }

                    if(prev_is_ground){    // Previous point is ground point
                        if(grad < grad_th && !lost_pt){
                            rmmap[col.first][vert.first] = true;
                            prev_is_ground = true;
                        }
                        else{   // Previous point is a threshold point
                            rmmap[col.first][vert.first] = false;
                            prev_is_ground = false;
                        }
                    }
                    else if(!prev_is_ground && p_curr[2] < -1000 && grad < grad_th){
                        rmmap[col.first][vert.first] = true;
                        prev_is_ground = true;
                        set_th_pt = false;
                    }

                    if(vert.second == 0){   // Current point is a lost point
                        rmmap[col.first][vert.first] = true;
                        lost_pt = true;
                        prev_is_ground = false;
                    }
                    else{
                        lost_pt = false;
                    }

                    if(vert.second < p_prev.norm() && vert.second != 0){
                        rmmap[col.first][vert.first] = false;
                        prev_is_ground = false;
                    }

                    // Set start point
                    if(set_th_pt && (p_curr[2]-p_th[2]) < height_th && p_curr[2] < p_prev[2]){
                        set_th_pt = false;
                        rmmap[col.first][vert.first] = true;
                        prev_is_ground = true;
                    }


                    p_prev = p_curr;
                    vert_prev = vert.first;
                }
            }

            int count = 0;
            for(auto& col: rimg){
                Vector3f p_prev(0, 0, -1200);
                for(auto& vert: col.second){
                    double x = vert.second*cos(vert.first)*sin(col.first);
                    double y = vert.second*cos(vert.first)*cos(col.first);
                    double z = vert.second*sin(vert.first);
                    Vector3f pt(x, y, z);
                    cv::Mat v;
                    cv::eigen2cv(pt, v);
                    if(rmmap[col.first][vert.first]){
                        buffer_g.push_back( v );
                    }
                    else{
                        buffer.push_back( v );
                    }
                    // if(count == 695){
                    //     double grad = asin((pt[2]-p_prev[2])/(pt-p_prev).norm())*180/CV_PI;
                    //     cout<<grad<<endl;
                    //     p_prev = pt;
                    // }
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

            // Create Widget: ground point cloud
            cv::Mat cloudMat_g = cv::Mat( static_cast<int>( buffer_g.size() ), 1, CV_64FC3, &buffer_g[0] );
            cv::viz::WCloud cloud_g( cloudMat_g, cv::viz::Color::yellow() );
            cloud_g.setRenderingProperty(cv::viz::POINT_SIZE, 4);
            // Show Point Cloud
            viewer.showWidget( "Cloud_g", cloud_g );

            // // Create Widget: Zero plane
            // cv::Point3d ct(0, 0, -1200);
            // cv::Vec3d nVec = cv::Vec3d(0, 0, 1);
            // cv::Vec3d yVec = cv::Vec3d(0, 1, 0);
            // cv::Size2d sz = cv::Size2d(70000, 70000);
            // cv::viz::WPlane VPlane(ct, nVec, yVec, sz, cv::viz::Color::green());          
            // VPlane.setRenderingProperty(cv::viz::OPACITY, 0.3);
            // viewer.showWidget("Plane", VPlane);


            cout<<"Frame:\t#"<<(frame_id++)<<endl;

        // isStop = true;
        }
        viewer.spinOnce();
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
