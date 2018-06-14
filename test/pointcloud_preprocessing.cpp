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

                unsigned long grid_id = getBlockID(Vector3f(x, y, z), false);
                unsigned long gridmap_id = getBlockID(Vector3f(x, y, z), true);
                if(gridmap.find(gridmap_id) == gridmap.end()){  // No (x, y) column
                    // Add new column and grid
                    vector<Vector3f> grid;
                    grid.push_back(Vector3f(x, y, z));
                    Grid gridcol;
                    gridcol.insert(make_pair(grid_id, grid));
                    gridmap.insert(make_pair(gridmap_id, gridcol));
                    // Update the lowest elevation of the grid column
                    int ground_z = int(round(z/prec))*prec;
                    groundval.insert(make_pair(gridmap_id, ground_z));
                }
                else{   // Find (x, y) column
                    if(gridmap[gridmap_id].find(grid_id) == gridmap[gridmap_id].end()){ // No (x, y, z) grid
                    // Add new grid
                        vector<Vector3f> grid;
                        grid.push_back(Vector3f(x, y, z));
                        gridmap[gridmap_id].insert(make_pair(grid_id, grid));
                        // Update the lowest elevation of the grid column
                        int ground_z = int(round(z/prec))*prec;
                        if(groundval[gridmap_id] > ground_z){
                            groundval[gridmap_id] = ground_z;
                        }
                    }
                    else{   // Find (x, y, z) grid
                        gridmap[gridmap_id][grid_id].push_back(Vector3f(x, y, z));
                    }
                }

                // Add a point into the point cloud
                pc->push_back( Vector3f( x, y, z ) );
                // Convert a point from Eigen type to OpenCV type
                Vector3f vs = pc->back(); 
                // cv::Mat v;
                // cv::eigen2cv(Vector3f( x, y, z ), v);
                // buffer.push_back( v );
            }

            // for(auto& col: gridmap){
            //     unsigned long col_buttom = (groundval[col.first] & 0x1FFFFF);
            //     for(auto& grid: col.second){
            //         unsigned long grid_z = (grid.first & 0x1FFFFF);
            //         if(grid_z == col_buttom){
            //             for(auto& pt: grid.second){
            //                 cv::Mat v;
            //                 cv::eigen2cv(pt, v);
            //                 buffer_g.push_back( v );
            //             }
            //             // continue;
            //         }
            //         else{
            //             for(auto& pt: grid.second){
            //                 cv::Mat v;
            //                 cv::eigen2cv(pt, v);
            //                 buffer.push_back( v );
            //             }
            //         }
            //     }
            // }

            // // Create a frame
            // myslam::Frame::Ptr fptr = myslam::Frame::createFrame();
            // fptr->setPointCloud(pc);
            // FrameSequence.push_back(fptr);
            // cout<<"size of pointcloud: "<<pc->size()<<endl;

            // // Create Widget: current point cloud
            // cv::Mat cloudMat = cv::Mat( static_cast<int>( buffer.size() ), 1, CV_64FC3, &buffer[0] );
            // cv::viz::WCloud cloud( cloudMat, cv::viz::Color::red() );
            // cloud.setRenderingProperty(cv::viz::POINT_SIZE, 4);
            // // Show Point Cloud
            // viewer.showWidget( "Cloud", cloud );

            // // Create Widget: ground point cloud
            // cv::Mat cloudMat_g = cv::Mat( static_cast<int>( buffer_g.size() ), 1, CV_64FC3, &buffer_g[0] );
            // cv::viz::WCloud cloud_g( cloudMat_g, cv::viz::Color::yellow() );
            // cloud_g.setRenderingProperty(cv::viz::POINT_SIZE, 4);
            // // Show Point Cloud
            // viewer.showWidget( "Cloud_g", cloud_g );




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
