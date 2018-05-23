#include "frame.h"

namespace myslam
{
    Frame::Frame()
    : id_(-1), timestamp_(-1), pointcloud_(nullptr), keypoints_(nullptr), descriptors_(nullptr), 
    is_key_frame_(false){

    }

    Frame::Frame 
    ( long id, double time_stamp, SE3 T_c_w, PCPtr pc, PCPtr kps, DCPPtr dcpts, bool isKeyframe): 
    id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), pointcloud_(pc), keypoints_(kps), 
    descriptors_(dcpts), is_key_frame_(isKeyframe){

    }

    Frame::~Frame()
    {

    }

    Frame::Ptr Frame::createFrame()
    {
        static long factory_id = 0;
        return Frame::Ptr( new Frame(factory_id++) );
    }

    void setTimestamp( const long long timestamp ){
        timestamp_ = timestamp;
    }

    void Frame::setPose ( const SE3& T_c_w )
    {
        T_c_w_ = T_c_w;
    }

    void Frame::setPointCloud(PCPtr pc){
        pointcloud_ = pc;
    }

    void Frame::setKeypoints(PCPtr kps){
        keypoints_ = kps;
    }

    void Frame::setDescriptors(DCPPtr dcpts){
        descriptors_ = dcpts;
    }

    // bool Frame::isInFrame ( const Vector3d& pt_world )
    // {
    //     // Vector3d p_cam = camera_->world2camera( pt_world, T_c_w_ );
    //     // // cout<<"P_cam = "<<p_cam.transpose()<<endl;
    //     // if ( p_cam(2,0)<0 ) return false;
    //     // Vector2d pixel = camera_->world2pixel( pt_world, T_c_w_ );
    //     // // cout<<"P_pixel = "<<pixel.transpose()<<endl<<endl;
    //     // return pixel(0,0)>0 && pixel(1,0)>0 
    //     //     && pixel(0,0)<color_.cols 
    //     //     && pixel(1,0)<color_.rows;
    // }

}