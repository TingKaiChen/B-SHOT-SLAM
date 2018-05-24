#ifndef FRAME_H
#define FRAME_H

#include "common_include.h"

namespace myslam 
{
    
    // forward declare 
    class Frame
    {
    public:
        typedef std::shared_ptr<Frame> Ptr;
        typedef std::shared_ptr<vector<Vector3d>> PCPtr;
        typedef std::shared_ptr<vector<std::bitset<352>>*> DCPPtr;
        unsigned long                  id_;         // id of this frame
        long long                      timestamp_;  // when it is recorded
        SE3                            T_c_w_;      // transform from world to camera
        PCPtr                          pointcloud_; // The point cloud of the frame
        PCPtr                          keypoints_;  // Keypoints of the frame 
        DCPPtr                         descriptors_;// Keypoints of the frame 
        bool                           is_key_frame_;  // whether a key-frame
        
    public: // data members 
        Frame();
        Frame( long id, double time_stamp=0, SE3 T_c_w=SE3(), 
            PCPtr pc=nullptr, PCPtr kps=nullptr, DCPPtr dcpts=nullptr, bool isKeyframe=false );
        ~Frame();
        
        static Frame::Ptr createFrame(); 
        
        void setTimestamp( const long long timestamp );
        void setPose( const SE3& T_c_w );
        void setPointCloud(PCPtr pc);
        void setKeypoints(PCPtr kps);
        void setDescriptors(DCPPtr dcpts);
        unsigned long   getID(){ return id_;};
        long long       getTimestamp(){ return timestamp_;};
        SE3             getPose(){ return T_c_w_;};
        PCPtr           getPointCloud(){ return pointcloud_;};
        PCPtr           getKeypoints(){ return keypoints_;};
        DCPPtr          getDescriptors(){ return descriptors_;};
        bool            isKeyframe(){ return is_key_frame_;};
        
        // check if a point is in this frame 
        // bool isInFrame( const Vector3d& pt_world );
    };

}

#endif // FRAME_H