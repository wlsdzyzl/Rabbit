#ifndef RABBIT_GRAPH_BASE_H
#define RABBIT_GRAPH_BASE_H
#include "Frame.h"
#include "Odometry.h"
#include "LCDetection.h"
#include "Optimization/FactorGraph.h"
#include "MovingBox.h"
namespace rabbit
{
namespace system
{
    using namespace util;
    class GraphBase
    {
        public:
        GraphBase():lcdetector(*this){};

        void SetSlidingWindow(int type)
        {
            sliding_window_type = type;
            if(type==2)
            {
                moving_box = MovingBox(sliding_window_cube_size_x,
                    sliding_window_cube_size_y, 
                    sliding_window_cube_size_z, 
                    sliding_window_cube_len);
            }
        }
        std::vector<int> GetFrameIDInSlidingWindow();
        //check the pose difference to decide if we should add a new keyframe
        bool NewKeyframe(const SE3 &delta_pose);
        SE3 AddNewFrame(const Frame &new_frame);
        SE3 AddNewFrameSubmap(const Frame &f);
        SE3 AddNewFrameSlidingWindow(const Frame &f);
        void ConstructSWVolume(const std::vector<int> &ids_in_sw, Frame &volume);
        void ConstructSubmapVolume(int submap_id, Frame &volume);
        void Optimize();
        void OptimizeSubmap();
        void OptimizeLocal();
        bool IsKeyframeInserted(){return keyframe_list.back().frame_id == (relative_pose_list.size() - 1);}
        bool Matching(const Frame &s, const Frame &t, SE3 &T);
        bool Mapping(const Frame &s, const Frame &t, SE3 &T);
        // bool GroundMapping(const Frame &s, const Frame &t, SE3 &T);
        void SetMatchingMethod(const std::string &m)
        {
            if(m == "loam")
            matching_method = OdometryMethod::LOAM;
            else if (m == "ndt")
            matching_method = OdometryMethod::NDT;
            else if (m == "icp")
            matching_method = OdometryMethod::ICP;
            else if(m == "gicp")
            matching_method = OdometryMethod::GICP;
            else if(m == "ndt_omp"||m == "ndtomp")
            matching_method = OdometryMethod::NDTOMP;
            else if(m == "gicp_omp" || m == "gicpomp")
            matching_method = OdometryMethod::GICPOMP;
            else
            {
                std::cout<<YELLOW<<"[WARNING]::[SetOdometry]::unsupported method. Use default: loam odometry."<<RESET<<std::endl;
            }
        }
        int GetCubeID(const Vec3 &pos);
        void SetMappingMethod(const std::string &m)
        {
            if(m == "loam")
            mapping_method = OdometryMethod::LOAM;
            else if (m == "ndt")
            mapping_method = OdometryMethod::NDT;
            else if (m == "icp")
            mapping_method = OdometryMethod::ICP;
            else if(m == "gicp")
            mapping_method = OdometryMethod::GICP;
            else if(m == "ndt_omp"||m == "ndtomp")
            mapping_method = OdometryMethod::NDTOMP;
            else if(m == "gicp_omp" || m == "gicpomp")
            mapping_method = OdometryMethod::GICPOMP;
            else
            {
                std::cout<<YELLOW<<"[WARNING]::[SetOdometry]::unsupported method. Use default: loam odometry."<<RESET<<std::endl;
            }
        }
        void SetGroundPriority(bool gp)
        {
            use_ground_priority = gp;
            Frame::ground_extraction = gp;
        }
        SE3List relative_pose_list;
        std::vector<Frame> keyframe_list;
        
        std::vector<optimization::FrameCorrespondence> keyframe_corrs;
        // local frame corrs, used to optimize local frames between two keyframe.
        std::vector<optimization::FrameCorrespondence> frame_corrs;
        std::vector<optimization::FrameCorrespondence> submap_corrs;
        std::vector<int> submap_id_for_keyframes;
        LidarOdometry odometry;
        double angle_threshold = 10;
        double distance_threshold =1.0; 
        double lcd_leaf_size = 0.5;
        // 1. continuous frames
        // 2. maintain a 3D cubes
        // 0. without slide window, use submap
        int sliding_window_type = 1;
        int sliding_window_volume_n = 25;

        size_t sliding_window_cube_size_x = 21;
        size_t sliding_window_cube_size_y = 21;
        size_t sliding_window_cube_size_z = 11;
        // each cube volume is 50 * 50 * 50 m^3
        double sliding_window_cube_len = 5;

        double sharp_leaf_size = 0.4;
        double flat_leaf_size = 0.5;
        optimization::Optimizer optimizer;
        Frame sliding_window_volume;
        Frame last_frame;
        LCDetector lcdetector;
        bool with_imu = false;
        SE3 relative_T_to_last_frame;
        SE3 relative_T_to_last_keyframe;
        int lcd_detection = 0;
        int min_mapping_n = 5;
        // n keyframe will form a submap
        int submap_len = 20;
        MovingBox moving_box;
        std::vector<Frame> submap_list;
        OdometryMethod matching_method = OdometryMethod::LOAM;
        OdometryMethod mapping_method = OdometryMethod::LOAM;
        OdometryMethod ground_mapping_method = OdometryMethod::LOAM;
        bool use_ground_priority = false;

        double matching_score_threshold = 0.5;
        double mapping_score_threshold = 0.1;

        double matching_inlier_ratio = 0.1;
        double mapping_inlier_ratio = 0.4;
        
        int lcd_detection_interval = 10;
        int last_lcd_id = 0;

        bool exaustive_mapping = false;
        double ground_weight = 1.5;
    };
}
}
#endif