#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nodelet/nodelet.h>
#include <stdio.h>
#include <mrs_lib/param_loader.h>
#include <std_msgs/Float32.h>
#include <mrs_msgs/ImagePointsWithFloatStamped.h>
#include <mrs_msgs/Point2DWithFloat.h>
#include <fstream>
#include <thread>

#include <cv_bridge/cv_bridge.h>
#include <alternativeHT/alternativeHT.h>

using pairPoints = std::pair<mrs_msgs::Point2DWithFloat, mrs_msgs::Point2DWithFloat>;
using vectorPair = std::vector<pairPoints>;

namespace uvdar {


    class UVDAR_BP_Tim : public nodelet::Nodelet {
       
       
        public:

            UVDAR_BP_Tim(); 
            ~UVDAR_BP_Tim();
        
        private:
            using vectPoint3D = std::vector<mrs_msgs::Point2DWithFloat>;
            // using img3DPointStamped = mrs_msgs::ImagePointsWithFloatStampedConstPtr;

            virtual void onInit();
            void loadParams(const bool & );
            bool parseSequenceFile(const std::string &);
            bool checkCameraTopicSizeMatch();
            void initAlternativeHTDataStructure();
            void subscribeToPublishedPoints();
            void insertPointToAHT(const mrs_msgs::ImagePointsWithFloatStampedConstPtr &, const size_t &);

            void updateBufferAndSetFirstCallBool(const size_t & img_index);

            void ProcessThread([[maybe_unused]] const ros::TimerEvent&, size_t);

            ros::NodeHandle private_nh_;

            std::vector<std::shared_ptr<alternativeHT>> aht_;
            std::vector<vectPoint3D> pVect;
            std::vector<vectPoint3D> potentialSequences; 

            std::atomic_bool initialized_ = false;  
            std::atomic_bool current_visualization_done_ = false;
            ros::Timer timer_visualization_;

            std::vector<std::vector<bool>> sequences_;
            std::vector<ros::Publisher> pub_blinkers_seen_;
            std::vector<ros::Publisher> pub_estimated_framerate_;
            std::vector<ros::Timer> timer_process_;


            std::vector<std::string> _blinkers_seen_topics;
            std::vector<std::string> _estimated_framerate_topics;
            std::vector<std::string> _points_seen_topics;
            std::string _sequence_file;
            int _number_sequences;
            std::vector<std::vector<vectPoint3D>> small_buffer_;
            int buffer_cnt_ = 0;
            bool first_call_; // bool for preventing access of non assigned values in small_buffer

            int process_rate = 1;


            const int max_buffer_size_ = 5; // max frames which are stored
            const int min_buffer_size_ = 3; // min consecutive frames - required due to Manchester Coding 
               

            using image_callback_t = boost::function<void (const sensor_msgs::ImageConstPtr&)>;
            std::vector<image_callback_t> cals_image_;
            std::vector<ros::Subscriber> sub_image_;

            using points_seen_callback_t = boost::function<void (const mrs_msgs::ImagePointsWithFloatStampedConstPtr&)>;
            std::vector<points_seen_callback_t> cals_points_seen_;
            std::vector<points_seen_callback_t> cals_sun_points_;
            std::vector<ros::Subscriber> sub_points_seen_;
            std::vector<ros::Subscriber> sub_sun_points_;

            
            // Params
            std::string _uav_name_;   
            bool        _debug_;
            bool        _visual_debug_;
            bool        _gui_;
            bool        _publish_visualization_;
            float       _visualization_rate_;
            bool        _use_camera_for_visualization_;
            bool        _enable_manchester_;
            int         _buffer_size_;


    };

}