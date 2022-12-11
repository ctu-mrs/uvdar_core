#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nodelet/nodelet.h>
#include <stdio.h>
#include <mrs_lib/param_loader.h>
#include <std_msgs/Float32.h>
#include <mrs_msgs/ImagePointsWithFloatStamped.h>
#include <fstream>




#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>


namespace uvdar {

    class UVDAR_BP_Tim : public nodelet::Nodelet {
       
       
        public:

            UVDAR_BP_Tim(); 
            ~UVDAR_BP_Tim();
        
        private:

            virtual void onInit();
            void loadParams(const bool &, ros::NodeHandle &);
            bool parseSequenceFile(const std::string &);
            bool checkVectorSizeMatch(const std::vector<std::string> &, const std::vector<std::string> &, const std::vector<std::string> & );
            void subscribeToPublishedPoints(ros::NodeHandle &);
            void getResults(ros::NodeHandle &);
            void processPoint(const mrs_msgs::ImagePointsWithFloatStampedConstPtr &, const size_t &);
            void processSunPoint(const mrs_msgs::ImagePointsWithFloatStampedConstPtr &, const size_t &) ;
            void processBuffer(std::vector<mrs_msgs::ImagePointsWithFloatStampedConstPtr> &);


            std::atomic_bool initialized_ = false;  
            std::atomic_bool current_visualization_done_ = false;
            ros::Timer timer_visualization_;
            std::vector<std::vector<bool>> _sequences_;
            std::vector<ros::Publisher> pub_blinkers_seen_;
            std::vector<ros::Publisher> pub_estimated_framerate_;
            std::vector<ros::Timer> timer_process_;
            std::vector<std::shared_ptr<std::mutex>>  mutex_camera_image_;


            std::vector<std::string> _blinkers_seen_topics;
            std::vector<std::string> _estimated_framerate_topics;
            std::vector<std::string> _points_seen_topics;
            std::string _sequence_file;

            std::vector<std::vector<mrs_msgs::ImagePointsWithFloatStampedConstPtr>> small_buffer_;

               

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
            int   _buffer_size_;


    };

}