#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <stdio.h>
#include <mrs_lib/param_loader.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>


namespace uvdar
{

    class Hello_World : public nodelet::Nodelet
    {
        public:
            Hello_World()
            {
            }

        private:
            void onInit()
            {
                ros::NodeHandle& private_nh = getPrivateNodeHandle();
                NODELET_DEBUG("Initializing nodelet...");

                bool loaded = false; 
                mrs_lib::ParamLoader param_loader(private_nh, loaded, "UVDARBlinkProcessor");

                param_loader.loadParam("uav_name", _uav_name_, std::string());

                // ROS_INFO_STREAM(_uav_name_);
                
                cv::namedWindow("view");

                image_transport::ImageTransport it(private_nh);
                // image_transport::Subscriber sub = it.subscribe( _uav_name_ + "/uvdar_bluefox_left/image_raw", 1, imageCallback);
                // cv::destroyWindow("view");
                
                auto callback = [this] (const sensor_msgs::ImageConstPtr& msg ) { 
                  imageCallback(msg);
                };

                sub_images_.push_back(it.subscribe( _uav_name_ + "/uvdar_bluefox_left/image_raw", 1, callback));


                // ROS_INFO_STREAM(typeid(callback).name());

                initialized_ = true;   
            }

                        
            void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
                if (!initialized_) return;

                try
                {
                    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
                    ROS_INFO_STREAM("Current number of subscribed images!" + sub_images_.size());
                    cv::waitKey(30);
                }
                catch (cv_bridge::Exception& e)
                {
                    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
                }
            }
            ros::Subscriber sub;
            std::atomic_bool initialized_ = false;  
            std::string _uav_name_;   
            std::vector<image_transport::Subscriber> sub_images_;
    };
} // uvdar namespace


PLUGINLIB_EXPORT_CLASS(uvdar::Hello_World, nodelet::Nodelet);

