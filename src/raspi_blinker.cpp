#include <ros/ros.h>

#include <mrs_lib/param_loader.h>

#include <mrs_msgs/Float64Srv.h>
#include <mrs_msgs/SetInt.h>
#include <std_srvs/SetBool.h>
#include <uvdar_core/SetInts.h>

#include <fstream>
#include <signal.h>
#include <fcntl.h>
#include <string.h>

#define BLINK_GPIO_PIN 25
#define INIT_BITRATE 60

#define LED std::string("/sys/class/leds/uvled/")

namespace uvdar {


  /**
   * @brief A processing class for controlling the blinking UV LEDs for UVDAR on Raspberry Pi-based platforms
   */
  class Raspi_UVDAR_Blinker {
    private:
      /* attributes //{ */

      bool _debug_ = false;
      std::string _uav_name_;

      //}
      //

      bool initialized_ = false;

      int _mode_ = 0;

      std::string _sequence_file_;

      std::mutex sequence_mutex;
      std::vector<std::vector<bool>> _sequences_;
      int selected_sequence_ = 0;
      bool _use_custom_sequence_;
      std::vector<bool> custom_sequence_ = {0,1}; //default custom sequence will just blink with squarewave
      int curr_index_ = -1;
      int prev_index_ = -2;
      bool blanking_ = false; //after change of the sequence we will broadcast a full one sequence of zeros to avoid false matching
      int blanking_index_ = 0; //used for counding down bits till we restart blinking after changing the sequence

      bool active_ = true;


      ros::Timer timer;

      ros::ServiceServer serv_set_active;
      ros::ServiceServer serv_frequency;
      ros::ServiceServer serv_select_sequence;
      ros::ServiceServer serv_use_custom;
      ros::ServiceServer serv_set_custom;

    public:
      /**
       * @brief Constructor - loads parameters and initializes necessary structures
       *
       * @param nh Private NodeHandle of this ROS node
       */
      /* Constructor //{ */
      Raspi_UVDAR_Blinker(ros::NodeHandle nh) {

        mrs_lib::ParamLoader param_loader(nh, "RaspiUVDARBlinker");

        param_loader.loadParam("uav_name", _uav_name_, std::string());

        param_loader.loadParam("mode", _mode_, 0);

        param_loader.loadParam("use_custom_sequence", _use_custom_sequence_, false);

        param_loader.loadParam("sequence_file", _sequence_file_, std::string());

        int _initial_id = -1;
        param_loader.loadParam("initial_id", _initial_id);

        if (_initial_id >= 0){
          selected_sequence_ = _initial_id;
        }

        ROS_INFO_STREAM("[Raspi_UVDAR_blinker]: Loading sequences from file " << _sequence_file_);
        if ((!parseSequenceFile(_sequence_file_)) || ((int)(_sequences_.size()) < 1)){
          ROS_INFO_STREAM("[Raspi_UVDAR_blinker]: Failed to load file " << _sequence_file_);
          return;
        }

        ROS_INFO_STREAM("[Raspi_UVDAR_blinker]: GPIO pin " << BLINK_GPIO_PIN << " has been set as OUTPUT.");

        serv_set_active = nh.advertiseService("set_active", &Raspi_UVDAR_Blinker::callbackSetActive, this);

        serv_frequency = nh.advertiseService("set_frequency", &Raspi_UVDAR_Blinker::callbackSetFrequency, this);

        serv_select_sequence = nh.advertiseService("select_sequence", &Raspi_UVDAR_Blinker::callbackSelectSequence, this);

        serv_use_custom = nh.advertiseService("use_custom_sequence", &Raspi_UVDAR_Blinker::callbackUseCustom, this);

        serv_set_custom = nh.advertiseService("set_custom_sequence", &Raspi_UVDAR_Blinker::callbackSetCustom, this);

        timer = nh.createTimer(INIT_BITRATE, &Raspi_UVDAR_Blinker::spin, this);

        initiateBlinker();

        initialized_ = true;
      }
      //}

      /**
       * @brief Thread for blinking - each iteration represents transmission of a bit
       *
       * @param te TimerEvent for the timer spinning this thread
       */
      /* spin //{ */
      void spin([[ maybe_unused ]] const ros::TimerEvent& te){
        if (!initialized_)
          return;

        if (!active_){
          setLED(false);
          return;
        }


        std::scoped_lock lock(sequence_mutex);

        std::vector<bool> sequence = (_use_custom_sequence_?custom_sequence_:_sequences_[selected_sequence_]);

        if ((blanking_) && (blanking_index_ < (int)(sequence.size()))){
          setLED(false);
          blanking_index_++;
          if (blanking_index_ >= (int)(sequence.size())){
            blanking_ = false;
            blanking_index_ = 0;
          }
          return;
        }

        curr_index_++;
        if (curr_index_ >= (int)(sequence.size())){
          curr_index_ = 0;
        }

        if (sequence[curr_index_] != sequence[prev_index_]){
          setLED(sequence[curr_index_]);
        }

        prev_index_ = curr_index_;
      }
      //}

      bool callbackSetActive(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
        if (!initialized_){
          ROS_ERROR("[Raspi_UVDAR_Blinker]: Blinker is NOT initialized!");
          res.success = false;
          res.message = "Blinker is NOT initialized!";
          return true;
        }

        active_ = req.data;

        res.message = std::string("Turning blinker "+std::string(active_?"ON":"OFF")).c_str();
        res.success = true;

        ROS_INFO_STREAM("[Raspi_UVDAR_Blinker]: " << res.message);

        return true;
      }

      bool callbackSetFrequency(mrs_msgs::Float64Srv::Request &req, mrs_msgs::Float64Srv::Response &res){
        if (!initialized_){
          ROS_ERROR("[Raspi_UVDAR_Blinker]: Blinker is NOT initialized!");
          res.success = false;
          res.message = "Blinker is NOT initialized!";
          return true;
        }


        unsigned short int_frequency = (unsigned short)(req.value); // Hz

        timer.setPeriod(ros::Duration(1.0/int_frequency));

        res.message = std::string("Setting the frequency to "+std::to_string((int)(int_frequency))+" Hz").c_str();
        res.success = true;

        ROS_INFO_STREAM("[Raspi_UVDAR_Blinker]: " << res.message);


        return true;
      }

      bool callbackSelectSequence(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res){
        if (!initialized_){
          ROS_ERROR("[Raspi_UVDAR_Blinker]: Blinker is NOT initialized!");
          res.success = false;
          res.message = "Blinker is NOT initialized!";
          return true;
        }

        if (_mode_ != 0){
          //TODO - this "mode" setting can be potentially replaced by simply calling in the custom sequence services
          ROS_ERROR("[Raspi_UVDAR_Blinker]: Requesting sequence selection, but the appropriate mode is not set!");
          res.success = false;
          res.message = "Requesting sequence selection, but the appropriate mode is not set!";
          return true;
        }

        unsigned char index = (unsigned char)(req.value);
        if (index >= (int)(_sequences_.size())){
          ROS_ERROR_STREAM("[Raspi_UVDAR_Blinker]: Failed to set sequence " << index << " - no such sequence!");
          res.message = "Failed to select sequence " + std::to_string((int)(index)) +" - no such sequence!";
          res.success = false;
          return true;
        }

        {
          std::scoped_lock lock(sequence_mutex);
          selected_sequence_ = index;
          blanking_ = true;
          blanking_index_ = 0;
        }




        return true;
      }

      bool callbackUseCustom(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
        if (!initialized_){
          ROS_ERROR("[Raspi_UVDAR_Blinker]: Blinker is NOT initialized!");
          res.success = false;
          res.message = "Blinker is NOT initialized!";
          return true;
        }

        {
          std::scoped_lock lock(sequence_mutex);
          _use_custom_sequence_ = req.data;
          blanking_ = true;
          blanking_index_ = 0;
        }

        return true;
      }

      bool callbackSetCustom(uvdar_core::SetInts::Request &req, uvdar_core::SetInts::Response &res){
        if (!initialized_){
          ROS_ERROR("[Raspi_UVDAR_Blinker]: Blinker is NOT initialized!");
          res.success = false;
          res.message = "Blinker is NOT initialized!";
          return true;
        }

        {
          std::scoped_lock lock(sequence_mutex);
          custom_sequence_.clear();
          for (auto bit : req.value){
            custom_sequence_.push_back((bool)(bit));
          }
          blanking_ = true;
          blanking_index_ = 0;
        }

        return true;
      }

      bool parseSequenceFile(std::string sequence_file){
        ROS_WARN_STREAM("[Raspi_UVDAR_Blinker]: Add sanitation - sequences must be of equal, non-zero length");
        ROS_INFO_STREAM("[Raspi_UVDAR_Blinker]: Loading sequence from file: [ " + sequence_file + " ]");
        std::ifstream ifs;
        ifs.open(sequence_file);
        std::string word;
        std::string line;

        std::vector<std::vector<bool>> sequences;
        if (ifs.good()) {
          ROS_INFO("[Raspi_UVDAR_Blinker]: Loaded Sequences: [: ");
          while (getline( ifs, line )){
            if (line[0] == '#'){
              continue;
            }
            std::string show_string = "";
            std::vector<bool> sequence;
            std::stringstream iss(line); 
            std::string token;
            while(std::getline(iss, token, ',')) {
              sequence.push_back(token=="1");
              if (sequence.back()){
                show_string += "1,";
              }
              else {
                show_string += "0,";
              }
            }
            sequences.push_back(sequence);
            ROS_INFO_STREAM("[Raspi_UVDAR_Blinker]:   [" << show_string << "]");
          }
          ROS_INFO("[Raspi_UVDAR_Blinker]: ]");
          ifs.close();

          _sequences_ = sequences;
        }
        else {
          ROS_ERROR_STREAM("[Raspi_UVDAR_Blinker]: Failed to load sequence file " << sequence_file << "! Returning.");
          ifs.close();
          return false;
        }
        return true;
      }

      void stopAndClear(){
        ROS_INFO("[Raspi_UVDAR_Blinker]: Stopping timer...");
        timer.stop();
        ROS_INFO("[Raspi_UVDAR_Blinker]: Clearing LED GPIO pin %d...",BLINK_GPIO_PIN);
        setLED(false);
        ROS_INFO("[Raspi_UVDAR_Blinker]: Done.");
        return;
      }


      void write_to_file(std::string path, std::string value) {
        int fd = open(path.c_str(), O_WRONLY);
        if (fd < 0) {
          perror("open");
          exit(EXIT_FAILURE);
        }
        if (write(fd, value.c_str(), strlen(value.c_str())) < 0) {
          perror("write");
          close(fd);
          exit(EXIT_FAILURE);
        }
        close(fd);
      }

      void initiateBlinker(){
        write_to_file(LED+"trigger", "none\n");
      }

      void setLED(bool state){
        write_to_file(LED+"brightness", state?"1":"0");
      }

  };
}

std::unique_ptr<uvdar::Raspi_UVDAR_Blinker> rub;

//cleanup - if I stop the program it should turn the LEDs off
void sigintHandler([[maybe_unused]] int s) 
{
  rub->stopAndClear();
  ros::shutdown();
  return;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "raspi_UVDAR_blinker");
  ros::NodeHandle nh("~");
  rub = std::make_unique<uvdar::Raspi_UVDAR_Blinker>(nh);

  signal(SIGINT, sigintHandler);

  ROS_INFO("[Raspi_UVDAR_blinker]: UWB-UVDAR fuser node initiated");

  ros::spin();

  return 0;
}
