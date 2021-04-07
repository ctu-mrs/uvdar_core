#include <ros/ros.h>
#include <ros/package.h>
#include <mrs_msgs/BacaProtocol.h>
#include <mrs_msgs/SetInt.h>
#include <mrs_msgs/Float64Srv.h>
#include <std_srvs/Trigger.h>
#include <mrs_lib/param_loader.h>
#include <fstream>

namespace uvdar {
  class SequenceSetter {
    private: 
      std::string _sequence_file_;
      std::vector<std::vector<bool>> sequences_;

      bool initialized = false;
      ros::Publisher baca_protocol_publisher;

      ros::Duration sleeper = ros::Duration(0.01);

      ros::ServiceServer serv_frequency;
      ros::ServiceServer serv_load_file;
      ros::ServiceServer serv_select_sequence;
      ros::ServiceServer serv_quick_start;
    public:
      SequenceSetter(ros::NodeHandle nh){
        baca_protocol_publisher = nh.advertise<mrs_msgs::BacaProtocol>("baca_protocol_out", 1);

        mrs_lib::ParamLoader param_loader(nh, "UVDARSequenceSetter");
        param_loader.loadParam("sequence_file", _sequence_file_, std::string());

        serv_frequency = nh.advertiseService("set_frequency", &SequenceSetter::callbackSetFrequency, this);
        serv_load_file = nh.advertiseService("load_sequence_file", &SequenceSetter::callbackLoadSequenceFile, this);
        serv_select_sequence = nh.advertiseService("select_sequence", &SequenceSetter::callbackSelectSequence, this);
        serv_quick_start = nh.advertiseService("quick_start", &SequenceSetter::callbackQuickStart, this);
        initialized = true;
      }

      ~SequenceSetter(){
      }


    private:
      bool callbackSetFrequency(mrs_msgs::Float64Srv::Request &req, mrs_msgs::Float64Srv::Response &res){
        unsigned char int_frequency = (unsigned char)(req.value);

        mrs_msgs::BacaProtocol serial_msg;
        serial_msg.stamp = ros::Time::now();

        serial_msg.payload.push_back(0x96); //set frequency
        serial_msg.payload.push_back(int_frequency); //# Hz
        baca_protocol_publisher.publish(serial_msg);

        res.message = std::string("Setting the frequency to "+std::to_string((int)(int_frequency))+" Hz").c_str();
        res.success = true;

        return true;
      }

      bool callbackLoadSequenceFile(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
        ROS_INFO_STREAM("[UVDARSequenceSetter]: Loading sequences from file " << _sequence_file_);
        if ((!parseSequenceFile(_sequence_file_)) || ((int)(sequences_.size()) < 1)){
          res.message = "Failed to load file "+_sequence_file_;
          res.success = false;
          return true;
        }

        mrs_msgs::BacaProtocol serial_msg;
        serial_msg.stamp = ros::Time::now();

        unsigned char sequence_length = (unsigned char)(sequences_[0].size());
        serial_msg.payload.push_back(0x97); //set sequence length
        serial_msg.payload.push_back(sequence_length); //# bits
        baca_protocol_publisher.publish(serial_msg);
        sleeper.sleep();


        unsigned char i = 0;
        for (auto sq : sequences_){
          serial_msg.payload.clear();
          serial_msg.payload.push_back(0x99); //write sequences
          serial_msg.payload.push_back(i); //sequence index is i

          for (auto b : sq){
            serial_msg.payload.push_back(b?0x01:0x00); //bit of the sequence
          }

          baca_protocol_publisher.publish(serial_msg);
          sleeper.sleep();
          i++;
        }

        res.message = "Loaded the sequences from file "+_sequence_file_;
        res.success = true;
        return true;
      }

      bool callbackSelectSequence(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res){
        unsigned char index = (unsigned char)(req.value);

        mrs_msgs::BacaProtocol serial_msg;
        serial_msg.stamp = ros::Time::now();

        serial_msg.payload.push_back(0x98); //select sequence index
        serial_msg.payload.push_back(index); //sequence #
        baca_protocol_publisher.publish(serial_msg);

        res.success = true;
        res.message="Selecting sequence "+std::to_string((int)(index));
        return true;
      }

      bool callbackQuickStart(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res){
        std_srvs::Trigger::Request dummy_trig_req;
        std_srvs::Trigger::Response dummy_trig_res;
        callbackLoadSequenceFile(dummy_trig_req, dummy_trig_res);
        if (dummy_trig_res.success == false){
          res.message = dummy_trig_res.message;
          res.success = false;
          return true;
        }
        mrs_msgs::Float64Srv::Request dummy_float_req;
        mrs_msgs::Float64Srv::Response dummy_float_res;
        dummy_float_req.value = (double)(60);
        sleeper.sleep();
        callbackSetFrequency(dummy_float_req,dummy_float_res);
        sleeper.sleep();
        callbackSelectSequence(req,res);

        return true;
      }

        bool parseSequenceFile(std::string sequence_file){
          ROS_WARN_STREAM("[UVDARSequenceSetter]: Add sanitation - sequences must be of equal, non-zero length");
          ROS_INFO_STREAM("[UVDARSequenceSetter]: Loading sequence from file: [ " + sequence_file + " ]");
          std::ifstream ifs;
          ifs.open(sequence_file);
          std::string word;
          std::string line;

          std::vector<std::vector<bool>> sequences;
          if (ifs.good()) {
            ROS_INFO("[UVDARSequenceSetter]: Loaded Sequences: [: ");
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
              ROS_INFO_STREAM("[UVDARSequenceSetter]:   [" << show_string << "]");
            }
            ROS_INFO("[UVDARSequenceSetter]: ]");
            ifs.close();

            sequences_ = sequences;
          }
          else {
            ROS_ERROR_STREAM("[UVDARSequenceSetter]: Failed to load sequence file " << sequence_file << "! Returning.");
            ifs.close();
            return false;
          }
          return true;
        }
  };
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "uvdar_sequence_setter");
  ros::NodeHandle nodeA("~");
  uvdar::SequenceSetter        kl(nodeA);

  ROS_INFO("[UVDARSequenceSetter]: blinking sequence setter node initiated");

  ros::spin();

  return 0;
}
