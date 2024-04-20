#include <ros/ros.h>
#include <ros/package.h>
#include <mrs_modules_msgs/BacaProtocol.h>
#include <mrs_msgs/SetInt.h>
#include <mrs_msgs/Float64Srv.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <mrs_lib/param_loader.h>
#include <uvdar_core/SetLedMessage.h>
/* #include <uvdar_core/SetIntIndex.h> */
#include <uvdar_core/SetInts.h>
#include <fstream>

#define UVDAR_CLASSIC false

namespace uvdar {
  class SequenceSetter {
    private: 
      std::string _sequence_file_;
      std::vector<std::vector<bool>> sequences_;

      bool initialized = false;

      int mode = 0;

      std::string _uav_name_;

      ros::Publisher baca_protocol_publisher;

      ros::Duration sleeper = ros::Duration(0.25);

      ros::ServiceServer serv_set_active;

      ros::ServiceServer serv_frequency;
      ros::ServiceServer serv_load_sequences;
      ros::ServiceServer serv_load_single_sequence;
      ros::ServiceServer serv_select_single_sequence;
      ros::ServiceServer serv_select_sequences;
      ros::ServiceServer serv_quick_start;

      ros::ServiceServer serv_set_mode;
      ros::ServiceServer serv_set_message;

      std::vector<ros::ServiceClient> clients_set_sq_gz;
      std::vector<ros::ServiceClient> clients_set_fr_gz;
      std::vector<ros::ServiceClient> clients_set_md_gz;
      std::vector<ros::ServiceClient> clients_set_ms_gz;
      std::vector<ros::ServiceClient> clients_set_ac_gz;

    public:
      SequenceSetter(ros::NodeHandle nh){


        baca_protocol_publisher = nh.advertise<mrs_modules_msgs::BacaProtocol>("baca_protocol_out", 1);

        mrs_lib::ParamLoader param_loader(nh, "UVDARLedManager");

        param_loader.loadParam("uav_name", _uav_name_, std::string());

        param_loader.loadParam("sequence_file", _sequence_file_, std::string());

        ROS_INFO_STREAM("[UVDARLedManager]: Loading sequences from file " << _sequence_file_);
        if ((!parseSequenceFile(_sequence_file_)) || ((int)(sequences_.size()) < 1)){
          ROS_INFO_STREAM("[UVDARLedManager]: Failed to load file " << _sequence_file_);
          return;
        }

        serv_set_active = nh.advertiseService("set_active", &SequenceSetter::callbackSetActive, this);

        serv_frequency = nh.advertiseService("set_frequency", &SequenceSetter::callbackSetFrequency, this);
        serv_load_sequences = nh.advertiseService("load_sequences", &SequenceSetter::callbackLoadSequences, this);
        serv_load_single_sequence = nh.advertiseService("load_single_sequence", &SequenceSetter::callbackLoadSingleSequence, this);
        serv_select_single_sequence = nh.advertiseService("select_single_sequence", &SequenceSetter::callbackSelectSingleSequence, this);
        serv_select_sequences = nh.advertiseService("select_sequences", &SequenceSetter::callbackSelectSequences, this);
        serv_quick_start = nh.advertiseService("quick_start", &SequenceSetter::callbackQuickStart, this);

        serv_set_mode = nh.advertiseService("set_mode", &SequenceSetter::callbackSetMode, this);
        serv_set_message = nh.advertiseService("set_message", &SequenceSetter::callbackSetMessage, this);

        //for simulation
        for (int i = 0; i < 8; i++) {
          clients_set_sq_gz.push_back(nh.serviceClient<mrs_msgs::SetInt>("/gazebo/ledSignalSetter/" + _uav_name_ + "_" + std::to_string(i + 1)));
          clients_set_fr_gz.push_back(nh.serviceClient<mrs_msgs::Float64Srv>("/gazebo/ledFrequencySetter/" + _uav_name_ + "_" + std::to_string(i + 1)));
          clients_set_md_gz.push_back(nh.serviceClient<mrs_msgs::SetInt>("/gazebo/ledModeSetter/" + _uav_name_ +  "_" +std::to_string(i + 1)));
          clients_set_ms_gz.push_back(nh.serviceClient<uvdar_core::SetLedMessage>("/gazebo/ledMessageSender/" + _uav_name_ + "_" + std::to_string(i + 1)));
          clients_set_ac_gz.push_back(nh.serviceClient<std_srvs::SetBool>("/gazebo/ledActiveSetter/" + _uav_name_ +  "_" +std::to_string(i + 1)));
        }

        initialized = true;
      }

      ~SequenceSetter(){
      }


    private:
      bool callbackSetActive(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
        if (!initialized){
          ROS_ERROR("[UVDARLedManager]: LED manager is NOT initialized!");
          res.success = false;
          res.message = "LED manager is NOT initialized!";
          return true;
        }


        unsigned char state = (req.data?0x01:0x00);

        mrs_modules_msgs::BacaProtocol serial_msg;
        serial_msg.stamp = ros::Time::now();

        serial_msg.payload.push_back(0x90); //set frequency
        serial_msg.payload.push_back(state); //# Hz
        baca_protocol_publisher.publish(serial_msg);

        if (state){
          res.message = std::string("Activating the LEDs").c_str();
        }
        else {
          res.message = std::string("Deactivating the LEDs").c_str();
        }
        res.success = true;


        std_srvs::SetBool led_state;
        led_state.request.data = req.data;
        for (auto& client : clients_set_ac_gz)
          client.call(led_state);

        return true;
      }

      bool callbackSetFrequency(mrs_msgs::Float64Srv::Request &req, mrs_msgs::Float64Srv::Response &res){
        if (!initialized){
          ROS_ERROR("[UVDARLedManager]: LED manager is NOT initialized!");
          res.success = false;
          res.message = "LED manager is NOT initialized!";
          return true;
        }
          

        unsigned short int_frequency = (unsigned short)(req.value); // Hz

        mrs_modules_msgs::BacaProtocol serial_msg;
        serial_msg.stamp = ros::Time::now();

        serial_msg.payload.push_back(0x96); //set frequency
        serial_msg.payload.push_back((unsigned char) (int_frequency & 0x00FF)); // LSB-first
        serial_msg.payload.push_back((unsigned char) ((int_frequency & 0xFF00) >> 8));
        baca_protocol_publisher.publish(serial_msg);

        res.message = std::string("Setting the frequency to "+std::to_string((int)(int_frequency))+" Hz").c_str();
        res.success = true;

        ROS_INFO_STREAM("[UVDARLedManager]: " << res.message);


        mrs_msgs::Float64Srv led_state;
        led_state.request.value = (double)int_frequency;
        for (auto& client : clients_set_fr_gz)
          client.call(led_state);

        return true;
      }

      bool callbackLoadSequences([[ maybe_unused ]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
        if (!initialized){
          ROS_ERROR("[UVDARLedManager]: LED manager is NOT initialized!");
          res.success = false;
          res.message = "LED manager is NOT initialized!";
          return true;
        }

        ROS_INFO_STREAM("[UVDARLedManager]: Loading sequences into the LED driver");

        mrs_modules_msgs::BacaProtocol serial_msg;
        serial_msg.stamp = ros::Time::now();

        unsigned char sequence_length = (unsigned char)(sequences_[0].size());
        serial_msg.payload.push_back(0x97); //set sequence length
        serial_msg.payload.push_back(sequence_length); //# bits
        baca_protocol_publisher.publish(serial_msg);
        sleeper.sleep();


        unsigned char i = 0;
        ros::Duration local_sleeper(0.25 + 0.1*(sequences_[0].size()));
        for (auto sq : sequences_){
          serial_msg.payload.clear();
          serial_msg.payload.push_back(0x99); //write sequences
          serial_msg.payload.push_back(i); //sequence index is i

            /* ROS_INFO_STREAM("[UVDARLedManager]: s:" << (int)(i)); */
          for (auto b : sq){
            serial_msg.payload.push_back(b?0x01:0x00); //bit of the sequence
            /* ROS_INFO_STREAM("[UVDARLedManager]: b:" << (b?"1":"0")); */
          }

          /* if (i == 3) */
          baca_protocol_publisher.publish(serial_msg);

          local_sleeper.sleep();
          i++;
        }

        res.message = "Loaded the sequences";
        res.success = true;
        return true;
      }

      bool callbackLoadSingleSequence(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res){
        if (!initialized){
          ROS_ERROR("[UVDARLedManager]: LED manager is NOT initialized!");
          res.success = false;
          res.message = "LED manager is NOT initialized!";
          return true;
        }

        unsigned char index = (unsigned char)(req.value);
        if (index >= (int)(sequences_.size())){
          ROS_ERROR_STREAM("[UVDARLedManager]: Failed to load sequence " << index << " into the LED driver - no such sequence!");
          res.message = "Failed to load sequence " + std::to_string((int)(index)) +" into the LED driver - no such sequence!";
          res.success = false;
          return true;
        }

        ROS_INFO_STREAM("[UVDARLedManager]: Loading sequence " << index << " into the LED driver");

        mrs_modules_msgs::BacaProtocol serial_msg;
        serial_msg.stamp = ros::Time::now();

        serial_msg.payload.clear();
        serial_msg.payload.push_back(0x99); //write sequences
        serial_msg.payload.push_back(index); //sequence index is i

        /* ROS_INFO_STREAM("[UVDARLedManager]: s:" << (int)(i)); */
        for (auto b : sequences_[index]){
          serial_msg.payload.push_back(b?0x01:0x00); //bit of the sequence
          /* ROS_INFO_STREAM("[UVDARLedManager]: b:" << (b?"1":"0")); */
        }

        /* if (i == 3) */
        baca_protocol_publisher.publish(serial_msg);
        sleeper.sleep();

        res.message = ("Loaded sequence "+std::to_string((int)(index))).c_str();
        res.success = true;
        return true;
      }

      bool callbackSelectSingleSequence(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res){
        if (!initialized){
          ROS_ERROR("[UVDARLedManager]: LED manager is NOT initialized!");
          res.success = false;
          res.message = "LED manager is NOT initialized!";
          return true;
        }

        if (mode != 0){
          ROS_ERROR("[UVDARLedManager]: Requesting sequence selection, but the appropriate mode is not set!");
          res.success = false;
          res.message = "Requesting sequence selection, but the appropriate mode is not set!";
          return true;
        }

        unsigned char index = (unsigned char)(req.value);
        if (index >= (int)(sequences_.size())){
          ROS_ERROR_STREAM("[UVDARLedManager]: Failed to set sequence " << index << " - no such sequence!");
          res.message = "Failed to select sequence " + std::to_string((int)(index)) +" - no such sequence!";
          res.success = false;
          return true;
        }

        if (UVDAR_CLASSIC) {
          mrs_modules_msgs::BacaProtocol serial_msg;
          serial_msg.stamp = ros::Time::now();

          serial_msg.payload.push_back(0x98); //select sequence index
          serial_msg.payload.push_back(index); //sequence #
          baca_protocol_publisher.publish(serial_msg);

          res.success = true;
          res.message="Selecting sequence "+std::to_string((int)(index));
          ROS_ERROR_STREAM("[UVDARLedManager]: " << res.message);

          mrs_msgs::SetInt led_state;
          led_state.request.value = index;
          for (auto& client : clients_set_sq_gz)
            client.call(led_state);

          return true;
        }
        else {
          mrs_modules_msgs::BacaProtocol serial_msg;
          serial_msg.stamp = ros::Time::now();

          serial_msg.payload.push_back(0x98); //select sequence index
          serial_msg.payload.push_back(index); //sequence #
          serial_msg.payload.push_back(index); //sequence #
          serial_msg.payload.push_back(index); //sequence #
          serial_msg.payload.push_back(index); //sequence #
          baca_protocol_publisher.publish(serial_msg);

          res.success = true;
          res.message="Selecting sequence "+std::to_string((int)(index));
          ROS_ERROR_STREAM("[UVDARLedManager]: " << res.message);

          mrs_msgs::SetInt led_state;
          led_state.request.value = index;
          for (auto& client : clients_set_sq_gz)
            client.call(led_state);

          return true;
        }

        return false;
      }

      bool callbackSelectSequences(uvdar_core::SetInts::Request &req, uvdar_core::SetInts::Response &res){
        if (!initialized){
          ROS_ERROR("[UVDARLedManager]: LED manager is NOT initialized!");
          res.success = false;
          res.message = "LED manager is NOT initialized!";
          return true;
        }

        if (mode != 0){
          ROS_ERROR("[UVDARLedManager]: Requesting sequence selection, but the appropriate mode is not set!");
          res.success = false;
          res.message = "Requesting sequence selection, but the appropriate mode is not set!";
          return true;
        }

        if (UVDAR_CLASSIC) {
          ROS_ERROR_STREAM("[UVDARLedManager]: Failed to set sequencess because the attached UVDAR board does not support multiple sequence setting!");
          res.message = "Failed to set sequencess because the attached UVDAR board does not support multiple sequence setting!";
          res.success = false;
          return true;
        }

        std::vector<unsigned char> selected_sequences;

        for (auto sq : req.value){
          unsigned char index = (unsigned char)(sq);
          if (index >= (int)(sequences_.size())){
            ROS_ERROR_STREAM("[UVDARLedManager]: Failed to set sequencess due to " << std::to_string((int)(index)) << " - no such sequence!");
            res.message = "Failed to select sequences due to " + std::to_string((int)(index)) +" - no such sequence!";
            res.success = false;
            return true;
          }
          else {
            selected_sequences.push_back(sq);
          }
        }


        mrs_modules_msgs::BacaProtocol serial_msg;
        serial_msg.stamp = ros::Time::now();

        res.message="Selecting sequences to [ ";

        serial_msg.payload.push_back(0x98); //select sequence index
        for (auto sq : selected_sequences){
          serial_msg.payload.push_back(sq); //sequence #
          res.message += std::to_string((int)(sq))+" ";
        }
        res.message += "]";

        ROS_ERROR_STREAM("[UVDARLedManager]: " << res.message);

        baca_protocol_publisher.publish(serial_msg);

        res.success = true;

        mrs_msgs::SetInt led_state;
        int i = 0;
        for (auto& client : clients_set_sq_gz){
          led_state.request.value = selected_sequences[i/2];
          client.call(led_state);
          i++;
        }

        return true;
      }

      bool callbackQuickStart(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res){
        if (!initialized){
          ROS_ERROR("[UVDARLedManager]: LED manager is NOT initialized!");
          res.success = false;
          res.message = "LED manager is NOT initialized!";
          return true;
        }

        std_srvs::Trigger::Request dummy_trig_req;
        std_srvs::Trigger::Response dummy_trig_res;
        callbackLoadSequences(dummy_trig_req, dummy_trig_res);
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

        /* uvdar_core::SetIntIndex::Request ind_req; */
        /* uvdar_core::SetIntIndex::Response ind_res; */
        /* ind_req.value = req.value; */
        /* callbackSelectSingleSequence(req,res); */
        uvdar_core::SetInts::Request req_seqences;
        uvdar_core::SetInts::Response res_seqences;
        unsigned char val = (unsigned char)(req.value);
        req_seqences.value.push_back(4*val+0);
        req_seqences.value.push_back(4*val+1);
        req_seqences.value.push_back(4*val+2);
        req_seqences.value.push_back(4*val+3);
        callbackSelectSequences(req_seqences, res_seqences);

        res.success = true;
        char message[100];
        sprintf(message, "Quickstart done. Sequences set to [%d,%d,%d,%d].", 4*val+0, 4*val+1, 4*val+2, 4*val+3); 
        return true;
      }

        bool parseSequenceFile(std::string sequence_file){
          ROS_WARN_STREAM("[UVDARLedManager]: Add sanitation - sequences must be of equal, non-zero length");
          ROS_INFO_STREAM("[UVDARLedManager]: Loading sequence from file: [ " + sequence_file + " ]");
          std::ifstream ifs;
          ifs.open(sequence_file);
          std::string word;
          std::string line;

          std::vector<std::vector<bool>> sequences;
          if (ifs.good()) {
            ROS_INFO("[UVDARLedManager]: Loaded Sequences: [: ");
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
              ROS_INFO_STREAM("[UVDARLedManager]:   [" << show_string << "]");
            }
            ROS_INFO("[UVDARLedManager]: ]");
            ifs.close();

            sequences_ = sequences;
          }
          else {
            ROS_ERROR_STREAM("[UVDARLedManager]: Failed to load sequence file " << sequence_file << "! Returning.");
            ifs.close();
            return false;
          }
          return true;
        }

        bool callbackSetMode(mrs_msgs::SetInt::Request &req, mrs_msgs::SetInt::Response &res){
          if (!initialized){

          ROS_ERROR("[UVDARLedManager]: LED manager is NOT initialized!");
          res.success = false;
          res.message = "LED manager is NOT initialized!";
          return true;
          }

          unsigned char index = (unsigned char)(req.value); //0 - tracking mode; 1 - communication mode

          mrs_modules_msgs::BacaProtocol serial_msg;
          serial_msg.stamp = ros::Time::now();

          serial_msg.payload.push_back(0xF0); //select sequence index
          serial_msg.payload.push_back(index); //sequence #
          baca_protocol_publisher.publish(serial_msg);

          switch (index){
            case 0:
              res.message="Selecting tracking mode";
              break;
            case 1:
              res.message="Selecting communication mode";
          }

          ROS_INFO_STREAM("[UVDARLedManager]: " << res.message);

          mrs_msgs::SetInt led_state;
          led_state.request.value = index;
          for (auto& client : clients_set_md_gz)
            client.call(led_state);

          mode = index;

          res.success = true;
          return true;
        }

        bool callbackSetMessage(uvdar_core::SetLedMessage::Request &req, uvdar_core::SetLedMessage::Response &res){
          if (!initialized){
          ROS_ERROR("[UVDARLedManager]: LED manager is NOT initialized!");
          res.success = false;
          res.message = "LED manager is NOT initialized!";
          return true;
          }

          if (mode != 1){
            ROS_ERROR("[UVDARLedManager]: Requesting message transmission, but the appropriate mode is not set!");
            res.success = false;
            res.message = "Requesting message transmission, but the appropriate mode is not set!";
            return true;
          }

          auto data_frame = req.data_frame;

          mrs_modules_msgs::BacaProtocol serial_msg;

          res.message = "Sending message";

          ROS_INFO_STREAM("[UVDARLedManager]: Sending message");

          serial_msg.payload.push_back(0x93);
          for (auto& bit : data_frame) {
            serial_msg.payload.push_back(bit);
          }
          baca_protocol_publisher.publish(serial_msg);

          uvdar_core::SetLedMessage led_message;
          led_message.request.data_frame = data_frame;
          for (auto& client : clients_set_ms_gz)
            client.call(led_message);
          
          res.success = true;
          return true;
        }

  };
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "uvdar_led_manager");
  ros::NodeHandle nodeA("~");
  uvdar::SequenceSetter        kl(nodeA);

  ROS_INFO("[UVDARLedManager]: blinking sequence setter node initiated");

  ros::spin();

  return 0;
}
