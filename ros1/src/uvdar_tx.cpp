#include <ros/ros.h>
#include <ros/package.h>
#include <mrs_lib/param_loader.h>
#include <uvdar_core/ImagePointsWithFloatStamped.h>
/* #include <uvdar_gazebo_plugin/LedInfo.h> */
#include <fstream>
#include <uvdar_core/USM.h>
#include <mrs_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <string>
#include <cmath>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <mrs_msgs/SetInt.h>
#include <mrs_msgs/Float64Srv.h>
#include <uvdar_core/SetLedMessage.h>
#include <uvdar_core/DefaultMsg.h>

#define MSG_BITS 8

std::string id_sequence_file;
std::vector<std::vector<bool>> sequences_;
std::vector<int> msg_manchester(2*MSG_BITS, 0);  

int                         uav_id = 0;
int                         set_rate = 0;
int                         bit_duplication_amount = 1;
std::string                 uav_name;
/* std::vector<ros::Publisher> pub_led_states; */
std::vector<std::string>    leds_topics;
std::vector<int>            curr_msg;
std::vector<int>            curr_frame;

uvdar_core::SetLedMessage led_msg;
int                              rate = (int)(80 / 3);  // three frames per bit. TODO variable rate based on estimated camera frequency
ros::ServiceClient               led_message_client;
ros::ServiceClient               led_mode_client;
ros::ServiceClient               decode_mode_client;
ros::ServiceClient               led_frequency_client;

std::string sig_setter_service;
std::string mode_setter_service;
std::string decode_mode_setter_service;
std::string frequency_setter_service;
std::string msgs_topic;

ros::Subscriber    sub_msg;

int payload_ = 0;

struct Msg2send
{
  bool               blank_msg;
  int                msg_type;
  std::vector<float> payload;
};

std::vector<Msg2send> msg_queue;


namespace TX
{

class TX_processor {
public:
  TX_processor(ros::NodeHandle& nh) {
    mrs_lib::ParamLoader param_loader(nh, "UVDARtx");

    param_loader.loadParam("uav_name", uav_name);
    param_loader.loadParam("uav_id", uav_id);
    /* param_loader.loadParam("leds_topics", leds_topics, leds_topics);  // gazebo topics for led frequency setting */
    param_loader.loadParam("id_sequence_file", id_sequence_file);
    param_loader.loadParam("sig_setter_service", sig_setter_service);
    param_loader.loadParam("mode_setter_service", mode_setter_service);
    param_loader.loadParam("decode_mode_setter_service", decode_mode_setter_service);
    param_loader.loadParam("frequency_setter_service", frequency_setter_service);
    param_loader.loadParam("msgs_topic", msgs_topic);
    param_loader.loadParam("set_rate", set_rate);
    param_loader.loadParam("bit_duplication_amount", bit_duplication_amount);

    /* rate = (int)(set_rate / bit_duplication_amount); */

    sub_msg  = nh.subscribe("/" + uav_name + "/uvcom/send_msg", 1, &TX_processor::subMsg, this);    // sub for get info about heading

    led_message_client = nh.serviceClient<uvdar_core::SetLedMessage>(sig_setter_service);
    led_mode_client = nh.serviceClient<mrs_msgs::SetInt>(mode_setter_service);
    decode_mode_client = nh.serviceClient<mrs_msgs::SetInt>(decode_mode_setter_service);
    led_frequency_client = nh.serviceClient<mrs_msgs::Float64Srv>(frequency_setter_service);

    parseSequenceFile(id_sequence_file);
    /* ROS_INFO("ID sequence file: %s", id_sequence_file.c_str()); */

    ROS_INFO("Node initialized %s", uav_name.c_str());
  }
  
  std::vector<int> manchesterEncode(const std::vector<int>& binaryVector) {
      std::vector<int> manchester;

      // Traverse through the original binary vector
      for (int bit : binaryVector) {
          if (bit == 0) {
              // For 0, append 01
              manchester.push_back(0);
              manchester.push_back(1);
          } else {
              // For 1, append 10
              manchester.push_back(1);
              manchester.push_back(0);
          }
      }

      return manchester;
  }

  void printBinaryVector(const std::vector<int>& binary) {
    for (int bit : binary) {
        std::cout << bit;
    }
    std::cout << std::endl;
  }


private:
  bool parseSequenceFile(const std::string &sequence_file) {

    ROS_INFO_STREAM("[UVDAR TX]: Loading sequence from file: [ " + sequence_file + " ]");
    std::ifstream ifs;
    ifs.open(sequence_file);
    std::string word;
    std::string line;
    std::vector<std::vector<bool>> sequences;
    if (ifs.good()){
      ROS_INFO("[UVDARBlinkProcessor]: Loaded Sequences: [: ");
      while (getline(ifs, line)){
        if (line[0] == '#'){
          continue;
        }
        std::string show_string = "";
        std::vector<bool> sequence;
        std::stringstream iss(line);
        std::string token;
        while (std::getline(iss, token, ',')){
          sequence.push_back(token == "1");
        }

        for (const auto bool_val : sequence){
          if (bool_val)
            show_string += "1,";
          else
            show_string += "0,";
        }

        sequences.push_back(sequence);
        ROS_INFO_STREAM("[UVDARBlinkProcessor]: [" << show_string << "]");
      }
      ROS_INFO("[UVDARBlinkProcessor]: ]");
      ifs.close();
      sequences_ = sequences;
    }else{
      ROS_ERROR_STREAM("[UVDARBlinkProcessor]: Failed to load sequence file " << sequence_file << "! Returning.");
      ifs.close();
      return false;
    }
    return true;
  }

  void subMsg(const std_msgs::Int32 msg) {
    if(msg.data < 0 || msg.data > 255){
      ROS_ERROR("Value %d is out of the range [0-255]", msg.data);
    } else{
      int decimal = msg.data;
      std::vector<int> binary(MSG_BITS, 0);  // Initialize a vector of size 8 with all 0s
      
      std::cout << "Msg. decimal: ";
      std::cout << decimal;
      std::cout << std::endl;

      for (int i = (MSG_BITS - 1); i >= 0; --i) {
        binary[i] = (decimal & 1);  // Get the least significant bit
        decimal >>= 1;              // Shift the number right by 1 to process the next bit
      }
      
      std::vector<int> manchester = manchesterEncode(binary);
      msg_manchester = manchester;

      
      std::cout << "Msg. binary: ";
      printBinaryVector(binary);

      std::cout << "Msg. manchester: ";
      printBinaryVector(manchester);
    }
  }

};
}  // namespace TX

void fillUavId() {
  /* ROS_INFO("id: %d, seq len: %d", uav_id, (int)size(sequences_)); */
  std::vector<bool> sequence; 
  if((int)size(sequences_) >= (uav_id + 1)){
    sequence = sequences_[uav_id];
  } else{
    ROS_WARN("UAV ID not accesible");
    exit(0);
  }
  
  for (int i = 0; i < (int)size(sequence); i++) {
    curr_msg.push_back((int)sequence[i]);
  }
}

void fillPayload() {
  for(auto& b : msg_manchester){
    curr_msg.push_back(b);
  }
}

// function for create physical data frame of message.
void create_curr_msg() {
  curr_frame.clear();
  curr_msg.clear();

  fillUavId();

  fillPayload();

  curr_msg.push_back(1);
  curr_msg.push_back(1);
  curr_msg.push_back(1);
  curr_msg.push_back(1);

  curr_frame = curr_msg;

  curr_msg.clear();  // init data msg
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "UVDARtx");
  ros::NodeHandle  nh("~");
  TX::TX_processor txko(nh);
  ROS_INFO("[TX_processor] Node initialized");

  mrs_msgs::SetInt ledMode;
  ledMode.request.value = 1;
  led_mode_client.call(ledMode);
  decode_mode_client.call(ledMode);

  ROS_WARN("Communication mode set");

  mrs_msgs::Float64Srv ledFrequency;
  ledFrequency.request.value = set_rate;
  ledFrequency.request.value /= bit_duplication_amount; // to dublicate each bit
  led_frequency_client.call(ledFrequency);

  double bit_rate = ((double)set_rate / (double)bit_duplication_amount);

  std::vector<int> init_msg(MSG_BITS, 0);  // Initialize a vector of size 8 with zeros
  msg_manchester = txko.manchesterEncode(init_msg);

  while (ros::ok()) {
    
    create_curr_msg(); 
    std::cout << "Transmitted msg frame: ";
    txko.printBinaryVector(curr_frame);
    std::cout << std::endl;

    led_msg.request.data_frame.clear();
    
    for (auto b : curr_frame){
      led_msg.request.data_frame.push_back((b==0)?0:255);
    }

    led_message_client.call(led_msg);

    ros::Duration sleeper = ros::Duration((double)(curr_frame.size()) / bit_rate);

    curr_frame.clear();
    sleeper.sleep();
    ros::spinOnce();
  }
}

