#include <ros/ros.h>
#include <ros/package.h>
#include <mrs_lib/param_loader.h>
#include <mrs_msgs/ImagePointsWithFloatStamped.h>
/* #include <uvdar_gazebo_plugin/LedInfo.h> */
#include <uvdar_core/USM.h>
#include <mrs_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <string>
#include <cmath>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <uvdar_core/SetLedState.h>
#include <uvdar_core/DefaultMsg.h>

#define SEPARATOR_BITS 5

int                         uav_id = 0;
std::string                 uav_name;
/* std::vector<ros::Publisher> pub_led_states; */
std::vector<std::string>    leds_topics;
std::vector<int>            curr_msg;
std::vector<int>            curr_frame;
// std::vector<int> curr_frame_raw;
uvdar_core::SetLedState ledMsg;
int                              rate = (int)(80 / 3);  // three frames per bit. TODO variable rate based on estimated camera frequency
ros::Subscriber                  USmsgSub;
ros::Subscriber                  OdomSub;
ros::ServiceClient               LedStateClient;

std::string fr_setter_topic;
std::string odom_topic;
std::string msgs_topic;

/* double default_msg; */
ros::Subscriber    sub_default_msg;

float act_heading;

struct DefMsg
{
  bool enable = false;;
  double msg = 0.0;
};

DefMsg default_msg;

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
    param_loader.loadParam("leds_topics", leds_topics, leds_topics);  // gazebo topics for led frequency setting
    param_loader.loadParam("fr_setter_topic", fr_setter_topic);
    param_loader.loadParam("odom_topic", odom_topic);
    param_loader.loadParam("msgs_topic", msgs_topic);

    USmsgSub = nh.subscribe(msgs_topic, 1, &TX_processor::usm_cb, this);  // sub for get new custom command to send
    OdomSub  = nh.subscribe(odom_topic, 1, &TX_processor::odom_cb, this);    // sub for get info about heading
    
    sub_default_msg  = nh.subscribe("/" + uav_name + "/uvdar_communication/default_angle_msg", 1, &TX_processor::defMsg, this);    // sub for get info about heading

    LedStateClient = nh.serviceClient<uvdar_core::SetLedState>(fr_setter_topic);

    // creating publishers for leds
    for (size_t i = 0; i < leds_topics.size(); ++i) {

      ROS_INFO("Topic loaded %s", leds_topics[i].c_str());
      /* pub_led_states.push_back(nh.advertise<uvdar_gazebo_plugin::LedInfo>(leds_topics[i], 1)); */
    }

    ROS_INFO("Node initialized %s", uav_name.c_str());
  }

private:
  // callback for heading calculation. Heading is coded and sent in the first data byte together with uav id and data type
  void odom_cb(const nav_msgs::Odometry& msg) {
    geometry_msgs::Quaternion q = msg.pose.pose.orientation;
    act_heading                 = toYaw(q.x, q.y, q.z, q.w);
  }
  
  void defMsg(const uvdar_core::DefaultMsg& msg) {
    default_msg.enable = msg.enable;
    default_msg.msg = msg.message;
  }

  // to calculate heading from quaternion
  float toYaw(float x, float y, float z, float w) {
    float siny = +2.0 * (w * z + x * y);
    float cosy = +1.0 - 2.0 * (y * y + z * z);
    return atan2(siny, cosy);
  }

  // callback for load custom msg into queue for send through uvdar comm. channel
  void usm_cb(const uvdar_core::USM& msg) {
    Msg2send rec;
    rec.blank_msg = msg.blank_msg;
    rec.msg_type  = msg.msg_type;
    rec.payload   = msg.payload;
    msg_queue.push_back(rec);
    // ROS_INFO("Received msg for send through uvdar channel %d, %f, %f", msg.msg_type.data, msg.azimuth.data, msg.velocity.data);
  }

};
}  // namespace TX

int setPayloadVelocity(float pl) {
  if (pl < 0)
    pl = 0;  // limit velocity
  if (pl > 1.5)
    pl = 1.5;
  return (int)(pl * 10);  // make a data in range 0-15
}

int setPayloadAzimuth(float pl) {
  if (pl < 0)
    pl = 0;
  if (pl > 360)
    pl = 360;
  return (int)(pl / 22.5);  // make a data in range 0-15
}

void fillPayload(int pl) {
  int payload = pl;
  for (int i = 0; i < 4; i++) {  // filling data message from back
    if (payload != 0) {
      curr_msg.push_back(payload % 2);  // fill velocity
      payload /= 2;
      continue;
    }
    curr_msg.push_back(0);  // fill with zeros to remain 4 bit value
  }
}

void fillMsgType(int type) {
  int msg_type = type;
  for (int i = 0; i < 2; i++) {
    if (msg_type != 0) {
      curr_msg.push_back(msg_type % 2);  // fill msg type (2bit)
      msg_type /= 2;
      continue;
    }
    curr_msg.push_back(0);
  }
}

void fillHeading() {
  //int discr_heading = (int)(((act_heading + M_PI) * (180 / M_PI) + 11.25) / 22.5);  // discretized to range 0-15
  int discr_heading = (int)(((default_msg.msg + M_PI) * (180 / M_PI) +11.25 )/ 22.5);  // discretized to range 0-15
  for (int i = 0; i < 4; i++) {
    if (discr_heading != 0) {
      curr_msg.push_back(discr_heading % 2);  // fill heading
      discr_heading /= 2;
      continue;
    }
    curr_msg.push_back(0);
  }
  if(default_msg.enable){
    curr_msg.push_back(1);
  }
  else{
    curr_msg.push_back(0);
  }
  /* std::cout << "Sended heading:"; */
  /* for (int i = 0; i < (int)curr_msg.size(); i++) { */
  /*   std::cout << curr_msg[i]; */
  /* } */
  /* std::cout << std::endl; */
}

void fillUavId() {
  int tmp_id = uav_id;
  for (int i = 0; i < 2; i++) {
    if (tmp_id != 0) {
      curr_msg.push_back(tmp_id % 2);  // fill uav_id (2bit)
      tmp_id /= 2;
      continue;
    }
    curr_msg.push_back(0);
  }
}

void fillDataAndParity() {
  int parity = 0;  // parity counter to make odd parity

  for (int i = 0; i < (int)curr_msg.size(); i++) {  // adding label and data bites
    curr_frame.push_back(curr_msg[i]);
    if (curr_msg[i] == 1)
      parity++;
  }

  if (parity % 2 == 0) {  // adding parity bit
    curr_frame.push_back(1);
  } else {
    curr_frame.push_back(0);
  }
}

void bitStuffing() {
  int curr_frame_length = (int)curr_frame.size() - SEPARATOR_BITS - 1;  // one frame bit - 00010 was before  // without separator and frame end bits
  for (int i = 3; i < curr_frame_length; i++) {                         // bit-stuffing of 3 bits
    if (curr_frame[i - 3] == curr_frame[i - 2] && curr_frame[i - 2] == curr_frame[i - 1]) {
      if (curr_frame[i - 1] == 0) {
        curr_frame.insert(curr_frame.begin() + i, 1);
      } else {
        curr_frame.insert(curr_frame.begin() + i, 0);
      }
      curr_frame_length++;
      i += 2;  // change
    }
  }
}

// function for create physical data frame of message.
// frame has following structure: 11111 (spacing bits) 01 (start bits) byte_1 (label byte) byte_2 (data byte) 0/1 (parity) 10 (stop bits)
void create_curr_msg() {
  // curr_frame_raw.clear(); //containers init
  curr_frame.clear();
  curr_msg.clear();
  Msg2send tmp_m2s = msg_queue.front();  // load the oldest msg from masgs queue
  msg_queue.erase(msg_queue.begin());

  bool             valid = false;
  std::vector<int> msg_payload;
  if (!tmp_m2s.blank_msg) {
    if (tmp_m2s.payload.size() == 0) {
      ROS_ERROR("No payload set, sending blank msg");
    } else if (tmp_m2s.payload.size() == 1) {
      switch (tmp_m2s.msg_type) {
        case 0:
          ROS_INFO("Sending velocity");
          msg_payload.push_back(setPayloadVelocity(tmp_m2s.payload[0]));
          valid = true;
          break;
        case 1:
          ROS_INFO("Sending azimuth");
          msg_payload.push_back(setPayloadAzimuth(tmp_m2s.payload[0]));
          valid = true;
          break;
        case 2:
          ROS_WARN("Unknown implementation of msg type, sending blank msg");
          break;
        case 3:
          ROS_WARN("Unknown implementation of msg type, sending blank msg");
          break;
        default:
          ROS_ERROR("Unexpected msg_type, sending blank msg");
          break;
      }
    } else if (tmp_m2s.payload.size() == 2) {
      switch (tmp_m2s.msg_type) {
        case 0:
          ROS_INFO("Velocity and azimuth sent");
          msg_payload.push_back(setPayloadVelocity(tmp_m2s.payload[1]));
          msg_payload.push_back(setPayloadAzimuth(tmp_m2s.payload[0]));
          valid = true;
          break;
        case 1:
          ROS_WARN("Unknown implementation of msg type, sending blank msg");
          break;
        case 2:
          ROS_WARN("Unknown implementation of msg type, sending blank msg");
          break;
        case 3:
          ROS_WARN("Unknown implementation of msg type, sending blank msg");
          break;
        default:
          ROS_ERROR("Unexpected msg_type, sending blank msg");
          break;
      }
    } else {
      ROS_ERROR("Sending payload with size higher that 2 is not reliable, sending blank msg");
    }
  }

  if (valid) {
    for (auto& pl : msg_payload) {
      fillPayload(pl);
    }
    fillMsgType(tmp_m2s.msg_type);
  } else {
    fillHeading();
    ROS_INFO("Sending blank message only with UAV ID and its heading");
  }
  fillUavId();

  std::reverse(curr_msg.begin(), curr_msg.end());  // reverse the data message into correct order

  curr_frame.push_back(0);  // frame start bits //creating data frame
  curr_frame.push_back(1);  // frame start bits

  fillDataAndParity();

  curr_frame.push_back(1);  // frame end bits
  curr_frame.push_back(0);  // frame end bits

  for (int i = 0; i < SEPARATOR_BITS; i++) {  // 4 separator bits
    curr_frame.push_back(1);
  }

  curr_msg.clear();  // init data msg

  bitStuffing();

  std::cout << "Sended data frame:";
  for (int i = 0; i < (int)curr_frame.size(); i++) {
    std::cout << curr_frame[i];
  }
  std::cout << std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "UVDARtx");
  ros::NodeHandle  nh("~");
  TX::TX_processor txko(nh);
  ROS_INFO("[TX_processor] Node initialized");
  ros::Rate my_rate(rate);

  int curr_bit_index = 0;  // order of currently sending bit

  while (ros::ok()) {
    ledMsg.request.start_of_message = false;
    ledMsg.request.data             = true;
    ledMsg.request.bit_value        = true;

    if (curr_frame.empty()) {    // if current data frame is empty - avoiding self channel collision
      if (!msg_queue.empty()) {  // checking content of msgs queue. If it is not empty, it creates new data frame from the oldest stored message
        create_curr_msg();
        ledMsg.request.start_of_message = true;
        ledMsg.request.data_frame       = curr_frame;
        curr_bit_index                  = 0;
      } else {  // if msg queue is empty, create blank msg just with uav id, its heading, msg_type = 0. The message is added to the msgs queue
        Msg2send sim_msg;
        sim_msg.blank_msg = true;
        msg_queue.push_back(sim_msg);
      }
    } else {
      if (curr_frame[curr_bit_index] == 0) {  // check vlaue of bit in current data frame and set frequency
        ledMsg.request.bit_value = false;
      }
      curr_bit_index++;                                // go to next bit in the next tranmiting round
      if (curr_bit_index >= (int)curr_frame.size()) {  // if the data frame was transmited, init values
        curr_bit_index = 0;
        curr_frame.clear();
      }
    }

    LedStateClient.call(ledMsg);

    my_rate.sleep();
    ros::spinOnce();
  }
}
