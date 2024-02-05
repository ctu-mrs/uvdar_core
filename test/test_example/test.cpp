#include <gtest/gtest.h>

#include <mrs_uav_testing/test_generic.h>

class Tester : public mrs_uav_testing::TestGeneric {

public:
  Tester();

  bool test();
};

Tester::Tester() : mrs_uav_testing::TestGeneric() {

  // create your own subscribers, publishers, etc here.
  //
  // check the mrs_uav_testing::TestGeneric for existing stuff
  // this->nh_ is available
}

bool Tester::test() {

  {
    auto [success, message] = spawnGazeboUav();

    if (!success) {
      ROS_ERROR("[%s]: gazebo UAV spawning failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  // wait for the MRS system
  while (true) {

    ROS_INFO_THROTTLE(1.0, "[%s]: waiting for the MRS system", ros::this_node::getName().c_str());

    if (!ros::ok()) {
      ROS_ERROR("[%s]: killed from outside", ros::this_node::getName().c_str());
      return false;
    }

    if (this->mrsSystemReady()) {
      break;
    }
  }

  {
    auto [success, message] = this->takeoff();

    if (!success) {
      ROS_ERROR("[%s]: takeoff failed with message: '%s'", ros::this_node::getName().c_str(), message.c_str());
      return false;
    }
  }

  this->sleep(5.0);

  if (this->isFlyingNormally()) {
    return true;
  } else {
    ROS_ERROR("[%s]: not flying normally", ros::this_node::getName().c_str());
    return false;
  }
}

// --------------------------------------------------------------
// |                     DO NOT MODIFY BELOW                    |
// --------------------------------------------------------------

TEST(TESTSuite, test) {

  Tester tester;

  bool result = tester.test();

  if (result) {
    GTEST_SUCCEED();
  } else {
    GTEST_FAIL();
  }
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  ros::init(argc, argv, "test");

  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
