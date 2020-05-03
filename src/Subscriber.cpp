#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace {

void printMessage(const std_msgs::String::ConstPtr &message) {
  ROS_INFO("%s", message->data.c_str());
}

}  // namespace

int main(int argc, char **argv) {
  try {
    ros::init(argc, argv, "palchukovsky_subscriber");
    ros::NodeHandle node;
    auto subscriber = node.subscribe("/demo/text", 1000, printMessage);
    ros::spin();
  } catch (const std::exception &ex) {
    ROS_FATAL(R"(Fatal error: "%s".)", ex.what());
    throw;
  } catch (...) {
    ROS_FATAL("Unknown fatal error.");
    throw;
  }
  return 0;
}