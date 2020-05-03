#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <memory>

namespace {

class Publisher {
 public:
  explicit Publisher(const std::string &topic)
      : m_publisher(m_node.advertise<std_msgs::String>(topic, 1000)) {}

  Publisher(Publisher &&) = default;
  Publisher(const Publisher &) = default;
  Publisher &operator=(Publisher &&) = default;
  Publisher &operator=(const Publisher &) = delete;
  ~Publisher() = default;

 public:
  void Start(ros::Duration period) {
    if (m_timer) {
      throw std::logic_error("already started");
    }
    m_timer = std::make_unique<ros::Timer>(m_node.createTimer(
        std::move(period),
        [this](const ros::TimerEvent &event) { Fire(event); }));
  }

 private:
  void Fire(const ros::TimerEvent &) { m_publisher.publish(CreateMessage()); }

  std_msgs::String CreateMessage() {
    std_msgs::String result;
    result.data = "message data";
    return result;
  }

 private:
  ros::NodeHandle m_node{""};
  std::unique_ptr<ros::Timer> m_timer;
  ros::Publisher m_publisher;
};

}  // namespace

int main(int argc, char **argv) {
  try {
    ros::init(argc, argv, "palchukovsky_publisher");

    Publisher pub("palchukovsky");
    pub.Start({0, 500});

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