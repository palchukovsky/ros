#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <boost/format.hpp>
#include <chrono>
#include <memory>
#include <random>
#include <vector>

namespace {

class Publisher {
 public:
  explicit Publisher(const std::string &topic, const size_t numberOfVars)
      : m_publisher(m_node.advertise<std_msgs::String>(topic, 1000)),
        m_randDist(0, std::max<decltype(numberOfVars)>(1, numberOfVars) - 1),
        m_messages(CrateMessageSet(numberOfVars)) {}

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

  const std_msgs::String &CreateMessage() const {
    return m_messages[m_randDist(m_randDev)];
  }

  static std::vector<std_msgs::String> CrateMessageSet(const size_t size) {
    std::vector<std_msgs::String> result(size);
    result.reserve(size);
    size_t i = 1;
    for (auto &item : result) {
      item.data =
          (boost::format("Publisher text string from slot #%1%.") % i).str();
      ++i;
    }
    return result;
  }

 private:
  ros::NodeHandle m_node;

  ros::Publisher m_publisher;

  mutable std::default_random_engine m_randDev{static_cast<size_t>(
      std::chrono::system_clock::now().time_since_epoch().count())};
  mutable std::uniform_int_distribution<size_t> m_randDist;

  const std::vector<std_msgs::String> m_messages;

  std::unique_ptr<ros::Timer> m_timer;
};

}  // namespace

int main(int argc, char **argv) {
  try {
    ros::init(argc, argv, "palchukovsky_publisher");

    Publisher pub("/demo/text", 20);
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