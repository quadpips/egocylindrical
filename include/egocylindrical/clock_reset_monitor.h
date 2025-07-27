#ifndef MESSAGE_FILTERS_CLOCK_RESET_MONITOR_H
#define MESSAGE_FILTERS_CLOCK_RESET_MONITOR_H

// #include <ros/time.h>
// #include <ros/console.h>

namespace message_filters
{

class ClockResetMonitor
{
public:
  ClockResetMonitor(rclcpp::Duration threshold=rclcpp::Duration::from_seconds(1)):
    threshold_(threshold)
    {}
    
  
  bool update()
  {
    // rclcpp::Time cur_time = rclcpp::Time::now();

    // bool res = (last_clock_time_ - cur_time) > threshold_;
    
    // if (res)
    // {
    //   // RCLCPP_WARN_STREAM("clock_monitor", "Resetting detector: New clock time (" << cur_time << ") is older than previous value (" << last_clock_time_ << ") by more greater than [" << threshold_ << "]");
    //   cur_time = rclcpp::Time();
    // }
   // last_clock_time_ = cur_time;

    bool res = true;

    return res;
  }

private:
  // ros::Time last_clock_time_;
  // ros::Duration threshold_;
  rclcpp::Duration threshold_;
  rclcpp::Time last_clock_time_;
};

}

#endif //MESSAGE_FILTERS_CLOCK_RESET_MONITOR_H
