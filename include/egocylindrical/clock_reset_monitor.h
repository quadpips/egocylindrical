#ifndef MESSAGE_FILTERS_CLOCK_RESET_MONITOR_H
#define MESSAGE_FILTERS_CLOCK_RESET_MONITOR_H

#include <ros/time.h>
#include <ros/console.h>

namespace message_filters
{

class ClockResetMonitor
{
public:
  ClockResetMonitor(ros::Duration threshold=ros::Duration(1)):
    threshold_(threshold)
    {}
    
  
  bool update()
  {
    ros::Time cur_time = ros::Time::now();
    bool res = (last_clock_time_ - cur_time) > threshold_;
    
    if(res)
    {
      ROS_WARN_STREAM_NAMED("clock_monitor", "Resetting detector: New clock time (" << cur_time << ") is older than previous value (" << last_clock_time_ << ") by more greater than [" << threshold_ << "]");
      cur_time = ros::Time();
    }
   
    last_clock_time_ = cur_time;

    return res;
  }

private:
  ros::Time last_clock_time_;
  ros::Duration threshold_;
};

}

#endif //MESSAGE_FILTERS_CLOCK_RESET_MONITOR_H
