#ifndef MESSAGE_FILTERS_TIME_FILTER_H
#define MESSAGE_FILTERS_TIME_FILTER_H

#include <message_filters/simple_filter.h>
#include <egocylindrical/clock_reset_monitor.h>

  template<class M>
  class TimeFilter: public message_filters::SimpleFilter<M>
  {
  public:
    TimeFilter() = default;
    
    template<class F>
    TimeFilter(F& f)
    {
      connectInput(f);
    }
    
     ~TimeFilter()
    {
      incoming_connection_.disconnect();
    }
    
    /**
    * \brief Connect this filter's input to another filter's output.
    */
    template<class F>
    void connectInput(F& f)
    {
      incoming_connection_.disconnect();
      incoming_connection_ = f.registerCallback(typename message_filters::SimpleFilter<M>::EventCallback(std::bind(&TimeFilter::cb, this, boost::placeholders::_1)));
    }

    void add(const typename message_filters::SimpleFilter<M>::EventType& evt)
    {
      namespace mt = ros::message_traits;
      if(clock_monitor_.update())
      {
        last_msg_time_ = ros::Time();
      }
      
      auto timestamp = mt::TimeStamp<M>::value(*evt.getMessage());
      if (timestamp <= last_msg_time_)
      {
        // ROS_DEBUG_STREAM_NAMED(message_filters::SimpleFilter<M>::getName(), "Dropping message with timestamp: " << timestamp << ", not newer than last message: " << last_msg_time_);
        return;
      }
      else
      {
        last_msg_time_ = timestamp;
      }
      
      this->signalMessage(evt);

    }

  protected:
    
    void cb(const typename message_filters::SimpleFilter<M>::EventType& evt)
    {
      add(evt);
    }
      
    ros::Time last_msg_time_;
    
    message_filters::Connection incoming_connection_;
    boost::mutex time_mutex_;
    message_filters::ClockResetMonitor clock_monitor_;

  };
  
#endif //MESSAGE_FILTERS_TIME_FILTER_H
