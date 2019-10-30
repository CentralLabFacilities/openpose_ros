#ifndef ROSCPP_TOPIC2_H
#define ROSCPP_TOPIC2_H

#include <ros/topic.h>
#include <ros/common.h>
#include <ros/node_handle.h>
#include <ros/callback_queue.h>
#include <boost/shared_ptr.hpp>

namespace ros
{
namespace topic
{

template<class M0, class M1, class M2>
class MessagesHelper {
private:
    std::vector<boost::shared_ptr<M0 const>> msgs0;//No order in timestamps guaranteed
    std::vector<boost::shared_ptr<M1 const>> msgs1;
    std::vector<boost::shared_ptr<M2 const>> msgs2;
    double timediff=INFINITY;
    int index0=-1, index1=-1, index2=-1;
    
    double computeTimeDiff(ros::Time t0, ros::Time t1, ros::Time t2) {
        // Sort the times, so that t0<=t1<=t2
        if(t0>t1) {
            ros::Time tmp=t1;
            t1=t0;
            t0=tmp;
        }
        if(t1>t2) {
            ros::Time tmp=t2;
            t2=t1;
            t1=tmp;
        }
        if(t0>t1) {
            ros::Time tmp=t1;
            t1=t0;
            t0=tmp;
        }
        if(t0>t1 || t1>t2) {
            ROS_ERROR("computeTimeDiff: times are not correctly ordered!");
        }
        return fabs((t2-t0).toSec());
    }

    void callback0(const boost::shared_ptr<M0 const>& message) {
        ROS_INFO_STREAM("callback0() called (" <<  ros::message_traits::datatype<M0>() << ") with timestamp " << message->header.stamp);
        msgs0.push_back(message);
        for(size_t i=0; i<msgs1.size(); i++) {
            for(size_t j=0; j<msgs2.size(); j++) {
                double diff=computeTimeDiff(message->header.stamp, msgs1[i]->header.stamp, msgs2[j]->header.stamp);
                if(diff<timediff) {
                    timediff=diff;
                    index0=msgs0.size()-1;
                    index1=i;
                    index2=j;
                    ROS_INFO("Setting new pair: timediff=%g, index0=%i, index1=%i, index2=%i", timediff, index0, index1, index2);
                }
            }
        }
    }

    void callback1(const boost::shared_ptr<M1 const>& message) {
        ROS_INFO_STREAM("callback1() called (" <<  ros::message_traits::datatype<M1>() << ") with timestamp " << message->header.stamp);
        msgs1.push_back(message);
        for(size_t i=0; i<msgs0.size(); i++) {
             for(size_t j=0; j<msgs2.size(); j++) {
                double diff=computeTimeDiff(msgs0[i]->header.stamp, message->header.stamp, msgs2[j]->header.stamp);
                if(diff<timediff) {
                    timediff=diff;
                    index0=i;
                    index1=msgs1.size()-1;
                    index2=j;
                    ROS_INFO("Setting new pair: timediff=%g, index0=%i, index1=%i, index2=%i", timediff, index0, index1, index2);
                }
            }
        }
    }

    void callback2(const boost::shared_ptr<M2 const>& message) {
        ROS_INFO_STREAM("callback2() called (" <<  ros::message_traits::datatype<M2>() << ") with timestamp " << message->header.stamp);
        msgs1.push_back(message);
        for(size_t i=0; i<msgs1.size(); i++) {
            for(size_t j=0; j<msgs2.size(); j++) {
                double diff=computeTimeDiff(msgs0[i]->header.stamp, msgs1[j]->header.stamp, message->header.stamp);
                if(diff<timediff) {
                    timediff=diff;
                    index0=i;
                    index1=j;
                    index2=msgs2.size()-1;
                    ROS_INFO("Setting new pair: timediff=%g, index0=%i, index1=%i, index2=%i", timediff, index0, index1, index2);
                }
            }
        }
    }

public:

/**
 * \brief Wait for two messages to arrive on topics, with timeout
 *
 * \param M <template> The message type
 * \param topic The topic to subscribe on
 * \param nh The NodeHandle to use to do the subscription
 * \param behavior -1: Mode 1, -2: Mode 2, >=0: Mode 3
 * \param timeout The amount of time to wait before returning if no message is received
 * \param msg0, msg1 The messages.  Empty boost::shared_ptr if waitForMessages is interrupted by the node shutting down
 */
//template<class M0, class M1, class M2>
void waitForMessages(const std::string& topic0, const std::string& topic1, const std::string& topic2, NodeHandle& nh, const float behavior, ros::Duration timeout, boost::shared_ptr<M0 const> &msg0, boost::shared_ptr<M1 const> &msg1, boost::shared_ptr<M2 const> &msg2)
{
  //TODO: trade-off between duration of this method, time stamp difference between the messages, and network traffic
  //Mode 1: Unsubcribe from topic when message received, return when message from each topic received. Traffic: Best. Duration: Best. Difference: Worst
  //Mode 2: Don't unsubscribe, keep receiving until at least one message from each topic has arrived. Traffic: Worse. Duration: Best. Difference: Better
  //Mode 3: Don't unsubscribe, keep receiving until messages with time stamp difference under a threshold have arrived. Traffic: Worst. Duration: Worst. Difference: Best 
  struct timeval start;
  struct timeval end;
  gettimeofday(&start, NULL);
  SubscribeOptions ops0, ops1, ops2;
  ops0.init<M0>(topic0, 10, boost::bind(&MessagesHelper<M0, M1, M2>::callback0, this, _1));
  ops1.init<M1>(topic1, 10, boost::bind(&MessagesHelper<M0, M1, M2>::callback1, this, _1));
  ops2.init<M2>(topic2, 10, boost::bind(&MessagesHelper<M0, M1, M2>::callback2, this, _1));

  ros::CallbackQueue queue0;
  ops0.callback_queue = &queue0;
  ros::CallbackQueue queue1;
  ops1.callback_queue = &queue1;
  ros::CallbackQueue queue2;
  ops2.callback_queue = &queue2;

  ros::Subscriber sub0 = nh.subscribe(ops0); // TODO unsubscribe later?
  ros::Subscriber sub1 = nh.subscribe(ops1);
  ros::Subscriber sub2 = nh.subscribe(ops2);

  ros::Time timeout_end = ros::Time::now() + timeout;
  if(behavior==-1.0f) {
      while (nh.ok() && (timeout.isZero() || ros::Time::now() < timeout_end)) {
        if (msgs0.size()==0) {
          queue0.callOne(ros::WallDuration(0.05));
          if (msgs1.size()==0) {
            queue1.callOne(ros::WallDuration(0.05));
            if (msgs2.size()==0) {
              queue2.callOne(ros::WallDuration(0.05));
            } else {
              sub2.shutdown();
            }
          } else {
            sub1.shutdown();
            if (msgs2.size()==0) {
              queue2.callOne(ros::WallDuration(0.05));
            } else {
              sub2.shutdown();
            }
          }
        } else {
          sub0.shutdown();
          if (msgs1.size()==0) {
            queue1.callOne(ros::WallDuration(0.05));
            if (msgs2.size()==0) {
              queue2.callOne(ros::WallDuration(0.05));
            } else {
              sub2.shutdown();
            }
          } else {
            sub1.shutdown();
            if (msgs2.size()==0) {
              queue2.callOne(ros::WallDuration(0.05));
            } else {
              sub2.shutdown();
              break;
            }
          }
        }
      }
  } else if(behavior==-2.0f) {
      while (nh.ok() && (timeout.isZero() || ros::Time::now() < timeout_end)) {
        queue0.callOne(ros::WallDuration(0.05));//TODO callAvailable? zero duration?
        queue1.callOne(ros::WallDuration(0.05));
        queue2.callOne(ros::WallDuration(0.05));
        if (msgs0.size()>0 && msgs1.size()>0 && msgs2.size()>0) {
          sub0.shutdown();
          sub1.shutdown();
          sub2.shutdown();
          break;
        }
      }
  } else if(behavior>=0.0f) {
      while (nh.ok() && (timeout.isZero() || ros::Time::now() < timeout_end)) {
        queue0.callOne(ros::WallDuration(0.05));//TODO callAvailable? zero duration?
        queue1.callOne(ros::WallDuration(0.05));
        queue2.callOne(ros::WallDuration(0.05));
        if (timediff<=behavior) {
          sub0.shutdown();
          sub1.shutdown();
          sub2.shutdown();
          break;
        }
      }
  } else {
    ROS_ERROR("behavior has wrong value");
  }

  msg0 = msgs0[index0];//TODO check indices
  msg1 = msgs1[index1];
  msg2 = msgs2[index2];
  gettimeofday(&end, NULL);
  ROS_INFO("timediff=%g, index0=%i, index1=%i, index2=%i, msgs0.size=%lu, msgs1.size=%lu, msgs2.size=%lu\n", timediff, index0, index1, index2, msgs0.size(), msgs1.size(), msgs2.size());
  ROS_INFO("waitForMessages finished in %g seconds", ((end.tv_sec-start.tv_sec)+(end.tv_usec-start.tv_usec)/1000000.0));
}
}; // class end

} // namespace topic
} // namespace ros

#endif // ROSCPP_TOPIC2_H
