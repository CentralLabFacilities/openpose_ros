// Generated by gencpp from file openpose_ros/DetectPeople.msg
// DO NOT EDIT!


#ifndef OPENPOSE_ROS_MESSAGE_DETECTPEOPLE_H
#define OPENPOSE_ROS_MESSAGE_DETECTPEOPLE_H

#include <ros/service_traits.h>


#include <openpose_ros/DetectPeopleRequest.h>
#include <openpose_ros/DetectPeopleResponse.h>


namespace openpose_ros
{

struct DetectPeople
{

typedef DetectPeopleRequest Request;
typedef DetectPeopleResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct DetectPeople
} // namespace openpose_ros


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::openpose_ros::DetectPeople > {
  static const char* value()
  {
    return "f9061c1888cb3379ce3b5278d009dde5";
  }

  static const char* value(const ::openpose_ros::DetectPeople&) { return value(); }
};

template<>
struct DataType< ::openpose_ros::DetectPeople > {
  static const char* value()
  {
    return "openpose_ros/DetectPeople";
  }

  static const char* value(const ::openpose_ros::DetectPeople&) { return value(); }
};


// service_traits::MD5Sum< ::openpose_ros::DetectPeopleRequest> should match 
// service_traits::MD5Sum< ::openpose_ros::DetectPeople > 
template<>
struct MD5Sum< ::openpose_ros::DetectPeopleRequest>
{
  static const char* value()
  {
    return MD5Sum< ::openpose_ros::DetectPeople >::value();
  }
  static const char* value(const ::openpose_ros::DetectPeopleRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::openpose_ros::DetectPeopleRequest> should match 
// service_traits::DataType< ::openpose_ros::DetectPeople > 
template<>
struct DataType< ::openpose_ros::DetectPeopleRequest>
{
  static const char* value()
  {
    return DataType< ::openpose_ros::DetectPeople >::value();
  }
  static const char* value(const ::openpose_ros::DetectPeopleRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::openpose_ros::DetectPeopleResponse> should match 
// service_traits::MD5Sum< ::openpose_ros::DetectPeople > 
template<>
struct MD5Sum< ::openpose_ros::DetectPeopleResponse>
{
  static const char* value()
  {
    return MD5Sum< ::openpose_ros::DetectPeople >::value();
  }
  static const char* value(const ::openpose_ros::DetectPeopleResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::openpose_ros::DetectPeopleResponse> should match 
// service_traits::DataType< ::openpose_ros::DetectPeople > 
template<>
struct DataType< ::openpose_ros::DetectPeopleResponse>
{
  static const char* value()
  {
    return DataType< ::openpose_ros::DetectPeople >::value();
  }
  static const char* value(const ::openpose_ros::DetectPeopleResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // OPENPOSE_ROS_MESSAGE_DETECTPEOPLE_H
