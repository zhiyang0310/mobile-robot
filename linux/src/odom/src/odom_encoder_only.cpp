//ROS头文件
#include "ros/ros.h"
//自定义msg产生的头文件
#include <nav_msgs/Odometry.h>
#include "encoders/encoders.h"
#include "imu/imu.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <string>
using namespace std;

#define meter_per_tick 0.0001309
#define wheel_track 0.156

int main(int argc, char **argv)
{
  //用于解析ROS参数，第三个参数为本节点名
  ros::init(argc, argv, "odom");

  //实例化句柄，初始化node
  ros::NodeHandle nh;

  //创建publisher
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
  //订阅server
  ros::ServiceClient encoders_client = nh.serviceClient<encoders::encoders>("encoders");
  ros::ServiceClient imu_client = nh.serviceClient<imu::imu>("imu");

  tf::TransformBroadcaster odom_broadcaster;
  //定义发布的频率 
  ros::Rate loop_rate(10.0);

  //prepare
  ros::Time last_time = ros::Time::now();
  long last_left_ticks = 0;
  long last_right_ticks = 0;
  geometry_msgs::Quaternion last_Quaternion;

  double d_left = 0.0,d_right = 0.0,d_center = 0.0,vx = 0.0,vy = 0.0,vth = 0.0;
  double x = 0.0,y = 0.0,th = 0.0;
  
  encoders::encoders encoders_srv;
  //imu::imu imu_srv;
  string response = "",response1 = "",response2 = "";
  //encoders_service
  encoders_srv.request.request = true;
  if (encoders_client.call(encoders_srv))
  {
    int flag = 0;
    response = encoders_srv.response.feedback;
    for (auto c : response)
    {
      if(c == '|'){
        flag = 1;
        continue;
      }
      if(c == '$')
        break;
      if(flag == 0)
        response1 += c;
      if(flag == 1)
        response2 += c;
    }
    last_left_ticks = stol(response1);
    last_right_ticks = stol(response2);
    response = "";
    response1 = "";
    response2 = "";
  }
  else
  {
	ROS_ERROR("Failed to call service encoders_service");
	return 1;
  }
  //imu_service
//   imu_srv.request.request = true;
//   if (imu_client.call(imu_srv))
//   {
//     last_Quaternion = imu_srv.response.imu_msg.orientation;
//     last_Quaternion.x = 0.0;
//     last_Quaternion.y = 0.0;
//   }
//   else
//   {
// 	ROS_ERROR("Failed to call service imu_service");
// 	return 1;
//   } 
  //循环发布msg
  //encoder_only
  geometry_msgs::Quaternion quat_init = tf::createQuaternionMsgFromYaw(th);
  geometry_msgs::TransformStamped odom_trans_init;
  odom_trans_init.header.stamp = last_time;
  odom_trans_init.header.frame_id = "odom";
  odom_trans_init.child_frame_id = "base_link";
  odom_trans_init.transform.translation.x = x;
  odom_trans_init.transform.translation.y = y;
  odom_trans_init.transform.translation.z = 0.0;
  odom_trans_init.transform.rotation = quat_init;

  odom_broadcaster.sendTransform(odom_trans_init);
  // publish msg
  nav_msgs::Odometry odom_init;
  odom_init.header.stamp = last_time;
  odom_init.header.frame_id = "odom";
  odom_init.pose.pose.position.x = x;
  odom_init.pose.pose.position.y = y;
  odom_init.pose.pose.position.z = 0.0;
  odom_init.pose.pose.orientation = quat_init;
  odom_init.child_frame_id = "base_link";
  odom_init.twist.twist.linear.x = vx;
  odom_init.twist.twist.linear.y = vy;
  odom_init.twist.twist.linear.z = 0.0;
  odom_init.twist.twist.angular.z = 0.0;

  odom_pub.publish(odom_init);

  while (ros::ok())
  {
    ros::Time current_time = ros::Time::now();
    long current_left_ticks = 0;
    long current_right_ticks = 0;
    geometry_msgs::Quaternion current_Quaternion;

    geometry_msgs::Vector3 angular_velocity;

    encoders_srv.request.request = true;
    if (encoders_client.call(encoders_srv))
    {
      int flag = 0;
      response = encoders_srv.response.feedback;
      for (auto c : response)
      {
        if(c == '|'){
          flag = 1;
          continue;
        }
        if(c == '$')
          break;
        if(flag == 0)
          response1 += c;
        if(flag == 1)
          response2 += c;
      }
      current_left_ticks = stol(response1);
      current_right_ticks = stol(response2);
      response = "";
      response1 = "";
      response2 = "";
    }
    else
    {
	  ROS_ERROR("Failed to call service encoders_service");
	  return 1;
    }

    // imu_srv.request.request = true;
    // if (imu_client.call(imu_srv))
    // {
    //   current_Quaternion = imu_srv.response.imu_msg.orientation;
    //   current_Quaternion.x = 0.0;
    //   current_Quaternion.y = 0.0;
    //   angular_velocity = imu_srv.response.imu_msg.angular_velocity;
    //   angular_velocity.x = 0.0;
    //   angular_velocity.y = 0.0;
    // }
    // else
    // {
	//   ROS_ERROR("Failed to call service imu_service");
	//   return 1;
    // }

    //publish
    d_left = (current_left_ticks - last_left_ticks)*meter_per_tick/2.0;
    d_right = (current_right_ticks - last_right_ticks)*meter_per_tick/2.0;
    d_center = (d_left + d_right)/2.0;
    //calculate angular
    // double last_cos = 1-2*(last_Quaternion.z)*(last_Quaternion.z);
    // double last_sin = 2*last_Quaternion.z*last_Quaternion.w;
    // double current_cos = 1-2*(current_Quaternion.z)*(current_Quaternion.z);
    // double current_sin = 2*current_Quaternion.z*current_Quaternion.w;
    // double cos_dth = last_cos*current_cos + last_sin*current_sin;
    // double sin_dth = current_sin*last_cos - current_cos*last_sin;
    double dth = (d_right - d_left) / wheel_track;
    th += dth;
    //calculate location
    double dx = cos(dth) * d_center;
    double dy = sin(dth) * d_center;
    x += (cos(th) * dx - sin(th) * dy);
    y += (sin(th) * dx + cos(th) * dy);
    //publish transform
    current_Quaternion = tf::createQuaternionMsgFromYaw(th);
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = current_Quaternion;

    odom_broadcaster.sendTransform(odom_trans);
    //publish odom
    double dt = (current_time - last_time).toSec();
    vx = d_center/dt;
    vy = 0.0;
    vth = dth/dt;

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = current_Quaternion;
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.z = vth;

    odom_pub.publish(odom);

    //record current value
    last_time = current_time;
    last_left_ticks = current_left_ticks;
    last_right_ticks = current_right_ticks;
    last_Quaternion = current_Quaternion;

    loop_rate.sleep();//根据前面的定义的loop_rate,设置的暂停
  }

  return 0;
} 
