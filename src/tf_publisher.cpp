#include <bert_tf/tf_publisher.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


TfPublisher::TfPublisher(ros::NodeHandle &nh) :
  nh_(nh)
{
  robot_config_.header.frame_id = "odom";
  robot_config_.header.seq = 0;
  robot_config_.header.stamp.nsec = 0;
  robot_config_.header.stamp.sec = 0;
  robot_config_.x = 0;
  robot_config_.y = 0;
  robot_config_.theta = 0;

  sub_robot_config_ = nh_.subscribe<bert_tf::Configuration>("robot_config", 10, &TfPublisher::configurationReceivedCallback, this);
}

void TfPublisher::sendTfOdomToBaseLink(void)
{
  static tf2_ros::TransformBroadcaster broadcaster;
  geometry_msgs::TransformStamped transform_odom_baselink;

  //transform_odom_baselink.header.stamp = ros::Time::now();
  transform_odom_baselink.header.stamp = robot_config_.header.stamp;
  transform_odom_baselink.header.frame_id = "odom";
  transform_odom_baselink.child_frame_id = "base_link";
  // set translation
  transform_odom_baselink.transform.translation.x = robot_config_.x;
  transform_odom_baselink.transform.translation.y = robot_config_.y;
  transform_odom_baselink.transform.translation.z = 0.0;
  // set rotation
  tf2::Quaternion q;
  q.setRPY(0, 0, robot_config_.theta);
  transform_odom_baselink.transform.rotation.x = q.x();
  transform_odom_baselink.transform.rotation.y = q.y();
  transform_odom_baselink.transform.rotation.z = q.z();
  transform_odom_baselink.transform.rotation.w = q.w();
  // send transform
  broadcaster.sendTransform(transform_odom_baselink);
}

void TfPublisher::sendTfBaseLinkToLaser(void)
{
  static tf2_ros::StaticTransformBroadcaster broadcaster;
  geometry_msgs::TransformStamped transform_baselink_laser;

  transform_baselink_laser.header.stamp = ros::Time::now();
  transform_baselink_laser.header.frame_id = "base_link";
  transform_baselink_laser.child_frame_id = "laser";
  // set translation (laser position relative to the robot reference point, which is the middle of the rear shaft)
  transform_baselink_laser.transform.translation.x = 0;
  transform_baselink_laser.transform.translation.y = 0;
  transform_baselink_laser.transform.translation.z = 0.2;
  // set rotation
  tf2::Quaternion quat;
  quat.setRPY(0, 0, 0);
  transform_baselink_laser.transform.rotation.x = quat.x();
  transform_baselink_laser.transform.rotation.y = quat.y();
  transform_baselink_laser.transform.rotation.z = quat.z();
  transform_baselink_laser.transform.rotation.w = quat.w();
  // send static transform
  broadcaster.sendTransform(transform_baselink_laser);
}

void TfPublisher::configurationReceivedCallback(const bert_tf::Configuration::ConstPtr &msg)
{
  // get message
  robot_config_.header = msg->header;
  robot_config_.x = msg->x;
  robot_config_.y = msg->y;
  robot_config_.theta = msg->theta;

  // send TF
  //TfPublisher::sendTfOdomToBaseLink();
  TfPublisher::sendTfBaseLinkToLaser();
}
