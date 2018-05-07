#ifndef TF_PUBLISHER_H
#define TF_PUBLISHER_H

#include <ros/ros.h>
#include <bert_tf/Configuration.h>

class TfPublisher {

public:
  TfPublisher(ros::NodeHandle& nh);
  void sendTfOdomToBaseLink(void);
  void sendTfBaseLinkToLaser(void);
  void configurationReceivedCallback(const bert_tf::Configuration::ConstPtr &msg);

private:
  ros::NodeHandle &nh_;
  ros::Subscriber sub_robot_config_;
  bert_tf::Configuration robot_config_;

};

#endif // TF_PUBLISHER_H
