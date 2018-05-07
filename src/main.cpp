#include <ros/ros.h>
#include <bert_tf/tf_publisher.h>

/* Main */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "bert_tf_broadcaster");
  ros::NodeHandle nh;

  ROS_INFO("TF broadcaster node started");
  //ROS_INFO("Broadcasted frame transforms: \n\"odom\" --> \"base_link\"\n\"base_link\" --> \"laser\"");
  ROS_INFO("Broadcasted frame transforms: \"base_link\" --> \"laser\"");

  TfPublisher tf_pub(nh);
  // send static tf of "base_link" --> "laser", needed only once as it is static
  tf_pub.sendTfBaseLinkToLaser();

  ros::spin();

  return 0;
}
