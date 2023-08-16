#include <ros/time.h>
#include "darknet_ros/tokamak.h"

Tokamak::Tokamak(const ros::NodeHandle& nh) : nh_(nh),
  it_(nh_),
  master_im_sub_(it_, "/kinect/master/rgb/image_raw", 3),
  sub_im_sub_(it_, "/kinect/sub/rgb/image_raw", 3),
  sync_(SerialImagePolicy(15), master_im_sub_, sub_im_sub_)
{
  image_fusion_pub_ = it_.advertise("/kinect/combined/image_raw", 6);
  sync_.registerCallback(boost::bind(&Tokamak::filterCallback, this, _1, _2));
}

void Tokamak::filterCallback(const sensor_msgs::ImageConstPtr& master_im, const sensor_msgs::ImageConstPtr& sub_im)
{
  image_fusion_pub_.publish(master_im);
  ros::Duration(0.03).sleep();
  image_fusion_pub_.publish(sub_im);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tokamak");
  ros::NodeHandle nh("~");

  Tokamak tokamak(nh);

  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::waitForShutdown();
}
