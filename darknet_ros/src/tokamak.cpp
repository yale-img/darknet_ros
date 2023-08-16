#include <pluginlib/class_list_macros.h>
#include <ros/time.h>
#include <darknet_ros/tokamak.h>

PLUGINLIB_EXPORT_CLASS(darknet_ros::Tokamak, nodelet::Nodelet)

namespace darknet_ros
{
void Tokamak::onInit()
{
  ros::NodeHandle& nh = getPrivateNodeHandle();
  image_transport::ImageTransport it(nh);
  image_fusion_pub_ = it.advertise("/kinect/combined/image_raw", 30);
  master_im_sub_.subscribe(it, "/kinect/master/rgb/image_raw", 15);
  sub_im_sub_.subscribe(it, "/kinect/sub/rgb/image_raw", 15);
  sync_.reset(new SerialImageSync(SerialImagePolicy(30), master_im_sub_, sub_im_sub_));
  sync_->registerCallback(boost::bind(&Tokamak::filterCallback, this, _1, _2));
}

void Tokamak::filterCallback(const sensor_msgs::ImageConstPtr& master_im, const sensor_msgs::ImageConstPtr& sub_im)
{
  image_fusion_pub_.publish(master_im);
  ros::Duration(0.0333).sleep();
  image_fusion_pub_.publish(sub_im);
}
}
