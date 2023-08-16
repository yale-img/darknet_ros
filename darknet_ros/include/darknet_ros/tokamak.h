#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

class Tokamak
{
  public:
    Tokamak(const ros::NodeHandle& nh);

  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_fusion_pub_;
    image_transport::SubscriberFilter master_im_sub_;
    image_transport::SubscriberFilter sub_im_sub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SerialImagePolicy;
    message_filters::Synchronizer<SerialImagePolicy> sync_;

    void filterCallback(const sensor_msgs::ImageConstPtr& master_im, const sensor_msgs::ImageConstPtr& sub_im);
};
