#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>

namespace darknet_ros
{
class Tokamak : public nodelet::Nodelet
{
  public:
    virtual void onInit();

  private:
    image_transport::Publisher image_fusion_pub_;
    image_transport::SubscriberFilter master_im_sub_;
    image_transport::SubscriberFilter sub_im_sub_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SerialImagePolicy;
    typedef message_filters::Synchronizer<SerialImagePolicy> SerialImageSync;
    boost::shared_ptr<SerialImageSync> sync_;

    void filterCallback(const sensor_msgs::ImageConstPtr& master_im, const sensor_msgs::ImageConstPtr& sub_im);
};
}
