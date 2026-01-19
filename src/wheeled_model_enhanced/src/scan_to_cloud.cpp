#include <laser_geometry/laser_geometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.hpp>

class ScanToCloud : public rclcpp::Node
{
  public:
    ScanToCloud() : Node("scan_to_cloud")
    {
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/lidar", 10, [this](const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
                try
                {
                    sensor_msgs::msg::PointCloud2 cloud_msg;
                    projector_.transformLaserScanToPointCloud(scan_msg->header.frame_id, *scan_msg, cloud_msg,
                                                              tf_buffer_, -1.0);

                    cloud_pub_->publish(cloud_msg);
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR_STREAM(get_logger(), "Conversion failed: " << e.what());
                }
            });

        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/sensing/lidar/concatenated/pointcloud", 10);
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    laser_geometry::LaserProjection projector_;
    tf2_ros::Buffer tf_buffer_{get_clock()};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanToCloud>());
    rclcpp::shutdown();
    return 0;
}