#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class LidarToImageProjection : public rclcpp::Node
{
public:
    LidarToImageProjection()
        : Node("lidar_to_image_projection"),
          tf_buffer_(this->get_clock()), 
          tf_listener_(tf_buffer_)
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/drone0/sensor_measurements/gimbal/camera/image_raw", 10,
            std::bind(&LidarToImageProjection::imageCallback, this, std::placeholders::_1));

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/drone0/sensor_measurements/lidar/points", 10,
            std::bind(&LidarToImageProjection::lidarCallback, this, std::placeholders::_1));

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/drone0/sensor_measurements/gimbal/camera/image_with_lidar", 10);

        fx_ = 1108.5;  // focal length x
        fy_ = 1108.5;  // focal length y
        cx_ = 640.5;  // optical center x
        cy_ = 480.5;  // optical center y
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    cv::Mat current_image_;
    bool image_received_ = false;

    double fx_, fy_, cx_, cy_;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            current_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
            image_received_ = true;
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!image_received_)
        {
            RCLCPP_WARN(this->get_logger(), "No image received yet, skipping Lidar projection");
            return;
        }

        // Perform the TF lookup
        geometry_msgs::msg::TransformStamped transformStamped;
        try {
            transformStamped = tf_buffer_.lookupTransform(
                "drone0/gimbal/_0/_1/_2/camera/hd_camera/camera/optical_frame",  // Target frame (camera)
                "drone0/lidar/lidar_3d/gpu_ray",                                  // Source frame (Lidar)
                tf2::TimePointZero
            );
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform lidar to camera frame: %s", ex.what());
            return;
        }

        // Create an OpenCV image to work with
        cv::Mat image_with_lidar = current_image_.clone();

        // Iterate over the point cloud data
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            // Get the point in Lidar frame
            geometry_msgs::msg::PointStamped lidar_point, camera_point;
            lidar_point.point.x = *iter_x;
            lidar_point.point.y = *iter_y;
            lidar_point.point.z = *iter_z;

            // Transform to the camera frame
            tf2::doTransform(lidar_point, camera_point, transformStamped);

            // Project the point onto the image plane (simple pinhole camera model)
            if (camera_point.point.z > 0)  // Only consider points in front of the camera
            {
                int u = static_cast<int>((fx_ * camera_point.point.x / camera_point.point.z) + cx_);
                int v = static_cast<int>((fy_ * camera_point.point.y / camera_point.point.z) + cy_);

                // Check if the projected point is within the image bounds
                if (u >= 0 && u < image_with_lidar.cols && v >= 0 && v < image_with_lidar.rows)
                {
                    // Draw the point on the image (using a red circle)
                    cv::circle(image_with_lidar, cv::Point(u, v), 2, cv::Scalar(0, 0, 255), -1);
                }
            }
        }

        // Convert the modified OpenCV image back to ROS Image message and publish
        cv_bridge::CvImage out_msg;
        out_msg.header = msg->header;
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = image_with_lidar;

        image_pub_->publish(*out_msg.toImageMsg());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarToImageProjection>());
    rclcpp::shutdown();
    return 0;
}
