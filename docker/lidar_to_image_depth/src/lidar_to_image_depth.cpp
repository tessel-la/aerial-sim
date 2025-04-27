#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <opencv2/opencv.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/flann/flann.hpp>
#include <vector>
#include <cmath>

class LidarToImageDepth : public rclcpp::Node
{
public:
    LidarToImageDepth()
        : Node("lidar_to_image_depth"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        // declate parameters for all the topics
        this->declare_parameter("image_sub", "/drone0/sensor_measurements/gimbal/camera/image_raw");
        this->declare_parameter("lidar_sub", "/octomap_point_cloud_centers");
        this->declare_parameter("camera_info_sub", "/drone0/sensor_measurements/gimbal/camera/camera_info");
        this->declare_parameter("resized_pub", "/drone0/sensor_measurements/gimbal/camera/image_resized");
        this->declare_parameter("pointcloud_pub", "/drone0/sensor_measurements/gimbal/camera/points");
        this->declare_parameter("camera_info_pub", "/drone0/sensor_measurements/gimbal/camera/camera_info_resized");

        // get the parameters
        std::string image_sub = this->get_parameter("image_sub").as_string();
        std::string lidar_sub = this->get_parameter("lidar_sub").as_string();
        std::string camera_info_sub = this->get_parameter("camera_info_sub").as_string();
        std::string resized_pub = this->get_parameter("resized_pub").as_string();
        std::string pointcloud_pub = this->get_parameter("pointcloud_pub").as_string();
        std::string camera_info_pub = this->get_parameter("camera_info_pub").as_string();



        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_sub, 10,
            std::bind(&LidarToImageDepth::imageCallback, this, std::placeholders::_1));

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_sub, 10,
            std::bind(&LidarToImageDepth::lidarCallback, this, std::placeholders::_1));

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(camera_info_sub, 10,std::bind(&LidarToImageDepth::camInfoCallback, this, std::placeholders::_1)) ;

        resized_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            resized_pub, 10);

        // create a pointcloud2 publisher
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            pointcloud_pub, 10);
        
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            camera_info_pub, 10);

        init_dist_ = 50.0; // Initial value for depth image in meters
        max_dist_threshold_ = 50.0; // Set max distance in pixels threshold for interpolation

        local_msg = nullptr;
        
    }

    ~LidarToImageDepth()
    {
        delete kdtree_; // Clean up the k-d tree
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    image_geometry::PinholeCameraModel camera_model_ros_;


    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr resized_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    cv::Mat current_image_;
    cv::Mat resized_image;
    bool image_received_ = false;
    bool cam_info_received_ = false;
    std::string cam_frame_name = "";

    double fx_, fy_, cx_, cy_;
    float init_dist_;
    float max_dist_threshold_;  // Maximum distance threshold

    cv::flann::Index *kdtree_;

    sensor_msgs::msg::PointCloud2::SharedPtr local_msg;
    std::mutex local_msg_mutex;


    cv::Mat createWeightedKernel(int size = 5, double sigma = 1.0) {
        cv::Mat kernel(size, size, CV_32F);
        float sum = 0.0;

        int halfSize = size / 2;
        for (int i = -halfSize; i <= halfSize; i++) {
            for (int j = -halfSize; j <= halfSize; j++) {
                kernel.at<float>(i + halfSize, j + halfSize) = exp(-(i * i + j * j) / (2 * sigma * sigma));
                sum += kernel.at<float>(i + halfSize, j + halfSize);
            }
        }

        // Normalize the kernel
        kernel /= sum;
        
        return kernel;
    }

    cv::Mat customConvolution(const cv::Mat &sparseDepth, const cv::Mat &kernel) {
        cv::Mat output = sparseDepth.clone();
        cv::Mat mask = (sparseDepth == 0);

        cv::Mat convolved;
        cv::filter2D(sparseDepth, convolved, CV_32F, kernel, cv::Point(-1, -1), 0, cv::BORDER_CONSTANT);

        cv::Mat weightSum;
        cv::filter2D(1 - mask, weightSum, CV_32F, kernel, cv::Point(-1, -1), 0, cv::BORDER_CONSTANT);

        for (int i = 0; i < output.rows; i++) {
            for (int j = 0; j < output.cols; j++) {
                if (mask.at<uchar>(i, j) && weightSum.at<float>(i, j) > 0) {
                    output.at<float>(i, j) = convolved.at<float>(i, j) / weightSum.at<float>(i, j);
                }
            }
        }

        return output;
    }


    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            current_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
            
            // resize the image the image to 640x480 and publish it
            cv::resize(current_image_, resized_image, cv::Size(640, 480));
            auto resized_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", resized_image).toImageMsg();
            
            // copy the header
            resized_msg->header = msg->header;
            cam_frame_name = msg->header.frame_id;  // Store the camera frame name

            resized_pub_->publish(*resized_msg);

            image_received_ = true;

            if (local_msg != nullptr && cam_info_received_) {
                projectAndPublishCloud(msg->header.stamp);
            }
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void camInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        // get the camera info and republish it for the 640x480 image
        sensor_msgs::msg::CameraInfo cam_info = *msg;
        
        
        cam_info.width = 640;
        cam_info.height = 480;

        // change the K matrix  
        float scale_x = (float)(cam_info.width) / (float)(msg->width);
        float scale_y = (float)(cam_info.height) / (float)(msg->height);

        cam_info.k[0] *= scale_x;
        cam_info.k[2] *= scale_x;
    
        cam_info.k[4] *= scale_y;
        cam_info.k[5] *= scale_y;

        cam_info.p[0] *= scale_x;
        cam_info.p[2] *= scale_x;
        cam_info.p[3] *= scale_x;   

        cam_info.p[4] *= scale_y;
        cam_info.p[5] *= scale_y;
        cam_info.p[6] *= scale_y;
        cam_info.p[7] *= scale_y;


        // cam_info.distortion_model = "";
        // cam_info.d.clear();
        camera_model_ros_.fromCameraInfo(cam_info);

        int binning_x = std::max<uint32_t>(camera_model_ros_.binningX(), 1);
        int binning_y = std::max<uint32_t>(camera_model_ros_.binningY(), 1);

        fx_ = camera_model_ros_.fx() / binning_x;
        fy_ = camera_model_ros_.fy() / binning_y;
        cx_ = camera_model_ros_.cx() / binning_x;
        cy_ = camera_model_ros_.cy() / binning_y;

        cam_info_received_ = true;
        camera_info_pub_->publish(cam_info);
    }

    void projectAndPublishCloud(const rclcpp::Time &stamp)
    {
        std::lock_guard<std::mutex> lock(local_msg_mutex);

        cv::Mat depth_image = cv::Mat::zeros(resized_image.size(), CV_32FC1);

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*local_msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*local_msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*local_msg, "z");

        std::vector<cv::Point2f> lidar_image_coords;
        std::vector<float> depth_values;

        // Create a point cloud message
        sensor_msgs::msg::PointCloud2 pointcloud_msg;
        pointcloud_msg.header = local_msg->header;
        pointcloud_msg.header.stamp = stamp;
        pointcloud_msg.header.frame_id = cam_frame_name;
        pointcloud_msg.height = 1;
        pointcloud_msg.is_bigendian = false;
        pointcloud_msg.is_dense = false;

        std::vector<float> pointcloud_data;

        geometry_msgs::msg::TransformStamped transformStamped;
        try {
            transformStamped = tf_buffer_.lookupTransform(
                cam_frame_name,
                local_msg->header.frame_id,
                stamp,
                rclcpp::Duration::from_seconds(2)
            );
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform lidar to camera frame: %s", ex.what());
            return;
        }

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            geometry_msgs::msg::PointStamped lidar_point, camera_point;
            lidar_point.point.x = *iter_x;
            lidar_point.point.y = *iter_y;
            lidar_point.point.z = *iter_z;
            lidar_point.header.frame_id = local_msg->header.frame_id;
            lidar_point.header.stamp = stamp;

            try {
                tf2::doTransform(lidar_point, camera_point, transformStamped);
            } catch (tf2::TransformException &ex) {
                continue;
            }

            float x = camera_point.point.x;
            float y = camera_point.point.y;
            float z = camera_point.point.z;

            if (z <= 0.0f) continue;

            int u = static_cast<int>((x * fx_ / z) + cx_);
            int v = static_cast<int>((y * fy_ / z) + cy_);

            if (u >= 0 && u < depth_image.cols && v >= 0 && v < depth_image.rows) {
                float &current_depth = depth_image.at<float>(v, u);
                if (current_depth == 0 || z < current_depth) {
                    current_depth = z;
                    lidar_image_coords.emplace_back(u, v);
                    depth_values.push_back(z);

                    // Add the point to the point cloud data
                    pointcloud_data.push_back(camera_point.point.x);
                    pointcloud_data.push_back(camera_point.point.y);
                    pointcloud_data.push_back(camera_point.point.z);
                }
            }
        }

        if (!pointcloud_data.empty()) {
            // Set point cloud metadata
            pointcloud_msg.width = pointcloud_data.size() / 3;
            pointcloud_msg.point_step = 12;
            pointcloud_msg.row_step = pointcloud_msg.point_step * pointcloud_msg.width;

            // Define fields (x, y, z)
            pointcloud_msg.fields.resize(3);
            pointcloud_msg.fields[0].name = "x";
            pointcloud_msg.fields[0].offset = 0;
            pointcloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
            pointcloud_msg.fields[0].count = 1;

            pointcloud_msg.fields[1].name = "y";
            pointcloud_msg.fields[1].offset = 4;
            pointcloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
            pointcloud_msg.fields[1].count = 1;

            pointcloud_msg.fields[2].name = "z";
            pointcloud_msg.fields[2].offset = 8;
            pointcloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
            pointcloud_msg.fields[2].count = 1;

            // Allocate data buffer
            pointcloud_msg.data.resize(pointcloud_msg.row_step);

            // Copy point cloud data
            std::memcpy(pointcloud_msg.data.data(), pointcloud_data.data(), pointcloud_msg.row_step);

            pointcloud_pub_->publish(pointcloud_msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "No valid points to publish in point cloud.");
        }
    }


    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!image_received_)
        {
            RCLCPP_WARN(this->get_logger(), "No image received yet, skipping process");
            return;
        }

        local_msg_mutex.lock();
        local_msg = msg;
        local_msg_mutex.unlock();
        
    }

    // Build the k-d tree from lidar image coordinates
    void buildKdTree(const std::vector<cv::Point2f> &points)
    {
        cv::Mat points_matrix(points.size(), 2, CV_32F);
        for (size_t i = 0; i < points.size(); ++i) {
            points_matrix.at<float>(i, 0) = points[i].x;
            points_matrix.at<float>(i, 1) = points[i].y;
        }

        kdtree_ = new cv::flann::Index(points_matrix, cv::flann::KDTreeIndexParams(1));
    }

    // Interpolate missing values using the k-d tree to find nearest neighbors
    void interpolateMissingValues(cv::Mat &depth_image, const std::vector<cv::Point2f> &lidar_coords, const std::vector<float> &depth_values)
    {
        int rows = depth_image.rows;
        int cols = depth_image.cols;
        cv::Mat interpolated_image = depth_image.clone();

        for (int v = 0; v < rows; ++v)
        {
            for (int u = 0; u < cols; ++u)
            {
                if (interpolated_image.at<float>(v, u) == init_dist_)
                {
                    // Query the k-d tree to find the closest Lidar points
                    std::vector<int> indices(4);
                    std::vector<float> dists(4);

                    cv::Mat query_point = (cv::Mat_<float>(1, 2) << u, v);
                    kdtree_->knnSearch(query_point, indices, dists, 4, cv::flann::SearchParams(16));

                    float weighted_sum = 0.0f;
                    float weight_total = 0.0f;

                    // Perform inverse distance weighting, considering the maximum distance threshold
                    for (size_t i = 0; i < indices.size(); ++i)
                    {
                        float dist = std::sqrt(dists[i]);
                        if (dist < max_dist_threshold_) // Only consider points within the max distance threshold
                        {
                            float weight = 1.0f / (dist + 1e-5f);  // Inverse distance weighting
                            weighted_sum += depth_values[indices[i]] * weight;
                            weight_total += weight;
                        }
                    }

                    if (weight_total > 0.0f)
                    {
                        interpolated_image.at<float>(v, u) = weighted_sum / weight_total;
                    }
                }
            }
        }

        depth_image = interpolated_image; // Update the depth image with interpolated values
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarToImageDepth>());
    rclcpp::shutdown();
    return 0;
}
