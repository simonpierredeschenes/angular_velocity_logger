#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <fstream>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class AngularVelocityLoggerNode : public rclcpp::Node
{
public:
    AngularVelocityLoggerNode():
            Node("angular_velocity_logger_node")
    {
        this->declare_parameter<std::string>("imu_measurements_file_name", "");
        this->get_parameter("imu_measurements_file_name", vectornavFileName);
        this->declare_parameter<std::string>("icp_measurements_file_name", "");
        this->get_parameter("icp_measurements_file_name", icpOdomFileName);

        if(vectornavFileName.empty() || icpOdomFileName.empty())
        {
            RCLCPP_WARN(this->get_logger(), "imu_measurements_file_name or icp_measurements_file_name parameter not provided, angular velocities will not be logged...");
            rclcpp::shutdown();
        }

        tfBuffer = std::unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer(this->get_clock(), std::chrono::seconds(1000000)));
        tfListener = std::unique_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(*tfBuffer));

        std::ofstream vectornavFile(vectornavFileName);
        vectornavFile << "stamp,angular_velocity_x,angular_velocity_y,angular_velocity_z" << std::endl;
        vectornavFile.close();
        std::ofstream icpOdomFile(icpOdomFileName);
        icpOdomFile << "stamp,angular_velocity_x,angular_velocity_y,angular_velocity_z" << std::endl;
        icpOdomFile.close();

        vectornavSubscription = this->create_subscription<sensor_msgs::msg::Imu>("/vectornav_imu/data", 0,
                                                                                 std::bind(&AngularVelocityLoggerNode::vectornavCallback, this, std::placeholders::_1));
        icpOdomSubscription = this->create_subscription<nav_msgs::msg::Odometry>("/icp_odom", 0,
                                                                                 std::bind(&AngularVelocityLoggerNode::icpOdomCallback, this, std::placeholders::_1));
    }

private:
    void vectornavCallback(const sensor_msgs::msg::Imu& msg)
    {
        try
        {
            geometry_msgs::msg::Vector3 angularVelocityInMapFrame;
            geometry_msgs::msg::TransformStamped imuToMap = tfBuffer->lookupTransform("map", msg.header.frame_id, msg.header.stamp, std::chrono::milliseconds(100));
            tf2::doTransform(msg.angular_velocity, angularVelocityInMapFrame, imuToMap);
            std::ofstream file(vectornavFileName, std::ios::app);
            file << std::setprecision(20) << rclcpp::Time(msg.header.stamp).seconds() << "," << angularVelocityInMapFrame.x << "," << angularVelocityInMapFrame.y << ","
                 << angularVelocityInMapFrame.z << std::endl;
            file.close();
        }
        catch(tf2::TransformException& ex)
        {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            return;
        }
    }

    void icpOdomCallback(const nav_msgs::msg::Odometry& msg)
    {
        std::ofstream file(icpOdomFileName, std::ios::app);
        file << std::setprecision(20) << rclcpp::Time(msg.header.stamp).seconds() << "," << msg.twist.twist.angular.x << "," << msg.twist.twist.angular.y << ","
             << msg.twist.twist.angular.z << std::endl;
        file.close();
    }

    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> tfListener;
    std::string vectornavFileName;
    std::string icpOdomFileName;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr vectornavSubscription;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr icpOdomSubscription;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AngularVelocityLoggerNode>());
    rclcpp::shutdown();
    return 0;
}

