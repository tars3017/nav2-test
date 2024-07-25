#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <functional> // for std::bind
#include <string>
#include <chrono>

double car[3] = {0};
double x = 0.5;
double y = 0.5;
double th = (double)3.1415926/2;

class OdomPublisher : public rclcpp::Node {
public:
    OdomPublisher()
    : Node("odometry_publisher") {
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1); 
        vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&OdomPublisher::vel_callback, this, std::placeholders::_1));
        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped> (
            "initialpose", 1, std::bind(&OdomPublisher::initial_pose_callback, this, std::placeholders::_1));
        odom_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        tf_prefix_ = this->declare_parameter("tf_prefix", std::string(""));
        if (tf_prefix_ != "") {
            tf_prefix_ = tf_prefix_ + "/";
        }
        current_time_ = this->now();
        last_time_ = this->now();

        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&OdomPublisher::publish_odom, this));
    }
private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
    std::string tf_prefix_;
    rclcpp::Time current_time_, last_time_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish_odom() {
        double vx = car[0];
        double vy = car[1];
        double vth = car[2];
        current_time_ = this->now();

        double dt = (current_time_ - last_time_).seconds();

        // vx, vy, vth is velocity relative to "child_frame_id"
        // pose is related to header.frame_id, so it need rotation matrix
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        // TransformBroadcaster
        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time_;
        odom_trans.header.frame_id = tf_prefix_ + "odom";
        odom_trans.child_frame_id = tf_prefix_ + "base_link";
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;

        tf2::Quaternion quat;
        quat.setRPY(0, 0, th);
        odom_trans.transform.rotation = tf2::toMsg(quat);
        odom_broadcaster_->sendTransform(odom_trans);

        last_time_ = current_time_;
    }

    void vel_callback(const geometry_msgs::msg::Twist& msg) const {
        car[0] = msg.linear.x;
        car[1] = msg.linear.y;
        car[2] = msg.angular.z;
    }

    void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped& msg) const {
        x = msg.pose.pose.position.x;
        y = msg.pose.pose.position.y;
        tf2::Quaternion quat;
        tf2::fromMsg(msg.pose.pose.orientation, quat);

        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        th = yaw;
    }


};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisher>());
    rclcpp::shutdown();
    return 0;
}