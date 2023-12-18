#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/float64.hpp"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

/* Depth Distance Constant */
const double kMAX_CHASE_DISTANCE = 2.0;    // Unit:meter
const double kMIN_CHASE_DISTANCE = 0.8;    // Unit:meter
// const double MIN_SKID_DISTANCE = 0.5;
const double kMAX_LINEAR_VEL_OUTPUT = 1.5;
/* Angle constant */
const double kMAX_ANGULER_VEL_OUTPUT = 2; 
/* Depth pid controller */
const double DEPTH_kp = 1.2;
const double DEPTH_kd =  0.85;
double depth_error1 = 0;
/* Angle pid controller */
const double ANGLE_kp = 2.32;
const double ANGLE_kd = 5.64;
double angle_error1 = 0;
// Veriable
double target_angle = 0.0;
double target_depth = 0.0;
double target_state = 0.0;
bool follow_flag = false;
// Decent factor for slow down
const double decent_factor = 0.56;

const int num_samples = 3;  // 設定移動平均的樣本數
double distance_samples[num_samples];  // 保存前 n 次的距離值

double depth_error=0.0;
double angle_error=0.0; 

double target_angle1 = 0;
double target_depth = 0;

double PD_Controller(double error, double error1, double max, double kp, double kd);

class HumanFollowerPID : public rclcpp::Node
{
  public:
    HumanFollowerPID():Node("human_follower_pid")
    {
        // Create Publisher
        follow_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("follow_cmd_vel", 10);
        // Create Subscriber
        robot_mode_sub = this->create_subscription<std_msgs::msg::String>(
            "robot_mode", 10, std::bind(&HumanFollowerPID::mode_callback, this, std::placeholders::_1));
        human_pose_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "human_pose", 10, std::bind(&HumanFollowerPID::human_pose_callback, this, std::placeholders::_1));
        depth_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "depth_raw", 10, std::bind(&HumanFollowerPID::depth_callback, this, std::placeholders::_1));
    }

  private:
    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr follow_vel_pub_;
    // Subscriber
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_mode_sub;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr human_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr depth_sub_;

    void mode_callback(const std_msgs::msg::String::SharedPtr mode_msgs) const
    {
        // Revice mode msg
        std::string mode = mode_msgs->data;
        // Message 
        auto cmd_vel_msgs = geometry_msgs::msg::Twist();
        /* Follow mode */
        if(mode=="FOLLOW"){
            follow_flag = true;
            // RCLCPP_INFO(this->get_logger(), "Following");
            /* Error cal */
            depth_error = target_depth - kMIN_CHASE_DISTANCE;
            angle_error = -target_angle;
            // Depth zone
            if(fabs(angle_error)<0.2) angle_error=0;
            /* ZONE Detect */
            if(target_depth > kMAX_CHASE_DISTANCE){
                /* STOP ZONE */
                cmd_vel_msgs.linear.x = 0.0;
                cmd_vel_msgs.angular.z = 0.0;
            }else{
                /* CHASING ZONE */
                cmd_vel_msgs.linear.x = PD_Controller(
                    depth_error, depth_error1, kMAX_LINEAR_VEL_OUTPUT, DEPTH_kp, DEPTH_kd
                );
                cmd_vel_msgs.angular.z = PD_Controller(
                    angle_error, angle_error1, kMAX_ANGULER_VEL_OUTPUT, ANGLE_kp, ANGLE_kd
                );
            }   
            /* LAST ERROR */
            depth_error1 = depth_error;
            angle_error1 = angle_error;
            /* Publish cmd_vel */
            follow_vel_pub_->publish(cmd_vel_msgs);
        }else{
            // RCLCPP_INFO(this->get_logger(), "Lost target, zero output");
            depth_error=0;
            angle_error=0;
            // Clean Output
            cmd_vel_msgs.linear.x = 0.0;
            cmd_vel_msgs.angular.z = 0.0;
            // Publish Data
            if (follow_flag == true){
                follow_vel_pub_->publish(cmd_vel_msgs);
                follow_flag = false;
            }
        }
    }

    /* Get the human_pose from topic */
    void human_pose_callback(const geometry_msgs::msg::Vector3::SharedPtr target_msgs) const
    {
        double target_x = target_msgs->x;
        double target_y = target_msgs->y;
        double target_state = target_msgs->z;
        // IF we got the target then update the target pos.
        if (target_state == 1.0){
            target_angle = (target_x-640)*0.001225; // Change pixel to radian 0.001225 = (45/640)*(pi/180)
            target_depth = std::max(0.0, std::min(2.0, target_y));
        }        
    }

    void depth_callback(const std_msgs::msg::Float64::SharedPtr depth_msg) const
    {
        // target_depth = double(depth_msg->data);
    }
};

double PD_Controller(double error, double error1, double max, double kp, double kd){
        double output = kp*error + kd*(error-error1);
        if (output > max) output = max;
        if (output < -max) output = -max;
        return output;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HumanFollowerPID>());
  rclcpp::shutdown();
  return 0;
}