#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <chrono>
#include <tuple>
#include <vector>
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <Eigen/Dense>
 
using Eigen::MatrixXd;
using namespace std::chrono_literals;
using std::placeholders::_1;
//std::chrono::nanoseconds fifty_milisec = 5000000;
class EightTrajectoryWheels : public rclcpp::Node {
public:

    //------- 1. wheel_speed topic publisher related -----------//
  EightTrajectoryWheels() : Node("eight_trajectory_wheels") {
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "wheel_speed", 10);

    //------- 2. timer_1 related -----------//
    timer_1_ = this->create_wall_timer(
        2000ms, std::bind(&EightTrajectoryWheels::timer1_callback, this));
    //------- 3. Odom related  -----------//
    callback_group_3_odom = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options3_odom;
    options3_odom.callback_group = callback_group_3_odom;
    subscription_3_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&EightTrajectoryWheels::odom_callback, this,
                  std::placeholders::_1), options3_odom);
    for (unsigned int i =0; i<waypoints.size()-1; i++){
        std::tuple<double,double> i_plus_1_th = waypoints[i+1];
        std::tuple<double,double> i_th = waypoints[i];
        double dx = std::get<0>(i_plus_1_th) - std::get<0>(i_th);
        double dy = std::get<1>(i_plus_1_th) - std::get<1>(i_th);
        double dphi = atan(dy/dx);
        motions.push_back(std::make_tuple(dx,dy,dphi));
    }
  
  }

private:
  void timer1_callback() {
    auto message = std_msgs::msg::Float32MultiArray();
    //rclcpp::Rate loop_rate(0.05);
    if(!motions.empty()){
        std::tuple<double,double,double> it = motions.front();
        RCLCPP_INFO(get_logger(), "Timer 1 Callbac");
        
        double dx = std::get<0>(it);
        double dy = std::get<1>(it);
        double dphi = std::get<2>(it);
        int max_iteration = 300;
        
        for (int i=0; i<max_iteration;i++){
            MatrixXd vb = velocity2twist(dphi, dx, dy);
            std::vector<float> u_vector = twist2wheels(vb);
            message.data = u_vector;
            publisher_->publish(message);  
            RCLCPP_INFO(get_logger(), "published dphi %f, dx %f, dy %f", dphi, dx,dy);          
            //loop_rate.sleep();
            usleep(10000);
        }
  
    timer1_counter++;
    motions.pop_front();
    }else{
        RCLCPP_INFO(get_logger(), "Timer shutdown");
        timer_1_->cancel();
    }

  }

  MatrixXd velocity2twist(double dphi, double dx, double dy){
    MatrixXd R(3,3);
    R(0,0) = 1; R(0,1) = 0; R(0,2) = 0;
    R(1,0) = 0; R(1,1) = cos(current_yaw_rad_); R(1,2) =  sin(current_yaw_rad_); 
    R(2,0) = 0; R(2,1) = -sin(current_yaw_rad_); R(2,2) =  cos(current_yaw_rad_);        
    MatrixXd v(3,1);
    v(0,0) = dphi;
    v(1,0) = dx;
    v(2,0) = dy;

    MatrixXd twist = R*v; 
    return twist;
  }
 std::vector<float> twist2wheels(MatrixXd twist){
    std::vector<float> u_vector;

    MatrixXd H(4,3);
    H(0,0) = (-l-w)/r; H(0,1) =  1/r; H(0,2) = -1/r;
    H(1,0) = (l+w)/r;  H(1,1) =  1/r; H(1,2) =  1/r;
    H(2,0) = (l+w)/r;  H(2,1) =  1/r; H(2,2) = -1/r;
    H(3,0) = (-l-w)/r; H(3,1) =  1/r; H(3,2) =  1/r;
    MatrixXd u = H * twist;
    u_vector.push_back(u(0,0));
    u_vector.push_back(u(1,0));
    u_vector.push_back(u(2,0));
    u_vector.push_back(u(3,0));
    return u_vector;
  
  }
  std::vector<float> twist2wheels(double wz, double vx, double vy){
    std::vector<float> u_vector;
    MatrixXd twist(3,1);
    twist(0,0) = wz;
    twist(1,0) = vx;
    twist(2,0) = vy;
    MatrixXd H(4,3);
    H(0,0) = (-l-w)/r; H(0,1) =  1/r; H(0,2) = -1/r;
    H(1,0) = (l+w)/r;  H(1,1) =  1/r; H(1,2) =  1/r;
    H(2,0) = (l+w)/r;  H(2,1) =  1/r; H(2,2) = -1/r;
    H(3,0) = (-l-w)/r; H(3,1) =  1/r; H(3,2) =  1/r;
    MatrixXd u = H * twist;
    u_vector.push_back(u(0,0));
    u_vector.push_back(u(1,0));
    u_vector.push_back(u(2,0));
    u_vector.push_back(u(3,0));
    return u_vector;
  
  }

  //------- 3. Odom related  Functions -----------//  
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    current_pos_ = msg->pose.pose.position;
    current_angle_ = msg->pose.pose.orientation;
    current_yaw_rad_ = yaw_theta_from_quaternion(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    RCLCPP_DEBUG(this->get_logger(), "current pos=['%f','%f','%f'",
                 current_pos_.x, current_pos_.y, current_yaw_rad_);
  }
  double yaw_theta_from_quaternion(double qx, double qy, double qz, double qw) {
    double roll_rad, pitch_rad, yaw_rad;
    tf2::Quaternion odom_quat(qx, qy, qz, qw);
    tf2::Matrix3x3 matrix_tf(odom_quat);
    matrix_tf.getRPY(roll_rad, pitch_rad, yaw_rad);
    return yaw_rad; // In radian
  }


  //------- 3. Odom related private variables  -----------//
  rclcpp::CallbackGroup::SharedPtr callback_group_3_odom;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_3_odom;
  geometry_msgs::msg::Point current_pos_;
  geometry_msgs::msg::Quaternion current_angle_;
  double current_yaw_rad_;

  //--------  Kinematic related private variables --------// 
  double l = 0.500/2;
  double r = 0.254/2;
  double w = 0.548/2;
  std::list <std::tuple<double, double, double>>motions;//dx,dy,dphi
  std::vector<std::tuple<double, double>> waypoints {std::make_tuple(0,0),std::make_tuple(1,1),std::make_tuple(2,0),
                                std::make_tuple(3,-1),std::make_tuple(4,0),std::make_tuple(3,1),std::make_tuple(2,0),
                                std::make_tuple(1,-1),std::make_tuple(0,0)};

 
  rclcpp::TimerBase::SharedPtr timer_1_;
  int timer1_counter;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;

};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto eight_trajectory_subscriber = std::make_shared<EightTrajectoryWheels>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(eight_trajectory_subscriber);
  executor.spin();

  //rclcpp::spin(std::make_shared<EightTrajectoryWheels>());
  rclcpp::shutdown();
  return 0;
}