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
        1000ms, std::bind(&EightTrajectoryWheels::timer1_callback, this));//nothing about 1 sec
    //------- 3. Odom related  -----------//
    callback_group_3_odom = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options3_odom;
    options3_odom.callback_group = callback_group_3_odom;
    subscription_3_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&EightTrajectoryWheels::odom_callback, this,
                  std::placeholders::_1), options3_odom);

    //ref_points.push_back(std::make_tuple(0,0));
    double cur_ref_x = 0;
    double cur_ref_y = 0;
    //RCLCPP_INFO(this->get_logger(), "initialize ref_point (x,y) = %f,%f ", cur_ref_x, cur_ref_y);
    for (auto iter = waypoints.begin();
     iter != waypoints.end(); iter++){
        std::tuple<double,double,double> i_th = *iter;
        double dx = std::get<1>(i_th);
        double dy = std::get<2>(i_th);
        cur_ref_x += dx;
        cur_ref_y += dy;
        ref_points.push_back(std::make_tuple(cur_ref_x,cur_ref_y));
        RCLCPP_INFO(this->get_logger(), "initialize ref_point (x,y) = %f,%f ", cur_ref_x, cur_ref_y);
    }
  
  }

private:
    //int max_iter = 20;

  void timer1_callback() {
    RCLCPP_INFO(this->get_logger(), "Timer Callback ");
    timer_1_->cancel();
    //assert(false);
    std::thread{std::bind(&EightTrajectoryWheels::execute, this)}.detach();
  }
  void execute() {
    auto message = std_msgs::msg::Float32MultiArray();
    //rclcpp::Rate loop_rate(0.01);
    while(!ref_points.empty()){

        std::tuple<double,double,double> it = waypoints.front();
        std::tuple<double,double> it2 = ref_points.front();
         RCLCPP_INFO(this->get_logger(), "ref_point (x,y) = %f,%f ", std::get<0>(it2), std::get<1>(it2));

        double dphi = std::get<0>(it)/3; 
        double distance_error = 1000;//just a large number
        double error_tolerance = 0.1; 
        unsigned int section_counter = 0;
        while( distance_error > error_tolerance){
            double dx = std::get<0>(it2) - current_pos_.x;
            double dy = std::get<1>(it2) - current_pos_.y;

    
            //int max_iteration = 300;
                MatrixXd vb = velocity2twist(dphi, dx, dy);
                std::vector<float> u_vector = twist2wheels(vb);
                double speed_x = vb(1,0);
                double speed_y = vb(2,0);
                double speed_norm = sqrt(speed_x*speed_x+speed_y*speed_y);
                double distance_to_travel = dphi == 0? sqrt(dx*dx+dy*dy) : abs(dphi*sqrt(dx*dx+dy*dy));
                double time_to_travel = distance_to_travel/speed_norm;//in sec

                int hz_inverse_us = 10000;//10 Hz = 0.01 sec = 10000 microsec 
                int max_iter = time_to_travel*1000000/hz_inverse_us;
                RCLCPP_INFO(get_logger(), "distance to travel %f, time to travel %f, iter %d", distance_to_travel,time_to_travel, max_iter ); 
        
            for (int i=0; i<max_iter;i++){
                message.data = u_vector;
                publisher_->publish(message);  
                //RCLCPP_INFO(get_logger(), "published dphi %f, dx %f, dy %f", vb(0,0), vb(1,0),vb(2,0));  
                //loop_rate.sleep();
                usleep(hz_inverse_us);
                section_counter++;
            }
            dx = std::get<0>(it2) - current_pos_.x;
            dy = std::get<1>(it2) - current_pos_.y;
            distance_error = dphi == 0? sqrt(dx*dx+dy*dy) : abs(dphi*sqrt(dx*dx+dy*dy));

        }
        sleep(1);
        timer1_counter++;

        waypoints.pop_front();
        ref_points.pop_front(); 
    }
    RCLCPP_INFO(get_logger(), "No more waypoints");  

  }

  MatrixXd velocity2twist(double dphi, double dx, double dy){
    RCLCPP_INFO(get_logger(), "velocity2twist current_yaw_rad_ %f",current_yaw_rad_);  
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

  std::list<std::tuple<double, double, double>> waypoints {std::make_tuple(0,1,-1),std::make_tuple(0,1,1),
                                std::make_tuple(0,1,1),std::make_tuple(1.5708, 1, -1),std::make_tuple(-3.1415, -1, -1),std::make_tuple(0.0, -1, 1),
                                std::make_tuple(0.0, -1, 1),std::make_tuple(0.0, -1, -1)};
//   std::list<std::tuple<double, double, double>> waypoints {std::make_tuple(0,1,-1),std::make_tuple(0,1,1),
//                                 std::make_tuple(0,1,1),std::make_tuple(0, 1, -1),std::make_tuple(0, -1, -1),std::make_tuple(0.0, -1, 1),
//                                 std::make_tuple(0.0, -1, 1),std::make_tuple(0.0, -1, -1)};
  //std::list<std::tuple<double, double, double>> waypoints {std::make_tuple(-3.14,1,0)};//,std::make_tuple(0,1,1)};
  std::list<std::tuple<double,double>> ref_points;
 
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