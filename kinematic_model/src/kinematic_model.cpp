#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
//#include "NumCpp.hpp"
#include <Eigen/Dense>
 
using Eigen::MatrixXd;
using std::placeholders::_1;
/*
# half of the wheel base distance
l = 0.500/2
# the radius of the wheels
r = 0.254/2
# half of track width
w = 0.548/2

    u_array = np.array(u).reshape((4, 1))
    H = np.array([[-l-w, 1, -1],
                  [ l+w, 1,  1],
                  [ l+w, 1, -1],
                  [-l-w, 1,  1]]) / r   

*/
class KinematicModel : public rclcpp::Node
{
public:
  KinematicModel()
  : Node("kinematic_model")
  {
    publisher_1_twist =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "wheel_speed", 10, std::bind(&KinematicModel::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    MatrixXd u(4,1);
    
     RCLCPP_INFO(this->get_logger(), "wheel_speed topic callback");
    for(unsigned int i = 0; i < msg->data.size(); i++){
    u(i,0) = msg->data[i];
    RCLCPP_INFO(this->get_logger(), "I heard: '%f'",u(i,0));
    }
    MatrixXd twist = KinematicLeastSquareNormalEq(u);
    RCLCPP_INFO(this->get_logger(), "twist wz: '%f'",twist(0,0));
    RCLCPP_INFO(this->get_logger(), "twist vx: '%f'",twist(1,0));
    RCLCPP_INFO(this->get_logger(), "twist vy: '%f'",twist(2,0));
    ling.linear.x = twist(1,0);
    ling.linear.y = twist(2,0);
    ling.angular.z = twist(0,0);
    this->move_robot(ling);
  }

MatrixXd KinematicLeastSquareNormalEq(MatrixXd & u){
    MatrixXd H(4,3);
    H(0,0) = (-l-w)/r; H(0,1) =  1/r; H(0,2) = -1/r;
    H(1,0) = (l+w)/r;  H(1,1) =  1/r; H(1,2) =  1/r;
    H(2,0) = (l+w)/r;  H(2,1) =  1/r; H(2,2) = -1/r;
    H(3,0) = (-l-w)/r; H(3,1) =  1/r; H(3,2) =  1/r;
    MatrixXd HTH_inv = (H.transpose()* H).inverse();
    MatrixXd HTHinv_least_square  = HTH_inv * H.transpose();
    MatrixXd twist = HTHinv_least_square * u;
    return twist;
}
void move_robot(geometry_msgs::msg::Twist &msg) {
    publisher_1_twist->publish(msg);
}
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_1_twist;
  geometry_msgs::msg::Twist ling;
  double l = 0.500/2;
  double r = 0.254/2;
  double w = 0.548/2;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicModel>());
  rclcpp::shutdown();
  return 0;
}