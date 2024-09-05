#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <chrono>
#include <vector>
#include <tuple>
#include <Eigen/Dense>
 
using Eigen::MatrixXd;
using namespace std::chrono_literals;

enum nodeState {
  move_forward,
  move_backward,
  move_sideways_left,
  move_sideways_right,
  turn_clockwise,
  turn_counterclockwise,
  stop
} nstate;

struct nstate_values {
  float u[4];
  std::string nstate_string;
};

struct nstate_values state_values[] = {
    {{1.0, 1.0, 1.0, 1.0}, "Move forward"},
    {{-1.0, -1.0, -1.0, -1.0}, "Move backward"},
    {{-1.0, 1.0, -1.0, 1.0}, "Move left"},
    {{1.0, -1.0, 1.0, -1.0}, "Move right"},
    {{1.0, -1.0, -1.0, 1.0}, "Turn clockwise"},
    {{-1.0, 1.0, 1.0, -1.0}, "Turn counter-clockwise"},
    {{0.0, 0.0, 0.0, 0.0}, "Stop"}};

class MoveRobotWheels : public rclcpp::Node {
public:
  MoveRobotWheels() : Node("move_robot_wheels") {
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "wheel_speed", 10);
    timer_1_ = this->create_wall_timer(
        500ms, std::bind(&MoveRobotWheels::timer1_callback, this));
  }

private:
  void timer1_callback() {
    auto message = std_msgs::msg::Float32MultiArray();

    if (timer1_counter < 6) {
      nstate = move_forward;
    } else if (timer1_counter >= 6 && timer1_counter < 12) {
      nstate = move_backward;
    } else if (timer1_counter >= 12 && timer1_counter < 18) {
      nstate = move_sideways_left;
    } else if (timer1_counter >= 18 && timer1_counter < 24) {
      nstate = move_sideways_right;
    } else if (timer1_counter >= 24 && timer1_counter < 30) {
      nstate = turn_clockwise;
    } else if (timer1_counter >= 30 && timer1_counter < 36) {
      nstate = turn_counterclockwise;
    } else if (timer1_counter >= 36 && timer1_counter < 42) {
      nstate = stop;
    } else {
      timer_1_->cancel();
    }
    RCLCPP_INFO(get_logger(), state_values[nstate].nstate_string.c_str());
    std::vector<float> u_vector(std::begin(state_values[nstate].u),
                                 std::end(state_values[nstate].u));
    //std::vector<float> u_vector =  twist2wheels(1.5,1,0);
    message.data = u_vector;
    publisher_->publish(message);
    timer1_counter++;
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
  int timer1_counter = 0;
  double l = 0.500/2;
  double r = 0.254/2;
  double w = 0.548/2;
  

  rclcpp::TimerBase::SharedPtr timer_1_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveRobotWheels>());
  rclcpp::shutdown();
  return 0;
}