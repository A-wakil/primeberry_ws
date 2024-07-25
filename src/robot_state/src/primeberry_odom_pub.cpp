#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float32.hpp" 
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <cmath>

// Create odometry data publishers
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_data_pub;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_data_pub_quat;
nav_msgs::msg::Odometry odomNew;
nav_msgs::msg::Odometry odomOld;

// Initial pose
const double initialX = 0.0;
const double initialY = 0.0;
const double initialTheta = 0.00000000001;
const double PI = 3.141592;

// Robot physical constants
const double TICKS_PER_REVOLUTION = 620;
const double WHEEL_RADIUS = 0.033;
const double WHEEL_BASE = 0.17;
const double TICKS_PER_METER = 3100;

// Distance both wheels have traveled
double distanceLeft = 0;
double distanceRight = 0;
double steeringAngle = 0;  // Steering angle in radians

// Flag to see if initial pose has been received
bool initialPoseReceived = false;

using namespace std;


void set_initial_2d(const geometry_msgs::msg::PoseStamped::SharedPtr rvizClick) {
  odomOld.pose.pose.position.x = rvizClick->pose.position.x;
  odomOld.pose.pose.position.y = rvizClick->pose.position.y;
  odomOld.pose.pose.orientation.z = rvizClick->pose.orientation.z;
  initialPoseReceived = true;
}


void Calc_Left(const std_msgs::msg::Int16::SharedPtr leftCount) {
  static int lastCountL = 0;
  if (leftCount->data != 0 && lastCountL != 0) {
    int leftTicks = (leftCount->data - lastCountL);
    if (leftTicks > 10000) {
      leftTicks = 0 - (65535 - leftTicks);
    } else if (leftTicks < -10000) {
      leftTicks = 65535 - leftTicks;
    }
    distanceLeft = leftTicks / TICKS_PER_METER;
  }
  lastCountL = leftCount->data;
}

void Calc_Right(const std_msgs::msg::Int16::SharedPtr rightCount) {
  static int lastCountR = 0;
  if (rightCount->data != 0 && lastCountR != 0) {
    int rightTicks = rightCount->data - lastCountR;
    if (rightTicks > 10000) {
      rightTicks = (0 - (65535 - rightTicks)) / TICKS_PER_METER;
    } else if (rightTicks < -10000) {
      rightTicks = 65535 - rightTicks;
    }
    distanceRight = rightTicks / TICKS_PER_METER;
  }
  lastCountR = rightCount->data;
}

// Callback for steering angle
void steering_angle_callback(const std_msgs::msg::Float32::SharedPtr msg) {
  steeringAngle = msg->data;
}


void publish_quat() {
  tf2::Quaternion q;
  q.setRPY(0, 0, odomNew.pose.pose.orientation.z);

  nav_msgs::msg::Odometry quatOdom;
  quatOdom.header.stamp = odomNew.header.stamp;
  quatOdom.header.frame_id = "odom";
  quatOdom.child_frame_id = "base_link";
  quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x;
  quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y;
  quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z;
  quatOdom.pose.pose.orientation.x = q.x();
  quatOdom.pose.pose.orientation.y = q.y();
  quatOdom.pose.pose.orientation.z = q.z();
  quatOdom.pose.pose.orientation.w = q.w();
  quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x;
  quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y;
  quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z;
  quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x;
  quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y;
  quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z;

  for (int i = 0; i < 36; i++) {
    if (i == 0 || i == 7 || i == 14) {
      quatOdom.pose.covariance[i] = .01;
    } else if (i == 21 || i == 28 || i == 35) {
      quatOdom.pose.covariance[i] += 0.1;
    } else {
      quatOdom.pose.covariance[i] = 0;
    }
  }

  odom_data_pub_quat->publish(quatOdom);
}

void update_odom() {
  double cycleDistance = (distanceRight + distanceLeft) / 2;
  double cycleAngle = tan(steeringAngle) * cycleDistance / WHEEL_BASE;
  double avgAngle = cycleAngle / 2 + odomOld.pose.pose.orientation.z;

  if (avgAngle > PI) {
    avgAngle -= 2 * PI;
  } else if (avgAngle < -PI) {
    avgAngle += 2 * PI;
  }

  odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + cos(avgAngle) * cycleDistance;
  odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + sin(avgAngle) * cycleDistance;
  odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z;

  if (isnan(odomNew.pose.pose.position.x) || isnan(odomNew.pose.pose.position.y) || isnan(odomNew.pose.pose.position.z)) {
    odomNew.pose.pose.position.x = odomOld.pose.pose.position.x;
    odomNew.pose.pose.position.y = odomOld.pose.pose.position.y;
    odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z;
  }

  if (odomNew.pose.pose.orientation.z > PI) {
    odomNew.pose.pose.orientation.z -= 2 * PI;
  } else if (odomNew.pose.pose.orientation.z < -PI) {
    odomNew.pose.pose.orientation.z += 2 * PI;
  }

  odomNew.header.stamp = rclcpp::Clock().now();
  odomNew.twist.twist.linear.x = cycleDistance / (odomNew.header.stamp.sec - odomOld.header.stamp.sec);
  odomNew.twist.twist.angular.z = cycleAngle / (odomNew.header.stamp.sec - odomOld.header.stamp.sec);

  odomOld.pose.pose.position.x = odomNew.pose.pose.position.x;
  odomOld.pose.pose.position.y = odomNew.pose.pose.position.y;
  odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z;
  odomOld.header.stamp = odomNew.header.stamp;

  odom_data_pub->publish(odomNew);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("primeberry_odom_pub");

  odom_data_pub = node->create_publisher<nav_msgs::msg::Odometry>("odom_data_euler", 100);
  odom_data_pub_quat = node->create_publisher<nav_msgs::msg::Odometry>("odom_data_quat", 100);

  auto subForRightCounts = node->create_subscription<std_msgs::msg::Int16>(
    "right_ticks", 100, Calc_Right
  );
  auto subForLeftCounts = node->create_subscription<std_msgs::msg::Int16>(
    "left_ticks", 100, Calc_Left
  );
  auto subInitialPose = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "initial_2d", 1, set_initial_2d
  );
  auto subSteeringAngle = node->create_subscription<std_msgs::msg::Float32>(
    "steering_angle", 100, steering_angle_callback
  );

  rclcpp::Rate loop_rate(30);

  while (rclcpp::ok()) {
    if (initialPoseReceived) {
      update_odom();
      publish_quat();
    }
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
