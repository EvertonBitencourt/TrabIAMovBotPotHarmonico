#include <iostream>
#include <pthread.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "Action.h"
#include "Perception.h"
#include "Utils.h"
#include "graphics.hpp"
#include "Mapping.hpp"
#include "PotentialField.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
char pressedKey;

class NavigationNode : public rclcpp::Node
{
  public:
    NavigationNode(Action &action, Perception &perception)
        : Node("navigation_p3dx"), action_(action), perception_(perception)
    {
      pub_twist = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
      sub_laserscan = this->create_subscription<sensor_msgs::msg::LaserScan>("/laser_scan", 100,
                                                                            std::bind(&Perception::receiveLaser, &perception, _1));
      sub_sonar = this->create_subscription<sensor_msgs::msg::PointCloud2>("/sonar", 100,
                                                                          std::bind(&Perception::receiveSonar, &perception, _1));
      sub_pose = this->create_subscription<nav_msgs::msg::Odometry>("/pose", 100,
                                                                    std::bind(&Perception::recievePose, &perception, _1));
      timer_ = this->create_wall_timer(100ms, std::bind(&NavigationNode::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      // Get latest sensor readings
      std::vector<float> lasers = perception_.getLatestLaserRanges();
      std::vector<float> sonars = perception_.getLatestSonarRanges();
      std::vector<float> pose = perception_.getLatestPose();
      std::cout << "Read " << lasers.size() << " laser measurements" << std::endl;
      std::cout << "Read " << sonars.size() << " sonar measurements" << std::endl;
      std::cout << "Read " << pose.size() << " pose measurements" << std::endl;

      // Get keyboard input
      char ch = pressedKey;
      MotionControl mc = action_.handlePressedKey(ch);
      std::cout << mc.mode << ' ' << mc.direction << std::endl;

      // Compute next action
      if (mc.mode == MANUAL)
      {
        action_.manualRobotMotion(mc.direction, sonars, pose);
      }
      else if (mc.mode == WANDER)
      {
        action_.avoidObstacles(lasers, sonars);
      }
      else if (mc.mode == FARFROMWALLS)
      {
        action_.keepAsFarthestAsPossibleFromWalls(lasers, sonars);
      }
      else if (mc.mode == FOLLOWWALLS)
      {
        action_.keepAsCloseAsPossibleToTheWalls(lasers, sonars);
      }else if (mc.mode == TESTMODE)
      {
        action_.testMode(lasers, sonars, pose);
      }

      action_.correctVelocitiesIfInvalid();

      // Publish robot motion
      geometry_msgs::msg::Twist twistROS;
      twistROS.linear.x = action_.getLinearVelocity();
      twistROS.angular.z = action_.getAngularVelocity();
      pub_twist->publish(twistROS);
      std::cout << "Published linVel " << twistROS.linear.x << " angVel " << twistROS.angular.z << std::endl;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laserscan;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_sonar;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_pose;

    Action &action_;
    Perception &perception_;
};

void *keyboardThreadFunction(void *arg)
{
  while (pressedKey != 27)
  {
    pressedKey = getCharWithoutWaitingENTER();
    std::cout << "PressedKey: " << pressedKey << std::endl;
  }

  std::cout << "Ending: " << pressedKey << std::endl;
  // exit(0); // force exit

  return NULL;
}

void *mainThreadFunction(void *arg)
{
  Action action;
  Perception perception;

  std::shared_ptr<NavigationNode> nh = std::make_shared<NavigationNode>(action, perception);
  rclcpp::spin(nh);

  return NULL;
}

void* graphicsThreadFunction();
void* mappingThreadFunction();
void* potentialFieldThreadFunction();

int main(int argc, char **argv)
{
  pressedKey = 'x';

  rclcpp::init(argc, argv);

  pthread_t mainThread, keyboardThread, graphicsThread, mappingThread, potentialFieldThread;

  pthread_create(&(mainThread), NULL, mainThreadFunction, NULL);
  pthread_create(&(keyboardThread), NULL, keyboardThreadFunction, NULL);
  pthread_create(&(graphicsThread), NULL, graphicsThreadFunction, NULL);
  pthread_create(&(mappingThread), NULL, mappingThreadFunction, NULL);
  pthread_create(&(potentialFieldThread), NULL, potentialFieldThreadFunction, NULL);


  pthread_join(mainThread, 0);
  pthread_join(keyboardThread, 0);
  pthread_join(graphicsThread, 0);
  pthread_join(mappingThread, 0);
  pthread_join(potentialFieldThread, 0);

  rclcpp::shutdown();

  return 0;
}