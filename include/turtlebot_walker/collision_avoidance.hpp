/**
 *
 * MIT License
 *
 * Copyright (c) 2019 Charan Karthikeyan Parthasarathy Vasanthi
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*
 * @file collision_avoidance.hpp
 * @Copyright (c) 2019 Charan Karthikeyan Pathasarathy Vasanthi
 * @author Charan Karthikeyan Parthasarathy Vasanthi
 * @brief The header file for the collision avoidance for the
 * turtlebot in ROS
 *
 */
#ifndef INCLUDE_TURTLEBOT_WALKER_COLLISION_AVOIDANCE_HPP_
#define INCLUDE_TURTLEBOT_WALKER_COLLISION_AVOIDANCE_HPP_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

/**
 * @brief A class to move the turtlebot without hitting any objects in the
 * workspace
 */

class collision_avoidance{
 private:
  // ROS node handle
  ros::NodeHandle n;
  // ROS subscriber for scan
  ros::Subscriber sub;
  //ROS publisher to the twist function
  ros::Publisher pub;
  // Boolean operator for the obstacle occupancy
  bool obstacleStatus;
  // Float value with the nearest obstacle distance
  float minDist;
  // Twist message
  geometry_msgs::Twist twist;
 public :
  /**
   * @brief Constructor for the collision_avoidance class.
   * @param None.
   * @return None.
   */
  collision_avoidance();
  /**
   * @brief Destructor for the collision_avoidance class.
   * @param None.
   * @return None.
   */
  ~collision_avoidance();n

  /**
   * @brief Initialize function for the twist publisher.
   * @param None.
   * @return None.
   */
  void initializePubTwist();
  /**
   * @brief Initialize function for the scan subscriber.
   * @param None.
   * @return None.
   */
  void initializeSubScan();
  /**
   * @brief Function to check the obstacle point and status and
   * moves according to the obstacle present.
   * @param obstacleStatus Status of obstacle value with respect
   * to the bot.
   * @return None.
   */
  void moveBot(bool obstacleStatus);
  /**
   * @brief Callback for the laser can of the robot and changes
   * trajectory accordingly.
   * @param scanVals Scan value pointer from the message.
   * @return None.
   */
  void scanCallOut(const sensor_msgs::LaserScan::ConstPtr &scanVals);


};




#endif /* INCLUDE_TURTLEBOT_WALKER_COLLISION_AVOIDANCE_HPP_ */
