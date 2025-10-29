/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  Pioneer3AT obstacle avoidance using LiDAR with state machine
 *               Based on Pioneer 3-DX sonar controller logic but adapted for LiDAR
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/lidar.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 32
#define MAX_SPEED 6.4
#define CRUISING_SPEED 5.0
#define MIN_DISTANCE 1.0              // minimal distance for obstacle to be considered (meters)
#define WHEEL_WEIGHT_THRESHOLD 100.0  // minimal weight for robot to turn

// enum to represent the state of the robot
typedef enum { FORWARD, LEFT, RIGHT } State;

int main(int argc, char **argv) {
  // init webots stuff
  wb_robot_init();

  // get devices
  WbDeviceTag lms291 = wb_robot_get_device("Sick LMS 291");
  WbDeviceTag front_left_wheel = wb_robot_get_device("front left wheel");
  WbDeviceTag front_right_wheel = wb_robot_get_device("front right wheel");
  WbDeviceTag back_left_wheel = wb_robot_get_device("back left wheel");
  WbDeviceTag back_right_wheel = wb_robot_get_device("back right wheel");

  // init lms291
  wb_lidar_enable(lms291, TIME_STEP);
  const int lms291_width = wb_lidar_get_horizontal_resolution(lms291);
  const int half_width = lms291_width / 2;
  const double max_range = wb_lidar_get_max_range(lms291);

  // init motors
  wb_motor_set_position(front_left_wheel, INFINITY);
  wb_motor_set_position(front_right_wheel, INFINITY);
  wb_motor_set_position(back_left_wheel, INFINITY);
  wb_motor_set_position(back_right_wheel, INFINITY);

  // init speed for each wheel
  double front_left_speed = 0.0;
  double front_right_speed = 0.0;
  double back_left_speed = 0.0;
  double back_right_speed = 0.0;

  // by default, the robot goes forward
  State state = FORWARD;

  // control loop
  while (wb_robot_step(TIME_STEP) != -1) {
    // get lidar values
    const float *lms291_values = wb_lidar_get_range_image(lms291);
    if (!lms291_values) continue;

    // Initialize wheel weights (left and right)
    double wheel_weight_left = 0.0;
    double wheel_weight_right = 0.0;

    // Convert LiDAR readings to left/right obstacle weights
    // Left side of LiDAR (indices 0 to half_width-1)
    for (int i = 0; i < half_width; ++i) {
      double distance = lms291_values[i];
      
      // Skip invalid readings (infinity or NaN)
      if (distance >= max_range || distance != distance) continue;

      // If obstacle is close enough, compute weight contribution
      if (distance < MIN_DISTANCE) {
        double speed_modifier = 1.0 - (distance / MIN_DISTANCE);
        // Left side obstacles affect left wheel (turn right to avoid)
        // Weight increases towards center (indices closer to half_width)
        double weight = (double)(half_width - i) / half_width * 300.0;
        wheel_weight_left += weight * speed_modifier;
      }
    }

    // Right side of LiDAR (indices half_width to lms291_width-1)
    for (int i = half_width; i < lms291_width; ++i) {
      double distance = lms291_values[i];
      
      // Skip invalid readings
      if (distance >= max_range || distance != distance) continue;

      // If obstacle is close enough, compute weight contribution
      if (distance < MIN_DISTANCE) {
        double speed_modifier = 1.0 - (distance / MIN_DISTANCE);
        // Right side obstacles affect right wheel (turn left to avoid)
        // Weight increases towards center (indices closer to half_width)
        double weight = (double)(i - half_width + 1) / half_width * 300.0;
        wheel_weight_right += weight * speed_modifier;
      }
    }

    // State machine to handle the direction of the robot
    switch (state) {
      // When going forward, start turning when obstacle detected
      case FORWARD:
        if (wheel_weight_left > WHEEL_WEIGHT_THRESHOLD) {
          // Turn right (slow left wheels, reverse right wheels)
          front_left_speed = 0.5 * MAX_SPEED;
          front_right_speed = -0.5 * MAX_SPEED;
          back_left_speed = 0.5 * MAX_SPEED;
          back_right_speed = -0.5 * MAX_SPEED;
          state = LEFT;
        } else if (wheel_weight_right > WHEEL_WEIGHT_THRESHOLD) {
          // Turn left (slow right wheels, reverse left wheels)
          front_left_speed = -0.5 * MAX_SPEED;
          front_right_speed = 0.5 * MAX_SPEED;
          back_left_speed = -0.5 * MAX_SPEED;
          back_right_speed = 0.5 * MAX_SPEED;
          state = RIGHT;
        } else {
          // No obstacle, go straight
          front_left_speed = CRUISING_SPEED;
          front_right_speed = CRUISING_SPEED;
          back_left_speed = CRUISING_SPEED;
          back_right_speed = CRUISING_SPEED;
        }
        break;

      // Continue turning left until obstacle clears
      case LEFT:
        if (wheel_weight_left > WHEEL_WEIGHT_THRESHOLD || wheel_weight_right > WHEEL_WEIGHT_THRESHOLD) {
          // Keep turning right
          front_left_speed = 0.5 * MAX_SPEED;
          front_right_speed = -0.5 * MAX_SPEED;
          back_left_speed = 0.5 * MAX_SPEED;
          back_right_speed = -0.5 * MAX_SPEED;
        } else {
          // Obstacle cleared, resume forward
          front_left_speed = CRUISING_SPEED;
          front_right_speed = CRUISING_SPEED;
          back_left_speed = CRUISING_SPEED;
          back_right_speed = CRUISING_SPEED;
          state = FORWARD;
        }
        break;

      // Continue turning right until obstacle clears
      case RIGHT:
        if (wheel_weight_left > WHEEL_WEIGHT_THRESHOLD || wheel_weight_right > WHEEL_WEIGHT_THRESHOLD) {
          // Keep turning left
          front_left_speed = -0.5 * MAX_SPEED;
          front_right_speed = 0.5 * MAX_SPEED;
          back_left_speed = -0.5 * MAX_SPEED;
          back_right_speed = 0.5 * MAX_SPEED;
        } else {
          // Obstacle cleared, resume forward
          front_left_speed = CRUISING_SPEED;
          front_right_speed = CRUISING_SPEED;
          back_left_speed = CRUISING_SPEED;
          back_right_speed = CRUISING_SPEED;
          state = FORWARD;
        }
        break;
    }

    // Set motor speeds
    wb_motor_set_velocity(front_left_wheel, front_left_speed);
    wb_motor_set_velocity(front_right_wheel, front_right_speed);
    wb_motor_set_velocity(back_left_wheel, back_left_speed);
    wb_motor_set_velocity(back_right_wheel, back_right_speed);
  }

  wb_robot_cleanup();
  return 0;
}
