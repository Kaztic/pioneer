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
 *               Modified to support ROS2 cmd_vel with priority system:
 *               Priority: Obstacle Avoidance > cmd_vel > Default Behavior
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <webots/camera.h>
#include <webots/lidar.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define TIME_STEP 32
#define MAX_SPEED 6.4
#define CRUISING_SPEED 5.0
#define MIN_DISTANCE 1.0              // minimal distance for obstacle to be considered (meters)
#define WHEEL_WEIGHT_THRESHOLD 100.0  // minimal weight for robot to turn
#define CMD_VEL_FILE_PATH_MAX 256     // max path length for cmd_vel file
#define LIDAR_FILE_PATH_MAX 256       // max path length for lidar file
#define CAMERA_FILE_PATH_MAX 256      // max path length for camera file

// Priority levels for control
typedef enum {
  PRIORITY_OBSTACLE_AVOIDANCE = 0,  // Highest priority - always active
  PRIORITY_CMD_VEL = 1,              // Medium priority - from path planner
  PRIORITY_DEFAULT = 2                // Lowest priority - forward cruising
} ControlPriority;

// enum to represent the state of the robot
typedef enum { FORWARD, LEFT, RIGHT } State;

// Structure to hold cmd_vel commands from file
typedef struct {
  double linear_x;
  double angular_z;
  double left_wheel;
  double right_wheel;
  double front_left;
  double front_right;
  double back_left;
  double back_right;
  int valid;  // 1 if cmd_vel data is valid
} CmdVelCommand;

// Read cmd_vel command from file
int read_cmd_vel_file(const char *file_path, CmdVelCommand *cmd) {
  FILE *file = fopen(file_path, "r");
  if (!file) {
    cmd->valid = 0;
    return 0;
  }

  // Format: linear_x:angular_z:left_wheel:right_wheel:front_left:front_right:back_left:back_right
  int result = fscanf(file, "%lf:%lf:%lf:%lf:%lf:%lf:%lf:%lf",
                      &cmd->linear_x,
                      &cmd->angular_z,
                      &cmd->left_wheel,
                      &cmd->right_wheel,
                      &cmd->front_left,
                      &cmd->front_right,
                      &cmd->back_left,
                      &cmd->back_right);

  fclose(file);

  if (result == 8) {
    cmd->valid = 1;
    return 1;
  } else {
    cmd->valid = 0;
    return 0;
  }
}

// Get robot name/ID for file path
// Priority: 1) Environment variable, 2) Command-line arg, 3) wb_robot_get_name(), 4) Default
// Root cause fix: wb_robot_get_name() sometimes returns prototype name instead of "name" field
void get_robot_id(char *robot_id, size_t max_len, int argc, char **argv) {
  // Method 1: Try environment variable (most reliable for multi-robot setups)
  const char *env_robot_id = getenv("WEBOTS_ROBOT_ID");
  if (env_robot_id && strlen(env_robot_id) > 0) {
    snprintf(robot_id, max_len, "%s", env_robot_id);
    printf("INFO: Robot ID from environment variable: '%s'\n", robot_id);
    return;
  }
  
  // Method 2: Try command-line argument (if Webots supports controllerArgs)
  // Format: controller --robot_id robot_1
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--robot_id") == 0 && i + 1 < argc) {
      snprintf(robot_id, max_len, "%s", argv[i + 1]);
      printf("INFO: Robot ID from command-line argument: '%s'\n", robot_id);
      return;
    }
  }
  
  // Method 3: Try wb_robot_get_name() - may return prototype name instead of "name" field
  const char *name = wb_robot_get_name();
  printf("DEBUG: wb_robot_get_name() returned: '%s'\n", name ? name : "(null)");
  
  if (name && strlen(name) > 0) {
    // Check if name is already in "robot_X" format (the name field value)
    if (strncmp(name, "robot_", 6) == 0) {
      snprintf(robot_id, max_len, "%s", name);
      printf("INFO: Robot ID from wb_robot_get_name(): '%s'\n", robot_id);
      return;
    }
    
    // Try to extract from DEF name format (e.g., "ROBOT_1" -> "robot_1")
    // This handles cases where the DEF name is used instead
    if (strncmp(name, "ROBOT_", 6) == 0) {
      char lower_id[64];
      snprintf(lower_id, sizeof(lower_id), "robot_%s", name + 6);
      snprintf(robot_id, max_len, "%s", lower_id);
      printf("INFO: Converted DEF name '%s' to robot_id: '%s'\n", name, robot_id);
      return;
    }
    
    printf("WARNING: wb_robot_get_name() returned '%s' (not robot_X format), using default\n", name);
  }
  
  // Method 4: Default fallback (for robot_1)
  snprintf(robot_id, max_len, "robot_1");
  printf("WARNING: Using default robot_id: '%s'. Set WEBOTS_ROBOT_ID env var or use --robot_id arg for multi-robot setup.\n", robot_id);
}

// Helper function to find camera device by trying common names and testing
WbDeviceTag find_camera_device(const char *robot_id) {
  // Try comprehensive list of camera device names
  // Note: LiDAR is "SickLms291" in .wbt but accessed as "Sick LMS 291" (with spaces)
  // Camera device name from Webots: "Webcam for Robotino 3"
  const char *camera_names[] = {
    "Webcam for Robotino 3",  // Exact name from Webots device tree
    "webcam for robotino 3",  // Lowercase variant
    "WEBCAM FOR ROBOTINO 3",  // Uppercase variant
    "Robotino 3 Webcam",      // Alternative - matches LiDAR pattern (Sick LMS 291)
    "Robotino3 Webcam",       // Alternative spacing
    "Robotino 3Webcam",       // Another spacing variant
    "Robotino3Webcam",        // Proto name (exact)
    "Robotino3WebCam",        // Mixed case
    "robotino 3 webcam",      // All lowercase with spaces
    "robotino3 webcam",       // Lowercase with space
    "camera",                 // Standard name
    "webcam",                 // Alternative name
    "front_camera",           // Descriptive name
    "Camera",                 // Capitalized
    "CAMERA",                 // All caps
    "robotino3webcam",        // Lowercase
    "ROBOTINO3WEBCAM",        // All caps
    NULL
  };
  
  // First, try all name variations
  for (int i = 0; camera_names[i] != NULL; i++) {
    WbDeviceTag device = wb_robot_get_device(camera_names[i]);
    if (device != 0) {
      // Device found - try to enable and verify it's a camera
      printf("DEBUG: Testing device '%s' for %s...\n", camera_names[i], robot_id);
      // Enable to test if it's a camera device (will be enabled again in main, that's OK)
      wb_camera_enable(device, TIME_STEP);
      int test_width = wb_camera_get_width(device);
      int test_height = wb_camera_get_height(device);
      
      if (test_width > 0 && test_height > 0) {
        // Successfully got dimensions - it's a camera!
        // Keep it enabled (main() will enable again which is safe)
        printf("INFO: Found camera device '%s' for %s (verified: %dx%d)\n", 
               camera_names[i], robot_id, test_width, test_height);
        return device;
      } else {
        // Not a camera or failed to get dimensions, disable and continue
        printf("DEBUG: Device '%s' found but not a camera (width=%d, height=%d)\n", 
               camera_names[i], test_width, test_height);
        wb_camera_disable(device);
      }
    }
  }
  
  // If no device found by name, try enumerating devices
  // Webots doesn't have a direct enumeration API, so we'll just report failure
  printf("WARNING: Camera device not found for %s after trying all name variations\n", robot_id);
  printf("DEBUG: Tried %d different device names\n", (int)(sizeof(camera_names)/sizeof(camera_names[0]) - 1));
  
  return 0;
}

int main(int argc, char **argv) {
  // init webots stuff
  wb_robot_init();

  // Get robot ID for cmd_vel file path
  char robot_id[64];
  get_robot_id(robot_id, sizeof(robot_id), argc, argv);

  // Construct cmd_vel file path
  char cmd_vel_file_path[CMD_VEL_FILE_PATH_MAX];
  snprintf(cmd_vel_file_path, sizeof(cmd_vel_file_path),
           "/tmp/pioneer_cmd_vel_%s.txt", robot_id);

  // Construct lidar file path
  char lidar_file_path[LIDAR_FILE_PATH_MAX];
  snprintf(lidar_file_path, sizeof(lidar_file_path),
           "/tmp/pioneer_lidar_%s.txt", robot_id);

  // Construct camera file path
  char camera_file_path[CAMERA_FILE_PATH_MAX];
  snprintf(camera_file_path, sizeof(camera_file_path),
           "/tmp/pioneer_camera_%s.txt", robot_id);

  // get devices
  WbDeviceTag lms291 = wb_robot_get_device("Sick LMS 291");
  
  // Find camera device using helper function
  WbDeviceTag camera = find_camera_device(robot_id);
  
  WbDeviceTag front_left_wheel = wb_robot_get_device("front left wheel");
  WbDeviceTag front_right_wheel = wb_robot_get_device("front right wheel");
  WbDeviceTag back_left_wheel = wb_robot_get_device("back left wheel");
  WbDeviceTag back_right_wheel = wb_robot_get_device("back right wheel");

  // init lms291
  wb_lidar_enable(lms291, TIME_STEP);
  
  // init camera
  int cam_width = 0;
  int cam_height = 0;
  if (camera != 0) {
    // Camera was already enabled in find_camera_device, but enable again to be safe
    wb_camera_enable(camera, TIME_STEP);
    cam_width = wb_camera_get_width(camera);
    cam_height = wb_camera_get_height(camera);
    printf("INFO: Camera enabled for %s: %dx%d\n", robot_id, cam_width, cam_height);
  } else {
    printf("WARNING: Camera device not found for %s\n", robot_id);
    printf("DEBUG: Check Webots console for device name. Tried 14 different name variations.\n");
    printf("DEBUG: LiDAR device name is 'Sick LMS 291' (note the spaces)\n");
    printf("DEBUG: Camera proto is 'Robotino3Webcam' - try checking Webots device tree for actual name\n");
  }
  const int lms291_width = wb_lidar_get_horizontal_resolution(lms291);
  const int half_width = lms291_width / 2;
  const double max_range = wb_lidar_get_max_range(lms291);
  const double min_range = 0.1;  // Default min range (Sick LMS 291 typically 0.1m)
  
  // Calculate field of view from sensor properties
  // Sick LMS 291 typically has 180-degree FOV (pi radians)
  // We default to 180 degrees since wb_lidar_get_fov may not be available in all Webots versions
  double fov = M_PI;  // Default to 180 degrees (pi radians)
  
  const double angle_increment = fov / lms291_width;
  const double angle_min = -fov / 2.0;
  const double angle_max = fov / 2.0;

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

  // Initialize cmd_vel structure
  CmdVelCommand cmd_vel = {0};

  // control loop
  while (wb_robot_step(TIME_STEP) != -1) {
    // Read cmd_vel from file (non-blocking)
    read_cmd_vel_file(cmd_vel_file_path, &cmd_vel);

    // get lidar values
    const float *lms291_values = wb_lidar_get_range_image(lms291);
    if (!lms291_values) continue;

    // Export LiDAR data to file for ROS2 bridge
    // Get current time from Webots
    double timestamp = wb_robot_get_time();
    
    // Write to temp file first, then rename (atomic operation)
    char lidar_temp_path[LIDAR_FILE_PATH_MAX + 4];
    snprintf(lidar_temp_path, sizeof(lidar_temp_path), "%s.tmp", lidar_file_path);
    
    FILE *lidar_file = fopen(lidar_temp_path, "w");
    if (lidar_file) {
      // Format: LIDAR:robot_name:timestamp:num_points:angle_min:angle_max:range_min:range_max:range1:range2:...
      fprintf(lidar_file, "LIDAR:%s:%.6f:%d:%.6f:%.6f:%.6f:%.6f",
              robot_id, timestamp, lms291_width, angle_min, angle_max, min_range, max_range);
      
      // Write all range values
      for (int i = 0; i < lms291_width; ++i) {
        double range = lms291_values[i];
        // Replace invalid readings (infinity/NaN) with -1 to indicate invalid
        if (range >= max_range || range != range) {
          range = -1.0;
        }
        fprintf(lidar_file, ":%.6f", range);
      }
      fprintf(lidar_file, "\n");
      fflush(lidar_file);
      fclose(lidar_file);
      
      // Atomic rename
      rename(lidar_temp_path, lidar_file_path);
    }

    // Export Camera data to file for ROS2 bridge
    if (camera != 0) {
      const unsigned char *image_data = wb_camera_get_image(camera);
      if (image_data && cam_width > 0 && cam_height > 0) {
        // Write to temp file first, then rename (atomic operation)
        char camera_temp_path[CAMERA_FILE_PATH_MAX + 4];
        snprintf(camera_temp_path, sizeof(camera_temp_path), "%s.tmp", camera_file_path);
        
        FILE *camera_file = fopen(camera_temp_path, "wb");
        if (camera_file) {
          // Webots camera returns BGRA format (4 bytes per pixel)
          // Format: CAMERA:robot_name:timestamp:width:height:encoding:
          fprintf(camera_file, "CAMERA:%s:%.6f:%d:%d:bgra8:\n", robot_id, timestamp, cam_width, cam_height);
          
          // Write binary image data (BGRA = 4 bytes per pixel)
          fwrite(image_data, 1, cam_width * cam_height * 4, camera_file);
          fflush(camera_file);
          fclose(camera_file);
          
          // Atomic rename
          rename(camera_temp_path, camera_file_path);
        }
      }
    }

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
          // Turn right (slow right wheels, keep left wheels forward)
          // This makes the robot turn right while moving forward
          front_left_speed = CRUISING_SPEED;
          front_right_speed = 0.3 * CRUISING_SPEED;  // Slow right side
          back_left_speed = CRUISING_SPEED;
          back_right_speed = 0.3 * CRUISING_SPEED;
          state = LEFT;
        } else if (wheel_weight_right > WHEEL_WEIGHT_THRESHOLD) {
          // Turn left (slow left wheels, keep right wheels forward)
          // This makes the robot turn left while moving forward
          front_left_speed = 0.3 * CRUISING_SPEED;  // Slow left side
          front_right_speed = CRUISING_SPEED;
          back_left_speed = 0.3 * CRUISING_SPEED;
          back_right_speed = CRUISING_SPEED;
          state = RIGHT;
        } else {
          // No obstacle, go straight
          front_left_speed = CRUISING_SPEED;
          front_right_speed = CRUISING_SPEED;
          back_left_speed = CRUISING_SPEED;
          back_right_speed = CRUISING_SPEED;
        }
        break;

      // Continue turning right until obstacle clears
      case LEFT:
        if (wheel_weight_left > WHEEL_WEIGHT_THRESHOLD || wheel_weight_right > WHEEL_WEIGHT_THRESHOLD) {
          // Keep turning right (slow right wheels)
          front_left_speed = CRUISING_SPEED;
          front_right_speed = 0.3 * CRUISING_SPEED;
          back_left_speed = CRUISING_SPEED;
          back_right_speed = 0.3 * CRUISING_SPEED;
        } else {
          // Obstacle cleared, resume forward
          front_left_speed = CRUISING_SPEED;
          front_right_speed = CRUISING_SPEED;
          back_left_speed = CRUISING_SPEED;
          back_right_speed = CRUISING_SPEED;
          state = FORWARD;
        }
        break;

      // Continue turning left until obstacle clears
      case RIGHT:
        if (wheel_weight_left > WHEEL_WEIGHT_THRESHOLD || wheel_weight_right > WHEEL_WEIGHT_THRESHOLD) {
          // Keep turning left (slow left wheels)
          front_left_speed = 0.3 * CRUISING_SPEED;
          front_right_speed = CRUISING_SPEED;
          back_left_speed = 0.3 * CRUISING_SPEED;
          back_right_speed = CRUISING_SPEED;
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

    // Priority system: Obstacle Avoidance > cmd_vel > Default
    // Variables to store final wheel speeds
    double final_front_left = front_left_speed;
    double final_front_right = front_right_speed;
    double final_back_left = back_left_speed;
    double final_back_right = back_right_speed;

    // Check if obstacle detected (highest priority)
    int obstacle_detected = (wheel_weight_left > WHEEL_WEIGHT_THRESHOLD ||
                             wheel_weight_right > WHEEL_WEIGHT_THRESHOLD);

    if (obstacle_detected) {
      // Priority 0: OBSTACLE_AVOIDANCE
      // Use obstacle avoidance speeds (already computed above)
      // Do nothing - speeds are already set from state machine
    } else if (cmd_vel.valid) {
      // Priority 1: CMD_VEL
      // Use cmd_vel commands from ROS2
      final_front_left = cmd_vel.front_left;
      final_front_right = cmd_vel.front_right;
      final_back_left = cmd_vel.back_left;
      final_back_right = cmd_vel.back_right;

      // Clamp to MAX_SPEED limits
      if (final_front_left > MAX_SPEED) final_front_left = MAX_SPEED;
      if (final_front_left < -MAX_SPEED) final_front_left = -MAX_SPEED;
      if (final_front_right > MAX_SPEED) final_front_right = MAX_SPEED;
      if (final_front_right < -MAX_SPEED) final_front_right = -MAX_SPEED;
      if (final_back_left > MAX_SPEED) final_back_left = MAX_SPEED;
      if (final_back_left < -MAX_SPEED) final_back_left = -MAX_SPEED;
      if (final_back_right > MAX_SPEED) final_back_right = MAX_SPEED;
      if (final_back_right < -MAX_SPEED) final_back_right = -MAX_SPEED;
    } else {
      // Priority 2: DEFAULT
      // Use default forward cruising (already set from state machine)
      // Do nothing - speeds are already set
    }

    // Set motor speeds
    wb_motor_set_velocity(front_left_wheel, final_front_left);
    wb_motor_set_velocity(front_right_wheel, final_front_right);
    wb_motor_set_velocity(back_left_wheel, final_back_left);
    wb_motor_set_velocity(back_right_wheel, final_back_right);
  }

  wb_robot_cleanup();
  return 0;
}
