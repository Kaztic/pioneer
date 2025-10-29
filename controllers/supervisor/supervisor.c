/*
 * Supervisor Controller for ROS 2 Integration
 * 
 * This supervisor controller:
 * 1. Reads positions of all 5 Pioneer robots
 * 2. Publishes robot positions via TCP socket (for ROS 2 node to read)
 * 3. Acts as a bridge between Webots and ROS 2
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

#define TIME_STEP 32
#define PORT 12345
#define NUM_ROBOTS 5
#define POSE_FILE "/tmp/pioneer_poses.txt"

// Robot DEF names as defined in the world file
const char* ROBOT_DEF_NAMES[NUM_ROBOTS] = {
  "ROBOT_1",
  "ROBOT_2", 
  "ROBOT_3",
  "ROBOT_4",
  "ROBOT_5"
};

// Structure to hold robot position and orientation
typedef struct {
  double x, y, z;
  double qx, qy, qz, qw;  // Quaternion
  double roll, pitch, yaw; // Euler angles (computed from quaternion)
  char name[32];
  int valid;  // 1 if robot exists and position is valid
} RobotPose;

// Convert quaternion to Euler angles (roll, pitch, yaw)
void quaternion_to_euler(double qx, double qy, double qz, double qw,
                         double *roll, double *pitch, double *yaw) {
  // Roll (x-axis rotation)
  double sinr_cosp = 2 * (qw * qx + qy * qz);
  double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
  *roll = atan2(sinr_cosp, cosr_cosp);

  // Pitch (y-axis rotation)
  double sinp = 2 * (qw * qy - qz * qx);
  if (fabs(sinp) >= 1)
    *pitch = copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
  else
    *pitch = asin(sinp);

  // Yaw (z-axis rotation)
  double siny_cosp = 2 * (qw * qz + qx * qy);
  double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
  *yaw = atan2(siny_cosp, cosy_cosp);
}

// Get robot position from Webots
int get_robot_pose(const char* robot_name, RobotPose* pose) {
  // Get robot node from DEF name
  WbNodeRef robot_node = wb_supervisor_node_get_from_def(robot_name);
  
  if (!robot_node) {
    pose->valid = 0;
    return 0;
  }

  // Get position field
  WbFieldRef translation_field = wb_supervisor_node_get_field(robot_node, "translation");
  if (!translation_field) {
    pose->valid = 0;
    return 0;
  }

  // Get rotation field (quaternion)
  WbFieldRef rotation_field = wb_supervisor_node_get_field(robot_node, "rotation");
  if (!rotation_field) {
    pose->valid = 0;
    return 0;
  }

  // Read position
  const double *translation = wb_supervisor_field_get_sf_vec3f(translation_field);
  pose->x = translation[0];
  pose->y = translation[1];
  pose->z = translation[2];

  // Read rotation (axis-angle format: [x, y, z, angle])
  const double *rotation = wb_supervisor_field_get_sf_rotation(rotation_field);
  double angle = rotation[3];
  double axis_x = rotation[0];
  double axis_y = rotation[1];
  double axis_z = rotation[2];

  // Convert axis-angle to quaternion
  double half_angle = angle * 0.5;
  double sin_half = sin(half_angle);
  pose->qw = cos(half_angle);
  pose->qx = axis_x * sin_half;
  pose->qy = axis_y * sin_half;
  pose->qz = axis_z * sin_half;

  // Normalize quaternion
  double norm = sqrt(pose->qx * pose->qx + pose->qy * pose->qy + 
                     pose->qz * pose->qz + pose->qw * pose->qw);
  if (norm > 0.0) {
    pose->qx /= norm;
    pose->qy /= norm;
    pose->qz /= norm;
    pose->qw /= norm;
  }

  // Convert to Euler angles
  quaternion_to_euler(pose->qx, pose->qy, pose->qz, pose->qw,
                      &pose->roll, &pose->pitch, &pose->yaw);

  // Get robot name from name field for logging
  WbFieldRef name_field = wb_supervisor_node_get_field(robot_node, "name");
  if (name_field) {
    const char* name_str = wb_supervisor_field_get_sf_string(name_field);
    if (name_str) {
      strncpy(pose->name, name_str, sizeof(pose->name) - 1);
    } else {
      strncpy(pose->name, robot_name, sizeof(pose->name) - 1);
    }
  } else {
    strncpy(pose->name, robot_name, sizeof(pose->name) - 1);
  }
  pose->valid = 1;

  return 1;
}

int main() {
  wb_robot_init();

  printf("Supervisor Controller: Starting ROS 2 Bridge\n");
  printf("Reading positions for %d robots...\n", NUM_ROBOTS);

  // Initialize robot poses array
  RobotPose robot_poses[NUM_ROBOTS];

  // Open file for writing robot poses
  FILE *pose_file = fopen(POSE_FILE, "w");
  if (!pose_file) {
    printf("Warning: Could not open pose file %s, using stdout instead\n", POSE_FILE);
    pose_file = stdout;
  }

  while (wb_robot_step(TIME_STEP) != -1) {
    // Read all robot positions
    for (int i = 0; i < NUM_ROBOTS; i++) {
      get_robot_pose(ROBOT_DEF_NAMES[i], &robot_poses[i]);
    }

    // Write to file (will be read by ROS 2 node)
    // Format: POSE:robot_name:x:y:z:qx:qy:qz:qw:roll:pitch:yaw
    // Write to temp file first, then rename to avoid race condition
    if (pose_file != stdout) {
      fclose(pose_file);
      // Write to temp file first
      pose_file = fopen(POSE_FILE ".tmp", "w");
      if (!pose_file) {
        printf("Warning: Could not open temp pose file, using stdout\n");
        pose_file = stdout;
      }
    }

    for (int i = 0; i < NUM_ROBOTS; i++) {
      if (robot_poses[i].valid) {
        fprintf(pose_file, "POSE:%s:%.6f:%.6f:%.6f:%.6f:%.6f:%.6f:%.6f:%.6f:%.6f:%.6f\n",
                robot_poses[i].name,
                robot_poses[i].x,
                robot_poses[i].y,
                robot_poses[i].z,
                robot_poses[i].qx,
                robot_poses[i].qy,
                robot_poses[i].qz,
                robot_poses[i].qw,
                robot_poses[i].roll,
                robot_poses[i].pitch,
                robot_poses[i].yaw);
      }
    }
    fflush(pose_file);  // Ensure data is flushed immediately
    
    // Atomic rename to avoid race condition
    if (pose_file != stdout) {
      fclose(pose_file);
      rename(POSE_FILE ".tmp", POSE_FILE);
      pose_file = fopen(POSE_FILE ".tmp", "w");
      if (!pose_file) {
        pose_file = stdout;
      }
    }
  }

  if (pose_file != stdout) {
    fclose(pose_file);
  }

  wb_robot_cleanup();
  return 0;
}

