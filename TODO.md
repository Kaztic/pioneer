# TODO - 

- 1. Foundation: Create custom ROS2 messages package (SearchAndRescue/DetectionReport, TerritoryAssignment, MissionStatus, ConsensusMessage), implement A* path planner node (global planner on occupancy grid, subscribe to odom/goals, publish to global_path), modify controller.c to subscribe to /robot_i/cmd_vel with priority system (obstacle avoidance > cmd_vel > default)

- 2. Exploration: Occupancy grid builder node (process LiDAR scans to grid 0.1-0.5m resolution, maintain local map, publish to occupancy_map), frontier detection algorithm (detect known/unknown boundaries, calculate utility, select best frontier), goal selector node (integrate frontier detection, coordinate with robots, publish goals to path planner)

- 3. TCE Integration: TCE consensus node per robot (message validation, vote submission, 4/5 vote aggregation, consensus broadcasting), TCE message wrapper (wrap critical messages, add vote tracking, publish to /consensus/* topics), integrate TCE with ROS2 topics (object_detection validation, territory assignments, mission status consensus)

- 4. Object Detection: Camera integration in Webots (mount/configure camera, publish to camera/image_raw), object detection node (image processing pipeline, YOLO/OpenCV detection, confidence scoring, pose estimation, publish to object_detection), multi-robot verification system (navigate to verify detections, vote aggregation, 4/5 consensus for object_found)

- 5. Coordination: Mission coordinator node (initial territory assignment 5 regions, dynamic re-allocation, state machine management), territory assignment algorithm (divide map, assign via TCE consensus, prevent overlap ensure coverage), collision avoidance system (publish planned paths, global collision checker, detect conflicts, priority-based resolution via TCE)

- 6. Integration & Testing: End-to-end integration testing (full mission simulation, multi-robot coordination, TCE consensus verification, object detection flow), parameter tuning (exploration, path planning, TCE timing, detection thresholds)

- 7. World building enhancements (add random object placement)

- 8. Launch file for all nodes

- 9. Documentation and usage guides

- 10. Visualization tools (RViz config for multi-robot view)

