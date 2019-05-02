rosservice call /controller_manager/load_controller "name: 'head_controller'"
rosservice call /controller_manager/switch_controller "{start_controllers: ['head_controller']}"
    rostopic pub /head_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names:
 ['head_1_joint', 'head_2_joint']
points:
- positions: [0,-0.35]
  velocities: []
  accelerations: []
  effort: []
  time_from_start: {secs: 1, nsecs: 0}" --once