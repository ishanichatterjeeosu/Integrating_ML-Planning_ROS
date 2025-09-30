search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=bravo_7_example.srdf
robot_name_in_srdf=bravo_7_example
moveit_config_pkg=bravo_7_example_moveit_config
robot_name=bravo_7_example
planning_group_name=arm
ikfast_plugin_pkg=bravo_7_example_arm_ikfast_plugin
base_link_name=bravo_base_link
eef_link_name=ee_link
ikfast_output_path=/home/tim/Research/bravo_ws/src/bpl_bravo_moveit_config/bravo_7_example_arm_ikfast_plugin/src/bravo_7_example_arm_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
