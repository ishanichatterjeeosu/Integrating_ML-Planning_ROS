#! /usr/bin/python3

import moveit_commander
import sys
import time
import trimesh

from geometry_msgs.msg import Pose, PoseStamped
import open3d as o3d
import ros_numpy
import numpy as np


import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import PointCloud2

from utils import TFHelper

from visualization_msgs.msg import Marker, MarkerArray

from scipy.spatial.transform import Rotation as R

marker_pub = rospy.Publisher('grasp_marker_individual', Marker, queue_size=1)
marker_pub_test = rospy.Publisher('pregrasp_marker_test', Marker, queue_size=1)

marker_array_pub = rospy.Publisher('bbox_points', MarkerArray, queue_size=100)

class Mover:
    """Wrapper around MoveIt functionality for the arm."""

    arm_move_group_cmdr: moveit_commander.MoveGroupCommander
    arm_robot_cmdr: moveit_commander.RobotCommander
    arm_group_name: str
    scene: moveit_commander.PlanningSceneInterface
    gripper_pub: rospy.Publisher
    box_name: str = None
    grasping_group_name: str
    tfh: TFHelper

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.arm_group_name = "arm"
        self.grasping_group_name = "hand"
        self.arm_robot_cmdr = moveit_commander.RobotCommander(robot_description="robot_description")
        self.arm_move_group_cmdr = moveit_commander.MoveGroupCommander(self.arm_group_name, robot_description="robot_description")

        self.scene = moveit_commander.PlanningSceneInterface(synchronous=True) # might break the non-blocking promise
        self.gripper_pub = rospy.Publisher('hand_position_controller/command', data_class=JointTrajectory, queue_size=1)
        self.add_ground_plane_to_planning_scene()

        #self.arm_move_group_cmdr.set_planner_id("RRTConnect") #for ompl
        self.arm_move_group_cmdr.set_planner_id("arm[arastar_jointdist_manip]") #for sbpl

        self.tfh = TFHelper()

        self.grasp_chooser = GraspChooser(self.tfh)

    def add_ground_plane_to_planning_scene(self):
        """Add a box object to the PlanningScene to prevent paths that collide
        with the ground. """

        box_pose = PoseStamped()
        box_pose.header.frame_id = self.arm_robot_cmdr.get_planning_frame()
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = -0.501
        box_name = "ground_plane"
        self.scene.add_box(box_name, box_pose, size=(3, 3, 1))

    def get_ee_pose(self):
        return self.arm_move_group_cmdr.get_current_pose()

    def go_joints(self, joints: np.ndarray, wait: bool = True):
        """Move robot to the given joint configuration.

        Args:
            joints (np.ndarray): joint angles
            wait (bool): whether to block until finished 
        """
        success = self.arm_move_group_cmdr.go(joints, wait=wait)
        self.arm_move_group_cmdr.stop()
        return success

    def go_gripper(self, pos: np.ndarray, wait: bool = True) -> bool:
        """Move the gripper fingers to the given actuator value.

        Args:
            pos (np.ndarray): desired position of `bravo_axis_a`
            wait (bool): whether to wait until it's probably done
        """
        jt = JointTrajectory()
        jt.joint_names = ['bravo_axis_a']
        jt.header.stamp = rospy.Time.now()

        jtp = JointTrajectoryPoint()
        jtp.positions = pos
        jtp.time_from_start = rospy.Duration(secs=5)
        jt.points.append(jtp)

        self.gripper_pub.publish(jt)

        if wait:
            time.sleep(6)

        return True

    def go_ee_pose(self, pose: PoseStamped, wait: bool = True) -> bool:
        """Move the end effector to the given pose.

        Args:
            pose (geometry_msgs.msg.Pose): desired pose for end effector
            wait (bool): whether to block until finished
        """

        pose.header.stamp = rospy.Time.now() # the timestamp may be out of date.
        self.arm_move_group_cmdr.set_pose_target(pose, end_effector_link="ee_link")

        motion_goal_pub = rospy.Publisher('motion_goal', PoseStamped, queue_size=10)
        motion_goal_pub.publish(pose)

        success = self.arm_move_group_cmdr.go(wait=wait)
        self.arm_move_group_cmdr.stop()
        self.arm_move_group_cmdr.clear_pose_targets()

        return success
    
    def go_named_group_state(self, state: str, wait: bool = True) -> bool:
        """Move the arm group to a named state from the SRDF.

        Args:
            state (str): the name of the state
            wait (bool, optional): whether to block until finished. Defaults to True.
        """
        self.arm_move_group_cmdr.set_named_target(state)
        #success = self.arm_move_group_cmdr.go(wait=wait)
        print("\tOpening jaws.")
        success = self.go_gripper(np.array([0.03]), wait=True) # Check if need to increase
        if not success: return False

        print("\tMoving to Joint state.")
        success, plan, planning_time, code = self.arm_move_group_cmdr.plan()
        # try executing a partial plan
        
        #print("path: ", plan.joint_trajectory.points[0].positions, plan.joint_trajectory.points[1].positions)
        # Can execute partial plans. can publish path length and truncate using that length
        # if cond then plan = existing success path, else plan = failrue path
        #plan.joint_trajectory.points = plan.joint_trajectory.points[0:120]
        self.arm_move_group_cmdr.execute(plan)
        self.arm_move_group_cmdr.stop()

        print("\tClosing jaws.")
        success = self.go_gripper(np.array([0.000]), wait=True)
        if not success: return False
        rospy.sleep(2)
        
        return success

    def add_object_for_pickup(self):
        """Add a box object to the PlanningScene so that collisions with the 
        hand are ignored. Otherwise, no collision-free trajectories can be found 
        after an object is picked up."""

        eef_link = self.arm_move_group_cmdr.get_end_effector_link()

        box_pose = PoseStamped()
        box_pose.header.frame_id = "ee_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.11  # above the hand frame
        self.box_name = "hand_collision_box"
        self.scene.add_box(self.box_name, box_pose, size=(0.075, 0.075, 0.075))

        touch_links = self.arm_robot_cmdr.get_link_names(group=self.grasping_group_name)
        self.scene.attach_box(eef_link, self.box_name, touch_links=touch_links)


    def remove_object_after_pickup(self):
        """Remove a PlanningScene box after adding it in add_object_for_pickup."""

        eef_link = self.arm_move_group_cmdr.get_end_effector_link()

        if self.box_name is not None:
            self.scene.remove_attached_object(eef_link, name=self.box_name)
            self.scene.remove_world_object(self.box_name)
            self.box_name = None
        else:
            raise ValueError("No box was added to the planning scene. Did you call add_object_for_pickup?")
    
    def execute_grasp_open_loop(self, orbital_pose: PoseStamped, final_pose: PoseStamped) -> bool:
        """Execute an open loop grasp by planning and executing a sequence of motions in turn:
           1) Open the jaws
           2) Move the end effector the the orbital pose
           3) Move the end effector to the final pose
           4) Close the jaws
           5) Move the end effector to the orbital pose
           6) Move the arm to the 'rest' configuration

        Args:
            orbital_pose (PoseStamped): the pre-grasp orbital pose of the end effector
            final_pose (PoseStamped): the end effector pose in which to close the jaws

        Returns:
            bool: whether every step succeeded according to MoveIt
        """

        print("Executing grasp.")

        print("\tOpening jaws.")
        success = self.go_gripper(np.array([0.03]), wait=True)
        if not success: return False

        print("\tMoving end effector to orbital pose.")
        success = self.go_ee_pose(orbital_pose, wait=True)
        if not success: return False

        print("\tMoving end effector to final pose.")
        success = self.go_ee_pose(final_pose, wait=True)
        if not success: return False

        print("\tClosing jaws.")
        success = self.go_gripper(np.array([0.008]), wait=True)
        if not success: return False
        rospy.sleep(2)

        print("\tMoving end effector to orbital pose.")
        success = self.go_ee_pose(orbital_pose, wait=True)
        if not success: return False

        print("\tMoving arm to 'rest' configuration.")
        success = self.go_named_group_state('rest', wait=True)

        print("\tOpening jaws.")
        success = self.go_gripper(np.array([0.03]), wait=True)
        if not success: return False

        return success

from cl_tsgrasp.msg import Grasps


class GraspChooser:

    final_goal_pose = None
    grasp_lpf = None

    def __init__(self, tfh: TFHelper):

        self.tfh = tfh

        grasp_sub = rospy.Subscriber(name='/tsgrasp/grasps', 
            data_class=Grasps, callback=self.grasp_cb, queue_size=1)

        self.best_grasp_pub = rospy.Publisher(name='/tsgrasp/final_goal_pose', 
            data_class=PoseStamped, queue_size=1)

        self.orbital_best_grasp_pub = rospy.Publisher(name='/tsgrasp/orbital_final_goal_pose', 
            data_class=PoseStamped, queue_size=1)
        pcl_sub = rospy.Subscriber(name='/tsgrasp/confs3d', 
            data_class=PointCloud2, callback=self.pcl_cb, queue_size=1)
            
        self.obb_pose_pub = rospy.Publisher(name='/tsgrasp/object_pose', 
            data_class=PoseStamped, queue_size=1)
        self.closest_grasp_lpf = None
        self.best_closest_grasp = None
        self.best_grasp = None

        self.grasp_closeness_importance = 0.5
    def pcl_cb(self, msg):
        """take the point cloud and find the oriented bb for the ."""
        #
        cld_3d = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg, remove_nans=False)
        # print(np.array(cld_3d).shape)
        pcl_o3d = o3d.utility.Vector3dVector(np.array(cld_3d))
        o3d_bbox = o3d.geometry.OrientedBoundingBox.create_from_points(pcl_o3d)
        # print(o3d_bbox.R.shape)
        bbox_pts = np.asarray(o3d.geometry.OrientedBoundingBox.get_box_points(o3d_bbox))

        #print(bbox_pts)
        

        marker_array = MarkerArray()
        for i in range(8):
            
            marker = Marker()
            marker.type = marker.CUBE
            marker.id = i
            marker.header = msg.header

            
            marker.pose.position.x = bbox_pts[i][0]
            marker.pose.position.y = bbox_pts[i][1]
            marker.pose.position.z = bbox_pts[i][2]
            
            marker.action = marker.MODIFY
            marker.color.a = 1.0
            marker.color.r = 1
            marker.color.g = 1
            marker.color.b = 0
            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            

            marker_array.markers.append(marker)

        
        marker_array_pub.publish(marker_array)

        obb_pose = PoseStamped()
        obb_pose_tmp = Pose()
        obb_pose_tmp.position.x = o3d_bbox.center[0]
        obb_pose_tmp.position.y = o3d_bbox.center[1]
        obb_pose_tmp.position.z = o3d_bbox.center[2]
        r = R.from_matrix(np.array(o3d_bbox.R))
        q = r.as_quat()
        obb_pose_tmp.orientation.x = q[0]
        obb_pose_tmp.orientation.y = q[1]
        obb_pose_tmp.orientation.z = q[2]
        obb_pose_tmp.orientation.w = q[3]
        obb_pose.pose = obb_pose_tmp
        obb_pose.header = msg.header
        self.obb_pose_pub.publish(obb_pose)
        
    
    def grasp_cb(self, msg):
        """Identify the (best) and (best, closest) grasps in the Grasps message and publish them."""
        # global ctr
        # if ctr >= 0:
        confs = msg.confs
        poses = msg.poses
        orbital_poses = msg.orbital_poses

        

        #find grasp in each region

       
        for i in range(len(poses)):
 
            print("ee positions ", poses[i].position.x, poses[i].position.y, poses[i].position.z)
            #print(" ee orientation ", poses[i].orientation.x, poses[i].orientation.y, poses[i].orientation.z, poses[i].orientation.w)
            #print("orbital positions ", orbital_poses[i].position.x, orbital_poses[i].position.y, orbital_poses[i].position.z)
            #print(" orbital orientation ", orbital_poses[i].orientation.x, orbital_poses[i].orientation.y, orbital_poses[i].orientation.z, orbital_poses[i].orientation.w)
            #viualize each EE pose to select
            marker = Marker()
            marker.type = marker.MESH_RESOURCE
            marker.action = marker.MODIFY
            marker.mesh_resource = "package://cl_tsgrasp/urdf/gripper_yawed.stl"
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.header = msg.header
            marker.pose.orientation = poses[i].orientation
            marker.pose.position = poses[i].position
            print("Publishing grasp poses one by one")
            print("Pose", i)
            marker_pub.publish(marker)
            sys.stdin.read(1)
            #     marker1 = Marker()
            #     marker1.type = marker.CUBE
            #     marker1.action = marker.MODIFY
            #     marker1.scale.x = 0.01
            #     marker1.scale.y = 0.01
            #     marker1.scale.z = 0.01
            #     marker1.color.a = 1.0
            #     marker1.color.r = 1.0
            #     marker1.color.g = 1.0
            #     marker1.color.b = 0.0
            #     marker1.header = msg.header
             
            #     marker1.pose.position = poses[i].position
             
            #     marker_pub_test.publish(marker1)
                #sys.stdin.read(1)
        #      #S#### Save the good grasp here itself, during execution execute this grasp.
        #      # But not be a good idea if too much deviation from predicted. TODO: think how to find nearest pose during execution
        #     sys.stdin.read(1) 
                
        # input()


        if len(confs) == 0: return

        # Find best (highest confidence) grasp and its orbital grasp
        best_grasp = PoseStamped()
        best_grasp.pose = poses[np.argmax(confs)]
        # Best grasp pose is the one selected during planning, TODO verify if correct
            
            # best_grasp.pose.position.x = 0.1012
            # best_grasp.pose.position.y = 0.0771
            # best_grasp.pose.position.z = 0.3326
            # best_grasp.pose.orientation.x = -0.4150
            # best_grasp.pose.orientation.y = 0.0768
            # best_grasp.pose.orientation.z = 0.7968
            # best_grasp.pose.orientation.w = -0.4221

        best_grasp.header = msg.header
        self.best_grasp_pub.publish(best_grasp)
        self.best_grasp = best_grasp

        ####################### If have the object pose, then transform grasps to object frame according to the following
        # obj_pose = np.array([
        #            [1, 0, 0, 0],
        #              [0, 1, 0, 0],
        #              [0, 0, 1, 0.527],
        #              [0, 0, 0, 1]
        #          ])
        # bg_quat = R.from_quat([best_grasp.pose.orientation.x, best_grasp.pose.orientation.y, best_grasp.pose.orientation.z, best_grasp.pose.orientation.w])
        # #best_grasp_array = np.array(best_grasp.pose)
        # #print(bg_quat.as_matrix())
        # trans = np.array([best_grasp.pose.position.x, best_grasp.pose.position.y, best_grasp.pose.position.z])
        # trans = trans.reshape(3, 1)
        # bg_pose = np.block([
        # [bg_quat.as_matrix(), trans],
        # [0, 0, 0, 1]
        # ])
        # print(bg_pose)
        # #poses_array = np.stack(best_grasp.pose)
        # tf_from_cam_to_scene = bg_pose.dot(
        #              trimesh.transformations.euler_matrix(np.pi, 0, 0)
        #          ) # camera_pose has a flipped z axis
        # rot = obj_pose[0:3, 0:3]
        # trans = rot.T @ obj_pose[0:3, 3].reshape(3, 1)
        # inv_homo = np.block([
        # [rot.T, -trans],
        # [0, 0, 0, 1]
        # ])
        # tf_from_scene_to_obj = inv_homo
        # tf_from_cam_to_obj = tf_from_scene_to_obj @ tf_from_cam_to_scene
        # print("poses array ", tf_from_cam_to_obj)
        ##############################################

        orbital_best_grasp = PoseStamped()
        orbital_best_grasp.pose = orbital_poses[np.argmax(confs)]
            # orbital_best_grasp.pose.position.x = 0.1958
            # orbital_best_grasp.pose.position.y = 0.1079
            # orbital_best_grasp.pose.position.z = 0.2490
            # orbital_best_grasp.pose.orientation.x = -0.4150
            # orbital_best_grasp.pose.orientation.y = 0.0768
            # orbital_best_grasp.pose.orientation.z = 0.7968
            # orbital_best_grasp.pose.orientation.w = -0.4221
        orbital_best_grasp.header = msg.header
        self.orbital_best_grasp_pub.publish(orbital_best_grasp)
        self.orbital_best_grasp = orbital_best_grasp
        #ctr += 1

    def reset_closest_target(self, posestamped: PoseStamped):
        if posestamped.header.frame_id != self.best_closest_grasp.header.frame_id: raise ValueError
        self.closest_grasp_lpf.best_grasp = posestamped.pose

    def get_best_grasp(self):
        return self.best_grasp
    
    def get_best_closest_grasp(self):
        return self.best_closest_grasp