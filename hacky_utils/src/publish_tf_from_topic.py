#!/usr/bin/python3
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf_conversions
import PySimpleGUI as sg
#set the theme for the screen/window
sg.theme("LightBlue")
#define layout
vars = ["x", "y", "z", "roll", "pitch", "yaw"]

defaults = {
    "x": 0.143,
    "y": 0.095,
    "z": 0.015,
    "roll": -2.322,
    "pitch": 0,
    "yaw": -1.599
}

layout = []
for var in vars:
    range = (-0.4, 0.4) if var in ["x", "y", "z"] else (-3.141, 3.141)
    res = 0.001 if var in ["x", "y", "z"] else 0.001
    default = defaults[var]

    layout.append(
        [
            sg.Text(var), 
            sg.Slider(orientation="horizontal", 
                resolution=res, 
                range=range,
                default_value=default,
                key=var)
        ] 
    )

#Define Window
window =sg.Window("Sliders for Transform",layout)

def print_tf(msg: TransformStamped) -> str:
    x, y, z = msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z
    qx, qy, qz, qw = msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w
    parent = msg.header.frame_id
    child = msg.child_frame_id
    return f"""<node pkg="tf2_ros" type="static_transform_publisher" name="trisect_camera_link_broadcaster"
      args="{x} {y} {z}  {qx} {qy} {qz} {qw} {parent} {child}" />
    """

"""
The GraspCmdRepublisher class gets a grasp command from /realarm_pub
and republishes them to the expected grasp_cmd topic for the bravo arm
"""
class GraspCmdRepublisher(object):

    def __init__(self, node_name):

        rospy.loginfo("Started Grasp tf republishing node")

        input_topic = '/cam_base_tf'

        # Create the node
        rospy.init_node(node_name)

        self.br = tf2_ros.TransformBroadcaster()


    def publish_tf(self, msg: TransformStamped):
        """
        Forwards the grasp pose message to the bravo grasp command topic
        """
        rospy.loginfo(f"Sending tf:\n{print_tf(msg)}")
        self.br.sendTransform(msg)

if __name__ == '__main__':

    republisher = GraspCmdRepublisher("grasp_cmd_forwarder")
    r = rospy.Rate(10)

    while True:
        #Read  values entered by user
        event,values=window.read(timeout=0)

        x = values['x']
        y = values['y']
        z = values['z']
        roll = values['roll']
        pitch = values['pitch']
        yaw = values['yaw']
        
        q = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)

        tf = TransformStamped()
        tf.child_frame_id = "left"
        tf.header.frame_id = "trisect_cam_link"
        # tf.child_frame_id = "camera_depth_optical_frame"
        # tf.header.frame_id = "left"
        tf.header.stamp = rospy.Time.now()
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]
        
        republisher.publish_tf(tf)
        r.sleep()