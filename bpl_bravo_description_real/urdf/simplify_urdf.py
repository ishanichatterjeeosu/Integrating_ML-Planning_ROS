# simplify_urdf.py
#
# Tim Player, playert@oregonstate.edu, May 6, 2022
#
# Script to parse a URDF and remove redundant links along a kinematic chain.
# Redundant links are links with fixed transforms to their parent. This script
# moves the redundant link's XML elements to the parent, transforming the
# <origin> tag of <collision> and <visual> elements appropriately. This script
# does NOT transform inertial tags appropriately.
#
# This solves a problem where the provided Bravo URDF defined a distinct link
# for every visual element, leading to dozens of distinct links for a simple 6R
# arm. The extra links were not being properly resolved within Gazebo's dynamics
# or IKFast.

from urdfpy import URDF
import argparse

robot = URDF.load('/home/tim/Research/bravo_ws/src/bpl_bravo_description_real/urdf/bravo_7_cam.urdf')

def link_is_fixed(robot, child):
    link1, link2, data = list(robot._G.out_edges(child, data=True))[0] # should be only one
    return data['joint'].joint_type == 'fixed'

def transform(elem, tf):
    """Transform the 6-DOF pose of the <origin> element of this URDFType."""
    if not hasattr(elem, 'origin'): raise AttributeError
    elem.origin = tf @ elem.origin
    return elem

def give_childs_stuff_to_parent(robot, child):
    """Shift the child URDF element's collision and visual attributes to its parent."""
    _child, parent, data = list(robot._G.out_edges(child, data=True))[0] # should be only one
    joint = data['joint']

    t_child_to_parent = joint.origin

    parent.visuals.extend([transform(v, t_child_to_parent) for v in child.visuals])
    parent.collisions.extend([transform(c, t_child_to_parent) for c in child.collisions])
    # TODO add inertias using proper physics

def remove_child(robot, child):
    """Remove the child URDF element from the URDF tree, rerouting connections
    to grandchildren."""
    print("----------------------------------")
    print(f"Removing child: {child.name}")

    _child, parent, data = list(robot._G.out_edges(child, data=True))[0] # should be only one
    joint = data['joint']
    t_child_to_parent = joint.origin

    name = robot.name
    links = robot.links
    joints = robot.joints
    materials = robot.materials
    transmissions = robot.transmissions
    other_xml = robot.other_xml

    links.remove(child)

    for grandchild, _child, data in robot._G.in_edges(child, data=True):
        joint = data['joint']
        new_joint = transform(joint, t_child_to_parent)
        # transforming joints this way only works for fixed joints

        new_joint.parent = parent.name

        print(f"Adding grand-child connection from {parent.name} to {grandchild.name}")
        joints.remove(joint)
        joints.append(new_joint)

    for _child, _parent, data in  robot._G.out_edges(child, data=True):
        joint = data['joint']
        print(f"Removing joint {joint.name}")
        joints.remove(joint)

    robot.__init__(name, links, joints, transmissions, materials, other_xml)

def eat_children(robot, link):
    """Take the immediate children of this <link>, and merge them
    if they attached with a fixed link. This is done recursively, leaving no extra
    fixed links in the URDF."""

    if len(list(robot._G.predecessors(link))) == 0: return # link has no children

    for child in robot._G.predecessors(link):

        eat_children(robot, child)

        if link_is_fixed(robot, child):

            give_childs_stuff_to_parent(robot, child)

            remove_child(robot, child)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("path", type=str,
                        help="Absolute path to URDF to simplify.")
    args, unknown = parser.parse_known_args()

    robot = URDF.load(args.path)
    eat_children(robot, robot.base_link)
    robot.save(f"{args.path}.simplified")