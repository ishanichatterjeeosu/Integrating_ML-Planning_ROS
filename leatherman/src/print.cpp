#include <leatherman/print.h>

namespace leatherman {

void printPoseMsg(
    const geometry_msgs::Pose& p,
    const std::string& text)
{
    ROS_INFO("[%s] xyz: %0.3f %0.3f %0.3f  quat: %0.3f %0.3f %0.3f %0.3f",
            text.c_str(), p.position.x, p.position.y, p.position.z,
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
}

void printJointTrajectory(
    const trajectory_msgs::JointTrajectory& traj,
    const std::string& text)
{
    if (!traj.points.empty()) {
        ROS_INFO("[%s] start_time: %0.3f", text.c_str(), traj.header.stamp.toSec());
    }
    for (unsigned int i = 0; i < traj.points.size(); i++) {
        ROS_INFO_STREAM("[" << text << "] [" << i << "] " <<
                traj.points[i].positions <<
                " time from start: " << traj.points[i].time_from_start.toSec());
    }
}

void printKDLChain(const KDL::Chain& c, const std::string& text)
{
    ROS_INFO("[%s] # segments: %d  # joints: %d", text.c_str(),
            c.getNrOfSegments(), c.getNrOfJoints());
    double r, p, y, jnt_pos = 0;
    for (size_t j = 0; j < c.getNrOfSegments(); ++j) {
        c.getSegment(j).pose(jnt_pos).M.GetRPY(r, p, y);
        ROS_INFO("[%s] [%2d] segment: %21s  xyz: %0.3f %0.3f %0.3f  rpy: %0.3f %0.3f %0.3f",
                text.c_str(), int(j), c.getSegment(j).getName().c_str(),
                c.getSegment(j).pose(jnt_pos).p.x(),
                c.getSegment(j).pose(jnt_pos).p.y(),
                c.getSegment(j).pose(jnt_pos).p.z(), r, p, y);

        c.getSegment(j).getJoint().pose(jnt_pos).M.GetRPY(r, p, y);
        ROS_INFO("[%s]        joint: %21s  xyz: %0.3f %0.3f %0.3f  rpy: %0.3f %0.3f %0.3f  type: %s",
                text.c_str(), c.getSegment(j).getJoint().getName().c_str(),
                c.getSegment(j).getJoint().pose(jnt_pos).p.x(),
                c.getSegment(j).getJoint().pose(jnt_pos).p.y(),
                c.getSegment(j).getJoint().pose(jnt_pos).p.z(), r, p, y,
                c.getSegment(j).getJoint().getTypeName().c_str());

    }
}

} // namespace leatherman

namespace Eigen {

std::string to_str(const Affine3d& transform)
{
    const Vector3d translation(transform.translation());
    const Quaterniond rotation(transform.rotation());
    std::stringstream ss;
    ss << "pos: (" << translation.x() << ", " << translation.y() << ", " << translation.z() << ")";
    ss << " ";
    ss << "rot: (" << rotation.w() << ", " << rotation.x() << ", " << rotation.y() << ", " << rotation.z() << ")";
    return ss.str();
}

std::string to_str(const Vector2d& v)
{
    std::stringstream ss;
    ss << "(" << v(0) << ", " << v(1) << ")";
    return ss.str();
}

std::string to_str(const Vector3d& v)
{
    std::stringstream ss;
    ss << "(" << v(0) << ", " << v(1) << ", " << v(2) << ")";
    return ss.str();
}

std::string to_str(const AngleAxisd& aa)
{
    std::stringstream ss;
    ss << "{ angle: " << 180.0 * aa.angle() / M_PI << " degs @ " << to_str(aa.axis()) << " }";
    return ss.str();
}

} // namespace Eigen

namespace geometry_msgs {

std::string to_string(const Pose& p)
{
    std::stringstream ss;
    ss << "{ " <<
            "position: " <<
            "{ " <<
            "x: " << p.position.x << ", " <<
            "y: " << p.position.y << ", " <<
            "z: " << p.position.z << " " <<
            "}, " <<
            "orientation: " <<
            "{ " <<
            "w: " << p.orientation.w << ", " <<
            "x: " << p.orientation.x << ", " <<
            "y: " << p.orientation.y << ", " <<
            "z: " << p.orientation.z << " ";
    return ss.str();
}

std::string to_string(const geometry_msgs::Transform& t)
{
    std::stringstream ss;
    ss << "{ " <<
            "translation: " <<
            "{ " <<
            "x: " << t.translation.x << ", " <<
            "y: " << t.translation.y << ", " <<
            "z: " << t.translation.z << " " <<
            "}, " <<
            "rotation: " <<
            "{ " <<
            "x: " << t.rotation.x << ", " <<
            "y: " << t.rotation.y << ", " <<
            "z: " << t.rotation.z << ", " <<
            "w: " << t.rotation.w << " " <<
            "} }";
    return ss.str();
}

} // namespace geometry_msgs
