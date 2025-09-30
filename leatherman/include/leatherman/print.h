#ifndef _LEATHERMAN_PRINT_
#define _LEATHERMAN_PRINT_

// standard includes
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

// system includes
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <ros/console.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace leatherman {

void printPoseMsg(const geometry_msgs::Pose& p, const std::string& text);

void printJointTrajectory(
    const trajectory_msgs::JointTrajectory& traj,
    const std::string& text);

void printKDLChain(const KDL::Chain& c, const std::string& text);

} // namespace leatherman

namespace std { // WARNING: non-standard

template <class T, class Allocator>
ostream& operator<<(ostream& o, const vector<T, Allocator>& v)
{
    o << "[ ";
    for (size_t i = 0; i < v.size(); ++i) {
        o << v[i];
        if (i != v.size() - 1) {
            o << ", ";
        }
    }
    o << " ]";
    return o;
}

template <class T1, class T2>
ostream& operator<<(ostream& o, const pair<T1, T2>& p)
{
    o << '(' << p.first << ", " << p.second << ')';
    return o;
}

template <class T, class Allocator>
string to_string(const vector<T, Allocator>& v)
{
    stringstream ss;
    ss << v;
    return ss.str();
}

} // namespace std

namespace Eigen {

std::string to_str(const Affine3d& transform);
std::string to_str(const Vector2d& v);
std::string to_str(const Vector3d& v);
std::string to_str(const AngleAxisd& aa);

} // namespace Eigen

namespace geometry_msgs {

std::string to_string(const Pose& p);
std::string to_string(const Transform& t);

} // namespace geometry_msgs

#endif
