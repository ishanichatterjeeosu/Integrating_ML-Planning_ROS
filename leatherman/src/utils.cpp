#include <leatherman/utils.h>

// system includes
#include <log4cxx/logger.h>
#include <resource_retriever/retriever.h>
#include <ros/console.h>
#include <tinyxml.h>

// project includes
#include <leatherman/print.h>

#define SMALL_NUM  0.00000001     // to avoid division overflow

namespace leatherman {

double distanceBetween3DLineSegments(
    const Eigen::Vector3d& l1a,
    const Eigen::Vector3d& l1b,
    const Eigen::Vector3d& l2a,
    const Eigen::Vector3d& l2b)
{
    // Copyright 2001, softSurfer (www.softsurfer.com)
    // This code may be freely used and modified for any purpose
    // providing that this copyright notice is included with it.
    // SoftSurfer makes no warranty for this code, and cannot be held
    // liable for any real or imagined damage resulting from its use.
    // Users of this code must verify correctness for their application.

    double u[3];
    double v[3];
    double w[3];
    double dP[3];

    u[0] = l1b[0] - l1a[0];
    u[1] = l1b[1] - l1a[1];
    u[2] = l1b[2] - l1a[2];

    v[0] = l2b[0] - l2a[0];
    v[1] = l2b[1] - l2a[1];
    v[2] = l2b[2] - l2a[2];

    w[0] = l1a[0] - l2a[0];
    w[1] = l1a[1] - l2a[1];
    w[2] = l1a[2] - l2a[2];

    double a = u[0] * u[0] + u[1] * u[1] + u[2] * u[2]; // always >= 0
    double b = u[0] * v[0] + u[1] * v[1] + u[2] * v[2]; // dot(u,v);
    double c = v[0] * v[0] + v[1] * v[1] + v[2] * v[2]; // dot(v,v);        // always >= 0
    double d = u[0] * w[0] + u[1] * w[1] + u[2] * w[2]; // dot(u,w);
    double e = v[0] * w[0] + v[1] * w[1] + v[2] * w[2]; // dot(v,w);
    double D = a*c - b*b;       // always >= 0
    double sc, sN, sD = D;      // sc = sN / sD, default sD = D >= 0
    double tc, tN, tD = D;      // tc = tN / tD, default tD = D >= 0

    // compute the line parameters of the two closest points
    if (D < SMALL_NUM) { // the lines are almost parallel
        sN = 0.0;        // force using point P0 on segment S1
        sD = 1.0;        // to prevent possible division by 0.0 later
        tN = e;
        tD = c;
    }
    else {                // get the closest points on the infinite lines
        sN = (b*e - c*d);
        tN = (a*e - b*d);
        if (sN < 0.0) {       // sc < 0 => the s=0 edge is visible
            sN = 0.0;
            tN = e;
            tD = c;
        }
        else if (sN > sD) {  // sc > 1 => the s=1 edge is visible
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.0) {           // tc < 0 => the t=0 edge is visible
        tN = 0.0;
        // recompute sc for this edge
        if (-d < 0.0)
            sN = 0.0;
        else if (-d > a)
            sN = sD;
        else {
            sN = -d;
            sD = a;
        }
    }
    else if (tN > tD) {      // tc > 1 => the t=1 edge is visible
        tN = tD;
        // recompute sc for this edge
        if ((-d + b) < 0.0)
            sN = 0;
        else if ((-d + b) > a)
             sN = sD;
        else {
            sN = (-d + b);
            sD = a;
        }
    }

    // finally do the division to get sc and tc
    sc = (fabs(sN) < SMALL_NUM ? 0.0 : sN / sD);
    tc = (fabs(tN) < SMALL_NUM ? 0.0 : tN / tD);

    // get the difference of the two closest points
    // dP = w + (sc * u) - (tc * v);  // = S1(sc) - S2(tc)

    dP[0] = w[0] + (sc * u[0]) - (tc * v[0]);
    dP[1] = w[1] + (sc * u[1]) - (tc * v[1]);
    dP[2] = w[2] + (sc * u[2]) - (tc * v[2]);

    return  sqrt(dP[0]*dP[0] + dP[1]*dP[1] + dP[2]*dP[2]);   // return the closest distance
}

bool isValidJointState(const sensor_msgs::JointState& state)
{
    return (state.position.empty() || state.position.size() == state.name.size()) ||
            (state.velocity.empty() || state.velocity.size() == state.name.size()) ||
            (state.effort.empty() || state.effort.size() == state.name.size());
}

bool isValidMultiDOFJointState(const sensor_msgs::MultiDOFJointState& state)
{
    return (state.transforms.empty() || state.transforms.size() == state.joint_names.size()) ||
            (state.twist.empty() || state.twist.size() == state.joint_names.size()) ||
            (state.wrench.empty() || state.wrench.size() == state.joint_names.size());
}

bool findJointPosition(const sensor_msgs::JointState &state, std::string name, double &position)
{
  for(size_t i = 0; i < state.name.size(); i++)
  {
    if(name.compare(state.name[i]) == 0)
    {
      position = state.position[i];
      return true;
    }
  }
  return false;
}

bool getJointPositions(
    const sensor_msgs::JointState& joint_state,
    const sensor_msgs::MultiDOFJointState& multi_dof_joint_state,
    const std::vector<std::string>& joint_names,
    std::vector<double>& positions)
{
    std::vector<std::string> missing;
    return getJointPositions(
            joint_state,
            multi_dof_joint_state,
            joint_names,
            positions,
            missing);
}

bool getJointPositions(
    const sensor_msgs::JointState& joint_state,
    const sensor_msgs::MultiDOFJointState& multi_dof_joint_state,
    const std::vector<std::string>& joint_names,
    std::vector<double>& positions,
    std::vector<std::string>& missing)
{
    if (!isValidJointState(joint_state) ||
        !isValidMultiDOFJointState(multi_dof_joint_state))
    {
        return false;
    }

    positions.resize(joint_names.size());
    missing.clear();

    for (size_t jidx = 0; jidx < joint_names.size(); ++jidx) {
        const std::string& joint_name = joint_names[jidx];
        const size_t idx = joint_name.find('/');
        if (idx != std::string::npos) {
            std::string jname = joint_name.substr(0, idx);
            std::string local_var_name = joint_name.substr(idx + 1);
            auto it = std::find(
                    multi_dof_joint_state.joint_names.begin(),
                    multi_dof_joint_state.joint_names.end(),
                    jname);
            if (it == multi_dof_joint_state.joint_names.end()) {
                missing.push_back(joint_name);
            } else {
                size_t jind = std::distance(multi_dof_joint_state.joint_names.begin(), it);
                const auto& trans = multi_dof_joint_state.transforms[jind];
                if (local_var_name == "x" || local_var_name == "trans_x") {
                    positions[jidx] = trans.translation.x;
                } else if (local_var_name == "y" || local_var_name == "trans_y") {
                    positions[jidx] = trans.translation.y;
                } else if (local_var_name == "z" || local_var_name == "trans_z") {
                    positions[jidx] = trans.translation.z;
                } else if (local_var_name == "theta") {
                    double theta;

                    Eigen::Quaterniond q(
                            trans.rotation.w,
                            trans.rotation.x,
                            trans.rotation.y,
                            trans.rotation.z);
                    double s_squared = 1.0 - (q.w() * q.w());
                    // from sbpl_collision_checking, from MoveIt, from BULLET
                    if (s_squared < 10.0 * std::numeric_limits<double>::epsilon()) {
                        theta = 0.0;
                    } else {
                        double s = 1.0 / sqrt(s_squared);
                        theta = (acos(q.w()) * 2.0f) * (q.z() * s);
                    }
                    positions[jidx] = theta;
                } else if (local_var_name == "rot_w") {
                    Eigen::Quaterniond q(
                            trans.rotation.w,
                            trans.rotation.x,
                            trans.rotation.y,
                            trans.rotation.z);
                    positions[jidx] = q.w();
                } else if (local_var_name == "rot_x") {
                    Eigen::Quaterniond q(
                            trans.rotation.w,
                            trans.rotation.x,
                            trans.rotation.y,
                            trans.rotation.z);
                    positions[jidx] = q.x();
                } else if (local_var_name == "rot_y") {
                    Eigen::Quaterniond q(
                            trans.rotation.w,
                            trans.rotation.x,
                            trans.rotation.y,
                            trans.rotation.z);
                    positions[jidx] = q.y();
                } else if (local_var_name == "rot_z") {
                    Eigen::Quaterniond q(
                            trans.rotation.w,
                            trans.rotation.x,
                            trans.rotation.y,
                            trans.rotation.z);
                    positions[jidx] = q.z();
                } else {
                    ROS_ERROR("Unrecognized local variable name '%s'", local_var_name.c_str());
                    return false;
                }
            }
        } else {
            auto it = std::find(joint_state.name.begin(), joint_state.name.end(), joint_name);
            if (it == joint_state.name.end()) {
                missing.push_back(joint_name);
            }
            else {
                size_t jind = std::distance(joint_state.name.begin(), it);
                positions[jidx] = joint_state.position[jind];
            }
        }
    }

    if (!missing.empty()) {
        positions.clear();
        return false;
    }

    return true;
}

void findAndReplaceJointPosition(std::string name, double position, sensor_msgs::JointState &state)
{
  bool exists = false;
  for(size_t i = 0; i < state.name.size(); i++)
  {
    if(state.name[i].compare(name) == 0)
    {
      state.position[i] = position;
      exists = true;
    }
  }
  if(!exists)
  {
    state.position.push_back(position);
    state.name.push_back(name);
  }
}

bool getJointIndex(const KDL::Chain &c, std::string name, int &index)
{
  for(size_t j = 0; j < c.getNrOfSegments(); ++j)
  {
    if(c.getSegment(j).getJoint().getName().compare(name) == 0)
    {
      index = j;
      return true;
    }
  }
  return false;
}

bool getSegmentIndex(const KDL::Chain &c, std::string name, int &index)
{
  for(size_t j = 0; j < c.getNrOfSegments(); ++j)
  {
    if(c.getSegment(j).getName().compare(name) == 0)
    {
      index = j;
      return true;
    }
  }
  return false;
}

bool getSegmentOfJoint(const KDL::Tree &tree, std::string joint, std::string &segment)
{
  KDL::SegmentMap smap = tree.getSegments();
  for(std::map<std::string, KDL::TreeElement>::const_iterator iter = smap.begin(); iter != smap.end(); ++iter)
  {
    if(iter->second.segment.getJoint().getName().compare(joint) == 0)
    {
      segment = iter->second.segment.getName();
      return true;
    }
  }
  return false;
}

bool getChainTip(const KDL::Tree &tree, const std::vector<std::string> &segments, std::string chain_root, std::string &chain_tip)
{
  KDL::Chain chain;

  // compute # of links each link would include if tip of chain
  for(size_t i = 0; i < segments.size(); ++i)
  {
    // create chain with link i as the tip
    if (!tree.getChain(chain_root, segments[i], chain))
    {
      ROS_ERROR("Failed to fetch the KDL chain. This code only supports a set of segments that live within a single kinematic chain. (root: %s, tip: %s)", chain_root.c_str(), segments[i].c_str());
      return false;
    }

    int index;
    size_t num_segments_included = 0;
    for(size_t j = 0; j < segments.size(); ++j)
    {
      if(getSegmentIndex(chain, segments[j], index))
        num_segments_included++;
    }

    if(num_segments_included == segments.size())
    {
      chain_tip = segments[i];
      return true;
    }
  }
  return false;
}

void HSVtoRGB(
    double* r, double* g, double* b,
    double h, double s, double v)
{
    int i;
    double f, p, q, t;
    if (s == 0) {
        // achromatic (grey)
        *r = *g = *b = v;
        return;
    }
    h /= 60.0;      // sector 0 to 5
    i = floor(h);
    f = h - i;      // factorial part of h
    p = v * (1.0 - s);
    q = v * (1.0 - s * f);
    t = v * (1.0 - s * (1.0 - f));
    switch (i) {
    case 0:
        *r = v;
        *g = t;
        *b = p;
        break;
    case 1:
        *r = q;
        *g = v;
        *b = p;
        break;
    case 2:
        *r = p;
        *g = v;
        *b = t;
        break;
    case 3:
        *r = p;
        *g = q;
        *b = v;
        break;
    case 4:
        *r = t;
        *g = p;
        *b = v;
        break;
    default:
        *r = v;
        *g = p;
        *b = q;
        break;
    }
}

void msgRGBToHSV(
    const std_msgs::ColorRGBA& color,
    double& h, double& s, double& v)
{
    // TODO: implement
}

void msgHSVToRGB(
    double h, double s, double v,
    std_msgs::ColorRGBA& color)
{
    double r, g, b;
    HSVtoRGB(&r, &g, &b, h, s, v);
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = 1.0f;
}

void setLoggerLevel(std::string package, std::string name, std::string level)
{
  ROSCONSOLE_AUTOINIT;

  //std::string logger_name = ROSCONSOLE_DEFAULT_NAME + std::string(".") + name;
  std::string logger_name = package + std::string(".") + name;

  ROS_INFO("Setting %s to %s level", logger_name.c_str(), level.c_str());

  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(logger_name);

  // Set the logger for this package to output all statements
  if(level.compare("debug") == 0)
    my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  else
    my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);

  ROS_DEBUG_NAMED(name, "This is a debug statement, and should print if you enabled debug.");
}

void setLoggerLevel(std::string name, std::string level)
{
  ROSCONSOLE_AUTOINIT;

  //std::string logger_name = ROSCONSOLE_DEFAULT_NAME + std::string(".") + name;
  std::string logger_name = name;

  ROS_INFO("Setting %s to %s level", logger_name.c_str(), level.c_str());

  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(logger_name);

  // Set the logger for this package to output all statements
  if(level.compare("debug") == 0)
    my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  else
    my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);

  ROS_DEBUG_NAMED(name, "This is a debug statement, and should print if you enabled debug.");
}

} // namespace leatherman
