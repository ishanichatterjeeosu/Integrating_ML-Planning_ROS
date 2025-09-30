#ifndef LEATHERMAN_MESH_PROCESSING_H
#define LEATHERMAN_MESH_PROCESSING_H

#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/Point.h>

namespace leatherman {

void scaleVertices(
    const std::vector<Eigen::Vector3d>& vin,
    double sx, double sy, double sz,
    std::vector<Eigen::Vector3d>& vout);

void scaleVertices(
    const std::vector<geometry_msgs::Point>& vin,
    double sx, double sy, double sz,
    std::vector<geometry_msgs::Point>& vout);

} // namespace leatherman

#endif
