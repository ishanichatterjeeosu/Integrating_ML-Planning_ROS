#include <leatherman/mesh_processing.h>

namespace leatherman {

void scaleVertices(
    const std::vector<Eigen::Vector3d> &vin,
    double sx, double sy, double sz,
    std::vector<Eigen::Vector3d> &vout)
{
    // find the mean of the points
    Eigen::Vector3d mean;
    double sumx = 0, sumy = 0, sumz = 0;
    for (size_t i = 0; i < vin.size(); ++i) {
        sumx += vin[i].x();
        sumy += vin[i].y();
        sumz += vin[i].z();
    }
    mean(0) = sumx / double(vin.size());
    mean(1) = sumy / double(vin.size());
    mean(2) = sumz / double(vin.size());

    // subtract mean and then scale
    vout.resize(vin.size());
    for (size_t i = 0; i < vin.size(); ++i) {
        vout[i](0) = (vin[i] - mean)(0) * sx;
        vout[i](1) = (vin[i] - mean)(1) * sy;
        vout[i](2) = (vin[i] - mean)(2) * sz;

        vout[i] += mean;
    }
}

void scaleVertices(
    const std::vector<geometry_msgs::Point> &vin,
    double sx, double sy, double sz,
    std::vector<geometry_msgs::Point> &vout)
{
    std::vector<Eigen::Vector3d> evin(vin.size()), evout;
    for (size_t p = 0; p < vin.size(); ++p) {
        evin[p](0) = vin[p].x;
        evin[p](1) = vin[p].y;
        evin[p](2) = vin[p].z;
    }

    scaleVertices(evin, sx, sy, sz, evout);

    vout.resize(evout.size());
    for (size_t p = 0; p < evout.size(); ++p) {
        vout[p].x = evout[p](0);
        vout[p].y = evout[p](1);
        vout[p].z = evout[p](2);
    }
}

} // namespace leatherman
