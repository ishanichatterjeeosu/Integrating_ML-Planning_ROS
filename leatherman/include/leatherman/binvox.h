/*
 * For reading in Binvox files. (http://www.cs.princeton.edu/~min/binvox/)
 * Binvox is a tool that reads in a mesh file and outputs a voxelized
 * version of it.
 *  * Ubuntu 12.04 and up (GLIBC 2.14):
 *    http://www.cs.princeton.edu/~min/binvox/linux64/binvox?rnd=137424032048747
 * Ubuntu 11.10 and earlier (GLIBC 2.13):
 *    http://www.cs.princeton.edu/~min/binvox/linux64b/binvox
*/

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <octomap/Pointcloud.h>
#include <octomap_msgs/Octomap.h>
#include <sensor_msgs/PointCloud.h>
#include <octomap/OcTree.h>

namespace leatherman {

struct VoxelGrid
{
    int     version = 0;
    int     depth   = 0;    ///< size along the x axis
    int     width   = 0;    ///< size along the y axis
    int     height  = 0;    ///< size along the z axis
    double  tx      = 0.0;
    double  ty      = 0.0;
    double  tz      = 0.0;
    double  scale   = 0.0;

    int     size    = 0;
    double  vdim    = 0.0;
    int     occ_count = 0;

    // dense array of voxels
    std::unique_ptr<std::uint8_t[]> voxels = nullptr;
};

int GetVoxelIndex(const VoxelGrid& grid, int x, int y, int z);
double GetWorldCoordX(const VoxelGrid& grid, int x);
double GetWorldCoordY(const VoxelGrid& grid, int y);
double GetWorldCoordZ(const VoxelGrid& grid, int z);

/// Deserialize a .binvox file.
bool ReadBinvox(const char* filename, VoxelGrid& grid);
bool ReadBinvox(const std::string& filename, VoxelGrid& grid);

/// Deserialize a .binvox file and insert the center points of all occupied
/// voxels into a point cloud.
bool AddBinvoxOccupiedVoxelsToPointCloud(
    const std::string& filename,
    octomap::Pointcloud& point_cloud);

/// FIXME
bool convertBinvoxToOctomapMsg(
    const std::string& filename,
    double resolution,
    octomap_msgs::Octomap& msg);

/// Convert a VoxelGrid to sensor_msgs/PointCloud. Return true if conversion
/// is successful. If conversion is successful, previously existing points and
/// channels in $pc are discarded, otherwise $pc is unmodified. In either case,
/// the $pc's header field is unmodified.
bool convertBinvoxToPointCloudMsg(
    const VoxelGrid& grid,
    sensor_msgs::PointCloud& pc);

/// Deserialize a .binvox file and convert to sensor_msgs/PointCloud.
bool convertBinvoxToPointCloudMsg(
    const std::string& filename,
    sensor_msgs::PointCloud& pc);

/// Convert a VoxelGrid to a vector of Eigen::Vector3d. Return true if
/// conversion is successful. If conversion is successful, previously existing
/// points in $voxels are discarded, otherwise $voxels is unmodified.
bool convertBinvoxToVector3d(
    const VoxelGrid& grid,
    std::vector<Eigen::Vector3d>& voxels);

/// Deserialize a .binvox file and convert to a vector of Eigen::Vector3d.
bool convertBinvoxToVector3d(
    const std::string& filename,
    std::vector<Eigen::Vector3d>& voxels);

/// Rasterize a 3D mesh stored at $mesh_filename. If rasterization is successful,
/// the resulting voxel grid is stored on disk in binvox format, and
/// $binvox_filename is assigned the path to the .binvox file. Rasterization is
/// performed by a system call to the 'binvox' command. Rasterization will fail
/// if no command interpreter is available, the 'binvox' command is not found,
/// or the 'binvox' command returns an error.
bool createBinvoxFile(
    const std::string& mesh_filename,
    std::string& binvox_filename);

/// Convert a .binvox file to .bt. If conversion is successful, $bt_filename
/// is assigned the path to the resulting .bt file. Conversion is performed by
/// a system call to the binvox2bt command, distributed with the 'octomap'
/// package.
bool convertBinvoxToBtSystem(
    const std::string& binvox_filename,
    std::string& bt_filename);

/// FIXME. Copied from octomap/binvox2bt.cpp
bool convertBinvoxToBt(
    const std::string& binvox_filename,
    std::string& bt_filename);

/// Rasterize a 3D mesh stored at $filename and store the center points of
/// occupied voxels in $voxels. Rasterization is performed by a call to
/// createBinvoxFile and voxel centers are extracted by a call to
/// convertBinvoxToVector3d(). Returns true if both rasterization and voxel
/// extraction are successful.
bool voxelizeMesh(
    const std::string& filename,
    std::vector<Eigen::Vector3d>& voxels);

/// FIXME
void getOccupiedVoxelsInOcTree(
    const octomap::OcTree& octree,
    std::vector<Eigen::Vector3d>& voxels);

} // namespace leatherman
