#include <leatherman/binvox.h>

// standard includes
#include <stdlib.h>
#include <fstream>
#include <utility>

// system includes
#include <ros/console.h>
#include <octomap_msgs/conversions.h>

// project includes
#include <leatherman/file.h>

namespace leatherman {

static bool ReadBinvoxHeader(
    std::ifstream& input,
    int& version,
    int& depth, int& width, int& height,
    double& tx, double& ty, double& tz,
    double& scale)
{
    std::string word;

    auto read_word = [&](const char* expected) {
        input >> word;
        if (!input.good()) {
            return false;
        }
        if (word != expected) {
            ROS_ERROR("Expected '%s'. Got '%s'", expected, word.c_str());
            return false;
        }

        return true;
    };

    // read header
    if (!read_word("#binvox")) {
        return false;
    }

    input >> version;
    if (!input.good()) {
        return false;
    }

    if (!read_word("dim")) {
        return false;
    }

    input >> depth >> width >> height;
    if (!input.good()) {
        return false;
    }
    if (depth != width || depth != height) {
        ROS_ERROR("Expected equal dimensions. Got depth = %d, width = %d, height = %d", depth, width, height);
        return false;
    }

    if (!read_word("translate")) {
        return false;
    }

    input >> tx >> ty >> tz;
    if (!input.good()) {
        return false;
    }

    if (!read_word("scale")) {
        return false;
    }

    input >> scale;
    if (!input.good()) {
        return false;
    }

    if (!read_word("data")) {
        return false;
    }

    return true;
}

int GetVoxelIndex(const VoxelGrid& grid, int x, int y, int z)
{
    return x * grid.width * grid.height + z * grid.width + y;
}

double GetWorldCoordX(const VoxelGrid& grid, int x)
{
    const double cx = ((double)x + 0.5) * grid.vdim;
    const double wx = grid.scale * cx + grid.tx;
    return wx;
}

double GetWorldCoordY(const VoxelGrid& grid, int y)
{
    const double cy = ((double)y + 0.5) * grid.vdim;
    const double wy = grid.scale * cy + grid.ty;
    return wy;
}

double GetWorldCoordZ(const VoxelGrid& grid, int z)
{
    const double cz = ((double)z + 0.5) * grid.vdim;
    const double wz = grid.scale * cz + grid.tz;
    return wz;
}

bool ReadBinvox(const char* filename, VoxelGrid& grid)
{
    ROS_INFO("Reading binvox file: %s", filename);
    std::ifstream input(filename, std::ios::in | std::ios::binary);

    if (!input.is_open()) {
        ROS_ERROR("Failed to open '%s' for reading", filename);
        return false;
    }

    int version;
    int depth, width, height;
    double tx, ty, tz;
    double scale;
    std::unique_ptr<std::uint8_t[]> voxels;

    if (!ReadBinvoxHeader(
            input, version, depth, width, height, tx, ty, tz, scale))
    {
        return false;
    }

    int size = depth * width * height;
    voxels.reset(new std::uint8_t[size]);

    // read voxel data
    std::uint8_t value;
    std::uint8_t count;
    int index = 0;
    int end_index = 0;
    int nr_voxels = 0;

    input.unsetf(std::ios::skipws); // need to read every byte now
    input >> value; // read the linefeed char

    while ((end_index < size) && input.good()) {
        input >> value >> count;

        // if file still ok
        if (input.good()) {
            end_index = index + count;
            if (end_index > size) {
                ROS_ERROR("Binvox file contains excess data");
                return false;
            }
            for (int i = index; i < end_index; i++) {
                voxels[i] = value;
            }

            if (value) {
                nr_voxels += count;
            }
            index = end_index;
        }
    }

    // final transaction
    grid.version = version;
    grid.depth = depth;
    grid.height = height;
    grid.width = width;
    grid.tx = tx;
    grid.ty = ty;
    grid.tz = tz;
    grid.scale = scale;

    grid.size = size;
    grid.vdim = 1.0 / (double)depth;
    grid.occ_count = nr_voxels;

    grid.voxels = std::move(voxels);

    return true;
}

bool ReadBinvox(const std::string& filename, VoxelGrid& grid)
{
    return ReadBinvox(filename.c_str(), grid);
}

template <class Callable>
void IterateOccupiedVoxels(const VoxelGrid& grid, Callable proc)
{
    int index = 0;
    for (int x = 0; x < grid.depth; ++x) {
    for (int z = 0; z < grid.height; ++z) {
    for (int y = 0; y < grid.width; ++y) {
        if (grid.voxels[index]) {
            proc(x, y, z);
        }
        ++index;
    }
    }
    }
}

bool AddBinvoxOccupiedVoxelsToPointCloud(
    const std::string& filename,
    octomap::Pointcloud& cloud_out)
{
    VoxelGrid grid;
    if (!ReadBinvox(filename, grid)) {
        ROS_ERROR("Failed to read '%s'", filename.c_str());
        return false;
    }

    auto add_occ_voxel_to_pc = [&](int x, int y, int z) {
        double wx = GetWorldCoordX(grid, x);
        double wy = GetWorldCoordY(grid, y);
        double wz = GetWorldCoordZ(grid, z);
        cloud_out.push_back(wx, wy, wz);
    };

    IterateOccupiedVoxels(grid, add_occ_voxel_to_pc);
    return true;
}

bool convertBinvoxToOctomapMsg(
    const std::string& filename,
    double resolution,
    octomap_msgs::Octomap& msg)
{
    return false;
#if 0
    octomap::Pointcloud point_cloud;
    AddBinvoxOccupiedVoxelsToPointCloud(filename, point_cloud);

    ROS_INFO("Inserting scan with %zu points.", point_cloud.size());
    octomap::OcTree octree(resolution);
//    octree.insertScan(point_cloud, octomap::point3d(0.0f, 0.0f, 0.0f));

    ROS_INFO("Converting to octomap_msgs/Octomap.");
    octomap_msgs::binaryMapToMsg(octree, msg);
    return false;
#endif
}

bool convertBinvoxToPointCloudMsg(
    const VoxelGrid& grid,
    sensor_msgs::PointCloud& pc)
{
    auto add_occ_voxel_to_pc = [&](int x, int y, int z) {
        geometry_msgs::Point32 p;
        p.x = GetWorldCoordX(grid, x);
        p.y = GetWorldCoordY(grid, y);
        p.z = GetWorldCoordZ(grid, z);
        pc.points.push_back(p);
    };

    pc.channels.clear();
    pc.points.clear();
    pc.points.reserve(grid.occ_count);
    IterateOccupiedVoxels(grid, add_occ_voxel_to_pc);
    return true;
}

bool convertBinvoxToPointCloudMsg(
    const std::string& filename,
    sensor_msgs::PointCloud& pc)
{
    VoxelGrid grid;
    if (!ReadBinvox(filename, grid)) {
        ROS_ERROR("Failed to read '%s'", filename.c_str());
        return false;
    }

    return convertBinvoxToPointCloudMsg(grid, pc);
}

bool convertBinvoxToVector3d(
    const VoxelGrid& grid,
    std::vector<Eigen::Vector3d>& voxels)
{
    auto add_occ_voxel_to_voxels = [&](int x, int y, int z) {
        Eigen::Vector3d v;
        v.x() = GetWorldCoordX(grid, x);
        v.y() = GetWorldCoordY(grid, y);
        v.z() = GetWorldCoordZ(grid, z);
        voxels.push_back(v);
    };

    voxels.clear();
    voxels.reserve(grid.occ_count);
    IterateOccupiedVoxels(grid, add_occ_voxel_to_voxels);
    return true;
}

bool convertBinvoxToVector3d(
    const std::string& filename,
    std::vector<Eigen::Vector3d>& voxels)
{
    VoxelGrid grid;
    if (!ReadBinvox(filename, grid)) {
        ROS_ERROR("Failed to read '%s'", filename.c_str());
        return false;
    }

    return convertBinvoxToVector3d(grid, voxels);
}

bool createBinvoxFile(
    const std::string& mesh_filename,
    std::string& binvox_filename)
{
    if (system(nullptr) == 0) {
        ROS_ERROR("Failed to convert mesh to binvox. No command processor available");
        return false;
    }

    std::string command = "binvox -e  " + mesh_filename;
    int err = system(command.c_str());
    if (err) {
        ROS_ERROR("Failed to convert mesh to binvox. Command '%s' returned %d", command.c_str(), err);
        return false;
    }

    // this is error prone if run twice in same folder on same file
    std::string path = getPathWithoutFilename(mesh_filename);
    std::string filename = getFilenameFromPath(mesh_filename, true);
    binvox_filename = path + filename + ".binvox";
    return true;
}

bool convertBinvoxToBtSystem(
    const std::string& binvox_filename,
    std::string& bt_filename)
{
    if (system(nullptr) == 0) {
        ROS_ERROR("Failed to convert binvox to bt. No command processor available");
        return false;
    }

    std::string path = getPathWithoutFilename(binvox_filename);
    std::string filename = getFilenameFromPath(binvox_filename, true);
    std::string bt_filename_ = path + filename + ".bt";

    std::string command =
            "binvox2bt --mark-free -o " + bt_filename_ + " " + binvox_filename;

    int err = system(command.c_str());
    if (err) {
        ROS_ERROR("Failed to convert binvox to bt. Command '%s' returned %d", command.c_str(), err);
        return false;
    }

    bt_filename = std::move(bt_filename_);
    return true;
}

bool convertBinvoxToBt(
    const std::string& binvox_filename,
    std::string& bt_filename)
{
    return false;
#if 0
    VoxelGrid grid;
    if (!ReadBinvox(binvox_filename, grid)) {
        return false;
    }

    double res = (double)grid.scale / (double)grid.depth;

    ROS_INFO("Build octree with leaf leaf resolution %f", res);
    octomap::OcTree tree(res);

    int index = 0;
    for (int x = 0; x < grid.width; ++x) {
    for (int y = 0; y < grid.height; ++y) {
    for (int z = 0; z < grid.depth; ++z) {
        if (grid.voxels[index] != 0) {
            octomap::point3d endpoint(
                x * res + grid.tx + 0.000001,
                y * res + grid.ty + 0.000001,
                z * res + grid.tz + 0.000001);
            tree.updateNode(endpoint, true, true);
        }
        index++;
    }
    }
    }

    // prune octree
    ROS_INFO("Pruning tree");
    tree.updateInnerOccupancy();
    tree.prune();

    bt_filename = binvox_filename + ".bt";
    tree.writeBinary(bt_filename.c_str());

    ROS_INFO("Done writing octree to bt file.");
    return true;
#endif
}

bool voxelizeMesh(
    const std::string& filename,
    std::vector<Eigen::Vector3d>& voxels)
{
    std::string binvox_filename;
    if (!createBinvoxFile(filename, binvox_filename)) {
        return false;
    }
    return convertBinvoxToVector3d(binvox_filename, voxels);
}

void getOccupiedVoxelsInOcTree(
    const octomap::OcTree& octree,
    std::vector<Eigen::Vector3d>& voxels)
{
#if 0
    Eigen::Vector3d v;
    for (auto it = octree.begin(octree.getTreeDepth()), end = octree.end();
        it != end; ++it)
    {
        if (octree.isNodeOccupied(*it)) {
            v(0) = it.getX();
            v(1) = it.getY();
            v(2) = it.getZ();
            voxels.push_back(v);
        }
    }
#endif
}

} // namespace leatherman
