#ifndef SMPL_URDF_ROBOT_MODEL_ROBOT_STATE_VISUALIZATION_H
#define SMPL_URDF_ROBOT_MODEL_ROBOT_STATE_VISUALIZATION_H

// standard includes
#include <vector>

// system includes
#include <smpl/debug/marker.h>

namespace smpl {
namespace urdf {

struct RobotState;
struct Shape;

auto MakeShapeVisualization(const Shape* shape) -> smpl::visual::Shape;

auto MakeRobotVisualization(
    const RobotState* state,
    smpl::visual::Color color = smpl::visual::Color{ 0.0f, 0.0f, 0.0f, 1.0f },
    const std::string& frame = std::string(),
    const std::string& ns = std::string(),
    int32_t* id = NULL)
    -> std::vector<smpl::visual::Marker>;

auto MakeCollisionVisualization(
    const RobotState* state,
    smpl::visual::Color color = smpl::visual::Color{ 0.0f, 0.0f, 0.0f, 1.0f },
    const std::string& frame = std::string(),
    const std::string& ns = std::string(),
    int32_t* id = NULL)
    -> std::vector<smpl::visual::Marker>;

} // namespace urdf
} // namespace sbpl

#endif
