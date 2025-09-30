/* \author Benjamin Cohen */

#ifndef LEATHERMAN_VIZ_H
#define LEATHERMAN_VIZ_H

// standard includes
#include <string>

// system includes
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <moveit_msgs/CollisionObject.h>
#include <ros/console.h>
#include <shape_msgs/SolidPrimitive.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace viz {

/// \brief Return a visualization marker representing a cube.
///
/// \param dim The dimensions of the cube with format { length, width, height }
/// \return The resulting visualization marker or a default-constructed
///     visualization marker if any parameter is incorrectly formatted.
visualization_msgs::Marker getCubeMarker(
    const geometry_msgs::PoseStamped& pose,
    const std::vector<double>& dim,
    int hue,
    const std::string& ns,
    int id = 0);

/// \brief Return a visualization marker representing a cube.
///
/// \param cube The cube with format { x, y, z, length, width, height }
/// \param color The color of the cube with format { r, g, b, a }. Components
///     are represented in the range [0, 1]. Any component outside this range
///     is assumed to be in the range [0, 255] and is normalized to [0, 1].
/// \return The resulting visualization marker or a default-constructed
///     visualization marker if any parameter is incorrectly formatted.
visualization_msgs::Marker getCubeMarker(
    const std::vector<double>& cube,
    const std::vector<double>& color,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0);

/// \brief Return a visualization marker representing a cube.
///
/// \param dim The dimensions of the cube with format { length, width, height }
/// \param color The color of the cube with format { r, g, b, a }. Components
///     are represented in the range [0, 1]. Any component outside this range
///     is assumed to be in the range [0, 255] and is normalized to [0, 1].
/// \return The resulting visualization marker or a default-constructed
///     visualization marker if any parameter is incorrectly formatted.
visualization_msgs::Marker getCubeMarker(
    const geometry_msgs::Pose& pose,
    const std::vector<double>& dim,
    const std::vector<double>& color,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0);

/// \brief Return a visualization marker representing a set of cubes.
visualization_msgs::Marker getCubesMarker(
    const std::vector<std::vector<double>>& points,
    double size,
    const std::vector<double>& color,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0);

visualization_msgs::Marker getCubesMarker(
    const std::vector<geometry_msgs::Point>& points,
    double size,
    const std_msgs::ColorRGBA& color,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0);

/// \brief Return a visualization marker representing a set of cubes.
visualization_msgs::MarkerArray getCubesMarkerArray(
    const std::vector<std::vector<double>>& poses,
    double size,
    const std::vector<std::vector<double>>& color,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0);

/// \brief Return a visualization marker array representing a pose.
///
/// The pose is represented as a sphere located at the pose origin with an arrow
/// pointing along the x-axis. An optional text marker above the pose displaying
/// the namespace via \p may be requested.
visualization_msgs::MarkerArray getPoseMarkerArray(
    const geometry_msgs::Pose& pose,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0,
    bool text = false);

/// \brief Return a visualization marker array representing a pose.
visualization_msgs::MarkerArray getPoseMarkerArray(
    const geometry_msgs::PoseStamped &pose,
    const std::string& ns,
    int id = 0,
    bool text = false);

/// \brief Return a visualization marker array representing a pose.
visualization_msgs::MarkerArray getPosesMarkerArray(
    const std::vector<std::vector<double>>& poses,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0,
    bool text = false);

/// \brief Return a visualization marker array representing a pose.
visualization_msgs::MarkerArray getPosesMarkerArray(
    const std::vector<geometry_msgs::Pose>& poses,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0,
    bool text = false);

visualization_msgs::MarkerArray getFrameMarkerArray(
    const Eigen::Affine3d& pose,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0);

/* Spheres */
visualization_msgs::Marker getSphereMarker(
    double x, double y, double z,
    double radius,
    int hue,
    std::string frame_id,
    std::string ns,
    int id = 0);

visualization_msgs::Marker getSpheresMarker(
    const std::vector<std::vector<double>>& poses,
    double radius,
    int hue,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0);

visualization_msgs::Marker getSpheresMarker(
    const std::vector<geometry_msgs::Point>& poses,
    double radius,
    int hue,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0);

visualization_msgs::MarkerArray getSpheresMarkerArray(
    const std::vector<std::vector<double> > &pose,
    std::vector<double> &radius,
    int hue,
    const std::string &frame_id,
    const std::string &ns,
    int id = 0);

visualization_msgs::MarkerArray getSpheresMarkerArray(
    const std::vector<std::vector<double> > &pose,
    const std::vector<int> &hue,
    std::string frame_id,
    std::string ns,
    int id = 0);

/* Line */
visualization_msgs::Marker getLineMarker(
    const std::vector<geometry_msgs::Point>& points,
    double thickness,
    int hue,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0);

/* Text */
visualization_msgs::Marker getTextMarker(
    const geometry_msgs::Pose& pose,
    const std::string& text,
    double size,
    int hue,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0);

visualization_msgs::Marker getTextMarker(
    const geometry_msgs::Pose& pose,
    const std::string& text,
    double size,
    const std::vector<double>& color,
    const std::string& frame_id,
    const std::string& ns,
    int id = 0);

/* Meshes */
visualization_msgs::Marker getMeshMarker(
    const geometry_msgs::PoseStamped& pose,
    const std::string& mesh_resource,
    int hue,
    std::string ns,
    int id = 0);

visualization_msgs::Marker getMeshMarker(
    const geometry_msgs::PoseStamped &pose,
    const std::vector<geometry_msgs::Point> &vertices,
    const std::vector<int> &triangles,
    int hue,
    bool psychadelic,
    std::string ns,
    int id = 0);

/* Collision Objects */
visualization_msgs::MarkerArray getShapesMarkerArray(
    const std::vector<shape_msgs::SolidPrimitive> &shapes,
    const std::vector<geometry_msgs::Pose> &poses,
    const std::vector<std::vector<double> >&color,
    std::string frame_id,
    std::string ns,
    int id);

visualization_msgs::MarkerArray getCollisionObjectMarkerArray(
    const moveit_msgs::CollisionObject &obj,
    const std::vector<double> &hue,
    std::string ns,
    int id);

/* Removal */
visualization_msgs::MarkerArray getRemoveMarkerArray(
    std::string ns,
    int max_id);

} // namespace viz

#endif
