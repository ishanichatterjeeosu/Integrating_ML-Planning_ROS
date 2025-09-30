/* \author Benjamin Cohen */

#include <leatherman/viz.h>

// system includes
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

// project includes
#include <leatherman/utils.h>

namespace viz {

visualization_msgs::Marker getCubeMarker(
    const geometry_msgs::PoseStamped& pose,
    const std::vector<double>& dim,
    int hue,
    const std::string& ns,
    int id)
{
    std::vector<double> color(4, 1.0);
    leatherman::HSVtoRGB(&(color[0]), &(color[1]), &(color[2]), hue, 1.0, 1.0);
    return getCubeMarker(pose.pose, dim, color, pose.header.frame_id, ns, id);
}

visualization_msgs::Marker getCubeMarker(
    const std::vector<double>& cube,
    const std::vector<double>& color,
    const std::string& frame_id,
    const std::string& ns,
    int id)
{
    if (cube.size() < 6) {
        ROS_ERROR("Expecting a cube vector with 6 values. { x, y, z, dimx, dimy, dimz }");
        return visualization_msgs::Marker();
    }

    std::vector<double> dim(3, 0.0);
    dim[0] = cube[3];
    dim[1] = cube[4];
    dim[2] = cube[5];

    geometry_msgs::Pose pose;
    pose.position.x = cube[0];
    pose.position.y = cube[1];
    pose.position.z = cube[2];
    pose.orientation.w = 1.0;
    return getCubeMarker(pose, dim, color, frame_id, ns, id);
}

visualization_msgs::Marker getCubeMarker(
    const geometry_msgs::Pose& pose,
    const std::vector<double>& dim,
    const std::vector<double>& color,
    const std::string& frame_id,
    const std::string& ns,
    int id)
{
    visualization_msgs::Marker marker;

    if (dim.size() < 3) {
        ROS_INFO("Three dimensions are needed to visualize a cube.");
        return marker;
    }

    if (color.size() < 4) {
        ROS_ERROR("No color specified.");
        return marker;
    }

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = dim[0];
    marker.scale.y = dim[1];
    marker.scale.z = dim[2];
    marker.color.r = color[0] > 1 ? color[0] / 255.0f : color[0];
    marker.color.g = color[1] > 1 ? color[1] / 255.0f : color[1];
    marker.color.b = color[2] > 1 ? color[2] / 255.0f : color[2];
    marker.color.a = color[3] > 1 ? color[3] / 255.0f : color[3];
    marker.lifetime = ros::Duration(0);
    return marker;
}

visualization_msgs::Marker getCubesMarker(
    const std::vector<std::vector<double>>& points,
    double size,
    const std::vector<double>& color,
    const std::string& frame_id,
    const std::string& ns,
    int id)
{
    if (color.size() < 4) {
        ROS_WARN("color has insufficient fields (expected: 5, actual: %zu)", color.size());
        return visualization_msgs::Marker();
    }

    std_msgs::ColorRGBA c;
    c.r = color[0];
    c.g = color[1];
    c.b = color[2];
    c.a = color[3];

    std::vector<geometry_msgs::Point> pts;
    pts.reserve(points.size());
    for (const auto& point : points) {
        if (points.size() >= 3) {
            geometry_msgs::Point pt;
            pt.x = point[0];
            pt.y = point[1];
            pt.z = point[2];
        }
        else {
            ROS_WARN("A cube pose was received with fewer than 3 values { x, y, z }");
        }
    }

    return getCubesMarker(pts, size, c, frame_id, ns, id);
}

visualization_msgs::Marker getCubesMarker(
    const std::vector<geometry_msgs::Point>& points,
    double size,
    const std_msgs::ColorRGBA& color,
    const std::string& frame_id,
    const std::string& ns,
    int id)
{
    visualization_msgs::Marker marker;

    marker.header.seq = 0;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;
    marker.color.r = color.r > 1.0 ? color.r / 255.0f : color.r;
    marker.color.g = color.g > 1.0 ? color.g / 255.0f : color.g;
    marker.color.b = color.b > 1.0 ? color.b / 255.0f : color.b;
    marker.color.a = color.a > 1.0 ? color.a / 255.0f : color.a;
    marker.lifetime = ros::Duration(0);
    marker.frame_locked = false;
    marker.points = points;

    return marker;
}

visualization_msgs::MarkerArray getCubesMarkerArray(
    const std::vector<std::vector<double>>& poses,
    double scale,
    const std::vector<std::vector<double>>& color,
    const std::string& frame_id,
    const std::string& ns,
    int id)
{
    if (color.size() < 2) {
        ROS_INFO("Not enough colors specified. Expecting two colors {2x4}.");
        return visualization_msgs::MarkerArray();
    }

    if (color[0].size() < 4 || color[1].size() < 4) {
        ROS_INFO("RGBA must be specified for each color.");
        return visualization_msgs::MarkerArray();
    }

    visualization_msgs::Marker marker;

    marker.header.seq = 0;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action =  visualization_msgs::Marker::ADD;
    // marker.pose;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    // marker.color;
    marker.lifetime = ros::Duration(0);
    marker.frame_locked = false;

    std::vector<geometry_msgs::Point> points;
    std::vector<std_msgs::ColorRGBA> colors;

    points.reserve(poses.size());
    colors.reserve(poses.size());

    for (size_t i = 0; i < poses.size(); ++i) {
        geometry_msgs::Point p;
        p.x = poses[i][0];
        p.y = poses[i][1];
        p.z = poses[i][2];

        std_msgs::ColorRGBA c;

        // interpolate color
        const double alpha = ((double)i) / ((double)poses.size());
        c.r = color[0][0] - (color[0][0] - color[1][0]) * alpha;
        c.g = color[0][1] - (color[0][1] - color[1][1]) * alpha;
        c.b = color[0][2] - (color[0][2] - color[1][2]) * alpha;
        c.a = color[0][3] - (color[0][3] - color[1][3]) * alpha;

        points.push_back(p);
        colors.push_back(c);
    }

    marker.points = std::move(points);
    marker.colors = std::move(colors);

    visualization_msgs::MarkerArray ma;
    ma.markers.push_back(marker);
    return ma;
}


visualization_msgs::MarkerArray getPosesMarkerArray(
    const std::vector<std::vector<double>>& poses,
    const std::string& frame_id,
    const std::string& ns,
    int id,
    bool text)
{
    std::vector<geometry_msgs::Pose> geo_poses;
    geo_poses.resize(poses.size());
    for (size_t i = 0; i < poses.size(); ++i) {
        geometry_msgs::Pose p;
        p.position.x = poses[i][0];
        p.position.y = poses[i][1];
        p.position.z = poses[i][2];

        tf::Quaternion pose_quaternion;
        pose_quaternion.setRPY(poses[i][3],poses[i][4],poses[i][5]);
        tf::quaternionTFToMsg(pose_quaternion, p.orientation);

        geo_poses[i] = p;
    }

    return getPosesMarkerArray(geo_poses, frame_id, ns, id, text);
}

visualization_msgs::MarkerArray getPosesMarkerArray(
    const std::vector<geometry_msgs::Pose>& poses,
    const std::string& frame_id,
    const std::string& ns,
    int id,
    bool text)
{
    // aggregate markers for each pose
    visualization_msgs::MarkerArray ma;
    int cid = id;
    for (const geometry_msgs::Pose& pose : poses) {
        visualization_msgs::MarkerArray ma_tmp =
                getPoseMarkerArray(pose, frame_id, ns, cid);
        ma.markers.insert(
                ma.markers.end(), ma_tmp.markers.begin(), ma_tmp.markers.end());
        cid = id + (int)ma.markers.size();
    }

    // add numbering to text field if applicable
    if (text) {
        int c = 0;
        for (visualization_msgs::Marker& m : ma.markers) {
            if (!m.text.empty()) {
                m.text += std::to_string(c++);
            }
        }
    }

    return ma;
}

visualization_msgs::MarkerArray getFrameMarkerArray(
    const Eigen::Affine3d& pose,
    const std::string& frame_id,
    const std::string& ns,
    int id)
{
    visualization_msgs::MarkerArray ma;

    visualization_msgs::Marker m;

    Eigen::Quaterniond q(pose.rotation());

    m.header.frame_id = frame_id;
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = pose.translation()[0];
    m.pose.position.y = pose.translation()[1];
    m.pose.position.z = pose.translation()[2];
    m.pose.orientation.w = q.w();
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.scale.x = 0.1;
    m.scale.y = m.scale.z = 0.01;
    m.color.r = m.color.a = 1.0;
    m.color.g = m.color.b = 0.0;
    m.lifetime = ros::Duration(0.0);
    ma.markers.push_back(m);

    q = q * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ());
    ++m.id;
    m.pose.orientation.w = q.w();
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.color.r = 0.0;
    m.color.g = 1.0;
    ma.markers.push_back(m);

    q = q * Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitY());
    ++m.id;
    m.pose.orientation.w = q.w();
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.color.g = 0.0;
    m.color.b = 1.0;
    ma.markers.push_back(m);

    return ma;
}

visualization_msgs::MarkerArray getPoseMarkerArray(
    const geometry_msgs::Pose& pose,
    const std::string& frame_id,
    const std::string& ns,
    int id,
    bool text)
{
    visualization_msgs::MarkerArray ma;
    ma.markers.resize(text ? 3 : 2);
    ros::Time time = ros::Time::now();

    ma.markers[0].header.stamp = time;
    ma.markers[0].header.frame_id = frame_id;
    ma.markers[0].ns = ns;
    ma.markers[0].type = visualization_msgs::Marker::ARROW;
    ma.markers[0].id = 0+id;
    ma.markers[0].action = visualization_msgs::Marker::ADD;
    ma.markers[0].pose = pose;
    ma.markers[0].scale.x = 0.1;
    ma.markers[0].scale.y = 0.015;
    ma.markers[0].scale.z = 0.015;
    ma.markers[0].color.r = 0.0;
    ma.markers[0].color.g = 0.7;
    ma.markers[0].color.b = 0.6;
    ma.markers[0].color.a = 0.7;
    ma.markers[0].lifetime = ros::Duration(0);

    ma.markers[1].header.stamp = time;
    ma.markers[1].header.frame_id = frame_id;
    ma.markers[1].ns = ns;
    ma.markers[1].id = 1+id;
    ma.markers[1].type = visualization_msgs::Marker::SPHERE;
    ma.markers[1].action = visualization_msgs::Marker::ADD;
    ma.markers[1].pose = pose;
    ma.markers[1].scale.x = 0.07;
    ma.markers[1].scale.y = 0.07;
    ma.markers[1].scale.z = 0.10;
    ma.markers[1].color.r = 1.0;
    ma.markers[1].color.g = 0.0;
    ma.markers[1].color.b = 0.6;
    ma.markers[1].color.a = 0.6;
    ma.markers[1].lifetime = ros::Duration(0);

    if (text) {
        ma.markers[2].header.stamp = time;
        ma.markers[2].header.frame_id = frame_id;
        ma.markers[2].ns = ns;
        ma.markers[2].id = 2+id;
        ma.markers[2].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        ma.markers[2].action = visualization_msgs::Marker::ADD;
        ma.markers[2].pose = pose;
        ma.markers[2].pose.position.z += 0.05;
        ma.markers[2].scale.x = 0.03;
        ma.markers[2].scale.y = 0.03;
        ma.markers[2].scale.z = 0.03;
        ma.markers[2].color.r = 1.0;
        ma.markers[2].color.g = 1.0;
        ma.markers[2].color.b = 1.0;
        ma.markers[2].color.a = 0.9;
        ma.markers[2].text = ns;
        ma.markers[2].lifetime = ros::Duration(0);
    }

    return ma;
}

visualization_msgs::MarkerArray getPoseMarkerArray(
    const geometry_msgs::PoseStamped& pose,
    const std::string& ns,
    int id,
    bool text)
{
    return getPoseMarkerArray(pose.pose, pose.header.frame_id, ns, id, false);
}

visualization_msgs::Marker getSphereMarker(
    double x,
    double y,
    double z,
    double radius,
    int hue,
    std::string frame_id,
    std::string ns,
    int id)
{
    double r = 0, g = 0, b = 0;
    visualization_msgs::Marker marker;
    leatherman::HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = radius * 2.0f;
    marker.scale.y = radius * 2.0f;
    marker.scale.z = radius * 2.0f;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0);
    return marker;
}

visualization_msgs::Marker getSpheresMarker(
    const std::vector<std::vector<double>>& poses,
    double radius,
    int hue,
    const std::string& frame_id,
    const std::string& ns,
    int id)
{
    std::vector<geometry_msgs::Point> centers;
    centers.reserve(poses.size());
    for (const auto& pose : poses) {
        if (pose.size() < 3) {
            continue;
        }
        geometry_msgs::Point p;
        p.x = pose[0];
        p.y = pose[1];
        p.z = pose[2];
        centers.push_back(p);
    }
    return getSpheresMarker(centers, radius, hue, frame_id, ns, id);
}

visualization_msgs::Marker getSpheresMarker(
    const std::vector<geometry_msgs::Point>& poses,
    double radius,
    int hue,
    const std::string& frame_id,
    const std::string& ns,
    int id)
{
    double r = 0, g = 0, b = 0;
    visualization_msgs::Marker marker;
    leatherman::HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time();
    marker.header.seq = 0;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = 0;
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = radius;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.8;
    marker.lifetime = ros::Duration(0);
    marker.points = poses;
    return marker;
}

visualization_msgs::MarkerArray getSpheresMarkerArray(
    const std::vector<std::vector<double> > &pose,
    std::vector<double> &radius,
    int hue,
    const std::string &frame_id,
    const std::string &ns,
    int id)
{
    double r = 0, g = 0, b = 0;
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray marker_array;
    leatherman::HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id;
    marker.ns = ns;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.6;
    marker.lifetime = ros::Duration(0);

    for (size_t i = 0; i < pose.size(); ++i) {
        marker.id = id+i;
        marker.scale.x = radius[i]*2.0;
        marker.scale.y = radius[i]*2.0;
        marker.scale.z = radius[i]*2.0;
        marker.pose.position.x = pose[i][0];
        marker.pose.position.y = pose[i][1];
        marker.pose.position.z = pose[i][2];
        marker.pose.orientation.w = 1.0;
        marker_array.markers.push_back(marker);
    }
    return marker_array;
}

visualization_msgs::MarkerArray getSpheresMarkerArray(
    const std::vector<std::vector<double> > &pose,
    const std::vector<int> &hue,
    std::string frame_id,
    std::string ns,
    int id)
{
    double r=0,g=0,b=0;
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray marker_array;

    if(pose.size() != hue.size())
    {
        ROS_WARN("Didn't receive as many colors as I did spheres. Not visualizing. (spheres: %d, colors: %d)", int(pose.size()), int(hue.size()));
        return marker_array;
    }

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id;
    marker.ns = ns;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0);
    marker.color.a = 0.6;

    for(std::size_t i = 0; i < pose.size(); ++i)
    {
        leatherman::HSVtoRGB(&r, &g, &b, hue[i], 1.0, 1.0);
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.id = id+i;
        marker.scale.x = pose[i][3]*2.0;
        marker.scale.y = pose[i][3]*2.0;
        marker.scale.z = pose[i][3]*2.0;
        marker.pose.position.x = pose[i][0];
        marker.pose.position.y = pose[i][1];
        marker.pose.position.z = pose[i][2];
        marker_array.markers.push_back(marker);
    }
    return marker_array;
}

visualization_msgs::MarkerArray getRemoveMarkerArray(std::string ns, int max_id)
{
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray marker_array;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.action = visualization_msgs::Marker::DELETE;
  marker_array.markers.resize(max_id,marker);
  for(int j = 0; j < max_id; ++j)
    marker_array.markers[j].id = j;
  return marker_array;
}

visualization_msgs::Marker getLineMarker(
    const std::vector<geometry_msgs::Point>& points,
    double thickness,
    int hue,
    const std::string& frame_id,
    const std::string& ns,
    int id)
{
    double r = 0.0, g = 0.0, b = 0.0;
    visualization_msgs::Marker marker;
    leatherman::HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.points = points;
    marker.scale.x = thickness;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.5;
    marker.lifetime = ros::Duration(0);
    return marker;
}

visualization_msgs::Marker getTextMarker(
    const geometry_msgs::Pose& pose,
    const std::string& text,
    double size,
    int hue,
    const std::string& frame_id,
    const std::string& ns,
    int id)
{
    std::vector<double> color(4, 1.0);
    leatherman::HSVtoRGB(&(color[0]), &(color[1]), &(color[2]), hue, 1.0, 1.0);

    return getTextMarker(pose, text, size, color, frame_id, ns, id);
}

visualization_msgs::Marker getTextMarker(
    const geometry_msgs::Pose& pose,
    const std::string& text,
    double size,
    const std::vector<double>& color,
    const std::string& frame_id,
    const std::string& ns,
    int id)
{
    visualization_msgs::Marker marker;

    if (color.size() < 4) {
        ROS_ERROR("No color specified");
        return marker;
    }

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;
    marker.pose = pose;

    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];
    marker.text = text;
    marker.lifetime = ros::Duration(0);
    return marker;
}

visualization_msgs::Marker getMeshMarker(const geometry_msgs::PoseStamped &pose, const std::string &mesh_resource, int hue, std::string ns, int id)
{
	double r = 0.0, g = 0.0, b = 0.0;
  if(hue > 0)
  	leatherman::HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

	visualization_msgs::Marker marker;
	marker.header.frame_id = pose.header.frame_id;
	marker.header.stamp = ros::Time::now();
	marker.ns = ns;
	marker.id = id;
	marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = pose.pose.position.x;
	marker.pose.position.y = pose.pose.position.y;
	marker.pose.position.z = pose.pose.position.z;
	marker.pose.orientation.x = pose.pose.orientation.x;
	marker.pose.orientation.y = pose.pose.orientation.y;
	marker.pose.orientation.z = pose.pose.orientation.z;
	marker.pose.orientation.w = pose.pose.orientation.w;
	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = 1.0;
	marker.mesh_resource = mesh_resource;

  if(hue < 0)
    marker.mesh_use_embedded_materials = true;

  return marker;
}

visualization_msgs::Marker getMeshMarker(const geometry_msgs::PoseStamped &pose, const std::vector<geometry_msgs::Point> &vertices, const std::vector<int> &triangles, int hue, bool psychadelic, std::string ns, int id)
{
	double r = 0.0, g = 0.0, b = 0.0;
	leatherman::HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

	std_msgs::ColorRGBA red; red.a = 1.0f; red.r = 1.0f; red.g = 0.0f; red.b = 0.0f;
	std_msgs::ColorRGBA green; green.a = 1.0f; green.r = 0.0f; green.g = 1.0f; green.b = 0.0f;
	std_msgs::ColorRGBA blue; blue.a = 1.0f; blue.r = 0.0f; blue.g = 0.0f; blue.b = 1.0f;

	std::vector<std_msgs::ColorRGBA> colors;
	for (int i = 0; i < (int)vertices.size(); i++)
  {
		if (i % 3 == 0) colors.push_back(red);
		if (i % 3 == 1) colors.push_back(green);
		if (i % 3 == 2) colors.push_back(blue);
	}

	visualization_msgs::Marker marker;
	marker.header.frame_id = pose.header.frame_id;
	marker.header.stamp = ros::Time::now();
	marker.ns = ns;
	marker.id = id;
	marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = pose.pose.position.x;
	marker.pose.position.y = pose.pose.position.y;
	marker.pose.position.z = pose.pose.position.z;
	marker.pose.orientation.x = pose.pose.orientation.x;
	marker.pose.orientation.y = pose.pose.orientation.y;
	marker.pose.orientation.z = pose.pose.orientation.z;
	marker.pose.orientation.w = pose.pose.orientation.w;
	marker.scale.x = 1.0;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;
	marker.points = vertices;

	if (psychadelic)
  {
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 1.0;
		marker.color.a = 1.0;
		marker.colors = colors;
	}
	else
  {
		marker.color.r = r;
		marker.color.g = g;
		marker.color.b = b;
		marker.color.a = 1.0;
	}
	return marker;
}

visualization_msgs::MarkerArray getShapesMarkerArray(
    const std::vector<shape_msgs::SolidPrimitive>& shapes,
    const std::vector<geometry_msgs::Pose>& poses,
    const std::vector<std::vector<double>>& color,
    std::string frame_id,
    std::string ns,
    int id)
{
    visualization_msgs::Marker m;
    visualization_msgs::MarkerArray ma;

    if (poses.size() != shapes.size()) {
        ROS_WARN("The shapes and poses arrays must be of the same length. (shapes: %zu, poses: %zu)", shapes.size(), poses.size());
        return ma;
    }

    m.header.frame_id = frame_id;
    m.header.stamp = ros::Time::now();
    m.ns = ns;
    m.action = visualization_msgs::Marker::ADD;
    m.lifetime = ros::Duration(0);

    for (std::size_t i = 0; i < shapes.size(); ++i) {
        m.id = id + i;
        m.color.r = color[i][0];
        m.color.g = color[i][1];
        m.color.b = color[i][2];
        m.color.a = color[i][3];
        m.pose = poses[i];

        if (shapes[i].type == shape_msgs::SolidPrimitive::BOX) {
            m.type = visualization_msgs::Marker::CUBE;
            m.scale.x = shapes[i].dimensions[0];
            m.scale.y = shapes[i].dimensions[1];
            m.scale.z = shapes[i].dimensions[2];
            ma.markers.push_back(m);
        }
        else if (shapes[i].type == shape_msgs::SolidPrimitive::SPHERE) {
            m.type = visualization_msgs::Marker::SPHERE;
            m.scale.x = shapes[i].dimensions[0];
            m.scale.y = shapes[i].dimensions[0];
            m.scale.z = shapes[i].dimensions[0];
            ma.markers.push_back(m);
        }
        else if (shapes[i].type == shape_msgs::SolidPrimitive::CYLINDER) {
            m.type = visualization_msgs::Marker::CYLINDER;
            m.scale.x = shapes[i].dimensions[0];
            m.scale.y = shapes[i].dimensions[0];
            m.scale.z = shapes[i].dimensions[1];
            ma.markers.push_back(m);
        }
        else {
            ROS_ERROR("Shapes of type '%u' are not supported yet.", shapes[i].type);
        }
    }
    return ma;
}

visualization_msgs::MarkerArray getCollisionObjectMarkerArray(
    const moveit_msgs::CollisionObject& obj,
    const std::vector<double>& hue,
    std::string ns,
    int id)
{
    std::vector<std::vector<double>> colors(hue.size(), std::vector<double>(4, 1.0));
    for (size_t i = 0; i < colors.size(); ++i) {
        leatherman::HSVtoRGB(
                &(colors[i][0]), &(colors[i][1]), &(colors[i][2]),
                hue[i], 1.0, 1.0);
    }

    return getShapesMarkerArray(
            obj.primitives,
            obj.primitive_poses,
            colors,
            obj.header.frame_id,
            ns,
            id);
}

} // namespace viz
