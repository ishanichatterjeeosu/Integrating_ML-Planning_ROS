#include <pedsim_simulator/common.h>
#include <cmath>
#include <vector>

float CommonConstants::getheur2D(int x, int y, int goal_x, int goal_y)
{
    // euclidean for large env
    return (sqrt((goal_x - x) * (goal_x - x) + (goal_y - y) * (goal_y - y)));
    // Manhattan for small env
    //return (abs(goal_x - x) + abs(goal_y - y));
    //return abs(goal_x -x) + abs(goal_y -y); // grid coordinates
}

int CommonConstants::getHashKey(int numcells_x, int gx, int gy)
{

    return (numcells_x * gy + gx);
}

int CommonConstants::getPredMotPrims(xy_planner_state *s, int act, std::vector<int> &dx_, std::vector<int> &dy_, xy_planner_state *s_pred)
{

    /////////////////////////// Add checks for out of bounds, collision checker, dynamic collision checker!!!!!!!!!!!!!!!!!!! /////////////////////////////////
    // ##############################################################3!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    int dx = 0;
    int dy = 0;
    float c = 0;

    dx += dx_[act];
    dy += dy_[act];
    c = c + sc * sqrt(dx_[act] * dx_[act] + dy_[act] * dy_[act]);

    s_pred->x = s->x - dx;
    s_pred->y = s->y - dy;

    return floor(c);
}

void CommonConstants::visualizeMarkerOpt(double x, double y, int id, int shape, visualization_msgs::MarkerArray &rob_cyl)
{

    visualization_msgs::Marker rob1;
    rob1.header.frame_id = "odom";
    rob1.header.stamp = ros::Time();
    rob1.id = id;
    //id = id+1;
    rob1.type = shape; // 1 for cube
    rob1.action = visualization_msgs::Marker::ADD;
    rob1.pose.position.x = x;
    rob1.pose.position.y = y;
    rob1.pose.position.z = 0;
    rob1.pose.orientation.x = 0;
    rob1.pose.orientation.y = 0;
    rob1.pose.orientation.z = 0;
    rob1.pose.orientation.w = 1;
    rob1.scale.x = 1.2;
    rob1.scale.y = 1.2;
    rob1.scale.z = 1.2;
    rob1.color.a = 1.0; // Don't forget to set the alpha!
    rob1.color.r = 1.0;
    rob1.color.g = 0.6;
    rob1.color.b = 0.0;

    rob_cyl.markers.push_back(rob1);
}