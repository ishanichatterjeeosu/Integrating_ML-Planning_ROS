////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2015, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
// may be used to endorse or promote products derived from this software without
// specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

#include "collision_env_sbpl.h"
// standard includes
#include <stdexcept>

#include <ros/ros.h>
#include <geometric_shapes/shape_operations.h>
#include <smpl/debug/visualize.h>

// module includes
#include "collision_common_sbpl.h"

namespace collision_detection {

// crp = collision robot plugin
static const char* CRP_LOGGER = "self_and_world_collisions";
static const char* LOG = "world_collisions";

static
auto MakeCollisionRobotVisualization(
    smpl::collision::RobotCollisionState* rcs,
    smpl::collision::AttachedBodiesCollisionState* abcs,
    int gidx,
    const std_msgs::ColorRGBA* color,
    const std::string* frame_id,
    const std::string* ns)
    -> visualization_msgs::MarkerArray
{
    auto ma = GetCollisionMarkers(*rcs, *abcs, gidx);
    for (auto& m : ma.markers) {
        m.ns = *ns;
        m.header.frame_id = *frame_id;
        m.color = *color;
    }
    return ma;
}

static
auto MakeCollisionRobotVisualization(
    const CollisionEnvSBPL* crobot,
    smpl::collision::RobotCollisionState* rcs,
    smpl::collision::AttachedBodiesCollisionState* abcs,
    int gidx,
    const std_msgs::ColorRGBA* color)
    -> visualization_msgs::MarkerArray
{
    auto* frame_id = &crobot->m_rcm->modelFrame();
    std::string ns("self_collision");
    return MakeCollisionRobotVisualization(
            rcs, abcs, gidx, color, frame_id, &ns);
}

static
auto MakeCollisionRobotValidityVisualization(
    const CollisionEnvSBPL* crobot,
    smpl::collision::RobotCollisionState* rcs,
    smpl::collision::AttachedBodiesCollisionState* abcs,
    int gidx,
    bool valid)
    -> visualization_msgs::MarkerArray
{
    std_msgs::ColorRGBA color;
    if (valid) {
        color.g = 1.0;
        color.r = color.b = 0.0;
        color.a = 1.0;
    } else {
        color.r = 1.0;
        color.g = color.b = 0.0;
        color.a = 1.0;
    }
    return MakeCollisionRobotVisualization(crobot, rcs, abcs, gidx, &color);
}

static
auto ComputeWorldAABB(const World& world) -> moveit_msgs::OrientedBoundingBox
{
    moveit_msgs::OrientedBoundingBox bb;
    bb.pose.orientation.w = 1.0;
    bb.pose.orientation.x = 0.0;
    bb.pose.orientation.y = 0.0;
    bb.pose.orientation.z = 0.0;

    geometry_msgs::Point min_pt;
    min_pt.x = std::numeric_limits<double>::max();
    min_pt.y = std::numeric_limits<double>::max();
    min_pt.z = std::numeric_limits<double>::max();
    geometry_msgs::Point max_pt;
    max_pt.x = std::numeric_limits<double>::lowest();
    max_pt.y = std::numeric_limits<double>::lowest();
    max_pt.z = std::numeric_limits<double>::lowest();

    if (world.size() == 0) {
        bb.pose.position.x = bb.pose.position.y = bb.pose.position.z = 0.0;
        bb.extents.x = bb.extents.y = bb.extents.z = 0.0;
        return bb;
    }

    for (auto oit = world.begin(); oit != world.end(); ++oit) {
        auto& object = *oit->second;
        auto& object_id = object.id_;
        auto num_shapes = object.shapes_.size();
        ROS_DEBUG_NAMED(LOG, "%zu shapes in object", num_shapes);
        for (auto i = (size_t)0; i < num_shapes; ++i) {
            auto& pose = object.shape_poses_[i];
            auto shape = object.shapes_[i];
            auto extents = shapes::computeShapeExtents(shape.get());

            auto shape_min = Eigen::Vector3d(Eigen::Vector3d(pose.translation()) - 0.5 * extents);
            auto shape_max = Eigen::Vector3d(Eigen::Vector3d(pose.translation()) + 0.5 * extents);

            min_pt.x = std::min(min_pt.x, shape_min.x());
            min_pt.y = std::min(min_pt.y, shape_min.y());
            min_pt.z = std::min(min_pt.z, shape_min.z());
            max_pt.x = std::max(max_pt.x, shape_max.x());
            max_pt.y = std::max(max_pt.y, shape_max.y());
            max_pt.z = std::max(max_pt.z, shape_max.z());
        }
    }

    bb.pose.position.x = 0.5 * (min_pt.x + max_pt.x);
    bb.pose.position.y = 0.5 * (min_pt.y + max_pt.y);
    bb.pose.position.z = 0.5 * (min_pt.z + max_pt.z);
    bb.extents.x = max_pt.x - min_pt.x;
    bb.extents.y = max_pt.y - min_pt.y;
    bb.extents.z = max_pt.z - min_pt.z;
    return bb;
}

static
bool IsEmptyBoundingBox(const moveit_msgs::OrientedBoundingBox& bb)
{
    return bb.extents.x == 0.0 && bb.extents.y == 0.0 && bb.extents.z == 0.0;
}

static
auto MakeGrid(const CollisionGridConfig& config) -> smpl::OccupancyGridPtr
{
    ROS_DEBUG_NAMED(LOG, "  Creating Distance Field");
    ROS_DEBUG_NAMED(LOG, "    size: (%0.3f, %0.3f, %0.3f)", config.size_x, config.size_y, config.size_z);
    ROS_DEBUG_NAMED(LOG, "    origin: (%0.3f, %0.3f, %0.3f)", config.origin_x, config.origin_y, config.origin_z);
    ROS_DEBUG_NAMED(LOG, "    resolution: %0.3f", config.res_m);
    ROS_DEBUG_NAMED(LOG, "    max_distance: %0.3f", config.max_distance_m);

    auto ref_counted = true;

    auto dmap = std::make_shared<smpl::OccupancyGrid>(
            config.size_x,
            config.size_y,
            config.size_z,
            config.res_m,
            config.origin_x,
            config.origin_y,
            config.origin_z,
            config.max_distance_m,
            ref_counted);
    dmap->setReferenceFrame(config.frame_id);
    return dmap;
}

CollisionEnvSBPL::CollisionEnvSBPL(
    const robot_model::RobotModelConstPtr& model,
    double padding,
    double scale)
:
    CollisionEnv(model, padding, scale)
{
    ROS_INFO_NAMED(CRP_LOGGER, "CollisionEnvSBPL(const RobotModelConstPtr&, double, double)");
    ros::NodeHandle ph("~");

    // search for the robot collision model on the param server
    auto robot_collision_model_param = "robot_collision_model";
    std::string rcm_key;
    if (!ph.searchParam(robot_collision_model_param, rcm_key)) {
        std::stringstream ss;
        ss << "Failed to find '" << robot_collision_model_param <<
                "' key on the param server";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }

    // retrieve the robot collision model param from the param server
    XmlRpc::XmlRpcValue rcm_config;
    if (!ph.getParam(rcm_key, rcm_config)) {
        std::stringstream ss;
        ss << "Failed to retrieve '" << rcm_key << "' from the param server";
        ROS_ERROR_STREAM(ss.str());
        throw std::runtime_error(ss.str());
    }

    // load the robot collision model configuration
    smpl::collision::CollisionModelConfig cm_config;
    if (!smpl::collision::CollisionModelConfig::Load(rcm_config, cm_config)) {
        auto msg = "Failed to load Collision Model Config";
        ROS_ERROR_STREAM(msg);
        throw std::runtime_error(msg);
    }

    // build robot collision model from configuration
    auto rcm = smpl::collision::RobotCollisionModel::Load(
            *model->getURDF(), cm_config);
    if (!rcm) {
        auto msg = "Failed to build Robot Collision Model from config";
        ROS_ERROR_STREAM(msg);
        throw std::runtime_error(msg);
    }

    auto self_collision_model_param = "self_collision_model";
    LoadCollisionGridConfig(ph, self_collision_model_param, m_scm_config);

    LoadJointCollisionGroupMap(ph, m_jcgm_map);

    if (!m_updater.init(*model, rcm)) {
        auto msg = "Failed to initialize Collision State Updater";
        ROS_ERROR_NAMED(CRP_LOGGER, "%s", msg);
        throw std::runtime_error(msg);
    }

    // ok! store the robot collision model
    m_rcm = rcm;
    m_rmcm = std::make_shared<smpl::collision::RobotMotionCollisionModel>(m_rcm.get());

    ros::NodeHandle nh;
}

// CollisionRobotSBPL::CollisionRobotSBPL(const CollisionRobotSBPL& other) :
//     CollisionRobot(other)
// {
//     ROS_INFO_NAMED(CRP_LOGGER, "CollisionRobotSBPL(other)");
//     m_scm_config = other.m_scm_config;
//     m_jcgm_map = other.m_jcgm_map;
//     m_rcm = other.m_rcm;
//     m_rmcm = other.m_rmcm;
//     m_updater = other.m_updater;
// }

CollisionEnvSBPL::CollisionEnvSBPL(
    const CollisionEnvSBPL& other,
    const WorldPtr& world)
:
    CollisionEnv(other, world) // copies over the world
{
//    ROS_DEBUG_NAMED(LOG, "CollisionEnvSBPL(other = %p, world = %p)", &other, world.get());

    m_wcm_config = other.m_wcm_config;
    m_jcgm_map = other.m_jcgm_map;

    m_parent_grid = other.m_grid ? other.m_grid : other.m_parent_grid;
    m_parent_wcm = other.m_wcm ? other.m_wcm : other.m_parent_wcm;

    m_updaters = other.m_updaters;
    // NOTE: no need to copy observer handle
    registerWorldCallback();
    // NOTE: no need to copy node handle
}

CollisionEnvSBPL::~CollisionEnvSBPL()
{
    ROS_INFO_NAMED(CRP_LOGGER, "~CollisionEnvSBPL");
}

auto CollisionEnvSBPL::robotCollisionModel() const
    -> const smpl::collision::RobotCollisionModelConstPtr&
{
    return m_rcm;
}

auto CollisionEnvSBPL::robotMotionCollisionModel() const
    -> const smpl::collision::RobotMotionCollisionModelConstPtr&
{
    return m_rmcm;
}

// void CollisionRobotSBPL::checkOtherCollision(
//     const CollisionRequest& req,
//     CollisionResult& res,
//     const robot_state::RobotState& robot_state,
//     const CollisionRobot& other_robot,
//     const robot_state::RobotState& other_state) const
// {
//     // TODO: implement
//     setVacuousCollision(res);
// }

// void CollisionRobotSBPL::checkOtherCollision(
//     const CollisionRequest& req,
//     CollisionResult& res,
//     const robot_state::RobotState& robot_state,
//     const CollisionRobot& other_robot,
//     const robot_state::RobotState& other_state,
//     const AllowedCollisionMatrix& acm) const
// {
//     // TODO: implement
//     setVacuousCollision(res);
// }

// void CollisionRobotSBPL::checkOtherCollision(
//     const CollisionRequest& req,
//     CollisionResult& res,
//     const robot_state::RobotState& state1,
//     const robot_state::RobotState& state2,
//     const CollisionRobot& other_robot,
//     const robot_state::RobotState& other_state1,
//     const robot_state::RobotState& other_state2) const
// {
//     // TODO: implement
//     setVacuousCollision(res);
// }

// void CollisionRobotSBPL::checkOtherCollision(
//     const CollisionRequest& req,
//     CollisionResult& res,
//     const robot_state::RobotState& state1,
//     const robot_state::RobotState& state2,
//     const CollisionRobot& other_robot,
//     const robot_state::RobotState& other_state1,
//     const robot_state::RobotState& other_state2,
//     const AllowedCollisionMatrix& acm) const
// {
//     // TODO: implement
//     setVacuousCollision(res);
// }

void CollisionEnvSBPL::checkSelfCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& state) const
{
    // TODO: implement
    setVacuousCollision(res);
}

auto CollisionEnvSBPL::distanceField(
    const std::string& robot_name,
    const std::string& group_name) const
    -> const smpl::DistanceMapInterface*
{
    if (m_grid) {
        return m_grid->getDistanceField().get();
    } else if (m_parent_grid) {
        return m_parent_grid->getDistanceField().get();
    } else {
        return nullptr;
    }
}

void CollisionEnvSBPL::checkSelfCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& state,
    const AllowedCollisionMatrix& acm) const
{
    const_cast<CollisionEnvSBPL*>(this)->checkSelfCollisionMutable(
            req, res, state, acm);
}

void CollisionEnvSBPL::checkRobotCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& state) const
{
    ROS_INFO_NAMED(LOG, "checkRobotCollision(req, res, state)");

    const_cast<CollisionEnvSBPL*>(this)->checkRobotCollisionMutable(
            req, res, state);
}

void CollisionEnvSBPL::checkRobotCollision(
    const CollisionRequest& req,
    CollisionResult& res,
   
    const robot_state::RobotState& state,
    const AllowedCollisionMatrix& acm) const
{
    const_cast<CollisionEnvSBPL*>(this)->checkRobotCollisionMutable(
            req, res, state, acm);
}

void CollisionEnvSBPL::checkRobotCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& state1,
    const robot_state::RobotState& state2) const
{
    ROS_INFO_NAMED(LOG, "checkRobotCollision(req, res, state1, state2)");

    const_cast<CollisionEnvSBPL*>(this)->checkRobotCollisionMutable(
            req, res, state1, state2);
}

void CollisionEnvSBPL::checkRobotCollision(
    const CollisionRequest& req,
    CollisionResult& res,
    
    const robot_state::RobotState& state1,
    const robot_state::RobotState& state2,
    const AllowedCollisionMatrix& acm) const
{
    const_cast<CollisionEnvSBPL*>(this)->checkRobotCollisionMutable(
            req, res, state1, state2, acm);
}

// void CollisionRobotSBPL::checkSelfCollision(
//     const CollisionRequest& req,
//     CollisionResult& res,
//     const robot_state::RobotState& state1,
//     const robot_state::RobotState& state2) const
// {
//     // TODO: implement
//     setVacuousCollision(res);
// }

// void CollisionRobotSBPL::checkSelfCollision(
//     const CollisionRequest& req,
//     CollisionResult& res,
//     const robot_state::RobotState& state1,
//     const robot_state::RobotState& state2,
//     const AllowedCollisionMatrix& acm) const
// {
//     // TODO: implement
//     const_cast<CollisionRobotSBPL*>(this)->checkSelfCollisionMutable(
//             req, res, state1, state2, acm);
// }

//#if COLLISION_DETECTION_SBPL_ROS_VERSION == COLLISION_DETECTION_SBPL_ROS_KINETIC

void CollisionEnvSBPL::distanceSelf(
    const collision_detection::DistanceRequest& req,
    collision_detection::DistanceResult& res,
    const moveit::core::RobotState&) const
{
    assert(0);
}


void CollisionEnvSBPL::distanceRobot(
    const collision_detection::DistanceRequest& req,
    collision_detection::DistanceResult& res,
    const moveit::core::RobotState&) const
{
    // TODO: implement
    assert(0);
}

// void CollisionRobotSBPL::distanceOther(
//     const collision_detection::DistanceRequest& req,
//     collision_detection::DistanceResult& res,
//     const moveit::core::RobotState& state,
//     const collision_detection::CollisionRobot&,
//     const moveit::core::RobotState&) const
// {
//     assert(0);
// }

//#else // COLLISION_DETECTION_SBPL_ROS_VERSION == COLLISION_DETECTION_SBPL_ROS_INDIGO

// double CollisionRobotSBPL::distanceOther(
//     const robot_state::RobotState& state,
//     const CollisionRobot& other_robot,
//     const robot_state::RobotState& other_state) const
// {
//     // TODO: implement
//     return -1.0;
// }

// double CollisionRobotSBPL::distanceOther(
//     const robot_state::RobotState& state,
//     const CollisionRobot& other_robot,
//     const robot_state::RobotState& other_state,
//     const AllowedCollisionMatrix& acm) const
// {
//     // TODO: implement
//     return -1.0;
// }

// double CollisionRobotSBPL::distanceSelf(
//     const robot_state::RobotState& state) const
// {
//     // TODO: implement
//     return -1.0;
// }

// double CollisionRobotSBPL::distanceSelf(
//     const robot_state::RobotState& state,
//     const AllowedCollisionMatrix& acm) const
// {
//     // TODO: implement
//     return -1.0;
// }

//#endif

void CollisionEnvSBPL::setWorld(const WorldPtr& world)
{
    // deregister update callback (we should always have a callback registered
    // if we have a world)
    auto& curr_world = getWorld();
    if (curr_world) {
        curr_world->removeObserver(m_observer_handle);
    }

    CollisionEnv::setWorld(world);
    ROS_INFO_NAMED(LOG, "setWorld(const WorldPtr&)");

    registerWorldCallback();
}

void CollisionEnvSBPL::construct()
{
    ros::NodeHandle ph("~");

    auto world_collision_model_param = "world_collision_model";
    LoadCollisionGridConfig(ph, world_collision_model_param, m_wcm_config);

    LoadJointCollisionGroupMap(ph, m_jcgm_map);

    m_grid = MakeGrid(m_wcm_config);
    m_wcm = std::make_shared<smpl::collision::WorldCollisionModel>(m_grid.get());

    // TODO: allowed collisions matrix

    ROS_INFO("Sleep to allow publish to set up");
    ros::Duration(0.5).sleep();
    ROS_INFO("Done sleeping");

    // publish collision world visualizations
    SV_SHOW_INFO_NAMED("collision_world_bounds", m_grid->getBoundingBoxVisualization());
}

void CollisionEnvSBPL::copyOnWrite()
{
    if (!m_wcm) {
        ROS_DEBUG_NAMED(LOG, "Spawn derivative world collision model");
        assert(!m_grid);

        // create our own grid
        m_grid = MakeGrid(m_wcm_config);

        // copy over state from parent world collision model
        if (m_parent_wcm) {
            m_wcm = std::make_shared<smpl::collision::WorldCollisionModel>(
                    *m_parent_wcm, m_grid.get());

            m_parent_grid.reset();
            m_parent_wcm.reset();
        }
    }
}

auto CollisionEnvSBPL::FindObjectRepPair(
    const World::ObjectConstPtr& object)
    -> std::vector<ObjectRepPair>::iterator
{
    auto has_object = [&](const ObjectRepPair& op) {
        return op.world_object == object;
    };
    return std::find_if(
            begin(m_collision_objects), end(m_collision_objects), has_object);
}

auto CollisionEnvSBPL::getCollisionStateUpdater(
    const moveit::core::RobotModel& robot_model)
    -> CollisionStateUpdaterPtr
{
    // return an existing updater if available
    auto it = m_updaters.find(robot_model.getName());
    if (it != m_updaters.end()) {
        return it->second;
    }

    ROS_INFO_NAMED(LOG, "Create Collision State Updater for '%s'", robot_model.getName().c_str());

    auto gm = std::make_shared<CollisionStateUpdater>();
    if (!gm->init(robot_model, robotCollisionModel())) {
        return CollisionStateUpdaterPtr();
    }

    // store the successfully initialized group model
    m_updaters[robot_model.getName()] = gm;
    return gm;
}

void CollisionEnvSBPL::registerWorldCallback()
{
//    ROS_DEBUG_NAMED(LOG, "Registering world observer callback");
    auto ocfn = boost::bind(&CollisionEnvSBPL::worldUpdate, this, _1, _2);
    m_observer_handle = getWorld()->addObserver(ocfn);
}

void CollisionEnvSBPL::worldUpdate(
    const World::ObjectConstPtr& object,
    World::Action action)
{
    ROS_DEBUG_NAMED(LOG, "CollisionEnvSBPL::worldUpdate()");
    ROS_DEBUG_NAMED(LOG, "  id: %s", object->id_.c_str());
    ROS_DEBUG_NAMED(LOG, "  shapes: %zu", object->shapes_.size());
    ROS_DEBUG_NAMED(LOG, "  shape_poses: %zu", object->shape_poses_.size());
    if (action & World::ActionBits::UNINITIALIZED) {
        ROS_DEBUG_NAMED(LOG, "  action: UNINITIALIZED");
        processWorldUpdateUninitialized(object);
    }
    else if (action & World::ActionBits::CREATE) {
        ROS_DEBUG_NAMED(LOG, "  action: CREATE");
        processWorldUpdateCreate(object);
    }
    else if (action & World::ActionBits::DESTROY) {
        ROS_DEBUG_NAMED(LOG, "  action: DESTROY");
        processWorldUpdateDestroy(object);
    }
    else if (action & World::ActionBits::MOVE_SHAPE) {
        ROS_DEBUG_NAMED(LOG, "  action: MOVE_SHAPE");
        processWorldUpdateMoveShape(object);
    }
    else if (action & World::ActionBits::ADD_SHAPE) {
        ROS_DEBUG_NAMED(LOG, "  action: ADD_SHAPE");
        processWorldUpdateAddShape(object);
    }
    else if (action & World::ActionBits::REMOVE_SHAPE)  {
        ROS_DEBUG_NAMED(LOG, "  action: REMOVE_SHAPE");
        processWorldUpdateRemoveShape(object);
    }
}



void CollisionEnvSBPL::updatedPaddingOrScaling(
    const std::vector<std::string>& links)
{
    CollisionEnv::updatedPaddingOrScaling(links);
}

void CollisionEnvSBPL::setVacuousCollision(CollisionResult& res) const
{
    res.collision = true;
    res.contact_count = 0;
    res.contacts.clear();
    res.cost_sources.clear();
    res.distance = 0.0;
}


void CollisionEnvSBPL::checkRobotCollisionMutable(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& state)
{
    // TODO: implement
    ROS_ERROR_NAMED(LOG, "checkRobotCollision(req, res, state)");
    setVacuousCollision(res);
}

void CollisionEnvSBPL::checkRobotCollisionMutable(
    const CollisionRequest& req,
    CollisionResult& res,
  
    const robot_state::RobotState& state,
    const AllowedCollisionMatrix& acm)
{
    //auto& crobot = (const CollisionRobotSBPL&)env;
    auto& rcm = robotCollisionModel();
    if (state.getRobotModel()->getName() != rcm->name()) {
        ROS_ERROR_NAMED(LOG, "Collision Robot Model does not match Robot Model");
        setVacuousCollision(res);
        return;
    }

    auto gm = getCollisionStateUpdater(*state.getRobotModel());
    if (!gm) {
        ROS_ERROR_NAMED(LOG, "Failed to get Group Model for robot '%s', group '%s'", state.getRobotModel()->getName().c_str(), req.group_name.c_str());
        setVacuousCollision(res);
        return;
    }

    auto jgcgit = m_jcgm_map.find(req.group_name);
    auto& collision_group_name =
            jgcgit == end(m_jcgm_map) ?  req.group_name : jgcgit->second;

    if (!rcm->hasGroup(collision_group_name)) {
        ROS_ERROR_NAMED(LOG, "No group '%s' found in the Robot Collision Model", collision_group_name.c_str());
        setVacuousCollision(res);
        return;
    }

    auto gidx = rcm->groupIndex(collision_group_name);

    gm->update(state);

    assert(m_wcm != NULL || m_parent_wcm != NULL);
    auto ewcm = m_wcm != NULL ? m_wcm.get() : m_parent_wcm.get();

    smpl::collision::WorldCollisionDetector wcd(rcm.get(), ewcm);

    double dist;
    auto valid = wcd.checkCollision(
            *gm->collisionState(),
            *gm->attachedBodiesCollisionState(),
            gidx,
            dist);

    ROS_INFO_STREAM_COND_NAMED(req.verbose, LOG, "world valid: " << std::boolalpha << valid << ", dist: " << dist);
    ROS_DEBUG_STREAM_COND_NAMED(!req.verbose, LOG, "world valid: " << std::boolalpha << valid << ", dist: " << dist);

    // NOTE: Visualizations used to trigger on verbose requests, but we're
    // opting for a debug channel here for scenarios where there is no interface
    // to set the verbose flag. This is the case when using the provided
    // service call to query the collision detector.
    SV_SHOW_DEBUG_NAMED(
            "world_collision",
            MakeCollisionRobotValidityVisualization(
                    this,
                    gm->collisionState(),
                    gm->attachedBodiesCollisionState(),
                    gidx,
                    valid));

    res.collision = !valid;
    if (req.distance) {
        res.distance = std::min(res.distance, dist);
    }
    if (req.cost) {
        ROS_WARN_ONCE("Cost sources not computed by sbpl collision checker");
    }
    if (req.contacts) {
        ROS_WARN_ONCE("Contacts not computed by sbpl collision checker");
    }
}

void CollisionEnvSBPL::checkRobotCollisionMutable(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& state1,
    const robot_state::RobotState& state2)
{
    // TODO: implement
    ROS_ERROR_NAMED(LOG, "checkRobotCollision(req, res, state1, state2)");
    setVacuousCollision(res);
}

void CollisionEnvSBPL::checkRobotCollisionMutable(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& state1,
    const robot_state::RobotState& state2,
    const AllowedCollisionMatrix& acm)
{
    //auto& crobot = (const CollisionRobotSBPL&)env;
    auto& rcm = robotCollisionModel();
    auto& rmcm = robotMotionCollisionModel();

    assert(state1.getRobotModel()->getName() == rcm->name());
    assert(state2.getRobotModel()->getName() == rcm->name());

    auto gm = getCollisionStateUpdater(*state1.getRobotModel());
    if (!gm) {
        ROS_ERROR_NAMED(LOG, "Failed to get Group Model for robot '%s', group '%s'", state1.getRobotModel()->getName().c_str(), req.group_name.c_str());
        setVacuousCollision(res);
        return;
    }

    auto jgcgit = m_jcgm_map.find(req.group_name);
    auto& collision_group_name =
            jgcgit == m_jcgm_map.end() ? req.group_name : jgcgit->second;

    if (!rcm->hasGroup(collision_group_name)) {
        ROS_ERROR_NAMED(LOG, "No group '%s' found in the Robot Collision Model", collision_group_name.c_str());
        setVacuousCollision(res);
        return;
    }

    int gidx = rcm->groupIndex(collision_group_name);

//    gm->update(state1);

    assert(m_wcm != NULL || m_parent_wcm != NULL);
    auto ewcm = (m_wcm != NULL) ? m_wcm.get() : m_parent_wcm.get();

    smpl::collision::WorldCollisionDetector wcd(rcm.get(), ewcm);

    double dist;

    auto startvars = gm->getVariablesFor(state1);
    auto goalvars = gm->getVariablesFor(state2);
    bool valid = wcd.checkMotionCollision(
        *gm->collisionState(),
        *gm->attachedBodiesCollisionState(),
        *rmcm,
        startvars,
        goalvars,
        gidx,
        dist);

    SV_SHOW_DEBUG_NAMED(
            "world_collision",
            MakeCollisionRobotValidityVisualization(
                    this,
                    gm->collisionState(),
                    gm->attachedBodiesCollisionState(),
                    gidx,
                    valid));

    res.collision = !valid;
    if (req.distance) {
        res.distance = std::min(res.distance, dist);
    }
    if (req.cost) {
        ROS_WARN_ONCE("Cost sources not computed by sbpl collision checker");
    }
    if (req.contacts) {
        ROS_WARN_ONCE("Contacts not computed by sbpl collision checker");
    }
}

void CollisionEnvSBPL::checkSelfCollisionMutable(
    const CollisionRequest& req,
    CollisionResult& res,
    const robot_state::RobotState& state,
    const AllowedCollisionMatrix& acm)
{
    using smpl::collision::AttachedBodiesCollisionModel;
    using smpl::collision::AttachedBodiesCollisionState;
    using smpl::collision::RobotCollisionState;
    using smpl::collision::SelfCollisionModel;

    if (state.getRobotModel()->getName() != m_rcm->name()) {
        ROS_ERROR_NAMED(CRP_LOGGER, "Collision Robot Model does not match Robot Model");
        setVacuousCollision(res);
        return;
    }

    // lookup the name of the corresponding collision group
    auto jgcgit = m_jcgm_map.find(req.group_name);
    auto& collision_group_name =
            jgcgit == m_jcgm_map.end() ? req.group_name : jgcgit->second;

    if (!m_rcm->hasGroup(collision_group_name)) {
        ROS_ERROR_NAMED(CRP_LOGGER, "No group '%s' found in the Robot Collision Model", collision_group_name.c_str());
        setVacuousCollision(res);
        return;
    }

    if (!m_grid) {
        ROS_DEBUG_NAMED(CRP_LOGGER, "Initialize self collision model");

        // lazily initialize self collision model
        m_grid = createGridFor(m_scm_config);
        m_grid->setReferenceFrame(m_rcm->modelFrame());

        auto bbm = m_grid->getOccupiedVoxelsVisualization();
        bbm.ns = "self_collision_model_bounds";
        SV_SHOW_INFO_NAMED("collision_robot_bounds", bbm);

        m_scm = std::make_shared<SelfCollisionModel>(
                m_grid.get(), m_rcm.get(), m_updater.attachedBodiesCollisionModel());
    }

    auto gidx = m_rcm->groupIndex(collision_group_name);

    auto state_copy = state;
    state_copy.setJointPositions(
            state_copy.getRobotModel()->getRootJoint(),
            Eigen::Isometry3d::Identity());
    m_updater.update(state_copy);

    double dist;
    auto valid = m_scm->checkCollision(
            *m_updater.collisionState(),
            *m_updater.attachedBodiesCollisionState(),
            AllowedCollisionMatrixAndTouchLinksInterface(
                    acm, m_updater.touchLinkSet()),
            gidx,
            dist);

    ROS_INFO_STREAM_COND_NAMED(req.verbose, CRP_LOGGER, "self valid: " << std::boolalpha << valid << ", dist: " << dist);
    ROS_DEBUG_STREAM_COND_NAMED(!req.verbose, CRP_LOGGER, "self valid: " << std::boolalpha << valid << ", dist: " << dist);

    SV_SHOW_DEBUG_NAMED(
            "self_collision",
            MakeCollisionRobotValidityVisualization(
                    this,
                    m_updater.collisionState(),
                    m_updater.attachedBodiesCollisionState(),
                    gidx,
                    valid));

    res.collision = !valid;
    if (req.distance) {
        res.distance = std::min(res.distance, dist);
    }
    if (req.cost) {
        ROS_WARN_ONCE("Cost sources not computed by sbpl collision checker");
    }
    if (req.contacts) {
        ROS_WARN_ONCE("Contacts not computed by sbpl collision checker");
    }
}

void CollisionEnvSBPL::checkSelfCollisionMutable(
    const CollisionRequest& req,
    CollisionResult& res,
    const moveit::core::RobotState& state1,
    const moveit::core::RobotState& state2,
    const AllowedCollisionMatrix& acm)
{
    using smpl::collision::AttachedBodiesCollisionModel;
    using smpl::collision::AttachedBodiesCollisionState;
    using smpl::collision::RobotCollisionState;
    using smpl::collision::SelfCollisionModel;

    assert(state1.getRobotModel()->getName() == m_rcm->name());
    assert(state2.getRobotModel()->getName() == m_rcm->name());

    // lookup the name of the corresponding collision group
    auto jgcgit = m_jcgm_map.find(req.group_name);
    auto& collision_group_name =
            jgcgit == m_jcgm_map.end() ? req.group_name : jgcgit->second;

    if (!m_rcm->hasGroup(collision_group_name)) {
        ROS_ERROR_NAMED(CRP_LOGGER, "No group '%s' found in the Robot Collision Model", collision_group_name.c_str());
        setVacuousCollision(res);
        return;
    }

    if (!m_grid) {
        ROS_DEBUG_NAMED(CRP_LOGGER, "Initialize self collision model");

        // lazily initialize self collision model
        m_grid = createGridFor(m_scm_config);
        m_grid->setReferenceFrame(m_rcm->modelFrame());

        auto bbma = m_grid->getOccupiedVoxelsVisualization();
        bbma.ns = "self_collision_model_bounds";
        SV_SHOW_INFO_NAMED("collision_robot_bounds", bbma);

        m_scm = std::make_shared<SelfCollisionModel>(
                m_grid.get(), m_rcm.get(), m_updater.attachedBodiesCollisionModel());
    }

    auto gidx = m_rcm->groupIndex(collision_group_name);

    auto state1_copy = state1;
    auto state2_copy = state2;
    state1_copy.setJointPositions(
            state1_copy.getRobotModel()->getRootJoint(),
            Eigen::Isometry3d::Identity());
    state2_copy.setJointPositions(
            state1_copy.getRobotModel()->getRootJoint(),
            Eigen::Isometry3d::Identity());
//    m_updater.update(state_copy);

    auto startvars = m_updater.getVariablesFor(state1_copy);
    auto goalvars = m_updater.getVariablesFor(state2_copy);

    double dist;
    auto valid = m_scm->checkMotionCollision(
            *m_updater.collisionState(),
            *m_updater.attachedBodiesCollisionState(),
            AllowedCollisionMatrixAndTouchLinksInterface(acm, m_updater.touchLinkSet()),
            *m_rmcm,
            startvars,
            goalvars,
            gidx,
            dist);

    ROS_INFO_STREAM_COND_NAMED(req.verbose, CRP_LOGGER, "valid: " << std::boolalpha << valid << ", dist: " << dist);
    ROS_DEBUG_STREAM_COND_NAMED(!req.verbose, CRP_LOGGER, "valid: " << std::boolalpha << valid << ", dist: " << dist);

    SV_SHOW_DEBUG_NAMED(
            "self_collision",
            MakeCollisionRobotValidityVisualization(
                    this,
                    m_updater.collisionState(),
                    m_updater.attachedBodiesCollisionState(),
                    gidx,
                    valid));

    res.collision = !valid;
    if (req.distance) {
        res.distance = std::min(res.distance, dist);
    }
    if (req.cost) {
        ROS_WARN_ONCE("Cost sources not computed by sbpl collision checker");
    }
    if (req.contacts) {
        ROS_WARN_ONCE("Contacts not computed by sbpl collision checker");
    }
}

auto CollisionEnvSBPL::getSelfCollisionPropagationDistance() const -> double
{
    // TODO: include the attached object models when computing the max expansion
    // distance

    // resolve the maximum expansion distance. this should be at least the
    // radius of the largest leaf sphere in the collision model but may be
    // overridden by the user to a larger value
    auto cfg_max_distance_m = m_scm_config.max_distance_m;
    if (cfg_max_distance_m > 0.0) {
        // allow the user to set the maximum expansion distance. a value between
        // the max leaf sphere radius and the max sphere radius abandons some
        // efficiency gained by hierarchical checking in exchange for fewer
        // distance propagations. a value larger than the max sphere radius
        // provides additional cost information about how far the robot is from
        // environment obstacles.
        auto required_radius = m_rcm->maxLeafSphereRadius() + sqrt(3) * m_scm_config.res_m;
        if (cfg_max_distance_m < required_radius) {
            ROS_WARN_NAMED(CRP_LOGGER, "configured max distance set to %0.3f. overriding to required radius %0.3f", cfg_max_distance_m, required_radius);
        }
        return std::max(required_radius, cfg_max_distance_m);
    } else {
        // default to the value of the largest sphere (leaf and internal nodes)
        return m_rcm->maxSphereRadius() + m_scm_config.res_m;
    }
}

auto CollisionEnvSBPL::createGridFor(const CollisionGridConfig& config) const
    -> smpl::OccupancyGridPtr
{
    ROS_DEBUG_NAMED(CRP_LOGGER, "  Create Distance Field");
    ROS_DEBUG_NAMED(CRP_LOGGER, "    size: (%0.3f, %0.3f, %0.3f)", config.size_x, config.size_y, config.size_z);
    ROS_DEBUG_NAMED(CRP_LOGGER, "    origin: (%0.3f, %0.3f, %0.3f)", config.origin_x, config.origin_y, config.origin_z);
    ROS_DEBUG_NAMED(CRP_LOGGER, "    resolution: %0.3f", config.res_m);
    ROS_DEBUG_NAMED(CRP_LOGGER, "    max_distance: %0.3f", config.max_distance_m);

    // TODO: this can be substantially smaller since it only has to encompass
    // the range of motion of the robot
    auto ref_counted = true;
    auto max_distance = getSelfCollisionPropagationDistance();
    return std::make_shared<smpl::OccupancyGrid>(
            config.size_x,
            config.size_y,
            config.size_z,
            config.res_m,
            config.origin_x,
            config.origin_y,
            config.origin_z,
            max_distance,
            ref_counted);
}

void CollisionEnvSBPL::processWorldUpdateUninitialized(
    const World::ObjectConstPtr& object)
{

}

using namespace smpl::collision;

void CollisionEnvSBPL::processWorldUpdateCreate(
    const World::ObjectConstPtr& object)
{
    copyOnWrite();

    // check for existing collision object
    auto it = FindObjectRepPair(object);
    if (it != end(m_collision_objects)) {
        ROS_WARN_NAMED(LOG, "Object '%s' already exists in Collision World", object->id_.c_str());
        return;
    }

    assert(object->shapes_.size() == object->shape_poses_.size());

    ObjectRepPair op;
    op.world_object = object;
    ConvertObjectToCollisionObjectShallow(object, op.shapes, op.collision_object);

    // attempt insertion into the world collision model
    auto inserted = m_wcm->insertObject(op.collision_object.get());
    assert(inserted);

    m_collision_objects.push_back(std::move(op));
}

void CollisionEnvSBPL::processWorldUpdateDestroy(
    const World::ObjectConstPtr& object)
{
    copyOnWrite();

    auto it = FindObjectRepPair(object);
    if (it == end(m_collision_objects)) {
        ROS_WARN_NAMED(LOG, "Object '%s' not in the Collision World", object->id_.c_str());
        return;
    }

    auto removed = m_wcm->removeObject(it->collision_object.get());
    assert(removed);

    m_collision_objects.erase(it);
}

void CollisionEnvSBPL::processWorldUpdateMoveShape(
    const World::ObjectConstPtr& object)
{
    copyOnWrite();

    auto it = FindObjectRepPair(object);
    if (it == end(m_collision_objects)) {
        ROS_WARN_NAMED(LOG, "Object '%s' not in the Collision World", object->id_.c_str());
        return;
    }

    auto res = m_wcm->moveShapes(it->collision_object.get());
    assert(res);
}

void CollisionEnvSBPL::processWorldUpdateAddShape(
    const World::ObjectConstPtr& object)
{
    copyOnWrite();
    auto it = FindObjectRepPair(object);
    if (it == end(m_collision_objects)) {
        ROS_WARN_NAMED(LOG, "Object '%s' not in the Collision World", object->id_.c_str());
        return;
    }

    auto res = m_wcm->insertShapes(it->collision_object.get());
    assert(res);
}

void CollisionEnvSBPL::processWorldUpdateRemoveShape(
    const World::ObjectConstPtr& object)
{
    copyOnWrite();
    auto it = FindObjectRepPair(object);
    if (it == end(m_collision_objects)) {
        ROS_WARN_NAMED(LOG, "Object '%s' not in the Collision World", object->id_.c_str());
        return;
    }

    auto res = m_wcm->removeShapes(it->collision_object.get());
    assert(res);
}

} // namespace collision_detection
