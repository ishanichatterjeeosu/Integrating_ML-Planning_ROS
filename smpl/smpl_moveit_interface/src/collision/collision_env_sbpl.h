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

#ifndef SMPL_MOVEIT_INTERFACE_COLLISION_ENV_SBPL_H
#define SMPL_MOVEIT_INTERFACE_COLLISION_ENV_SBPL_H

// system includes
// Check if all pure virtual functions ( = 0) of moveit collisionEnv are defined as union of botb collisionRobotSBPL and collisionWorldSBPL
// Make sure any calls to functions of these classes are still valid through an object of the collsionEnvSBPL class
// Functions that directly call moveit functions through moveIt objects need not be altered
#include <moveit/collision_detection/collision_env.h>
#include <moveit/distance_field/propagation_distance_field.h>
#include <moveit_msgs/OrientedBoundingBox.h>
#include <ros/ros.h>
#include <smpl/occupancy_grid.h>
#include <sbpl_collision_checking/attached_bodies_collision_model.h>
#include <sbpl_collision_checking/robot_collision_model.h>
#include <sbpl_collision_checking/robot_motion_collision_model.h>
#include <sbpl_collision_checking/self_collision_model.h>
#include <sbpl_collision_checking/collision_space.h>
#include <sbpl_collision_checking/collision_model_config.h>
#include <sbpl_collision_checking/shapes.h>
#include <sbpl_collision_checking/world_collision_detector.h>
#include <smpl/forward.h>

// module includes
#include "collision_common_sbpl.h"
#include "config.h"

namespace smpl
{
    SBPL_CLASS_FORWARD(OccupancyGrid);
}

namespace collision_detection
{

    class CollisionEnvSBPL : public CollisionEnv
    {
    public:
        CollisionEnvSBPL(
            const robot_model::RobotModelConstPtr &model,
            double padding = 0.0,
            double scale = 1.0);

        CollisionEnvSBPL(
            const robot_model::RobotModelConstPtr &model, const WorldPtr &world,
            double padding = 0.0,
            double scale = 1.0);

        CollisionEnvSBPL(const CollisionEnvSBPL &other, const WorldPtr &world);

        virtual ~CollisionEnvSBPL();

        auto robotCollisionModel() const
            -> const smpl::collision::RobotCollisionModelConstPtr &;

        auto robotMotionCollisionModel() const
            -> const smpl::collision::RobotMotionCollisionModelConstPtr &;

        auto distanceField(
            const std::string &robot_name,
            const std::string &group_name) const
            -> const smpl::DistanceMapInterface *;

        /// \name CollisionRobot Interface
        ///@{
        void checkSelfCollision(
            const CollisionRequest &req,
            CollisionResult &res,
            const robot_state::RobotState &state) const override;

        void checkSelfCollision(
            const CollisionRequest &req,
            CollisionResult &res,
            const robot_state::RobotState &state,
            const AllowedCollisionMatrix &acm) const override;

        /// \name CollisionWorld Interface
        ///@{
        void checkRobotCollision(
            const CollisionRequest &req,
            CollisionResult &res,
            const robot_state::RobotState &state) const override;

        void checkRobotCollision(
            const CollisionRequest &req,
            CollisionResult &res,

            const robot_state::RobotState &state,
            const AllowedCollisionMatrix &acm) const override;

        void checkRobotCollision(
            const CollisionRequest &req,
            CollisionResult &res,
            const robot_state::RobotState &state1,
            const robot_state::RobotState &state2) const override;

        void checkRobotCollision(
            const CollisionRequest &req,
            CollisionResult &res,
            const robot_state::RobotState &state1,
            const robot_state::RobotState &state2,
            const AllowedCollisionMatrix &acm) const override;

        // void checkSelfCollision(
        //     const CollisionRequest& req,
        //     CollisionResult& res,
        //     const robot_state::RobotState& state1,
        //     const robot_state::RobotState& state2) const override;

        // void checkSelfCollision(
        //     const CollisionRequest& req,
        //     CollisionResult& res,
        //     const robot_state::RobotState& state1,
        //     const robot_state::RobotState& state2,
        //     const AllowedCollisionMatrix& acm) const override;

        // void checkOtherCollision(
        //     const CollisionRequest& req,
        //     CollisionResult& res,
        //     const robot_state::RobotState& state,
        //     const CollisionRobot& other_robot,
        //     const robot_state::RobotState& other_state) const override;

        // void checkOtherCollision(
        //     const CollisionRequest& req,
        //     CollisionResult& res,
        //     const robot_state::RobotState& state,
        //     const CollisionRobot& other_robot,
        //     const robot_state::RobotState& other_state,
        //     const AllowedCollisionMatrix& acm) const override;

        // void checkOtherCollision(
        //     const CollisionRequest& req,
        //     CollisionResult& res,
        //     const robot_state::RobotState& state1,
        //     const robot_state::RobotState& state2,
        //     const CollisionRobot& other_robot,
        //     const robot_state::RobotState& other_state1,
        //     const robot_state::RobotState& other_state2) const override;

        // void checkOtherCollision(
        //     const CollisionRequest& req,
        //     CollisionResult& res,
        //     const robot_state::RobotState& state1,
        //     const robot_state::RobotState& state2,
        //     const CollisionRobot& other_robot,
        //     const robot_state::RobotState& other_state1,
        //     const robot_state::RobotState& other_state2,
        //     const AllowedCollisionMatrix& acm) const override;

        // #if COLLISION_DETECTION_SBPL_ROS_VERSION == COLLISION_DETECTION_SBPL_ROS_KINETIC

        void distanceSelf(
            const DistanceRequest &req,
            DistanceResult &res,
            const robot_state::RobotState &state) const override;

        void distanceRobot(
            const DistanceRequest &req,
            DistanceResult &res,
            const robot_state::RobotState &state) const override;

        void setWorld(const WorldPtr &world) override;

        // void distanceOther(
        //     const DistanceRequest& req,
        //     DistanceResult& res,
        //     const robot_state::RobotState& state,
        //     const CollisionRobot& other_robot,
        //     const robot_state::RobotState& other_state) const override;

        // #else

        // double distanceSelf(const robot_state::RobotState& state) const override;

        // double distanceSelf(
        //     const robot_state::RobotState& state,
        //     const AllowedCollisionMatrix& acm) const override;

        // double distanceOther(
        //     const robot_state::RobotState& state,
        //     const CollisionRobot& other_robot,
        //     const robot_state::RobotState& other_state) const override;

        // double distanceOther(
        //     const robot_state::RobotState& state,
        //     const CollisionRobot& other_robot,
        //     const robot_state::RobotState& other_state,
        //     const AllowedCollisionMatrix& acm) const override;
        // #endif

        ///@}

    protected:
        /// \name Reimplemented Protected Functions
        ///@{
        void updatedPaddingOrScaling(const std::vector<std::string> &links) override;
        ///@}

    public:
        using CollisionModelConfigConstPtr =
            std::shared_ptr<const smpl::collision::CollisionModelConfig>;

        CollisionGridConfig m_scm_config;
        CollisionGridConfig m_wcm_config;

        // mapping from joint group name to collision group name
        std::unordered_map<std::string, std::string> m_jcgm_map;

        smpl::collision::RobotCollisionModelConstPtr m_rcm;
        smpl::collision::RobotMotionCollisionModelConstPtr m_rmcm;

        // world model stuff
        smpl::OccupancyGridConstPtr m_parent_grid;
        smpl::collision::WorldCollisionModelConstPtr m_parent_wcm;
        smpl::collision::WorldCollisionDetectorConstPtr m_parent_wcd;

        CollisionStateUpdater m_updater;

        // self colllision models
        smpl::OccupancyGridPtr m_grid;
        smpl::collision::SelfCollisionModelPtr m_scm;

        smpl::collision::WorldCollisionModelPtr m_wcm;

        std::unordered_map<std::string, CollisionStateUpdaterPtr> m_updaters;

        World::ObserverHandle m_observer_handle;

        struct ObjectRepPair
        {
            // originating world object. hold onto it here so we are guaranteed to
            // maintain valid references to shared data e.g. octomap, mesh data,
            // and don't have to make copies.
            World::ObjectConstPtr world_object;

            // CollisionShapes matching world object's shapes
            std::vector<std::unique_ptr<smpl::collision::CollisionShape>> shapes;

            // CollisionObject matching world object
            std::unique_ptr<smpl::collision::CollisionObject> collision_object;
        };

        std::vector<ObjectRepPair> m_collision_objects;

        void construct();

        void copyOnWrite();

        auto FindObjectRepPair(const World::ObjectConstPtr &object)
            -> std::vector<ObjectRepPair>::iterator;

        auto getCollisionStateUpdater(
            const moveit::core::RobotModel &robot_model)
            -> CollisionStateUpdaterPtr;

        void registerWorldCallback();
        void worldUpdate(const World::ObjectConstPtr &object, World::Action action);

        void setVacuousCollision(CollisionResult &res) const;

        void checkSelfCollisionMutable(
            const CollisionRequest &req,
            CollisionResult &res,
            const robot_state::RobotState &state,
            const AllowedCollisionMatrix &acm);

        void checkSelfCollisionMutable(
            const CollisionRequest &req,
            CollisionResult &res,
            const moveit::core::RobotState &state1,
            const moveit::core::RobotState &state2,
            const AllowedCollisionMatrix &acm);

        bool updateAttachedBodies(const moveit::core::RobotState &state);

        auto getSelfCollisionPropagationDistance() const -> double;

        auto createGridFor(const CollisionGridConfig &config) const
            -> smpl::OccupancyGridPtr;

        void clearAllCollisions(CollisionResult &res) const;

        void checkRobotCollisionMutable(
            const CollisionRequest &req,
            CollisionResult &res,
            const robot_state::RobotState &state);

        void checkRobotCollisionMutable(
            const CollisionRequest &req,
            CollisionResult &res,

            const robot_state::RobotState &state,
            const AllowedCollisionMatrix &acm);

        void checkRobotCollisionMutable(
            const CollisionRequest &req,
            CollisionResult &res,
            const robot_state::RobotState &state1,
            const robot_state::RobotState &state2);

        void checkRobotCollisionMutable(
            const CollisionRequest &req,
            CollisionResult &res,

            const robot_state::RobotState &state1,
            const robot_state::RobotState &state2,
            const AllowedCollisionMatrix &acm);

        void processWorldUpdateUninitialized(const World::ObjectConstPtr &object);
        void processWorldUpdateCreate(const World::ObjectConstPtr &object);
        void processWorldUpdateDestroy(const World::ObjectConstPtr &object);
        void processWorldUpdateMoveShape(const World::ObjectConstPtr &object);
        void processWorldUpdateAddShape(const World::ObjectConstPtr &object);
        void processWorldUpdateRemoveShape(const World::ObjectConstPtr &object);
    };

} // namespace collision_detection

#endif
