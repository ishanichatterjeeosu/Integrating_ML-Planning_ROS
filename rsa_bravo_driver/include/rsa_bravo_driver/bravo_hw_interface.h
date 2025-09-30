#ifndef BRAVO_HW_INTERFACE_H
#define BRAVO_HW_INTERFACE_H


// ROS
#include <ros/ros.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>

// ROS Control
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

#include "libbpl_protocol/io_service_thread.h"
#include "libbpl_protocol/async_client.h"

// Enable service
#include "rsa_bravo_msgs/EnableBravo.h"


namespace bravo_base
{
    using libbpl_protocol::Packet;

    const unsigned int NUM_JOINTS = 7;

    struct JointState
    {
        float angular_position_;
        float angular_velocity_;
    };

    /// \brief Hardware interface for a robot
    class BravoHWInterface : public hardware_interface::RobotHW
    {
    public:
        /**
         * \brief Constructor
         * \param nh - Node handle for topics.
         * \param urdf_model - optional pointer to a parsed robot model
         */
        BravoHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);

        /** \brief Destructor */
        virtual ~BravoHWInterface();

        /** \brief The init function is called to initialize the RobotHW from a
         *         non-realtime thread.
         *
         * Initialising a custom robot is done by registering joint handles
         * (\ref hardware_interface::ResourceManager::registerHandle) to hardware
         * interfaces that group similar joints and registering those individual
         * hardware interfaces with the class that represents the custom robot
         * (derived from this \ref hardware_interface::RobotHW)
         *
         * \note Registering of joint handles and interfaces can either be done in the
         * constructor or this \ref init method.
         *
         * \param root_nh A NodeHandle in the root of the caller namespace.
         *
         * \param robot_hw_nh A NodeHandle in the namespace from which the RobotHW
         * 
         * 
         * should read its configuration.
         *
         * \returns True if initialization was successful
         */
        virtual bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh);

        /** \brief Read data from the robot hardware.
         *
         * The read method is part of the control loop cycle (\ref read, update, \ref write) 
         * and is used to populate the robot state from the robot's hardware resources
         * (joints, sensors, actuators). This method should be called before 
         * controller_manager::ControllerManager::update() and \ref write.
         * 
         * \note The name \ref read refers to reading state from the hardware.
         * This complements \ref write, which refers to writing commands to the hardware.
         *
         * Querying WallTime inside \ref read is not realtime safe. The parameters
         * \p time and \p period make it possible to inject time from a realtime source.
         *
         * \param time The current time
         * \param period The time passed since the last call to \ref read
         */
        virtual void read(const ros::Time& time, const ros::Duration& period) override;

        /** \brief Write commands to the robot hardware.
         * 
         * The write method is part of the control loop cycle (\ref read, update, \ref write) 
         * and is used to send out commands to the robot's hardware 
         * resources (joints, actuators). This method should be called after 
         * \ref read and controller_manager::ControllerManager::update.
         * 
         * \note The name \ref write refers to writing commands to the hardware.
         * This complements \ref read, which refers to reading state from the hardware.
         *
         * Querying WallTime inside \ref write is not realtime safe. The parameters
         * \p time and \p period make it possible to inject time from a realtime source.
         *
         * \param time The current time
         * \param period The time passed since the last call to \ref write
         */
        virtual void write(const ros::Time& time, const ros::Duration& period);

        void packetRxCallback( libbpl_protocol::Packet packet );

    protected:

        /** \brief Arn teardown like disabling heartbeat messages.*/
        void reset_bravo();

        /** \brief Get the URDF XML from the parameter server */
        virtual void loadURDF(const ros::NodeHandle& nh, std::string param_name);

        // Short name of this class
        std::string name_;

        // Startup and shutdown of the internal node inside a roscpp program
        ros::NodeHandle nh_;

        // Hardware interfaces
        // hardware_interface::JointStateInterface gives read access to all joint values 
        // without conflicting with other controllers.
        hardware_interface::JointStateInterface joint_state_interface_;

        // hardware_interface::PositionJointInterface inherits from 
        // hardware_interface::JointCommandInterface and is used for reading and writing
        // joint positions. Because this interface reserves the joints for write access,
        // conflicts with other controllers writing to the same joints might occur.
        // To only read joint velocities, avoid conflicts using 
        // hardware_interface::JointStateInterface.
        hardware_interface::PositionJointInterface position_joint_interface_;

        // Configuration
        std::vector<std::string> joint_names_;
        std::size_t num_joints_;
        urdf::Model *urdf_model_;

        // Enable/Disable debug output
        // Setting only possible during first start of the hw interface
        // to avoid reading permanently form the parameter server
        bool debug_;


        // Data member array to store the controller commands which are sent to the 
        // robot's resources (joints, actuators)
        double joint_position_commands_[NUM_JOINTS];
 
        // Data member arrays to store the state of the robot's resources (joints, sensors)
        // These values are filled in the read() method and are registered to the 
        // joint_state_interface_ of type hardware_interface::JointStateInterface.
        double joint_positions_[NUM_JOINTS];
        double joint_velocities_[NUM_JOINTS];
        double joint_efforts_[NUM_JOINTS];

        // these are updated by callbacks and periodically copied into joint_positions_, etc.
        double async_joint_positions_[NUM_JOINTS];
        double async_joint_velocities_[NUM_JOINTS];
        double async_joint_efforts_[NUM_JOINTS];

        libbpl_protocol::IoServiceThread io_thread_;
        std::shared_ptr<libbpl_protocol::AsynchronousClient> bpl_client_;

        // the enabled_ variable is set by the EnableBravo service,
        // and enables the write() functionality.
        bool enabled_ = false;
        bool enableSrvCallback(rsa_bravo_msgs::EnableBravoRequest& req,
                rsa_bravo_msgs::EnableBravoResponse& res);
        ros::ServiceServer _enableBravoServer;

        // heartbeat frequency to query sensors. Cast to a uint8 later.
        double heartbeat_freq_ = 0.0;

    };  // class BravoHWInterface

}  // namespace

#endif // BRAVO_HW_INTERFACE_H
