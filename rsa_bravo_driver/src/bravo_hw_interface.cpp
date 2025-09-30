#include <rsa_bravo_driver/bravo_hw_interface.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

//#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>

#include <iomanip>
#include <fstream>
#include <iostream>



namespace bravo_base
{
    using libbpl_protocol::PacketTypes;

    BravoHWInterface::BravoHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
        : name_("hardware_interface")
        , nh_(nh),
        io_thread_()
    { 
        ROS_DEBUG("Initializing node!");

        // Initialization of the robot's resources (joints, sensors, actuators) and
        // interfaces can be done here or inside init().
        // E.g. parse the URDF for joint names & interfaces, then initialize them
        // Check if the URDF model needs to be loaded
        if (urdf_model == NULL)
            loadURDF(nh, "robot_description");
        else
            urdf_model_ = urdf_model;

        // Load rosparams
        ros::NodeHandle rpnh(nh_, name_);
        std::size_t error = 0;
        // Code API of rosparam_shortcuts:
        // http://docs.ros.org/en/noetic/api/rosparam_shortcuts/html/namespacerosparam__shortcuts.html#aa6536fe0130903960b1de4872df68d5d
        error += !rosparam_shortcuts::get(name_, rpnh, "joints", joint_names_);
        error += !rosparam_shortcuts::get(name_, nh_, "debug/hardware_interface", debug_);
        error += !rosparam_shortcuts::get(name_, nh_, "heartbeat_freq", heartbeat_freq_);
        rosparam_shortcuts::shutdownIfError(name_, error);

        // Initialize the hardware interface
        init(nh_, nh_);
        
        


        ROS_INFO_STREAM("Initialized RobotHW!");

    }

    void BravoHWInterface::reset_bravo() {

        bpl_client_->disable(); // set actuators to MODE_DISABLE

        // Disable heartbeat messages
        Packet sent(PacketTypes::HEARTBEAT_FREQUENCY, libbpl_protocol::BroadcastDeviceId);
        sent.push_back<uint8_t>(0);
        bpl_client_->send(sent);
    }

    BravoHWInterface::~BravoHWInterface() {
        reset_bravo();
        io_thread_.stop();
        io_thread_.join();
    }
 
    bool BravoHWInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
    {
        ROS_INFO("Initializing Bravo Hardware Interface ...");

        _enableBravoServer = root_nh.advertiseService("enable_bravo",
                                &BravoHWInterface::enableSrvCallback, this);


        num_joints_ = joint_names_.size();
        ROS_INFO("Number of joints: %d", (int)num_joints_);
        for (unsigned int i = 0; i < num_joints_; i++)
        {
            // Create a JointStateHandle for each joint and register them with the 
            // JointStateInterface.
            hardware_interface::JointStateHandle joint_state_handle(joint_names_[i],
                                                                    &joint_positions_[i], 
                                                                    &joint_velocities_[i],
                                                                    &joint_efforts_[i]);
            joint_state_interface_.registerHandle(joint_state_handle);

            // Create a JointHandle (read and write) for each controllable joint
            // using the read-only joint handles within the JointStateInterface and 
            // register them with the JointPositionInterface.
            hardware_interface::JointHandle joint_handle(joint_state_handle, &joint_position_commands_[i]);
            position_joint_interface_.registerHandle(joint_handle);

            // Initialize joint states with zero values
            joint_positions_[i] = 0.0;
            joint_velocities_[i] = 0.0;
            joint_efforts_[i] = 0.0;
            async_joint_positions_[i] = 0.0;
            async_joint_velocities_[i] = 0.0;
            async_joint_efforts_[i] = 0.0;

            joint_position_commands_[i] = 0.0;

        }

        // Register the JointStateInterface containing the read only joints
        // with this robot's hardware_interface::RobotHW.
        registerInterface(&joint_state_interface_);

        // Register the JointVelocityInterface containing the read/write joints
        // with this robot's hardware_interface::RobotHW.
        registerInterface(&position_joint_interface_);

        const std::string ipAddress("192.168.2.3"); // Default IP address for Bravo
        const unsigned int port = 6789;             // Default port for Bravo
        bpl_client_ = std::make_shared<libbpl_protocol::AsynchronousClient>(io_thread_.context(), ipAddress, port);
        bpl_client_->add_callback( std::bind(&BravoHWInterface::packetRxCallback,this,std::placeholders::_1) );

        // Configure all devices to broadcast position only
        // Not sure if it should go here
        for (std::size_t i = 0; i < num_joints_; ++i) {
            Packet sent(PacketTypes::HEARTBEAT_SET, i+1); // set devices 1-7 (= joints A-F)
            sent.push_back(PacketTypes::POSITION);
            sent.push_back(PacketTypes::CURRENT);
            bpl_client_->send(sent);
        }

        for (std::size_t i = 0; i < num_joints_; ++i) {
            const uint8_t freq = (uint8_t) heartbeat_freq_; // Hz
            Packet sent(PacketTypes::HEARTBEAT_FREQUENCY, i+1);
            sent.push_back<uint8_t>(freq);
            bpl_client_->send(sent);
        }

        // Request the current limits for the gripper
        // Steps:
        // Formulate a REQUEST packet (6.2.1) for a CURRENT_LIMIT packets
        // Identify the Device ID of the gripper actuation
        // Send the REQUEST packet
        // Ensure that the packet will be printed to the screen
        
        Packet sent(PacketTypes::REQUEST, 0x01); // 0x01 is arm
        sent.push_back<uint8_t>(PacketTypes::CURRENT_LIMITS);
        bpl_client_->send(sent);

        // Set the current limits for the gripper
        sent = Packet(PacketTypes::CURRENT_LIMITS, 0x01);
        sent.push_back<float>(800, -800);
        bpl_client_->send(sent);

        // Check that the current limit was successfully updated
        sent = Packet(PacketTypes::REQUEST, 0x01); // 0x01 is arm
        sent.push_back<uint8_t>(PacketTypes::CURRENT_LIMITS);
        bpl_client_->send(sent);

        io_thread_.start();

        ROS_INFO("... Done Initializing Bravo Hardware Interface");

        return true;
    }


    void BravoHWInterface::packetRxCallback( libbpl_protocol::Packet packet ) {

        // Assumes the device_id in the packet is the index into the
        // array
        uint8_t id = packet.deviceId();

        

        // deviceID 1 through 7 corresponds to joints A (jaw) through F (base swivel)
        if (id < 1 || id > num_joints_){ 
            return;
        }

        if (packet.type() == PacketTypes::POSITION) {
            if (id == 1) {
                // convert linear actuator from mm to meters
                async_joint_positions_[0] = ( (double) packet.pop_front<float>() ) / 1000.0;
            } else {
                async_joint_positions_[id - 1] = (double) packet.pop_front<float>();
            }
        } 
        else if(packet.type() == PacketTypes::CURRENT)
        {
            async_joint_efforts_[id - 1] = (double)packet.pop_front<float>();
            if (id == 1)
            {
                ROS_INFO_STREAM_NAMED(name_, "Received gripper current value: " << async_joint_efforts_[0] << "," << "for device id: " << (int) id);
            }
        }
        else {
            if (packet.type() == PacketTypes::CURRENT_LIMITS) {
                ROS_INFO_STREAM_NAMED(name_, "Received current limits [" 
                    << packet.pop_front<float>() << ", " 
                    << packet.pop_front<float>() << "] for deviceID "
                    << packet.deviceId() << ".");
            }
            // ROS_DEBUG_STREAM_NAMED(name_, "Received RX: " << packet); // NYI operator<<
        }

    }

    void BravoHWInterface::read(const ros::Time& time, const ros::Duration& period){
        for (std::size_t i = 0; i < num_joints_; ++i)
        {
            joint_positions_[i] = async_joint_positions_[i];
            joint_velocities_[i] = async_joint_velocities_[i];
            joint_efforts_[i] = async_joint_efforts_[i];
        }
    }

    void BravoHWInterface::write(const ros::Time& time, const ros::Duration& period)
    {
        if (!enabled_){ // TODO make disabling also issue a "stop" command
            return;
        }

        // ROS_INFO_STREAM_THROTTLE(1, "write()");

        // This is some seriously untested code
        for (std::size_t i = 0; i < num_joints_; ++i)
        {

            //ROS_INFO_STREAM_NAMED(name_, "writing joint " << i+1 << " to " << joint_position_commands_[i]);
            double command;

            if (i == 0) {
                // convert linear actuator command from meters to mm
                //ROS_INFO_STREAM_NAMED(name_, "writing gripper" << i << " to " << joint_position_commands_[0]);

                command = joint_position_commands_[0] * 1000;
            } else {
                command = joint_position_commands_[i];
            }

            bpl_client_->sendPosition(i+1, (float) command);
        }

    }

    bool BravoHWInterface::enableSrvCallback(rsa_bravo_msgs::EnableBravoRequest& req,
                rsa_bravo_msgs::EnableBravoResponse& res){

        enabled_ = req.enable;

        if (req.enable) {
            bpl_client_->enable();
        } else {
            bpl_client_->disable();
        }

        ROS_INFO_STREAM_NAMED(name_, "Bravo is enabled: " << enabled_);
                
        return true;
    }

    void BravoHWInterface::loadURDF(const ros::NodeHandle &nh, std::string param_name)
    {
        std::string urdf_string;
        urdf_model_ = new urdf::Model();
        // search and wait for robot_description on param server
        while (urdf_string.empty() && ros::ok())
        {
            std::string search_param_name;
            if (nh.searchParam(param_name, search_param_name))
            {
                ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
                                        nh.getNamespace() << search_param_name);
                nh.getParam(search_param_name, urdf_string);
            }
            else
            {
                ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
                                        nh.getNamespace() << param_name);
                nh.getParam(param_name, urdf_string);
            }

            usleep(100000);
        }

        if (!urdf_model_->initString(urdf_string))
            ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model");
        else
            ROS_DEBUG_STREAM_NAMED(name_, "Received URDF from param server");
    }

};
