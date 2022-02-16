#ifndef _ROS_SUBSCRIBER_PLUGIN_HH_
#define _ROS_SUBSCRIBER_PLUGIN_HH_

//  Gazebo api
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// ROS
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/String.h"
#include "log.h"

#include "arm_lib/FK.h"
#include "arm_lib/IK.h"

namespace gazebo
{
/// \brief A plugin to control a SUBSCRIBER sensor.
    class JointRosSubscriberPlugin : public ModelPlugin {

    private:

        /// \brief Pointer to the models.
        physics::ModelPtr model;

        /// \brief List of joints. We have 8 joints the 4 four on robot arm the last four on the grip part
        physics::Joint_V jointList;

        /// \brief List of 8 links;
        physics::Link_V linkList;

        /// \brief A PID controller for the joint.
        common::PID pid;

        /// \brief joint controller
        physics::JointControllerPtr jointController;

        /// \brief A node use for ROS transport
        std::unique_ptr<ros::NodeHandle> rosNode;

        /// \brief A ROS subscriber
        ros::Subscriber rosSub, gripSub, ikSub;

        /// \brief A ROS callbackqueue that helps process messages
        ros::CallbackQueue rosQueue, gripQueue, ikQueue;

        /// \brief A thread the keeps running the rosQueue
        std::thread rosQueueThread;

        /// \brief ROS helper function that processes messages
        void QueueThread() {
            static const double timeout = 0.01;
            while (this->rosNode->ok()) {
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
                this->gripQueue.callAvailable(ros::WallDuration(timeout));
//                this->calculateFK();
            }
        }

        void calculateFK() {
            ros::NodeHandle n;
            ros::ServiceClient client = n.serviceClient<arm_lib::FK>("fk");
            arm_lib::FK srv;
            srv.request.link_length = {0.05, 2, 1, 0.5, 0.04, 0.2};
            srv.request.joint_position = {0, 0, 0, 0, 0, 0};
            srv.request.joint_axis = {2, 0, 0, 0, 2, 0}; // all joint except the last one (index == 5) are
            for (int i = 0; i < 6; i++) {
                srv.request.joint_position[i] = GetJointPosition(jointList[i], srv.request.joint_axis[i]);
            }

            if (client.call(srv)) {
                auto pose = srv.response.actuator_pose;
                std::cout << pose[0] << " " << pose[1] << " "  << pose[2] << std::endl;
            }
            else
                std::cout << "error" << std::endl;
        }

        void calculateIK(double x, double y, double z) {
            // service client
            ros::NodeHandle n;
            ros::ServiceClient client = n.serviceClient<arm_lib::IK>("ik");
            arm_lib::IK srv;
            srv.request.end_effector = {x, y, z};

            if (client.call(srv)) {
                for (int i = 0; i < 6; i++) {
                    jointController->SetPositionTarget(jointList[i]->GetScopedName(), srv.response.new_angles[i]);
                }
            }
            else {
                std::cout << "error" << std::endl;
            }
        }
    public:
        /// \brief Constructor
        JointRosSubscriberPlugin(){ }

        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
            printf("Joint ros subscriber \n");
            // Safety check
            if (_model->GetJointCount() == 0) {
                std::cerr << "Invalid joint count, SUBSCRIBER plugin not loaded\n";
                return;
            }

            // Store the models pointer for convenience.
            this->model = _model;

            // Get list of joints
            this->jointList = _model->GetJoints();

            // Get list of links
            this->linkList = _model->GetLinks();

            // Setup a PID  .
            this->pid = common::PID(200, 65, 35);

            this->jointController = this->model->GetJointController();

            // Apply the PID to the joints.
            for (auto const &joint : jointList) {
                jointController->SetPositionPID(joint->GetScopedName(), this->pid);
            }

            auto pid2 = common::PID(18.2, 10, 10);
            jointController->SetPositionPID(jointList[6]->GetScopedName(), pid2);
            jointController->SetPositionPID(jointList[7]->GetScopedName(), pid2);

            jointController->SetPositionTarget(jointList[6]->GetScopedName(), -0.5 * M_PI);
            jointController->SetPositionTarget(jointList[7]->GetScopedName(), 0.5 * M_PI);

            // Initialize ros, if it has not already bee initialized.
            if (!ros::isInitialized()) {
                int argc = 0;
                char** argv = NULL;
                ros::init(argc, argv, "gazebo_client",
                          ros::init_options::NoSigintHandler);
            }

            // Create our ROS node. This acts in a similar manner to
            // the Gazebo node
            this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

            // Create a named topic, and subscribe to it.
            ros::SubscribeOptions so =
                    ros::SubscribeOptions::create<geometry_msgs::Quaternion>(
                            "/arm/angle_cmd",
                            1,
                            boost::bind(&JointRosSubscriberPlugin::OnPosMsg, this, _1),
                            ros::VoidPtr(), &this->rosQueue);

            this->rosSub = this->rosNode->subscribe(so);

            // Create a grip topic, and subscribe to it.
            so = ros::SubscribeOptions::create<std_msgs::String>(
                    "/arm/grip_cmd",
                    1,
                    boost::bind(&JointRosSubscriberPlugin::OnGripMsg, this, _1),
                    ros::VoidPtr(), &this->gripQueue);

            this->gripSub = this->rosNode->subscribe(so);

            // Spin up the queue helper thread.
            this->rosQueueThread =
                    std::thread(std::bind(&JointRosSubscriberPlugin::QueueThread, this));

            calculateIK(2, 2, 0.25);
        }

        double GetJointPosition(const physics::JointPtr joint, const double axis_index = 0) {
            return joint->Position(axis_index);
        }

        void OnPosMsg(const geometry_msgs::QuaternionConstPtr &_msg) {
            calculateIK(_msg->x, _msg->y, _msg->z);
        }

        void OnGripMsg(const std_msgs::StringConstPtr &_msg) {
            std::stringstream ss(_msg->data.c_str());
            std::string s = ss.str();

            double angle = (s.find("grip") != std::string::npos) ? 0.41 : 0.0;

            jointController->SetPositionTarget(jointList[6]->GetScopedName(), +angle);
            jointController->SetPositionTarget(jointList[7]->GetScopedName(), -angle);
        }
    };

    GZ_REGISTER_MODEL_PLUGIN(JointRosSubscriberPlugin)
}
#endif