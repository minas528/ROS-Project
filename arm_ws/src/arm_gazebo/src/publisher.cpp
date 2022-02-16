//
// Created by TumsaUmata on 19/05/2021.
//

#include <iostream>
#include <string>
#include <functional>
#include <memory>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include "log.h"

namespace gazebo {
    class JointStatePublisher : public ModelPlugin {
    private:
        physics::ModelPtr model;
        physics::Joint_V jointList;
        sdf::ElementPtr sdf;
        event::ConnectionPtr updateConnection;

        int jointsCount = 0;
        std::vector<double> jPos;

        // ROS members
        std::unique_ptr<ros::NodeHandle> rosNodeUniquePtr;
        // ros::NodeHandle *node_handle_;
        ros::Publisher rosPubJointStates;
        ros::Timer timer;
        sensor_msgs::JointState jointStateMsg;

        void InitializeROSMembers() {
            if (!ros::isInitialized()) {
                int argc = 0;
                char **argv = nullptr;
                ros::init(argc, argv, "JSP", ros::init_options::NoSigintHandler);
                DebugMessage("ROS Node is initialized manually.", NO_COLOR);
            }
            this->rosNodeUniquePtr = std::make_unique<ros::NodeHandle>(this->model->GetName());
            this->rosPubJointStates = this->rosNodeUniquePtr->advertise<sensor_msgs::JointState>("/joint_states", 10, false);
            this->timer = this->rosNodeUniquePtr->createTimer(ros::Duration(0.2), &JointStatePublisher::timerCallback,
                                                                this);

            this->jointStateMsg = sensor_msgs::JointState();

            this->jointsCount = this->jointList.size();

            this->jointStateMsg.position.resize(this->jointsCount);

            this->jPos.resize(jointsCount);

            for (auto const &j : jointList) {
                this->jointStateMsg.name.push_back(j->GetName());
            }
        }


        void timerCallback(const ros::TimerEvent &event) {
            for (int i = 0; i < jointsCount; i++) {
                this->jointStateMsg.position[i] = this->jPos[i];
            }
            jointStateMsg.header.stamp = ros::Time::now();
            this->rosPubJointStates.publish(this->jointStateMsg);
        }

        double GetJointPosition(const physics::JointPtr joint, const double axis_index = 0) {
            return joint->Position(axis_index);
        }

    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
            this->model = _parent;
            this->sdf = _sdf;
            jointList = model->GetJoints();

            InitializeROSMembers();

            this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&JointStatePublisher::OnUpdate, this));

            DebugMessage("Plugin is loaded");
        }
        void OnUpdate() {
            for (int i = 0; i < jointsCount; i++) {
                this->jPos[i] = GetJointPosition(jointList[i], i ? 0 : 2);
            }
        }
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(JointStatePublisher)
}
