//
// Created by desktop on 21/05/2021.
//

#ifndef ARM_GAZEBO_LOG_H
#define ARM_GAZEBO_LOG_H

#define BLUE_TXT1 "\e[1;34m"
#define NO_COLOR "\033[0m"

void DebugMessage(const std::string Message, const std::string color = BLUE_TXT1, const bool set_no_color = true) {
    std::string st = color + "[JointsPublisher] " +Message + NO_COLOR;
    ROS_INFO("%s", st.c_str());
}

void DebugMessageERROR(const std::string Message) {
    ROS_ERROR("%s", Message.c_str());
}

#endif //ARM_GAZEBO_LOG_H
