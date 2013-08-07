/*
* Software License Agreement (BSD License) 
*
* Copyright (c) 2011, Southwest Research Institute
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   * Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*   * Neither the name of the Southwest Research Institute, nor the names 
*   of its contributors may be used to endorse or promote products derived
*   from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef JOINT_FEEDBACK_HANDLER_H
#define JOINT_HANDLER_H

#include <string>
#include <vector>
#include <map>

#include "ros/ros.h"
#include "control_msgs/FollowJointTrajectoryFeedback.h"
#include "sensor_msgs/JointState.h"
#include "simple_message/joint_data.h"
#include "simple_message/message_handler.h"
#include "simple_message/simple_message.h"
#include "simple_message/messages/joint_feedback_message.h"

namespace industrial_robot_client
{
namespace joint_feedback_handler
{

using industrial::joint_data::JointData;
using industrial::joint_feedback_message::JointFeedbackMessage;
using industrial::simple_message::SimpleMessage;
using industrial::smpl_msg_connection::SmplMsgConnection;


/**
 * \brief Message Handler that handles joint feedback messages for multiple
 * control groups.
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class JointFeedbackHandler : public industrial::message_handler::MessageHandler
{

    public:
        /** \brief Empty class constructor. */
        JointFeedbackHandler() {};

        /**
         * \brief Initialize joint feedback handler for single control group.
         *
         * \param[in] connection
         * \param[in] joint_manes vector of joint names
         *
         */
        bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection, std::vector<std::string> &joint_names);

        /**
         * \brief Initialize joint feedback handler for multiple cotrol groups.
         *
         * \param[in] connection
         * \param[in] joint_manes_map map of joint names vectors by robot id
         *
         */
        bool init(industrial::smpl_msg_connection::SmplMsgConnection* connection, std::map<int, std::vector<std::string> > &joint_names_map);

    protected:

        /** \brief ROS Node handle */
        ros::NodeHandle node_;

        /** \brief Map of joint names by the robot id */
        std::map<int, std::vector<std::string> > joint_names_map_;

        ros::Publisher pub_joint_control_state_;
        ros::Publisher pub_joint_sensor_state_;

        /**
         * \brief Convert ROS-I JointMessage to std ROS messages.
         *
         * \param[in] msg_in
         * \param[out] control_state
         * \param[out] sensor_state
         *
         * \return true if ok, false in case of error
         */
        virtual bool create_messages(JointFeedbackMessage& msg_in,
                                     control_msgs::FollowJointTrajectoryFeedback* control_state,
                                     sensor_msgs::JointState* sensor_state);


        /**
         * \brief Get joint names for particular control group.
         *
         * \param[in]  robot_id
         * \param[out] joint_names
         *
         * \return true if ok, false in case of error
         */
        virtual bool get_joint_names(int robot_id, std::vector<std::string>* joint_names);

        /**
         * \brief internal callback
         */
        bool internalCB(JointFeedbackMessage& in);

    private:

        /**
         * \brief Internal callback called by MessageManager, it performs
         * conversion to the JointFeedbackMessage.
         */
        bool internalCB(SimpleMessage& in);

};

}
}


#endif
