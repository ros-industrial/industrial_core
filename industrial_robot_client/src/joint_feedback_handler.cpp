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

#include "industrial_robot_client/joint_feedback_handler.h"
#include "simple_message/log_wrapper.h"
#include "industrial_utils/utils.h"

using industrial::joint_data::JointData;
using namespace industrial::simple_message;
using namespace industrial_utils;

namespace industrial_robot_client
{
namespace joint_feedback_handler
{
bool JointFeedbackHandler::init(industrial::smpl_msg_connection::SmplMsgConnection* connection, std::vector<std::string> &joint_names)
{
    std::map<int, std::vector<std::string> > joint_names_map;
    joint_names_map[0] = joint_names;
    return init(connection, joint_names_map);
}

bool JointFeedbackHandler::init(industrial::smpl_msg_connection::SmplMsgConnection* connection, std::map<int, std::vector<std::string> > &joint_names_map)
{
    // Register publishers
    this->pub_joint_control_state_ = this->node_.advertise<control_msgs::FollowJointTrajectoryFeedback>("feedback_states", 1);
    this->pub_joint_sensor_state_ = this->node_.advertise<sensor_msgs::JointState>("joint_states", 1);

    // Store joint names
    this->joint_names_map_ = joint_names_map;

    // Initialize message handler
    return MessageHandler::init((int)StandardMsgTypes::JOINT_FEEDBACK, connection);
}

bool JointFeedbackHandler::create_messages(JointFeedbackMessage& msg_in,
                                           control_msgs::FollowJointTrajectoryFeedback* control_state,
                                           sensor_msgs::JointState* sensor_state)
{

    std::vector<std::string> joint_names;
    get_joint_names(msg_in.getRobotID(), &joint_names);
    JointData values;
    int num_jnts = joint_names.size();

    *control_state = control_msgs::FollowJointTrajectoryFeedback();  // always start with a "clean" message
    control_state->header.stamp = ros::Time::now();
    control_state->joint_names = joint_names;

    // copy position data
    if (msg_in.getPositions(values))
    {
        if (!jointData2Vector(values, num_jnts, control_state->actual.positions))
        {
            LOG_ERROR("Failed to parse position data from JointFeedbackMessage");
            return false;
        }
    } else {
        control_state->actual.positions.clear();
    }

    // copy velocity data
    if (msg_in.getVelocities(values))
    {
        if (!jointData2Vector(values, num_jnts, control_state->actual.velocities))
        {
            LOG_ERROR("Failed to parse velocity data from JointFeedbackMessage");
            return false;
        }
    } else {
        control_state->actual.velocities.clear();
    }

    // copy acceleration data
    if (msg_in.getAccelerations(values))
    {
        if (!jointData2Vector(values, num_jnts, control_state->actual.accelerations))
        {
            LOG_ERROR("Failed to parse acceleration data from JointFeedbackMessage");
            return false;
        }
    } else {
        control_state->actual.accelerations.clear();
    }

    control_state->actual.time_from_start = ros::Duration(0);

    *sensor_state = sensor_msgs::JointState();  // always start with a "clean" message
    sensor_state->header.stamp = ros::Time::now();
    sensor_state->name = joint_names;
    sensor_state->position = control_state->actual.positions;
    sensor_state->velocity = control_state->actual.velocities;

    return true;
}
bool JointFeedbackHandler::get_joint_names(int robot_id, std::vector<std::string>* joint_names)
{
    bool ret = true;
    joint_names->clear();

    if (this->joint_names_map_.count(robot_id) > 0) {
        std::vector<std::string> tmp;
        tmp = this->joint_names_map_[robot_id];
        for (std::vector<std::string>::iterator it = tmp.begin(); it != tmp.end(); ++it)
                joint_names->push_back(*it);
    } else {
        ROS_ERROR("Undefined control group: %d", robot_id);
        ret = false;
    }

    if (joint_names->size() <= 0) {
        ROS_ERROR("Empty joint names");
        ret = false;
    }

    return ret;
}

bool JointFeedbackHandler::internalCB(JointFeedbackMessage& in)
{

  control_msgs::FollowJointTrajectoryFeedback control_state;
  sensor_msgs::JointState sensor_state;
  bool rtn = true;

  if (create_messages(in, &control_state, &sensor_state))
  {
    this->pub_joint_control_state_.publish(control_state);
    this->pub_joint_sensor_state_.publish(sensor_state);
  } else {
    rtn = false;
  }

  // Reply back to the controller if the sender requested it.
  if (CommTypes::SERVICE_REQUEST == in.getMessageType())
  {
    SimpleMessage reply;
    reply.init(in.getMessageType(), CommTypes::SERVICE_REPLY, rtn ? ReplyTypes::SUCCESS : ReplyTypes::FAILURE);
    this->getConnection()->sendMsg(reply);
  }

  return rtn;
}


bool JointFeedbackHandler::internalCB(SimpleMessage& in)
{
    JointFeedbackMessage tmp_msg;

    if (!tmp_msg.init(in)) {
        return false;
    }

    return internalCB(tmp_msg);
}


} // end joint_feedback_relay_handler
} // end industrial_robot_client
